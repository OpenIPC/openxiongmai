// SPDX-License-Identifier: GPL-2.0
/*
 * xm530_mve: the XM530/XM550's H.265 encoder device (an Arm Mali-V550, host
 * protocol 2), meant as a drop-in for the vendor's mve_base.ko with the same
 * /dev/mv500 ABI. It works on the vendor's mve_rsrc.ko, which schedules
 * sessions on the cores, and loads the same firmware, hevcenc.fwb.
 *
 * One ioctl runs a command: {command, size} and size bytes of payload, all of
 * it overwritten by {status, firmware error, size} and the reply. The other
 * hands in the 160 bytes of numbers /proc/umap/h265 shows for a channel.
 *
 * Written from mve_base.ko's disassembly. On an XM550 its H.265 streams are
 * byte for byte mve_base.ko's.
 *
 * It encodes H.265 from NV12 in MMZ (/dev/mmz mappings), what the vendor
 * library and libxmvenc do. Decoding, other codecs, secure sessions, dma-buf
 * and ashmem buffers, interlaced frames and user space answering the
 * firmware's memory calls are refused, not implemented.
 *
 * What else differs from mve_base.ko:
 *  - /proc/umap/h265 is released on close; there, seq_open() is never
 *    released and leaks 4 KiB each time (OpenIPC/firmware#2956);
 *  - a reply that cannot be copied out is freed;
 *  - the channel number of the statistics is checked (it indexes kernel
 *    memory), and the regions of interest are counted;
 *  - an unknown ioctl is -ENOTTY, not 0;
 *  - waits for a lock are not cut short by a signal;
 *  - the watchdog does not take the scheduler's lock under a session's.
 */

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "mve.h"

#define MVE_IOC_CMD 0xc008fb00
#define MVE_IOC_STATS 0xc0a0fb01
#define PAYLOAD_MAX 4096

enum {
	CMD_CREATE,
	CMD_DESTROY,
	CMD_ACTIVATE,
	CMD_FLUSH,
	CMD_STATE,
	CMD_EVENT,
	CMD_SET_PARAMETER,
	CMD_GET_PARAMETER,
	CMD_SET_CONFIG,
	CMD_GET_CONFIG,
	CMD_REGISTER,
	CMD_UNREGISTER,
	CMD_FILL,
	CMD_EMPTY,
	CMD_RELEASE_REF,
	CMD_MAX_FREQUENCY,
	CMD_HW_INFO,
	CMD_RPC_ALLOC,
	CMD_RPC_RESIZE,
};

struct reply {
	u32 status, fw_error, size;
};

/* The numbers for /proc/umap/h265 */
#define CHNS 3
#define STATS 40
static u32 stats[CHNS][STATS];
static DEFINE_MUTEX(stats_lock);

/* A command; the reply data, if any, is allocated into *data */
static struct reply run(struct file *file, u32 cmd, const u32 *in, u32 size,
			void **data, void **owner)
{
	struct reply r = {MVE_OK, 0, 0};
	struct mve_session *s;
	struct mve_event *ev;
	u32 *w;

	if (cmd == CMD_MAX_FREQUENCY) {
		mver_dvfs_request_max_frequency();
		return r;
	}
	if (cmd == CMD_HW_INFO) {
		w = kmalloc(12, GFP_KERNEL);
		if (!w) {
			r.status = MVE_NOMEM;
			return r;
		}
		w[0] = mver_reg_get_fuse();
		w[1] = mver_reg_get_version();
		w[2] = mver_scheduler_get_ncores();
		*data = *owner = w;
		r.size = 12;
		return r;
	}
	if (cmd == CMD_DESTROY) {
		mve_session_destroy(file);
		return r;
	}
	s = mve_session_get(file);
	if (cmd == CMD_CREATE) {
		if (s) {
			r.status = MVE_NOMEM; /* one for each open */
			mve_session_put(s);
		} else if (!size || ((const char *)in)[size - 1]) {
			r.status = MVE_NOT_IMPLEMENTED;
		} else {
			r.status = mve_session_create(file, (const char *)in);
		}
		return r;
	}
	if (!s) {
		r.status = MVE_BAD_PARAM;
		return r;
	}
	switch (cmd) {
	case CMD_ACTIVATE:
		if (size) { /* a secure session's firmware */
			r.status = MVE_NOT_IMPLEMENTED;
			break;
		}
		w = kmalloc(4, GFP_KERNEL);
		if (!w) {
			r.status = MVE_NOMEM;
			break;
		}
		r.status = mve_session_activate(s, w);
		if (r.status) {
			kfree(w);
			break;
		}
		*data = *owner = w;
		r.size = 4;
		break;
	case CMD_FLUSH:
		r.status = size < 4 ? MVE_BAD_PARAM :
				      mve_session_flush(s, in[0]);
		break;
	case CMD_STATE:
		r.status = size < 4 ? MVE_BAD_PARAM :
				      mve_session_set_state(s, in[0]);
		break;
	case CMD_EVENT:
		ev = size < 4 ? NULL : mve_session_event(s, in[0]);
		if (!ev) {
			r.status = MVE_TIMEOUT;
			break;
		}
		*owner = ev;
		*data = &ev->e;
		r.size = 4 + ev->e.size;
		break;
	case CMD_SET_PARAMETER:
	case CMD_SET_CONFIG:
		r.status = mve_session_config(s, in, size, &r.fw_error);
		break;
	case CMD_REGISTER:
		if (size < 4 + sizeof(struct mve_user_buffer))
			r.status = MVE_BAD_PARAM;
		else if (in[0] > 1)
			r.status = MVE_BAD_PORT;
		else {
			struct mve_user_buffer b;

			memcpy(&b, in + 1, sizeof(b));
			r.status = mve_session_register(s, in[0], &b);
		}
		break;
	case CMD_UNREGISTER:
	case CMD_RELEASE_REF: {
		u64 handle;

		if (size < 8) {
			r.status = MVE_BAD_PARAM;
			break;
		}
		memcpy(&handle, in, 8);
		r.status = cmd == CMD_UNREGISTER ?
				   mve_session_unregister(s, handle) :
				   mve_session_release_ref(s, handle);
		break;
	}
	case CMD_FILL:
	case CMD_EMPTY:
		if (size < sizeof(struct mve_user_queue))
			r.status = MVE_BAD_PARAM;
		else
			r.status = mve_session_enqueue(
				s, (const struct mve_user_queue *)in,
				cmd == CMD_EMPTY);
		break;
	default: /* GET_PARAMETER and GET_CONFIG: protocol 2 has none */
		r.status = MVE_NOT_IMPLEMENTED;
		break;
	}
	mve_session_put(s);
	return r;
}

static long mve_command(struct file *file, void __user *arg)
{
	void *in = NULL, *data = NULL, *owner = NULL;
	struct reply r;
	long ret = 0;
	u32 req[2];

	if (copy_from_user(req, arg, sizeof(req)))
		return -EFAULT;
	if (req[1] > PAYLOAD_MAX)
		return -EINVAL;
	if (req[1]) {
		in = memdup_user(arg + sizeof(req), req[1]);
		if (IS_ERR(in))
			return PTR_ERR(in);
	}
	r = run(file, req[0], in, req[1], &data, &owner);
	if (copy_to_user(arg, &r, sizeof(r)) ||
	    (r.size && copy_to_user(arg + sizeof(r), data, r.size)))
		ret = -EFAULT;
	kfree(owner);
	kfree(in);
	return ret;
}

static long mve_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 s[STATS];

	switch (cmd) {
	case MVE_IOC_CMD:
		return mve_command(file, (void __user *)arg);
	case MVE_IOC_STATS:
		if (copy_from_user(s, (void __user *)arg, sizeof(s)))
			return -EFAULT;
		if (s[0] >= CHNS)
			return -EINVAL;
		mutex_lock(&stats_lock);
		memcpy(stats[s[0]], s, sizeof(s));
		mutex_unlock(&stats_lock);
		return 0;
	}
	return -ENOTTY;
}

static unsigned int mve_poll(struct file *file, poll_table *wait)
{
	return mve_session_poll(file, wait);
}

static int mve_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int mve_release(struct inode *inode, struct file *file)
{
	mve_session_destroy(file);
	return 0;
}

static int h265_show(struct seq_file *m, void *unused)
{
	const u32 *s;
	int i;

	mutex_lock(&stats_lock);
	seq_puts(m, "\n\n[H265] openxiongmai xm530_mve\n");
	seq_puts(m, "----------------------------------CHN ATTR------------"
		    "------------------------\n");
	seq_puts(m, "ID  Width  Height  InLBase     InCbBase    InCrBase    "
		    "MainStr\n");
	for (i = 0; i < CHNS; i++) {
		s = stats[i];
		seq_printf(m,
			   "%-2d  %-5d  %-6d  0x%-8x  0x%-8x  0x%-8x  %-7s\n",
			   s[0], s[28], s[29], s[30], s[31], s[32],
			   i ? "NO" : "YES");
	}
	seq_puts(m, "------------------------RC  INFO "
		    "---------------------------\n");
	seq_puts(m, "ID  QPMin  QPMax  GOPLen  QpDelta  rcMod  bitRate  "
		    "frameRate\n");
	for (i = 0; i < CHNS; i++) {
		s = stats[i];
		seq_printf(m,
			   "%-2d  %-5d  %-5d  %-6d  %-7d  %-5s  %-7d  %-9d\n",
			   s[0], s[33], s[34], s[35], s[36],
			   s[37] == 1 ? "CBR" : "VBR", s[38], s[39]);
	}
	seq_puts(m, "-----CHN EVENT----------------------------------------"
		    "-------------------\n");
	seq_puts(m, "Event     CHN0      CHN1\n");
	seq_printf(m, "%-8s  %-8d  %-8d\n", "GetErr", stats[0][1],
		   stats[1][1]);
	seq_printf(m, "%-8s  %-8d  %-8d\n", "timeOut", stats[0][2],
		   stats[1][2]);
	for (i = 0; i < 25; i++)
		if (stats[0][3 + i] || stats[1][3 + i])
			seq_printf(m, "%-8d  %-8d  %-8d\n", i, stats[0][3 + i],
				   stats[1][3 + i]);
	mutex_unlock(&stats_lock);
	return 0;
}

static int h265_open(struct inode *inode, struct file *file)
{
	return single_open(file, h265_show, NULL);
}

static const struct file_operations h265_fops = {
	.owner = THIS_MODULE,
	.open = h265_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations mve_fops = {
	.owner = THIS_MODULE,
	.open = mve_open,
	.release = mve_release,
	.unlocked_ioctl = mve_ioctl,
	.poll = mve_poll,
};

static struct miscdevice mve_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mv500",
	.fops = &mve_fops,
};

static struct proc_dir_entry *h265_proc;

static int __init xm530_mve_init(void)
{
	int ret;

	BUILD_BUG_ON(sizeof(struct mve_user_buffer) != 120);
	BUILD_BUG_ON(sizeof(struct mve_user_queue) != 208);
	ret = misc_register(&mve_misc);
	if (ret) {
		pr_err("mve: register device failed\n");
		return ret;
	}
	mve_sessions_init(mve_misc.this_device);
	/* /proc/umap is the MMZ module's */
	h265_proc = proc_create("umap/h265", 0444, NULL, &h265_fops);
	if (!h265_proc)
		pr_warn("mve: no /proc/umap/h265, MMZ not loaded?\n");
	return 0;
}

static void __exit xm530_mve_exit(void)
{
	if (h265_proc)
		remove_proc_entry("umap/h265", NULL);
	misc_deregister(&mve_misc);
	mve_sessions_exit();
}

module_init(xm530_mve_init);
module_exit(xm530_mve_exit);
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("hevcenc.fwb");
MODULE_DESCRIPTION("XM530/XM550 H.265 encoder, compatible with mve_base.ko");
