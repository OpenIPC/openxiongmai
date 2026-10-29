// SPDX-License-Identifier: GPL-2.0
/*
 * xm_v200_venc: the V200 series' VENC frame queue, a drop-in replacement for
 * the vendor's venc.ko (/dev/venc_info; built as open_venc.ko).
 *
 * VPSS (or VI) hands the pictures bound to VENC to vb.ko's VENC list. A
 * thread takes them from there, has rgn.ko draw the OSD regions on them
 * (osd_process) and queues them, up to fifo_depth; past that they go back
 * to the free list. User space takes them from the queue and gives them
 * back when they are encoded:
 *
 *   0xc0284902  the next picture, waiting up to 2 s: a 40-byte record
 *               {VENC chn, phys, width, height, stride, 0, pts lo, pts hi,
 *               ...} with the buffer's descriptor address at word 8 (the
 *               XM535: word 9, after a word of the descriptor's own)
 *   0x40284903  that record back: the picture goes to the free list
 *   0x4900      start (the queue empty); 0x41044901 (0x40c44901 on the
 *               XM533, 0x40c04901 on the XM535W1) a channel's settings, for
 *               /proc; 0x40104904 16 bytes kept; 0x40284905 (XM535) the
 *               OSD regions composed on a channel's picture
 *
 * The thread waits from the first open of the device until 0x4900. The
 * buffer descriptors are vb.ko's: VENC_LAYOUT=v200 (XM530V200, XM533) or
 * xm535 at build time, as for open_vb.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

/* vb.ko and rgn.ko */
int vb_get_buf(u32 list, u32 size, void **buf, int ms);
int vb_put_buf(u32 list, void *buf);
int osd_process(void *frame, void *bind);
#ifdef VENC_LAYOUT_XM535
int comp_osd_process(void *frame, void *bind);
#endif

#ifdef VENC_LAYOUT_XM535
#define W_OWN 9	   /* the XM535's word: set, the OSD is done already */
#define W_BIND 14  /* id, dev, chn */
#define W_PTS 12
#define REC_BUF 9  /* the descriptor's address in the record */
#else
#define W_BIND 12
#define W_PTS 10
#define REC_BUF 8
#endif
#define W_PHYS 4
#define W_STRIDE 6
#define W_WIDTH 7
#define W_HEIGHT 8

#define VB_FREE 0
#define VB_VENC 2

#define VENC_GET 0xc0284902
#define VENC_RELEASE 0x40284903
#define VENC_INIT 0x4900
#define VENC_SET4 0x40104904
#define VENC_OSD 0x40284905
#define VENC_NR_CFG 1
#define N_CHN 33
#define CFG_MAX 0x104

static unsigned int fifo_depth = 4;
module_param(fifo_depth, uint, 0644);
MODULE_PARM_DESC(fifo_depth, "venc list fifo depth");

/* a picture: vb.ko's descriptor, words 0 and 1 its list link */
struct pic {
	struct list_head node;
	u32 w[];
};
#define WORD(p, n) ((p)->w[(n) - 2])

static LIST_HEAD(fifo);
static DEFINE_SPINLOCK(fifo_lock);
static DECLARE_WAIT_QUEUE_HEAD(fifo_wq);
static struct {
	u32 vb_fail, fifo_full, fifo_empty, count, max;
} state;
static u8 chn_cfg[N_CHN][CFG_MAX];
static u32 chn_cfg_size;
static u32 set4[4];
static atomic_t opens = ATOMIC_INIT(0);
static volatile int wait_init;
static struct task_struct *thread;

/* the pictures user space holds, to take back only those */
#define MAX_OUT 64
static void *out[MAX_OUT];

static void out_add(void *p)
{
	int i;

	for (i = 0; i < MAX_OUT; i++)
		if (!out[i]) {
			out[i] = p;
			return;
		}
}

static bool out_take(void *p)
{
	int i;

	for (i = 0; i < MAX_OUT; i++)
		if (p && out[i] == p) {
			out[i] = NULL;
			return true;
		}
	return false;
}

static void drop(struct pic *p)
{
	memset(&WORD(p, W_BIND), 0, 12);
	if (vb_put_buf(VB_FREE, p))
		pr_err("venc: put buffer %p failed\n", p);
}

static int venc_thread(void *arg)
{
	while (!kthread_should_stop()) {
		struct pic *p;
		unsigned long flags;
		u32 frame[4];

		if (!atomic_read(&opens) || wait_init) {
			schedule_timeout_interruptible(1);
			continue;
		}
		if (vb_get_buf(VB_VENC, 0, (void **)&p, 2000)) {
			state.vb_fail++;
			schedule_timeout_interruptible(1);
			continue;
		}
		if (state.count >= fifo_depth) {
			state.fifo_full++;
			drop(p);
			continue;
		}
#ifdef VENC_LAYOUT_XM535
		if (!WORD(p, W_OWN))
#endif
		{
			frame[0] = WORD(p, W_STRIDE);
			frame[1] = WORD(p, W_HEIGHT);
			frame[2] = frame[3] = WORD(p, W_PHYS);
			osd_process(frame, &WORD(p, W_BIND));
		}
		spin_lock_irqsave(&fifo_lock, flags);
		list_add_tail(&p->node, &fifo);
		state.count++;
		if (state.count > state.max)
			state.max = state.count;
		spin_unlock_irqrestore(&fifo_lock, flags);
		wake_up(&fifo_wq);
	}
	return 0;
}

static void venc_start(void)
{
	unsigned long flags;

	spin_lock_irqsave(&fifo_lock, flags);
	INIT_LIST_HEAD(&fifo);
	memset(&state, 0, sizeof(state));
	spin_unlock_irqrestore(&fifo_lock, flags);
	memset(chn_cfg, 0, sizeof(chn_cfg));
	memset(set4, 0, sizeof(set4));
	memset(out, 0, sizeof(out));
	wait_init = 0;
}

static long venc_get(void __user *arg)
{
	struct pic *p = NULL;
	unsigned long flags;
	u32 r[10];
	long t;

	t = wait_event_interruptible_timeout(fifo_wq, !list_empty(&fifo),
					     msecs_to_jiffies(2000));
	if (t <= 0) {
		state.fifo_empty++;
		return -ERESTARTSYS;
	}
	spin_lock_irqsave(&fifo_lock, flags);
	if (!list_empty(&fifo)) {
		p = list_first_entry(&fifo, struct pic, node);
		list_del(&p->node);
		state.count--;
	}
	spin_unlock_irqrestore(&fifo_lock, flags);
	if (!p)
		return -ERESTARTSYS;
	memset(r, 0, sizeof(r));
	r[0] = WORD(p, W_BIND + 2);
	r[1] = WORD(p, W_PHYS);
	r[2] = WORD(p, W_WIDTH);
	r[3] = WORD(p, W_HEIGHT);
	r[4] = WORD(p, W_STRIDE);
	r[6] = WORD(p, W_PTS);
	r[7] = WORD(p, W_PTS + 1);
#ifdef VENC_LAYOUT_XM535
	r[8] = WORD(p, W_OWN);
#endif
	r[REC_BUF] = (u32)(uintptr_t)p;
	out_add(p);
	if (copy_to_user(arg, r, sizeof(r))) {
		pr_err("venc: copy_to_user failed\n");
		return -1;
	}
	return 0;
}

static long venc_release(void __user *arg)
{
	u32 r[10];
	struct pic *p;

	if (copy_from_user(r, arg, sizeof(r))) {
		pr_err("venc: copy_from_user failed\n");
		return -1;
	}
	p = (struct pic *)(uintptr_t)r[REC_BUF];
	if (!out_take(p)) {
		pr_err("venc: release of no picture %#x\n", r[REC_BUF]);
		return -1;
	}
	memset(&WORD(p, W_BIND), 0, 12);
	if (vb_put_buf(VB_FREE, p)) {
		pr_err("venc: put buffer failed\n");
		return -1;
	}
	return 0;
}

static long venc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	u32 size = _IOC_SIZE(cmd);

	if (_IOC_TYPE(cmd) != 0x49 || _IOC_NR(cmd) > 5)
		return -ENOTTY;
	switch (cmd) {
	case VENC_GET:
		return venc_get(uarg);
	case VENC_RELEASE:
		return venc_release(uarg);
	case VENC_INIT:
		venc_start();
		return 0;
	case VENC_SET4:
		if (copy_from_user(set4, uarg, sizeof(set4)))
			break;
		return 0;
#ifdef VENC_LAYOUT_XM535
	case VENC_OSD: {
		u32 r[10], frame[4] = {0}, bind[3] = {7, 0, 0};

		if (copy_from_user(r, uarg, sizeof(r)))
			break;
		frame[0] = r[2];
		frame[1] = r[3];
		frame[2] = frame[3] = r[1];
		bind[2] = r[0];
		comp_osd_process(frame, bind);
		return 0;
	}
#endif
	}
	if (_IOC_NR(cmd) == VENC_NR_CFG && _IOC_DIR(cmd) == _IOC_WRITE &&
	    size >= 8 && size <= CFG_MAX) {
		/* a channel's settings: {chn, enabled, ...} */
		u8 c[CFG_MAX];
		u32 chn, on;

		if (copy_from_user(c, uarg, size))
			goto fault;
		memcpy(&chn, c, 4);
		memcpy(&on, c + 4, 4);
		if (chn >= N_CHN) {
			pr_err("venc: chn %u is invalid\n", chn);
			return 0;
		}
		chn_cfg_size = size;
		if (on)
			memcpy(chn_cfg[chn], c, size);
		else
			memset(chn_cfg[chn] + 4, 0, 4);
		return 0;
	}
	pr_err("venc: command error %x\n", cmd);
	return -1;
fault:
	pr_err("venc: copy_from_user failed\n");
	return -1;
}

static int venc_open(struct inode *inode, struct file *file)
{
	if (atomic_inc_return(&opens) == 1)
		wait_init = 1; /* until the pipeline says it starts */
	return 0;
}

static int venc_close(struct inode *inode, struct file *file)
{
	atomic_dec(&opens);
	return 0;
}

static int venc_show(struct seq_file *s, void *v)
{
	u32 c, w[6];

	seq_puts(s, "[VENC] openxiongmai xm_v200_venc\n\n");
	seq_puts(s, "-------VIBUF FIFO STATUS----------------------------------------------------------\n");
	seq_puts(s, "VbFail  FifoFull  FifoEmpty  FifoDepth  MaxDepth\n");
	seq_printf(s, "%-6u  %-8u  %-9u  %-9u  %-8u\n", state.vb_fail,
		   state.fifo_full, state.fifo_empty, state.count, state.max);
	seq_puts(s, "\n-------CHANNELS--------------------------------------------------------------------\n");
	seq_puts(s, "Chn  Type   Width  Height  BufSize\n");
	for (c = 0; c < N_CHN; c++) {
		memcpy(w, chn_cfg[c], sizeof(w));
		if (w[1])
			seq_printf(s, "%-3u  %#-5x  %-5u  %-6u  %#x\n", c, w[2], w[3],
				   w[4], chn_cfg_size > 0x24 ? *(u32 *)(chn_cfg[c] + 0x20) : 0);
	}
	return 0;
}

static int venc_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, venc_show, NULL);
}

static const struct file_operations venc_proc_fops = {
	.owner = THIS_MODULE,
	.open = venc_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations venc_fops = {
	.owner = THIS_MODULE,
	.open = venc_open,
	.release = venc_close,
	.unlocked_ioctl = venc_ioctl,
};

static struct miscdevice venc_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "venc_info",
	.fops = &venc_fops,
};

static int __init xm_v200_venc_init(void)
{
	int ret;

	venc_start();
	if (!proc_create("umap/venc", 0444, NULL, &venc_proc_fops))
		pr_err("venc: create venc proc fail!\n");
	ret = misc_register(&venc_misc);
	if (ret) {
		pr_err("venc: register device failed!\n");
		remove_proc_entry("umap/venc", NULL);
		return ret;
	}
	thread = kthread_run(venc_thread, NULL, "venc_thread");
	if (IS_ERR(thread)) {
		thread = NULL;
		misc_deregister(&venc_misc);
		remove_proc_entry("umap/venc", NULL);
		return -ENOMEM;
	}
	pr_info("venc: openxiongmai xm_v200_venc init ok\n");
	return 0;
}

static void __exit xm_v200_venc_exit(void)
{
	if (thread)
		kthread_stop(thread);
	misc_deregister(&venc_misc);
	remove_proc_entry("umap/venc", NULL);
}

module_init(xm_v200_venc_init);
module_exit(xm_v200_venc_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V200-series VENC frame queue, compatible with venc.ko");
