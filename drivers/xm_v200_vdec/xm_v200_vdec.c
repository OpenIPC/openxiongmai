// SPDX-License-Identifier: GPL-2.0
/*
 * The XM530V200's video decoder bridge (/dev/vdec): a drop-in for the
 * vendor vdec.ko. The decoding is user space's; this hands it frame
 * buffers from the VB pools and the VO channels, passes the decoded ones
 * on, keeps the decoder's state table for /proc/umap/vdec and gives a
 * microsecond delay from a hardware timer.
 *
 * ioctls take a 36-byte record, words:
 *
 *   0, 1  out: the buffer's physical address, size; VB get: 1 the size
 *   2..4  VB put: stride, width, height
 *   5     VB put: the bind id (7 VENC, 8 VPSS, 17 var; anything else the
 *         free list); VO put: 15 shows the frame, else it is dropped
 *   6, 7  VB put: bind dev, chn; VO get and put: VO dev, chn
 *   8     the buffer (descriptor address), out of a get, into a put
 *
 *   0xc0244402  a VO channel's free buffer (vo_get_buf)
 *   0x40244403  a VO buffer back, shown or not (vo_put_buf)
 *   0xc0244404  a free VB buffer at least that big (vb_get_buf, no wait)
 *   0x40244405  a VB buffer to the module it is bound to (vb_put_buf)
 *   0x40040144  {us}: sleep that long on timer 5 (0x100D1020, IRQ 38),
 *               a second at most
 *   0x40e80044  the 8 x 232-byte state table /proc/umap/vdec shows
 *
 * The buffers are checked to be VB's (vb_find_buf) or the channel's (VO);
 * vdec.ko takes them on trust.
 */

#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

int vb_get_buf(u32 list, u32 size, void **buf, int ms);
int vb_put_buf(u32 list, void *buf);
void *vb_find_buf(u32 addr);
int vo_get_buf(u32 dev, u32 chn, void **buf);
int vo_put_buf(u32 dev, u32 chn, void *buf, int show);

#define VDEC_VO_GET	0xc0244402
#define VDEC_VO_PUT	0x40244403
#define VDEC_VB_GET	0xc0244404
#define VDEC_VB_PUT	0x40244405
#define VDEC_DELAY	0x40040144
#define VDEC_SET_INFO	0x40e80044

/* the second timer of the 0x100D1000 SP804 pair */
#define TIMER_BASE	0x100d1020
#define TIMER_IRQ	38
#define T_LOAD		0x00
#define T_CTRL		0x08
#define T_INTCLR	0x0c
#define T_MIS		0x14
#define T_ENABLE	0x80
/* one-shot, 32-bit, interrupting (vdec.ko runs it free and stops it in
 * the interrupt: the same on the chip) */
#define T_MODE		0x23
#define T_TICKS_US	24	/* as vdec.ko counts */

/* a buffer descriptor (xm_v200_vb's v200 layout) */
#define W_PHYS		4
#define W_SIZE		5
#define W_STRIDE	6
#define W_BIND		12

#define N_DEV		8
#define INFO_SIZE	0xe8

struct vdec_rec {
	u32 phys, size;
	u32 stride, width, height;
	u32 bind;
	u32 dev, chn;
	u32 buf;
};

static void __iomem *timer;
static DECLARE_WAIT_QUEUE_HEAD(delay_wq);
static DEFINE_MUTEX(delay_lock);
static int delay_done;
static u32 info[N_DEV][INFO_SIZE / 4];
static DEFINE_MUTEX(info_lock);

static irqreturn_t vdec_delay_irq(int irq, void *dev)
{
	if (!(readl(timer + T_MIS) & 1))
		return IRQ_NONE;
	writel(1, timer + T_INTCLR);
	writel(T_MODE, timer + T_CTRL);
	delay_done = 1;
	wake_up(&delay_wq);
	return IRQ_HANDLED;
}

static int vdec_delay(u32 us)
{
	u32 ticks = us * T_TICKS_US;

	mutex_lock(&delay_lock);
	delay_done = 0;
	writel(T_MODE, timer + T_CTRL);
	writel(ticks, timer + T_LOAD);
	writel(T_MODE | T_ENABLE, timer + T_CTRL);
	wait_event_interruptible_timeout(delay_wq, delay_done, HZ);
	mutex_unlock(&delay_lock);
	return 0;
}

static int rec_out(void __user *u, struct vdec_rec *r, u32 *b)
{
	r->phys = b[W_PHYS];
	r->size = b[W_SIZE];
	r->buf = (u32)(uintptr_t)b;
	return copy_to_user(u, r, sizeof(*r)) ? -EFAULT : 0;
}

static long vdec_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *u = (void __user *)arg;
	struct vdec_rec r;
	u32 *b;
	u32 us, list;

	switch (cmd) {
	case VDEC_DELAY:
		if (get_user(us, (u32 __user *)u))
			return -EFAULT;
		return vdec_delay(us);
	case VDEC_SET_INFO:
		mutex_lock(&info_lock);
		if (copy_from_user(info, u, sizeof(info))) {
			mutex_unlock(&info_lock);
			return -EFAULT;
		}
		mutex_unlock(&info_lock);
		return 0;
	case VDEC_VO_GET:
	case VDEC_VO_PUT:
	case VDEC_VB_GET:
	case VDEC_VB_PUT:
		break;
	default:
		pr_err("vdec: ioctl cmd %x error\n", cmd);
		return -ENOTTY;
	}
	if (copy_from_user(&r, u, sizeof(r)))
		return -EFAULT;
	switch (cmd) {
	case VDEC_VO_GET:
		if (vo_get_buf(r.dev, r.chn, (void **)&b) || !b)
			return -1;
		return rec_out(u, &r, b);
	case VDEC_VO_PUT:
		if (vo_put_buf(r.dev, r.chn, (void *)(uintptr_t)r.buf,
			       r.bind == 15)) {
			pr_err("vo_put_buf fail\n");
			return -1;
		}
		return 0;
	case VDEC_VB_GET:
		if (vb_get_buf(0, r.size, (void **)&b, 0))
			return -1;
		return rec_out(u, &r, b);
	}
	/* VDEC_VB_PUT */
	b = vb_find_buf(r.buf);
	if (!b) {
		pr_err("vdec: put of no buffer %#x\n", r.buf);
		return -EINVAL;
	}
	/* the NV12 picture within the buffer */
	if (r.stride > 8192 || r.height > 8192 ||
	    (u64)r.stride * r.height * 3 / 2 > b[W_SIZE]) {
		pr_err("vdec: %ux%u does not fit buffer %#x\n", r.stride,
		       r.height, r.buf);
		return -EINVAL;
	}
	b[W_STRIDE] = r.stride;
	b[W_STRIDE + 1] = r.width;
	b[W_STRIDE + 2] = r.height;
	b[W_STRIDE + 4] = 0;
	b[W_STRIDE + 5] = 0;
	switch (r.bind) {
	case 7:
		list = 2;
		break;
	case 8:
		list = 1;
		break;
	case 17:
		list = 3;
		break;
	default:
		list = 0;
		r.bind = 0;
	}
	b[W_BIND] = r.bind;
	b[W_BIND + 1] = r.dev;
	b[W_BIND + 2] = r.chn;
	return vb_put_buf(list, b);
}

static const struct file_operations vdec_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = vdec_ioctl,
};

static struct miscdevice vdec_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vdec",
	.fops = &vdec_fops,
};

/* the state table, words: 0 type, 1, 2 max w, h, 3, 4 w, h, 5 buffer
 * size, 6 on; 7 started, 8 received, 9 decoded, 10 frame rate, 11 max,
 * 12 min, 13 decode time; 15 crop, 16..19 crop x, y, w, h, 20 dst w,
 * 21 stride, 22 dst h, 23, 24 src, dst rate, 25..27 bind id, dev, chn,
 * 28 channel on, 29 sent, 30 frame rate, 31, 32 set, current depth;
 * 34..39 the input buffer: start, end, write and read node, addresses */
static const char *bind_name(u32 id)
{
	switch (id) {
	case 0: return "free";
	case 5: return "vdec";
	case 7: return "venc";
	case 8: return "vpss";
	case 15: return "vo";
	case 16: return "vi";
	case 17: return "var";
	}
	return "unknow";
}

static int vdec_show(struct seq_file *s, void *v)
{
	static const char *const type[] = { "H264", "H265", "JPEG" };
	int i;

	mutex_lock(&info_lock);
	seq_puts(s, "[VDEC] open_vdec\n\n");
	seq_puts(s, "-----DEV ATTR------------------------------------------------------------\n");
	seq_puts(s, "ID  TYPE  MaxW  MaxH  Width  Height  BufSize\n");
	for (i = 0; i < N_DEV; i++) {
		u32 *d = info[i];

		if (d[6])
			seq_printf(s, "%-2d  %-4s  %-4d  %-4d  %-5d  %-6d  %-7d\n",
				   i, d[0] < 3 ? type[d[0]] : "unknow",
				   d[1], d[2], d[3], d[4], d[5]);
	}
	seq_puts(s, "\n-----CHN ATTR-----------------------------------------------------------\n");
	seq_puts(s, "ID Chn Crop CropX CropY CropW CropH DstW Stride DstH SrcRate DstRate BindMod BindDev BindChn\n");
	for (i = 0; i < N_DEV; i++) {
		u32 *d = info[i];

		if (d[6] && d[28])
			seq_printf(s, "%-2d %-3d %-4s %-5d %-5d %-5d %-5d %-4d %-7d %-4d %-7d %-7d %-7s %-7d %-7d\n",
				   i, 0, d[15] ? "Y" : "N", d[16], d[17],
				   d[18], d[19], d[20], d[21], d[22], d[23],
				   d[24], bind_name(d[25]), d[26], d[27]);
	}
	seq_puts(s, "\n-----DEV STATE-----------------------------------------------------------\n");
	seq_puts(s, "ID  Start  RecvCnt  DecCnt   FrameRate MinTime MaxTime DecTime\n");
	for (i = 0; i < N_DEV; i++) {
		u32 *d = info[i];

		if (d[6])
			seq_printf(s, "%-2d  %-5s  %-7d  %-8d %-9d %-7d %-7d %-7d \n",
				   i, d[7] ? "Y" : "N", d[8], d[9], d[10],
				   d[12], d[11], d[13]);
	}
	seq_puts(s, "\n-----CHN STATE-----------------------------------------------------------\n");
	seq_puts(s, "ID Chn Enable SendCnt  FrameRate SetDepth CurDepth\n");
	for (i = 0; i < N_DEV; i++) {
		u32 *d = info[i];

		if (d[6] && d[28])
			seq_printf(s, "%-2d %-3d %-6s %-7d  %-9d %-8d %-8d\n",
				   i, 0, "Y", d[29], d[30], d[31], d[32]);
	}
	seq_puts(s, "\n-----DEV BUFFER-----------------------------------------------------------\n");
	seq_puts(s, "ID  InStart   InEnd     InWNode  InRNode  InWAddr   InRAddr\n");
	for (i = 0; i < N_DEV; i++) {
		u32 *d = info[i];

		if (d[6])
			seq_printf(s, "%-2d  %08x  %08x  %-7d  %-7d  %08x  %08x\n",
				   i, d[34], d[35], d[36], d[37], d[38], d[39]);
	}
	mutex_unlock(&info_lock);
	return 0;
}

static int vdec_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, vdec_show, NULL);
}

static const struct file_operations vdec_proc_fops = {
	.owner = THIS_MODULE,
	.open = vdec_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init xm_v200_vdec_init(void)
{
	int ret;

	timer = ioremap(TIMER_BASE, 0x20);
	if (!timer)
		return -ENOMEM;
	writel(T_MODE, timer + T_CTRL);
	writel(1, timer + T_INTCLR);
	ret = request_irq(TIMER_IRQ, vdec_delay_irq, IRQF_SHARED, "vdec_delay",
			  &vdec_misc);
	if (ret) {
		pr_err("vdec: irq %d failed\n", TIMER_IRQ);
		goto unmap;
	}
	ret = misc_register(&vdec_misc);
	if (ret) {
		pr_err("vdec: register device failed!\n");
		goto irq;
	}
	if (!proc_create("umap/vdec", 0, NULL, &vdec_proc_fops))
		pr_err("vdec: create vdec proc fail!\n");
	return 0;
irq:
	free_irq(TIMER_IRQ, &vdec_misc);
unmap:
	iounmap(timer);
	return ret;
}

static void __exit xm_v200_vdec_exit(void)
{
	remove_proc_entry("umap/vdec", NULL);
	misc_deregister(&vdec_misc);
	writel(T_MODE, timer + T_CTRL);
	free_irq(TIMER_IRQ, &vdec_misc);
	iounmap(timer);
}

module_init(xm_v200_vdec_init);
module_exit(xm_v200_vdec_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM530V200 video decoder bridge, compatible with vdec.ko");
