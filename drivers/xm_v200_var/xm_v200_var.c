// SPDX-License-Identifier: GPL-2.0
/*
 * The XM530V200's rotator, VAR (/dev/var): a drop-in for the vendor
 * var.ko. VPSS channels bound to it (bind id 17, the VAR channel as the
 * bind chn) put their NV12 frames on VB list 3; a thread takes them, turns
 * them a quarter turn in hardware into a new buffer (90 or 270 degrees; 0
 * and 180 pass the frame on as it is) and hands that to the VAR channel's
 * own binding (VENC, VPSS or a VO channel), or queues it for user space.
 *
 * The engine at 0x36000000, IRQ 93, byte offsets:
 *   0x00  direction: 0 a quarter turn clockwise (the source read from its
 *         last row up), 1 anticlockwise (the destination written from its
 *         last row up)
 *   0x04  source width (its stride), 0x08 its height
 *   0x10, 0x14  source luma, chroma;  0x18, 0x1c  destination luma, chroma
 *   0x30  1 starts;  0x40 done;  0x44 0 acknowledges
 *   0x48  interrupt enable;  0x4c interrupt status;  0x408 engine enable
 * The destination's stride and width are the source's height, its height
 * the source's stride.
 *
 * ioctls take a 56-byte record {chn, ms, a, b, c, d, stride, width,
 * height, -, pts lo, hi, buffer, -}:
 *   0x4100      every channel off and forgotten
 *   0x40384101  rotation a (0..3, quarter turns clockwise)
 *   0x40384102  channel on,  0x40384103  off
 *   0x40384104  queue depth a for user space (0: none), the queue emptied
 *   0xc0384105  a frame from the queue, waiting ms: a, b, c its luma,
 *               chroma and (c) addresses, stride, width, height, pts and
 *               the buffer; a bound channel's frame is a copy, the frame
 *               itself going on to the binding
 *   0x40384106  that frame back
 *   0x40384107  bind to {a id, b dev, c chn}, while the depth is 0
 * Frames back from user space are checked to be VB's (vb_find_buf); var.ko
 * takes them on trust.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

int vb_get_buf(u32 list, u32 size, void **buf, int ms);
int vb_put_buf(u32 list, void *buf);
void *vb_find_buf(u32 addr);
int vo_get_buf(u32 dev, u32 chn, void **buf);
int vo_put_buf(u32 dev, u32 chn, void *buf, int show);

#define VAR_BASE	0x36000000
#define VAR_IRQ		93
#define R_DIR		0x00
#define R_SRC_W		0x04
#define R_SRC_H		0x08
#define R_SRC_Y		0x10
#define R_SRC_C		0x14
#define R_DST_Y		0x18
#define R_DST_C		0x1c
#define R_START		0x30
#define R_DONE		0x40
#define R_ACK		0x44
#define R_INT_EN	0x48
#define R_INT		0x4c
#define R_ENABLE	0x408

#define VAR_RESET	0x4100
#define VAR_ROTATE	0x40384101
#define VAR_ENABLE	0x40384102
#define VAR_DISABLE	0x40384103
#define VAR_DEPTH	0x40384104
#define VAR_GET		0xc0384105
#define VAR_RELEASE	0x40384106
#define VAR_BIND	0x40384107

/* a buffer descriptor (xm_v200_vb's v200 layout) */
#define W_PHYS		4
#define W_SIZE		5
#define W_STRIDE	6
#define W_WIDTH		7
#define W_HEIGHT	8
#define W_PTS		10
#define W_BIND		12

#define MOD_VENC	7
#define MOD_VPSS	8
#define MOD_VO		15
#define MOD_VAR		17

#define N_CHN		64

struct var_rec {
	u32 chn, ms, a, b, c, d;
	u32 stride, width, height, pad;
	u32 pts[2];
	u32 buf, pad2;
};

struct var_chn {
	u32 enable, rot, bind[3], depth;
	struct list_head q; /* through the descriptors' words 0, 1 */
	u32 count;
	u32 recv, send, timeouts;
	u32 src[3];
	u64 time;
	wait_queue_head_t wq;
};

static void __iomem *regs;
static struct var_chn chn[N_CHN];
static DEFINE_MUTEX(var_lock);
static DECLARE_WAIT_QUEUE_HEAD(done_wq);
static struct task_struct *thread;
static atomic_t opens = ATOMIC_INIT(0);

#define NODE(b) ((struct list_head *)(b))

static u32 list_of(u32 id)
{
	switch (id) {
	case MOD_VENC:
		return 2;
	case MOD_VPSS:
		return 1;
	}
	return 0;
}

static void var_reset(void)
{
	int i;

	mutex_lock(&var_lock);
	for (i = 0; i < N_CHN; i++) {
		struct var_chn *c = &chn[i];

		while (c->count) {
			u32 *b = (u32 *)c->q.next;

			list_del(NODE(b));
			c->count--;
			vb_put_buf(list_of(b[W_BIND]), b);
		}
		memset(c, 0, offsetof(struct var_chn, wq));
		INIT_LIST_HEAD(&c->q);
	}
	writel(1, regs + R_ENABLE);
	writel(1, regs + R_INT_EN);
	mutex_unlock(&var_lock);
}

static irqreturn_t var_irq(int irq, void *dev)
{
	if (!readl(regs + R_INT))
		return IRQ_NONE;
	writel(0, regs + R_ACK);
	wake_up(&done_wq);
	return IRQ_HANDLED;
}

/* a frame to the channel's binding or its queue; under var_lock */
static void send(struct var_chn *c, u32 *b)
{
	b[W_BIND] = c->bind[0];
	b[W_BIND + 1] = c->bind[1];
	b[W_BIND + 2] = c->bind[2];
	c->send++;
	if (c->bind[0] == MOD_VO) {
		vo_put_buf(c->bind[1], c->bind[2], b, 1);
	} else if (!c->depth) {
		vb_put_buf(list_of(c->bind[0]), b);
	} else {
		list_add_tail(NODE(b), &c->q);
		if (++c->count > c->depth) {
			u32 *old = (u32 *)c->q.next;

			list_del(NODE(old));
			c->count--;
			vb_put_buf(list_of(old[W_BIND]), old);
		}
		wake_up(&c->wq);
	}
}

/* the source turned into a new buffer; under var_lock */
static u32 *rotate(struct var_chn *c, u32 *s)
{
	u32 w = s[W_STRIDE], h = s[W_HEIGHT], luma = w * h, size = luma * 3 / 2;
	u32 sy = s[W_PHYS], dy;
	u32 *d;

	if (c->bind[0] == MOD_VO) {
		if (vo_get_buf(c->bind[1], c->bind[2], (void **)&d) || !d)
			return NULL;
		if (d[W_SIZE] < size) {
			vo_put_buf(c->bind[1], c->bind[2], d, 0);
			return NULL;
		}
	} else if (vb_get_buf(0, size, (void **)&d, 0)) {
		return NULL;
	}
	d[W_STRIDE] = h;
	d[W_WIDTH] = h;
	d[W_HEIGHT] = w;
	dy = d[W_PHYS];
	writel(w, regs + R_SRC_W);
	writel(h, regs + R_SRC_H);
	if (c->rot == 1) {
		writel(0, regs + R_DIR);
		writel(sy + luma - w, regs + R_SRC_Y);
		writel(sy + luma - w + luma / 2, regs + R_SRC_C);
		writel(dy, regs + R_DST_Y);
		writel(dy + luma, regs + R_DST_C);
	} else {
		writel(1, regs + R_DIR);
		writel(sy, regs + R_SRC_Y);
		writel(sy + luma, regs + R_SRC_C);
		writel(dy + luma - h, regs + R_DST_Y);
		writel(dy + luma - h + luma / 2, regs + R_DST_C);
	}
	writel(1, regs + R_START);
	if (!wait_event_timeout(done_wq, readl(regs + R_DONE), 2 * HZ)) {
		c->timeouts++;
		if (c->bind[0] == MOD_VO)
			vo_put_buf(c->bind[1], c->bind[2], d, 0);
		else
			vb_put_buf(0, d);
		return NULL;
	}
	d[W_PTS] = s[W_PTS];
	d[W_PTS + 1] = s[W_PTS + 1];
	return d;
}

static void process(u32 *s)
{
	struct var_chn *c;
	ktime_t t0 = ktime_get();
	u32 *d;

	if (s[W_BIND] != MOD_VAR || s[W_BIND + 2] >= N_CHN)
		goto free;
	c = &chn[s[W_BIND + 2]];
	mutex_lock(&var_lock);
	c->recv++;
	memcpy(c->src, &s[W_STRIDE], sizeof(c->src));
	if (!c->enable) {
		mutex_unlock(&var_lock);
		goto free;
	}
	if (!(c->rot & 1)) {
		/* not turned: the frame itself, except to VO (not its buffer) */
		if (c->bind[0] == MOD_VO) {
			mutex_unlock(&var_lock);
			goto free;
		}
		send(c, s);
		c->time = ktime_us_delta(ktime_get(), t0);
		mutex_unlock(&var_lock);
		return;
	}
	d = rotate(c, s);
	if (d) {
		send(c, d);
		c->time = ktime_us_delta(ktime_get(), t0);
	}
	mutex_unlock(&var_lock);
free:
	memset(&s[W_BIND], 0, 12);
	vb_put_buf(0, s);
}

static int var_thread(void *data)
{
	void *b;

	while (!kthread_should_stop()) {
		if (!atomic_read(&opens) || vb_get_buf(3, 0, &b, 20)) {
			schedule_timeout_interruptible(1);
			continue;
		}
		process(b);
	}
	return 0;
}

/* a copy of b in a free buffer */
static u32 *copy_frame(u32 *b)
{
	u32 size = b[W_STRIDE] * b[W_HEIGHT] * 3 / 2;
	void __iomem *src, *dst;
	u32 *d;

	if (vb_get_buf(0, size, (void **)&d, 0))
		return NULL;
	src = ioremap(b[W_PHYS], size);
	dst = ioremap(d[W_PHYS], size);
	if (src && dst)
		memcpy_fromio((void __force *)dst, src, size);
	if (src)
		iounmap(src);
	if (dst)
		iounmap(dst);
	memcpy(&d[W_STRIDE], &b[W_STRIDE], 12);
	d[W_PTS] = b[W_PTS];
	d[W_PTS + 1] = b[W_PTS + 1];
	return d;
}

static long var_get(struct var_chn *c, struct var_rec *r)
{
	long t;
	u32 *b;

	t = wait_event_interruptible_timeout(c->wq, c->count,
					     msecs_to_jiffies(r->ms));
	if (t < 0)
		return t;
	mutex_lock(&var_lock);
	if (!c->count) {
		mutex_unlock(&var_lock);
		return -ETIMEDOUT;
	}
	b = (u32 *)c->q.next;
	list_del(NODE(b));
	c->count--;
	mutex_unlock(&var_lock);
	if (c->bind[0]) {
		u32 *orig = b;

		b = copy_frame(orig);
		vb_put_buf(list_of(orig[W_BIND]), orig);
		if (!b)
			return -ENOMEM;
	}
	r->a = b[W_PHYS];
	r->b = r->a + b[W_STRIDE] * b[W_HEIGHT];
	r->c = r->b + b[W_STRIDE] * b[W_HEIGHT];
	r->d = 0;
	r->stride = b[W_STRIDE];
	r->width = b[W_WIDTH];
	r->height = b[W_HEIGHT];
	r->pts[0] = b[W_PTS];
	r->pts[1] = b[W_PTS + 1];
	r->buf = (u32)(uintptr_t)b;
	return 0;
}

static long var_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *u = (void __user *)arg;
	struct var_rec r;
	struct var_chn *c;
	long ret = 0;
	u32 *b;

	if (cmd == VAR_RESET) {
		var_reset();
		return 0;
	}
	if (copy_from_user(&r, u, sizeof(r)))
		return -EFAULT;
	if (r.chn >= N_CHN)
		return -EINVAL;
	c = &chn[r.chn];
	switch (cmd) {
	case VAR_ROTATE:
		mutex_lock(&var_lock);
		c->rot = r.a & 3;
		writel(1, regs + R_INT_EN);
		mutex_unlock(&var_lock);
		return 0;
	case VAR_ENABLE:
	case VAR_DISABLE:
		mutex_lock(&var_lock);
		c->enable = cmd == VAR_ENABLE;
		mutex_unlock(&var_lock);
		return 0;
	case VAR_DEPTH:
		mutex_lock(&var_lock);
		while (c->count) {
			b = (u32 *)c->q.next;
			list_del(NODE(b));
			c->count--;
			vb_put_buf(list_of(b[W_BIND]), b);
		}
		c->depth = r.a;
		mutex_unlock(&var_lock);
		return 0;
	case VAR_BIND:
		mutex_lock(&var_lock);
		if (c->depth) {
			ret = -EBUSY;
		} else {
			c->bind[0] = r.a;
			c->bind[1] = r.b;
			c->bind[2] = r.c;
		}
		mutex_unlock(&var_lock);
		return ret;
	case VAR_RELEASE:
		b = vb_find_buf(r.buf);
		if (!b)
			return -EINVAL;
		return vb_put_buf(0, b);
	case VAR_GET:
		ret = var_get(c, &r);
		if (ret)
			return ret;
		return copy_to_user(u, &r, sizeof(r)) ? -EFAULT : 0;
	}
	pr_err("var: ioctl cmd %x error\n", cmd);
	return -ENOTTY;
}

static int var_open(struct inode *inode, struct file *file)
{
	atomic_inc(&opens);
	return 0;
}

static int var_release(struct inode *inode, struct file *file)
{
	atomic_dec(&opens);
	return 0;
}

static const struct file_operations var_fops = {
	.owner = THIS_MODULE,
	.open = var_open,
	.release = var_release,
	.unlocked_ioctl = var_ioctl,
};

static struct miscdevice var_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "var",
	.fops = &var_fops,
};

static int var_show(struct seq_file *s, void *v)
{
	int i;

	seq_puts(s, "[VAR] open_var\n\n");
	seq_puts(s, "---------------------------------VAR INFO----------------------------------------------------\n");
	seq_puts(s, "ChnID  Enable  Rotate  RecvPic  SrcS  SrcW  SrcH  SendPic  ModId  DevID  ChnID  Timeout  Time\n");
	mutex_lock(&var_lock);
	for (i = 0; i < N_CHN; i++) {
		struct var_chn *c = &chn[i];

		if (c->enable)
			seq_printf(s, "%-5d  %-6s  %-6d  %-7d  %-4d  %-4d  %-4d  %-7d  %-5d  %-5d  %-5d  %-7d  %-8lld\n",
				   i, "Y", c->rot * 90, c->recv, c->src[0],
				   c->src[1], c->src[2], c->send, c->bind[0],
				   c->bind[1], c->bind[2], c->timeouts,
				   (long long)c->time);
	}
	mutex_unlock(&var_lock);
	return 0;
}

static int var_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, var_show, NULL);
}

static const struct file_operations var_proc_fops = {
	.owner = THIS_MODULE,
	.open = var_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init xm_v200_var_init(void)
{
	int i, ret;

	regs = ioremap(VAR_BASE, 0x1000);
	if (!regs)
		return -ENOMEM;
	for (i = 0; i < N_CHN; i++) {
		INIT_LIST_HEAD(&chn[i].q);
		init_waitqueue_head(&chn[i].wq);
	}
	writel(1, regs + R_ENABLE);
	writel(1, regs + R_INT_EN);
	ret = request_irq(VAR_IRQ, var_irq, 0, "var", NULL);
	if (ret) {
		pr_err("var: irq %d failed\n", VAR_IRQ);
		goto unmap;
	}
	ret = misc_register(&var_misc);
	if (ret) {
		pr_err("var: register device failed!\n");
		goto irq;
	}
	thread = kthread_run(var_thread, NULL, "var");
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		goto misc;
	}
	if (!proc_create("umap/var", 0, NULL, &var_proc_fops))
		pr_err("var: create proc fail!\n");
	return 0;
misc:
	misc_deregister(&var_misc);
irq:
	free_irq(VAR_IRQ, NULL);
unmap:
	iounmap(regs);
	return ret;
}

static void __exit xm_v200_var_exit(void)
{
	remove_proc_entry("umap/var", NULL);
	kthread_stop(thread);
	var_reset();
	misc_deregister(&var_misc);
	free_irq(VAR_IRQ, NULL);
	iounmap(regs);
}

module_init(xm_v200_var_init);
module_exit(xm_v200_var_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM530V200 rotator, compatible with var.ko");
