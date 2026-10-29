// SPDX-License-Identifier: GPL-2.0
/*
 * xm_v200_vb: the V200 series' video buffer pools, a drop-in replacement for
 * the vendor's vb.ko (built as open_vb.ko).
 *
 * The media modules (isp, vpss or vi, venc) take their frame buffers from
 * here and hand them on to each other. A buffer is a descriptor the modules
 * read and write directly, so its layout is the vendor's:
 *
 *   word 0, 1   the list it is on (free, or a queue)
 *   word 2, 3   the list of every buffer
 *   word 4      physical address    word 5  size
 *   word 6..8   stride, width, height (the module that fills it sets them)
 *   then the reference count, and the bind (id, dev, chn) further on
 *
 * The XM530V200 and XM533 have a 64-byte descriptor (count at word 9, bind
 * at 12..14); the XM535's is 72 bytes, a word more before the count (count
 * at 10, bind at 14..16). VB_LAYOUT=v200 or xm535 at build time.
 *
 * Lists: 0 the free buffers, in size order; 1 VPSS's, 2 VENC's, 3 one more
 * queue (var), each with a wait queue. vb_get_buf(list, size, &buf, ms)
 * takes the first buffer (from the free list the first at least that big,
 * or of that size in mode 1), waiting forever (ms < 0), not at all (0) or
 * ms; vb_put_buf(list, buf) gives it to a list (to the free list once the
 * last reference goes).
 *
 * /dev/vb: 0x41044201 the pools ({count, {size, count}[32]}), 0x4200 makes
 * them (an MMZ block "vb[size][n]" per buffer, of size + 0x10 to 32 bytes,
 * the "MEMORY BARRIERS" marker in its last 16), 0x4204 frees the free and
 * VENC lists' blocks, 0x40044205 the mode; the get and put calls take a
 * record with a copy of the descriptor and the descriptor's address, which
 * is checked here (the vendor's takes it on trust).
 */

#include <linux/fs.h>
#include <linux/io.h>
#include <linux/jiffies.h>
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
#include <asm/cacheflush.h>

int mmz_alloc(const char *name, u32 size, u32 *phys);
int mmz_free(u32 phys, u32 *size);

#ifdef VB_LAYOUT_XM535
#define DESC_WORDS 18
#define W_REF 10
#define W_BIND 14
#define PUT_FROM 12 /* the put call copies words 6..8 and these 5 */
#define DUMMIES 2   /* empty buffers at the head of the free list */
#else
#define DESC_WORDS 16
#define W_REF 9
#define W_BIND 12
#define PUT_FROM 10
#define DUMMIES 0
#endif
#define W_PHYS 4
#define W_SIZE 5
#define W_STRIDE 6

#define VB_SET_CONF 0x41044201
#define VB_INIT 0x4200
#define VB_FREE 0x4204
#define VB_SET_MODE 0x40044205
#define VB_GET (0xc0004202 | ((DESC_WORDS * 4 + 16) << 16))
#define VB_PUT (0x40004203 | ((DESC_WORDS * 4 + 16) << 16))

#define MAX_POOLS 32
#define N_LISTS 4

/* a buffer: the descriptor as the modules see it, and its links */
struct vb_buf {
	struct list_head node; /* words 0, 1 */
	struct list_head all;  /* words 2, 3 */
	u32 w[DESC_WORDS - 4];
};
#define WORD(b, n) ((b)->w[(n) - 4])

static struct {
	u32 count;
	struct {
		u32 size, count;
	} pool[MAX_POOLS];
} conf;
static struct {
	int cur, min;
} pool_state[MAX_POOLS];
static u32 mode;

static struct list_head lists[N_LISTS];
static wait_queue_head_t waits[N_LISTS];
static LIST_HEAD(all_bufs);
static DEFINE_SPINLOCK(vb_lock);
static const char marker[16] = "MEMORY BARRIERS";

static int pool_of(u32 size)
{
	u32 i;

	for (i = 0; i < conf.count && i < MAX_POOLS; i++)
		if (conf.pool[i].size == size)
			return i;
	return -1;
}

/* before the first free buffer at least as big: size order */
static struct list_head *free_slot(u32 size)
{
	struct vb_buf *b;

	list_for_each_entry (b, &lists[0], node)
		if (WORD(b, W_SIZE) >= size)
			return &b->node;
	return &lists[0];
}

int vb_get_buf(u32 list, u32 size, struct vb_buf **out, int ms)
{
	struct vb_buf *b, *found = NULL;
	unsigned long flags;
	long t;
	int p;

	*out = NULL;
	if (list >= N_LISTS) {
		pr_err("vb: list %d error\n", list);
		return -1;
	}
	if (ms < 0) {
		if (wait_event_interruptible(waits[list], !list_empty(&lists[list])))
			return -ERESTARTSYS;
	} else if (ms > 0) {
		t = wait_event_interruptible_timeout(
			waits[list], !list_empty(&lists[list]),
			msecs_to_jiffies(ms));
		if (t <= 0)
			return -ERESTARTSYS;
	}
	spin_lock_irqsave(&vb_lock, flags);
	if (list_empty(&lists[list])) {
		spin_unlock_irqrestore(&vb_lock, flags);
		return -1;
	}
	if (list == 0) {
		list_for_each_entry (b, &lists[0], node)
			if (mode == 1 ? WORD(b, W_SIZE) == size :
					WORD(b, W_SIZE) >= size) {
				found = b;
				break;
			}
		if (!found) {
			spin_unlock_irqrestore(&vb_lock, flags);
			return -1;
		}
		WORD(found, W_REF)++;
		p = pool_of(WORD(found, W_SIZE));
		if (p >= 0) {
			pool_state[p].cur--;
			if (pool_state[p].cur < pool_state[p].min)
				pool_state[p].min = pool_state[p].cur;
		}
	} else {
		found = list_first_entry(&lists[list], struct vb_buf, node);
	}
	list_del(&found->node);
	spin_unlock_irqrestore(&vb_lock, flags);
	*out = found;
	return 0;
}
EXPORT_SYMBOL(vb_get_buf);

int vb_put_buf(u32 list, struct vb_buf *b)
{
	struct list_head *at;
	unsigned long flags;
	int p;

	if (list >= N_LISTS) {
		pr_err("vb: list %d error\n", list);
		return -1;
	}
	spin_lock_irqsave(&vb_lock, flags);
	if (list == 0) {
		if (!WORD(b, W_REF)) {
			spin_unlock_irqrestore(&vb_lock, flags);
			pr_err("vb: put a free buffer\n");
			return -1;
		}
		if (--WORD(b, W_REF)) {
			spin_unlock_irqrestore(&vb_lock, flags);
			return 0;
		}
		at = free_slot(WORD(b, W_SIZE));
		p = pool_of(WORD(b, W_SIZE));
		if (p >= 0)
			pool_state[p].cur++;
	} else {
		at = &lists[list];
	}
	list_add_tail(&b->node, at);
	spin_unlock_irqrestore(&vb_lock, flags);
	wake_up(&waits[list]);
	return 0;
}
EXPORT_SYMBOL(vb_put_buf);

int vb_add_buf_reference(struct vb_buf *b)
{
	unsigned long flags;

	spin_lock_irqsave(&vb_lock, flags);
	WORD(b, W_REF)++;
	spin_unlock_irqrestore(&vb_lock, flags);
	return 0;
}
EXPORT_SYMBOL(vb_add_buf_reference);

int vb_get_length(u32 list, int *n)
{
	struct list_head *pos;
	unsigned long flags;

	*n = 0;
	if (list >= N_LISTS) {
		pr_err("vb: list %d error\n", list);
		return -1;
	}
	spin_lock_irqsave(&vb_lock, flags);
	list_for_each (pos, &lists[list])
		(*n)++;
	spin_unlock_irqrestore(&vb_lock, flags);
	return 0;
}
EXPORT_SYMBOL(vb_get_length);

int vb_flush_dcache(u32 phys, void *virt, u32 size)
{
	__cpuc_flush_dcache_area(virt, size);
	return 0;
}
EXPORT_SYMBOL(vb_flush_dcache);

static void lists_init(void)
{
	int i;

	for (i = 0; i < N_LISTS; i++)
		INIT_LIST_HEAD(&lists[i]);
	INIT_LIST_HEAD(&all_bufs);
}

static struct vb_buf *new_buf(void)
{
	return kzalloc(sizeof(struct vb_buf), GFP_KERNEL);
}

/* the pools: what was there forgotten (its MMZ blocks go with the MMZ
 * reset the pipeline starts with), and conf's made */
static int vb_init(void)
{
	struct vb_buf *b, *n;
	unsigned long flags;
	LIST_HEAD(gone);
	u32 i, k;
	int d;

	spin_lock_irqsave(&vb_lock, flags);
	list_for_each_entry_safe (b, n, &all_bufs, all)
		list_move(&b->all, &gone);
	lists_init();
	spin_unlock_irqrestore(&vb_lock, flags);
	list_for_each_entry_safe (b, n, &gone, all)
		kfree(b);

	for (i = 0; i < conf.count && i < MAX_POOLS; i++) {
		u32 size = conf.pool[i].size, room = (size + 0x2f) & ~0x1fu;

		for (k = 0; k < conf.pool[i].count; k++) {
			char name[32];
			u32 phys;
			void __iomem *m;

			snprintf(name, sizeof(name), "vb[%x][%d]", size, k);
			if (mmz_alloc(name, room, &phys)) {
				pr_err("vb: mmz_alloc %s failed\n", name);
				return -1;
			}
			b = new_buf();
			if (!b) {
				pr_err("vb: kmalloc failed\n");
				return -1;
			}
			WORD(b, W_PHYS) = phys;
			WORD(b, W_SIZE) = size;
			m = ioremap(phys + room - sizeof(marker), sizeof(marker));
			if (m) {
				memcpy_toio(m, marker, sizeof(marker));
				iounmap(m);
			}
			spin_lock_irqsave(&vb_lock, flags);
			list_add_tail(&b->node, free_slot(size));
			list_add_tail(&b->all, &all_bufs);
			spin_unlock_irqrestore(&vb_lock, flags);
		}
		pool_state[i].cur = pool_state[i].min = conf.pool[i].count;
	}
	for (d = 0; d < DUMMIES; d++) {
		b = new_buf();
		if (!b)
			return -1;
		spin_lock_irqsave(&vb_lock, flags);
		list_add(&b->node, &lists[0]);
		list_add_tail(&b->all, &all_bufs);
		spin_unlock_irqrestore(&vb_lock, flags);
	}
	return 0;
}

/* the free and VENC lists' blocks back to MMZ */
static void vb_free_blocks(void)
{
	struct vb_buf *b, *n;
	unsigned long flags;
	LIST_HEAD(gone);
	u32 size;
	int l;

	spin_lock_irqsave(&vb_lock, flags);
	for (l = 0; l <= 2; l += 2)
		list_for_each_entry_safe (b, n, &lists[l], node)
			list_move_tail(&b->node, &gone);
	spin_unlock_irqrestore(&vb_lock, flags);
	list_for_each_entry_safe (b, n, &gone, node) {
		if (WORD(b, W_PHYS))
			mmz_free(WORD(b, W_PHYS), &size);
		list_del_init(&b->node);
	}
}

/* a descriptor address from user space, if it is one of ours */
static struct vb_buf *known(u32 addr)
{
	struct vb_buf *b;

	list_for_each_entry (b, &all_bufs, all)
		if ((u32)(uintptr_t)b == addr)
			return b;
	return NULL;
}

struct vb_rec {
	u32 list, size;
	u32 desc[DESC_WORDS];
	u32 buf, ms;
};

static long vb_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct vb_rec r;
	struct vb_buf *b;
	unsigned long flags;
	int ret, i;

	switch (cmd) {
	case VB_SET_MODE:
		if (copy_from_user(&mode, uarg, 4))
			break;
		return 0;
	case VB_SET_CONF:
		if (copy_from_user(&conf, uarg, sizeof(conf)))
			break;
		return 0;
	case VB_INIT:
		return vb_init();
	case VB_FREE:
		vb_free_blocks();
		return 0;
	case VB_GET:
		if (copy_from_user(&r, uarg, sizeof(r)))
			break;
		ret = vb_get_buf(r.list, r.size, &b, r.ms);
		if (!ret) {
			memcpy(r.desc, b, sizeof(r.desc));
			r.buf = (u32)(uintptr_t)b;
		}
		if (copy_to_user(uarg, &r, sizeof(r))) {
			pr_err("vb: ioctl copy_to_user error\n");
			return -1;
		}
		return ret;
	case VB_PUT:
		if (copy_from_user(&r, uarg, sizeof(r)))
			break;
		spin_lock_irqsave(&vb_lock, flags);
		b = known(r.buf);
		if (b) {
			for (i = W_STRIDE; i <= W_STRIDE + 2; i++)
				WORD(b, i) = r.desc[i];
			for (i = PUT_FROM; i < PUT_FROM + 5; i++)
				WORD(b, i) = r.desc[i];
		}
		spin_unlock_irqrestore(&vb_lock, flags);
		if (!b) {
			pr_err("vb: put of no buffer %#x\n", r.buf);
			return -1;
		}
		return vb_put_buf(r.list, b);
	default:
		pr_err("vb: ioctl cmd error %x\n", cmd);
		return 0;
	}
	pr_err("vb: ioctl copy_from_user error\n");
	return -1;
}

static void show_list(struct seq_file *s, struct list_head *l)
{
	struct vb_buf *b;

	seq_puts(s, "Node      PhysAddr  Size      Stride  Width  Height  BindId  BindDev  BindChn\n");
	list_for_each_entry (b, l, node)
		seq_printf(s, "%-8p  %-8x  %-8x  %-6d  %-5d  %-6d  %-6d  %-7d  %-7d\n",
			   b, WORD(b, 4), WORD(b, 5), WORD(b, 6), WORD(b, 7),
			   WORD(b, 8), WORD(b, W_BIND), WORD(b, W_BIND + 1),
			   WORD(b, W_BIND + 2));
}

static int vb_show(struct seq_file *s, void *v)
{
	static const char *const names[N_LISTS] = {"FREE", "VPSS", "VENC",
						   "VAR"};
	unsigned long flags;
	struct vb_buf *b;
	u32 i;

	spin_lock_irqsave(&vb_lock, flags);
	seq_puts(s, "[VB] openxiongmai xm_v200_vb\n\n");
	seq_puts(s, "-----VB PUB CONFIG-------------------------------------------------------------\n");
	seq_printf(s, "Max Count of Pools:        %d\n", MAX_POOLS);
	seq_printf(s, "Mode: %d\n", mode);
	seq_puts(s, "-----COMMON POOL CONFIG--------------------------------------------------------\n");
	seq_puts(s, "PoolId  Count  Size      CurCnt  MinCnt\n");
	for (i = 0; i < conf.count && i < MAX_POOLS; i++)
		seq_printf(s, "%-6d  %-5d  %-8x  %-6d  %-6d\n", i,
			   conf.pool[i].count, conf.pool[i].size,
			   pool_state[i].cur, pool_state[i].min);
	seq_puts(s, "-----VB ALL LIST STATUS--------------------------------------------------------\n");
	seq_puts(s, "Node      PhysAddr  Size      Stride  Width  Height  BindId  BindDev  BindChn  Reference\n");
	list_for_each_entry (b, &all_bufs, all)
		seq_printf(s, "%-8p  %-8x  %-8x  %-6d  %-5d  %-6d  %-6d  %-7d  %-7d  %-9d\n",
			   b, WORD(b, 4), WORD(b, 5), WORD(b, 6), WORD(b, 7),
			   WORD(b, 8), WORD(b, W_BIND), WORD(b, W_BIND + 1),
			   WORD(b, W_BIND + 2), WORD(b, W_REF));
	for (i = 0; i < N_LISTS; i++) {
		seq_printf(s, "-----VB %s LIST STATUS-------------------------------------------------------\n",
			   names[i]);
		show_list(s, &lists[i]);
	}
	spin_unlock_irqrestore(&vb_lock, flags);
	return 0;
}

static int vb_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, vb_show, NULL);
}

static const struct file_operations vb_proc_fops = {
	.owner = THIS_MODULE,
	.open = vb_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations vb_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = vb_ioctl,
};

static struct miscdevice vb_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vb",
	.fops = &vb_fops,
};

static int __init xm_v200_vb_init(void)
{
	int i, ret;

	BUILD_BUG_ON(sizeof(struct vb_buf) != DESC_WORDS * 4);
	lists_init();
	for (i = 0; i < N_LISTS; i++)
		init_waitqueue_head(&waits[i]);
	if (!proc_create("umap/vb", 0444, NULL, &vb_proc_fops))
		pr_err("vb: create vb proc fail!\n");
	ret = misc_register(&vb_misc);
	if (ret) {
		pr_err("vb: register device failed!\n");
		remove_proc_entry("umap/vb", NULL);
		return ret;
	}
	pr_info("vb: openxiongmai xm_v200_vb init ok, %d-byte buffers\n",
		DESC_WORDS * 4);
	return 0;
}

static void __exit xm_v200_vb_exit(void)
{
	struct vb_buf *b, *n;

	misc_deregister(&vb_misc);
	remove_proc_entry("umap/vb", NULL);
	list_for_each_entry_safe (b, n, &all_bufs, all)
		kfree(b);
}

module_init(xm_v200_vb_init);
module_exit(xm_v200_vb_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V200-series video buffer pools, compatible with vb.ko");
