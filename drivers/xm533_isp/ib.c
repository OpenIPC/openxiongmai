// SPDX-License-Identifier: GPL-2.0
/*
 * The ISP buffers (ib): one pool in MMZ, set up by 0x4962 (w/10 * h * 16
 * bytes a buffer, blkNum of them), each ending in a 16-byte guard. Other
 * modules take them with ib_get_buf() from the free list (0, sorted by
 * size) or the BPU list (0x10), and give them back with ib_put_buf().
 */

#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/wait.h>

#include "isp.h"

#define IB_FREE 0
#define IB_BPU 0x10
#define IB_GUARD "MEMORY BARRIERS" /* 16 bytes with its 0 */

/* isp.ko's node, 0x4c bytes; other modules read it */
struct ib_node {
	struct list_head node; /* on the free or the BPU list */
	struct list_head all;
	u32 phys, size, w, h;
	u32 bind[3]; /* id, dev, chn */
	u32 pad[8];
};

static LIST_HEAD(ib_free);
static LIST_HEAD(ib_bpu);
static LIST_HEAD(ib_all);
static DECLARE_WAIT_QUEUE_HEAD(ib_free_wq);
static DECLARE_WAIT_QUEUE_HEAD(ib_bpu_wq);
static DEFINE_SPINLOCK(ib_lock);
static u32 ib_pools, ib_size, ib_count;

int mmz_alloc(const char *name, u32 size, u32 *phys);

static bool ib_empty(struct list_head *l)
{
	unsigned long flags;
	bool e;

	spin_lock_irqsave(&ib_lock, flags);
	e = list_empty(l);
	spin_unlock_irqrestore(&ib_lock, flags);
	return e;
}

/* ms < 0: wait for a buffer; 0: don't; > 0: that long (-512 after it) */
int ib_get_buf(u32 unused, int type, u32 size, void **out, int ms)
{
	struct list_head *l;
	wait_queue_head_t *wq;
	struct ib_node *n, *pick = NULL;
	unsigned long flags;
	long t;

	*out = NULL;
	if (type == IB_FREE) {
		l = &ib_free;
		wq = &ib_free_wq;
	} else if (type == IB_BPU) {
		l = &ib_bpu;
		wq = &ib_bpu_wq;
	} else {
		pr_err("IspKo:Err: IB: isp_list_type %d not support \n", type);
		return -1;
	}
	if (ms < 0) {
		if (wait_event_interruptible(*wq, !ib_empty(l)))
			return -ERESTARTSYS;
	} else if (ms > 0) {
		t = wait_event_interruptible_timeout(*wq, !ib_empty(l),
						     msecs_to_jiffies(ms));
		if (!t)
			return -ERESTARTSYS;
	}
	spin_lock_irqsave(&ib_lock, flags);
	list_for_each_entry(n, l, node) {
		if (type == IB_FREE && n->size < size)
			continue;
		pick = n;
		break;
	}
	if (!pick) {
		spin_unlock_irqrestore(&ib_lock, flags);
		return -1;
	}
	list_del(&pick->node);
	spin_unlock_irqrestore(&ib_lock, flags);
	*out = pick;
	return 0;
}
EXPORT_SYMBOL(ib_get_buf);

int ib_put_buf(u32 unused, int type, void *buf)
{
	struct ib_node *b = buf, *n;
	unsigned long flags;

	if (type != IB_FREE && type != IB_BPU) {
		pr_err("IspKo:Err: IB: isp_list_type %d not support \n", type);
		return -1;
	}
	spin_lock_irqsave(&ib_lock, flags);
	if (type == IB_FREE) {
		/* before the first at least as big */
		list_for_each_entry(n, &ib_free, node)
			if (n->size >= b->size)
				break;
		list_add_tail(&b->node, &n->node);
	} else {
		list_add_tail(&b->node, &ib_bpu);
	}
	spin_unlock_irqrestore(&ib_lock, flags);
	wake_up(type == IB_FREE ? &ib_free_wq : &ib_bpu_wq);
	return 0;
}
EXPORT_SYMBOL(ib_put_buf);

/* pools: {count, size, buffers}; isp.ko sets up the first pool only */
int ib_init(const u32 *pools, u32 w, u32 h)
{
	struct ib_node *n, *tmp, *pos;
	u32 size, phys, i;
	char name[32];
	void __iomem *g;

	/* the buffers stay in MMZ, as isp.ko leaves them */
	list_for_each_entry_safe(n, tmp, &ib_all, all) {
		list_del(&n->all);
		kfree(n);
	}
	INIT_LIST_HEAD(&ib_all);
	INIT_LIST_HEAD(&ib_free);
	INIT_LIST_HEAD(&ib_bpu);
	ib_pools = pools[0];
	ib_size = pools[1];
	ib_count = pools[2];
	if (!ib_pools)
		return 0;
	size = (ib_size + 0x2f) & ~0x1f;
	if (ib_count * size == 0)
		return 0;
	snprintf(name, sizeof(name), "ib[0x%x][%d]", size, ib_count);
	if (mmz_alloc(name, ib_count * size, &phys)) {
		pr_err("IspKo:Err: IB: mmz malloc memory failed\n");
		return -1;
	}
	list_for_each_entry(pos, &ib_free, node)
		if (pos->size >= size)
			break;
	for (i = 0; i < ib_count; i++) {
		n = kzalloc(sizeof(*n), GFP_KERNEL);
		if (!n) {
			pr_err("IspKo:Err: IB: kmalloc failed\n");
			return -1;
		}
		n->size = size;
		n->w = w;
		n->h = h;
		n->phys = phys;
		phys += size;
		g = ioremap(phys - 0x10, 0x10);
		if (g) {
			memcpy_toio(g, IB_GUARD, 0x10);
			iounmap(g);
		}
		list_add_tail(&n->node, &pos->node);
		list_add_tail(&n->all, &ib_all);
	}
	return 0;
}

static void ib_list_show(struct seq_file *m, struct list_head *l, bool all)
{
	struct ib_node *n;

	seq_puts(m, "Node \t  PhysAddr  Size      Width  Height  BindId  BindDev  BindChn\n");
	if (all)
		list_for_each_entry(n, l, all)
			seq_printf(m, "%-8p  %-8x  %-8x  %-5d  %-6d  %-6d  %-7d  %-7d\n",
				   n, n->phys, n->size, n->w, n->h, n->bind[0],
				   n->bind[1], n->bind[2]);
	else
		list_for_each_entry(n, l, node)
			seq_printf(m, "%-8p  %-8x  %-8x  %-5d  %-6d  %-6d  %-7d  %-7d\n",
				   n, n->phys, n->size, n->w, n->h, n->bind[0],
				   n->bind[1], n->bind[2]);
}

void ib_proc_show(struct seq_file *m, int *k)
{
	unsigned long flags;
	u32 i;

	seq_printf(m, "\n[%d.IB(isp buffer info)]:\n", (*k)++);
	seq_puts(m, "-----COMMON POOL CONFIG--------------------------------------------------------\n");
	seq_puts(m, "PoolId  Count  Size\n");
	for (i = 0; i < ib_pools && i < 1; i++)
		seq_printf(m, "%-6d  %-5d  %-4d\n", i, ib_count, ib_size);
	spin_lock_irqsave(&ib_lock, flags);
	seq_puts(m, "-----IB FREE LIST STATUS-------------------------------------------------------\n");
	ib_list_show(m, &ib_free, false);
	seq_puts(m, "-----IB BPU LIST STATUS-------------------------------------------------------\n");
	ib_list_show(m, &ib_bpu, false);
	seq_puts(m, "-----IB ALL LIST STATUS-------------------------------------------------------\n");
	ib_list_show(m, &ib_all, true);
	spin_unlock_irqrestore(&ib_lock, flags);
}
