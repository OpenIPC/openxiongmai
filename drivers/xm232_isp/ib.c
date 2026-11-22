// SPDX-License-Identifier: GPL-2.0
/*
 * The ISP buffers (ib): the pools 0x4962 asks for, in MMZ, each buffer
 * ending in a 16-byte guard. The VICU stores a raw frame into a buffer from
 * the free list (0, sorted by size) and hands it to the BPU list (0x10).
 * Once the pools are set up isp.ko keeps memory: each channel keeps the
 * first buffer it gets, and nothing goes back.
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
#define IB_GUARD "MEMORY BARRIERS"
#define IB_POOLS 32

/* isp.ko's node, 0x48 bytes; the VICU and BPU code read its words */
struct ib_node {
	struct list_head node;
	struct list_head all;
	u32 phys, size, w, h;
	u32 bind[3]; /* id, dev, chn */
	u32 w11[7];  /* 0x2c: stored; 0x38/0x3c the frame's pts, 0x40/0x44 the
		      * store's */
};

static LIST_HEAD(ib_free);
static LIST_HEAD(ib_bpu);
static LIST_HEAD(ib_all);
static DECLARE_WAIT_QUEUE_HEAD(ib_free_wq);
static DECLARE_WAIT_QUEUE_HEAD(ib_bpu_wq);
static DEFINE_SPINLOCK(ib_lock);
static u32 ib_conf[1 + IB_POOLS * 2]; /* count, then size, buffers */
static u8 ib_mem_flag;
static struct ib_node *ib_kept[N_CHN];

static bool ib_empty(struct list_head *l)
{
	unsigned long flags;
	bool e;

	spin_lock_irqsave(&ib_lock, flags);
	e = list_empty(l);
	spin_unlock_irqrestore(&ib_lock, flags);
	return e;
}

int ib_get_buf(u32 chn, int type, u32 size, void **out, int ms)
{
	struct list_head *l;
	wait_queue_head_t *wq;
	struct ib_node *n, *pick = NULL;
	unsigned long flags;
	long t;

	*out = NULL;
	if (chn >= N_CHN)
		return -1;
	if (type == IB_FREE) {
		l = &ib_free;
		wq = &ib_free_wq;
	} else if (type == IB_BPU) {
		l = &ib_bpu;
		wq = &ib_bpu_wq;
	} else {
		pr_err("IK:Err: IB: isp_list_type %d not support \n", type);
		return -1;
	}
	if (ib_mem_flag < 2) {
		if (ms < 0) {
			if (wait_event_interruptible(*wq, !ib_empty(l)))
				return -ERESTARTSYS;
		} else if (ms > 0) {
			t = wait_event_interruptible_timeout(*wq, !ib_empty(l),
							     msecs_to_jiffies(ms));
			if (t < 1)
				return -ERESTARTSYS;
		}
	} else if (ib_kept[chn]) {
		spin_lock_irqsave(&ib_lock, flags);
		*out = ib_kept[chn];
		spin_unlock_irqrestore(&ib_lock, flags);
		return 0;
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
	ib_kept[chn] = pick;
	list_del(&pick->node);
	spin_unlock_irqrestore(&ib_lock, flags);
	*out = pick;
	return 0;
}
EXPORT_SYMBOL(ib_get_buf);

int ib_put_buf(u32 chn, int type, void *buf)
{
	struct ib_node *b = buf, *n;
	unsigned long flags;

	if (ib_mem_flag >= 2)
		return 0;
	if (type != IB_FREE && type != IB_BPU) {
		pr_err("IK:Err: IB: isp_list_type %d not support \n", type);
		return -1;
	}
	spin_lock_irqsave(&ib_lock, flags);
	if (type == IB_FREE) {
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

int ib_init(const u32 *pools, u32 w, u32 h)
{
	struct ib_node *n, *tmp, *pos;
	u32 size, phys, i, k, cnt;
	char name[32];
	void __iomem *g;

	ib_mem_flag = 2;
	memcpy(ib_conf, pools, sizeof(ib_conf));
	list_for_each_entry_safe(n, tmp, &ib_all, all) {
		list_del(&n->all);
		kfree(n);
	}
	INIT_LIST_HEAD(&ib_all);
	INIT_LIST_HEAD(&ib_free);
	INIT_LIST_HEAD(&ib_bpu);
	memset(ib_kept, 0, sizeof(ib_kept));
	for (k = 0; k < ib_conf[0] && k < IB_POOLS; k++) {
		cnt = ib_conf[2 + k * 2];
		size = (ib_conf[1 + k * 2] + 0x2f) & ~0x1f;
		if (cnt * size == 0)
			continue;
		snprintf(name, sizeof(name), "ib[0x%x][%d]", size, cnt);
		if (mmz_alloc(name, cnt * size, &phys)) {
			pr_err("IK:Err: IB: mmz malloc memory failed\n");
			return -1;
		}
		list_for_each_entry(pos, &ib_free, node)
			if (pos->size >= size)
				break;
		for (i = 0; i < cnt; i++) {
			n = kzalloc(sizeof(*n), GFP_KERNEL);
			if (!n) {
				pr_err("IK:Err: IB: kmalloc failed\n");
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
	}
	return 0;
}

static void ib_show(struct seq_file *m, struct ib_node *n)
{
	seq_printf(m, "%-8p  %-8x  %-8x  %-5d  %-6d  %-6d  %-7d  %-7d\n", n,
		   n->phys, n->size, n->w, n->h, n->bind[0], n->bind[1],
		   n->bind[2]);
}

void ib_proc_show(struct seq_file *m, int *k)
{
	static const char hdr[] =
		"Node \t  PhysAddr  Size      Width  Height  BindId  BindDev  BindChn\n";
	struct ib_node *n;
	unsigned long flags;
	void __iomem *g;
	bool over;
	u32 i;

	seq_printf(m, "\n[%d.IB(isp buffer info)]:\n", (*k)++);
	seq_puts(m, "-----COMMON POOL CONFIG--------------------------------------------------------\n");
	seq_puts(m, "PoolId  Count  Size\n");
	for (i = 0; i < ib_conf[0] && i < IB_POOLS; i++)
		seq_printf(m, "%-6d  %-5d  %-4d\n", i, ib_conf[2 + i * 2],
			   ib_conf[1 + i * 2]);
	spin_lock_irqsave(&ib_lock, flags);
	seq_puts(m, "-----IB FREE LIST STATUS-------------------------------------------------------\n");
	seq_puts(m, hdr);
	list_for_each_entry(n, &ib_free, node)
		ib_show(m, n);
	seq_puts(m, "-----IB BPU LIST STATUS-------------------------------------------------------\n");
	seq_puts(m, hdr);
	list_for_each_entry(n, &ib_bpu, node)
		ib_show(m, n);
	spin_unlock_irqrestore(&ib_lock, flags);
	seq_puts(m, "-----IB ALL LIST STATUS-------------------------------------------------------\n");
	seq_puts(m, "Node \t  PhysAddr\tSize\t  Width  Height  BindId  BindDev  BindChn  Overflow\n");
	/* each buffer's guard checked, and put back if overwritten */
	list_for_each_entry(n, &ib_all, all) {
		over = false;
		g = ioremap(n->phys + n->size - 0x10, 0x10);
		if (g) {
			char s[0x10];

			memcpy_fromio(s, g, sizeof(s));
			over = memcmp(s, IB_GUARD, 0x10) != 0;
			if (over)
				memcpy_toio(g, IB_GUARD, 0x10);
			iounmap(g);
		}
		seq_printf(m, "%-8p  %-8x      %-8x  %-5d  %-6d  %-6d  %-7d  %-7d  %-7d\n",
			   n, n->phys, n->size, n->w, n->h, n->bind[0],
			   n->bind[1], n->bind[2], over);
	}
}
