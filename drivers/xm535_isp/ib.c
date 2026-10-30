// SPDX-License-Identifier: GPL-2.0
/*
 * The ISP buffers (ib): up to three pools in MMZ (0x4962), each buffer
 * ending in a 16-byte guard. The VICU stores a raw frame into a buffer from
 * the free list (0, sorted by size) and hands it to the BPU list (0x10);
 * the BPU gives it back. With mode >= 2 (the memory saver) each channel
 * keeps the first buffer it gets, and nothing goes back.
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

/* isp.ko's node, 0x48 bytes; the VICU and BPU code read its words */
struct ib_node {
	struct list_head node;
	struct list_head all;
	u32 phys, size, w, h;
	u32 bind[3]; /* id, dev, chn */
	u32 w11[7];  /* 0x2c: stored; 0x34/0x38 the frame's pts, 0x40/0x44 the
		      * store's */
};

static LIST_HEAD(ib_free);
static LIST_HEAD(ib_bpu);
static LIST_HEAD(ib_all);
static DECLARE_WAIT_QUEUE_HEAD(ib_free_wq);
static DECLARE_WAIT_QUEUE_HEAD(ib_bpu_wq);
static DEFINE_SPINLOCK(ib_lock);
static u32 ib_pool[7]; /* count, then size, buffers per pool */
static u8 ib_mode;
static struct ib_node *ib_kept[N_CHN];
static u32 ib_mmz[3][2]; /* phys, size */
static u32 ib_nmmz;

int mmz_alloc(const char *name, u32 size, u32 *phys);
int mmz_free(u32 phys, u32 *size);

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
	int k = chn;
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
		pr_err("IspKo:Err: IB: isp_list_type %d not support \n", type);
		return -1;
	}
	if (ib_mode < 2) {
		if (ms < 0) {
			if (wait_event_interruptible(*wq, !ib_empty(l)))
				return -ERESTARTSYS;
		} else if (ms > 0) {
			t = wait_event_interruptible_timeout(*wq, !ib_empty(l),
							     msecs_to_jiffies(ms));
			if (!t)
				return -ERESTARTSYS;
		}
	} else if (ib_kept[k]) {
		spin_lock_irqsave(&ib_lock, flags);
		*out = ib_kept[k];
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
	ib_kept[k] = pick;
	list_del(&pick->node);
	spin_unlock_irqrestore(&ib_lock, flags);
	*out = pick;
	return 0;
}

int ib_put_buf(u32 chn, int type, void *buf)
{
	struct ib_node *b = buf, *n;
	unsigned long flags;

	if (ib_mode >= 2)
		return 0;
	if (type != IB_FREE && type != IB_BPU) {
		pr_err("IspKo:Err: IB: isp_list_type %d not support \n", type);
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

int ib_init(const u32 *pools, u32 w, u32 h, u8 mode)
{
	struct ib_node *n, *tmp, *pos;
	u32 size, phys, i, k, cnt;
	char name[32];
	void __iomem *g;

	memcpy(ib_pool, pools, sizeof(ib_pool));
	ib_mode = mode;
	list_for_each_entry_safe(n, tmp, &ib_all, all) {
		list_del(&n->all);
		kfree(n);
	}
	INIT_LIST_HEAD(&ib_all);
	INIT_LIST_HEAD(&ib_free);
	INIT_LIST_HEAD(&ib_bpu);
	memset(ib_kept, 0, sizeof(ib_kept));
	ib_nmmz = 0;
	for (k = 0; k < ib_pool[0] && k < 3; k++) {
		cnt = ib_pool[2 + k * 2];
		size = (ib_pool[1 + k * 2] + 0x2f) & ~0x1f;
		if (cnt * size == 0)
			continue;
		snprintf(name, sizeof(name), "ib%d[0x%x][%d]", k, size, cnt);
		if (mmz_alloc(name, cnt * size, &phys)) {
			pr_err("IspKo:Err: IB: mmz malloc memory failed\n");
			return -1;
		}
		ib_mmz[ib_nmmz][0] = phys;
		ib_mmz[ib_nmmz++][1] = cnt * size;
		list_for_each_entry(pos, &ib_free, node)
			if (pos->size >= size)
				break;
		for (i = 0; i < cnt; i++) {
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
	}
	return 0;
}

void ib_exit(void)
{
	u32 i, size;

	for (i = 0; i < ib_nmmz; i++) {
		size = ib_mmz[i][1];
		if (ib_mmz[i][0] && size)
			mmz_free(ib_mmz[i][0], &size);
		ib_mmz[i][0] = ib_mmz[i][1] = 0;
	}
	pr_info("IspKo: ib_exit[%d]\n", ib_nmmz);
	ib_nmmz = 0;
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
	u32 i;

	seq_printf(m, "\n[%d.IB(isp buffer info)]:\n", (*k)++);
	seq_puts(m, "-----COMMON POOL CONFIG--------------------------------------------------------\n");
	seq_puts(m, "PoolId  Count  Size\n");
	for (i = 0; i < ib_pool[0] && i < 3; i++)
		seq_printf(m, "%-6d  %-5d  %-4d\n", i, ib_pool[2 + i * 2],
			   ib_pool[1 + i * 2]);
	spin_lock_irqsave(&ib_lock, flags);
	seq_puts(m, "-----IB FREE LIST STATUS-------------------------------------------------------\n");
	seq_puts(m, hdr);
	list_for_each_entry(n, &ib_free, node)
		ib_show(m, n);
	seq_puts(m, "-----IB BPU LIST STATUS-------------------------------------------------------\n");
	seq_puts(m, hdr);
	list_for_each_entry(n, &ib_bpu, node)
		ib_show(m, n);
	seq_puts(m, "-----IB ALL LIST STATUS-------------------------------------------------------\n");
	seq_puts(m, hdr);
	list_for_each_entry(n, &ib_all, all)
		ib_show(m, n);
	spin_unlock_irqrestore(&ib_lock, flags);
}
