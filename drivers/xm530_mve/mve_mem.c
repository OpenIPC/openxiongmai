// SPDX-License-Identifier: GPL-2.0
/*
 * xm530_mve: the firmware's page tables, the firmware image, and the event
 * queues.
 */

#include <linux/firmware.h>
#include <linux/jiffies.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "mve.h"

const struct mve_range mve_regions[MVE_REGIONS] = {
	{0x00000000, 0x10000000}, /* firmware, core 0 */
	{0x10079000, 0x1007a000}, /* message queues, host's and firmware's */
	{0x1007a000, 0x1007b000},
	{0x1007b000, 0x1007c000}, /* input buffers */
	{0x1007c000, 0x1007d000},
	{0x1007d000, 0x1007e000}, /* output buffers */
	{0x1007e000, 0x1007f000},
	{0x1007f000, 0x10080000}, /* RPC */
	{0x20000000, 0x50000000}, /* bitstreams, the firmware's own memory */
	{0x50000000, 0x80000000}, /* frames */
	{0x80000000, 0x90000000}, /* firmware, cores 1 to 7 */
	{0x90000000, 0xa0000000}, {0xa0000000, 0xb0000000},
	{0xb0000000, 0xc0000000}, {0xc0000000, 0xd0000000},
	{0xd0000000, 0xe0000000}, {0xe0000000, 0xf0000000},
};

/*
 * MMU: a page of 1024 L1 entries, each a page of 1024 L2 entries, allocated
 * when first used.
 */
struct mve_map {
	struct list_head node;
	u32 va, n, max;
	u32 *pages; /* the MMU's to free, or NULL */
};

struct mve_mmu {
	u32 l1;
	u32 dummy; /* behind reserved addresses */
	struct list_head maps;
};

/* the address of va's L2 entry, 0 without one (and no memory for it) */
static u32 l2_addr(struct mve_mmu *mmu, u32 va)
{
	u32 l1 = mmu->l1 + 4 * (va >> 22), e = mve_rsrc_mem_read32(l1), page;

	if (!e) {
		page = alloc_mve_page(); /* zeroed, and cleaned */
		if (!page)
			return 0;
		e = PTE(page, PTE_RO);
		mve_rsrc_mem_write32(l1, e);
		mve_rsrc_mem_clean_cache_range(l1, 4);
	}
	return (((e >> 2) & 0xfffff) << 12) + 4 * ((va >> 12) & 0x3ff);
}

static void set_entry(u32 addr, u32 val)
{
	mve_rsrc_mem_write32(addr, val);
	mve_rsrc_mem_clean_cache_range(addr, 4);
}

/* whether n pages at va are free; allocates their L2 pages */
static int range_free(struct mve_mmu *mmu, u32 va, u32 n, u32 *used)
{
	u32 i, e;

	for (i = 0; i < n; i++) {
		e = l2_addr(mmu, va + (i << 12));
		if (!e)
			return -ENOMEM;
		if (mve_rsrc_mem_read32(e)) {
			*used = va + (i << 12);
			return -EBUSY;
		}
	}
	return 0;
}

static struct mve_map *add_map(struct mve_mmu *mmu, u32 va, u32 n, u32 max,
			       u32 *pages)
{
	struct mve_map *m = kzalloc(sizeof(*m), GFP_KERNEL);

	if (!m)
		return NULL;
	m->va = va;
	m->n = n;
	m->max = max;
	m->pages = pages;
	list_add(&m->node, &mmu->maps);
	return m;
}

static void del_map(struct mve_map *m)
{
	list_del(&m->node);
	if (m->pages)
		free_mve_pages(m->pages, m->n);
	kfree(m);
}

static struct mve_map *find_map(struct mve_mmu *mmu, u32 va)
{
	struct mve_map *m;

	list_for_each_entry(m, &mmu->maps, node)
		if (m->va == va)
			return m;
	return NULL;
}

struct mve_mmu *mve_mmu_create(void)
{
	struct mve_mmu *mmu = kzalloc(sizeof(*mmu), GFP_KERNEL);

	if (!mmu)
		return NULL;
	INIT_LIST_HEAD(&mmu->maps);
	mmu->l1 = alloc_mve_page();
	mmu->dummy = alloc_mve_page();
	if (mmu->l1 && mmu->dummy)
		return mmu;
	if (mmu->l1)
		free_mve_page(mmu->l1);
	if (mmu->dummy)
		free_mve_page(mmu->dummy);
	kfree(mmu);
	return NULL;
}

void mve_mmu_destroy(struct mve_mmu *mmu)
{
	struct mve_map *m, *tmp;
	u32 i, e;

	if (!mmu)
		return;
	list_for_each_entry_safe(m, tmp, &mmu->maps, node)
		del_map(m);
	for (i = 0; i < 1024; i++) {
		e = mve_rsrc_mem_read32(mmu->l1 + 4 * i);
		if (e)
			free_mve_page(((e >> 2) & 0xfffff) << 12);
	}
	free_mve_page(mmu->dummy);
	free_mve_page(mmu->l1);
	kfree(mmu);
}

u32 mve_mmu_ctrl(struct mve_mmu *mmu)
{
	return PTE(mmu->l1, 3);
}

int mve_mmu_map(struct mve_mmu *mmu, u32 va, const u32 *pages, u32 n,
		u32 flags)
{
	u32 i, used;
	int ret = range_free(mmu, va, n, &used);

	if (ret)
		return ret;
	if (!add_map(mmu, va, n, n, NULL))
		return -ENOMEM;
	for (i = 0; i < n; i++)
		set_entry(l2_addr(mmu, va + (i << 12)), PTE(pages[i], flags));
	return 0;
}

int mve_mmu_map_table(struct mve_mmu *mmu, u32 va, const u32 *entries)
{
	u32 i, used;
	int ret = range_free(mmu, va, 1024, &used);

	if (ret)
		return ret;
	if (!add_map(mmu, va, 1024, 1024, NULL))
		return -ENOMEM;
	for (i = 0; i < 1024; i++)
		set_entry(l2_addr(mmu, va + (i << 12)), entries[i]);
	return 0;
}

u32 mve_mmu_map_in(struct mve_mmu *mmu, const struct mve_range *range,
		   u32 *pages, u32 n, u32 max, u32 align, bool own)
{
	const u32 flags = PTE_SHARED_RW | PTE_RW;
	u32 va, used, i;
	int ret;

	if (!n || max < n || max > (range->end - range->start) >> 12)
		return 0;
	if (align < PAGE_SIZE)
		align = PAGE_SIZE;
	va = ALIGN(range->start, align);
	for (;;) {
		if (va < range->start || va >= range->end ||
		    range->end - va < max << 12)
			return 0;
		ret = range_free(mmu, va, max, &used);
		if (!ret)
			break;
		if (ret != -EBUSY)
			return 0;
		va = ALIGN(used + PAGE_SIZE, align);
	}
	if (!add_map(mmu, va, n, max, own ? pages : NULL))
		return 0;
	for (i = 0; i < max; i++)
		set_entry(l2_addr(mmu, va + (i << 12)),
			  PTE(i < n ? pages[i] : mmu->dummy, flags));
	return va;
}

/* The firmware's own memory grown or shrunk within what it reserved */
int mve_mmu_resize(struct mve_mmu *mmu, u32 va, u32 n)
{
	const u32 flags = PTE_SHARED_RW | PTE_RW;
	struct mve_map *m = find_map(mmu, va);
	u32 *more, *pages;
	u32 i, old;

	if (!m || !m->pages || !n || n > m->max)
		return -EINVAL;
	old = m->n;
	if (n == old)
		return 0;
	/* a new list: its allocator and its free know no resize */
	pages = alloc_mve_pages(n);
	if (!pages)
		return -ENOMEM;
	more = m->pages;
	for (i = 0; i < min(n, old); i++)
		swap(pages[i], more[i]);
	for (i = min(n, old); i < m->max; i++)
		set_entry(l2_addr(mmu, va + (i << 12)),
			  PTE(i < n ? pages[i] : mmu->dummy, flags));
	/* the old list now holds new, unused pages and those cut off */
	free_mve_pages(more, old);
	m->pages = pages;
	m->n = n;
	return 0;
}

void mve_mmu_unmap(struct mve_mmu *mmu, u32 va)
{
	struct mve_map *m = find_map(mmu, va);
	u32 i, e;

	if (!m)
		return;
	for (i = 0; i < m->max; i++) {
		e = l2_addr(mmu, va + (i << 12));
		if (e)
			set_entry(e, 0);
	}
	del_map(m);
}

/*
 * The firmware: text mapped for every core from VA 0x1000 of its region, then
 * BSS, some of it a core's own and some shared by the cores.
 */
#define FW_NAME "hevcenc.fwb"
#define FW_PAGES_MAX 1024

struct mve_fw_header {
	u32 rasc_jmp;
	u8 minor, major, reserved[2];
	u8 info[56];
	u8 version[32];
	u32 text_length;
	u32 bss_start;
	u32 bss_bitmap_size;
	u32 bss_bitmap[16];
	u32 master_rw_start;
	u32 master_rw_size;
};

struct mve_fw {
	u32 ncores;
	u32 ntext, nshared, nbss; /* nbss: for all the cores */
	u32 *text, *shared, *bss;
	u32 mapped; /* cores */
};

static const int fw_region[] = {
	MVE_REGION_FW0,	    MVE_REGION_FW1,	MVE_REGION_FW1 + 1,
	MVE_REGION_FW1 + 2, MVE_REGION_FW1 + 3, MVE_REGION_FW1 + 4,
	MVE_REGION_FW1 + 5, MVE_REGION_FW1 + 6,
};

void mve_fw_unload(struct mve_mmu *mmu, struct mve_fw *fw)
{
	u32 i;

	if (!fw)
		return;
	for (i = 0; i < fw->mapped; i++)
		mve_mmu_unmap(mmu, mve_regions[fw_region[i]].start);
	if (fw->text)
		free_mve_pages(fw->text, fw->ntext);
	if (fw->shared)
		free_mve_pages(fw->shared, fw->nshared);
	if (fw->bss)
		free_mve_pages(fw->bss, fw->nbss);
	kfree(fw);
}

enum { BSS_NONE, BSS_CORE, BSS_SHARED };

static int bss_kind(const struct mve_fw_header *h, u32 page)
{
	u32 va = h->bss_start + (page << 12);

	if (va >= h->master_rw_start &&
	    va - h->master_rw_start < h->master_rw_size)
		return BSS_SHARED;
	return h->bss_bitmap[page >> 5] & (1u << (page & 31)) ? BSS_CORE :
								BSS_NONE;
}

struct mve_fw *mve_fw_load(struct mve_mmu *mmu, struct device *dev, int ncores,
			   u16 *version)
{
	const struct firmware *blob;
	const struct mve_fw_header *h;
	struct mve_fw *fw = NULL;
	u32 i, core, per_core = 0, nbss = 0, b, sh, text, bss, *table = NULL;

	if (ncores < 1 || ncores > ARRAY_SIZE(fw_region))
		return NULL;
	if (request_firmware(&blob, FW_NAME, dev) < 0) {
		pr_err("mve: no firmware " FW_NAME "\n");
		return NULL;
	}
	h = (const void *)blob->data;
	if (blob->size < sizeof(*h))
		goto bad;
	text = DIV_ROUND_UP(h->text_length, PAGE_SIZE);
	bss = h->bss_start >> 12;
	if (h->major != 2 || (h->bss_start & ~PAGE_MASK) ||
	    h->bss_bitmap_size > 8 * sizeof(h->bss_bitmap) ||
	    h->text_length > blob->size || 1 + text > bss ||
	    bss > FW_PAGES_MAX || h->bss_bitmap_size > FW_PAGES_MAX - bss)
		goto bad;
	*version = h->major << 8 | h->minor;

	fw = kzalloc(sizeof(*fw), GFP_KERNEL);
	table = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!fw || !table)
		goto fail;
	fw->ncores = ncores;
	for (i = 0; i < h->bss_bitmap_size; i++) {
		if (bss_kind(h, i) == BSS_SHARED)
			fw->nshared++;
		else if (bss_kind(h, i) == BSS_CORE)
			per_core++;
	}
	nbss = per_core * ncores;
	fw->ntext = text; /* what is mapped; the rest of the file is not */
	fw->text = alloc_mve_pages(fw->ntext);
	if (!fw->text)
		goto fail;
	if (fw->nshared) {
		fw->shared = alloc_mve_pages(fw->nshared);
		if (!fw->shared)
			goto fail;
	}
	if (nbss) {
		fw->bss = alloc_mve_pages(nbss);
		if (!fw->bss)
			goto fail;
		fw->nbss = nbss;
	}
	for (i = 0; i < fw->ntext; i++) {
		memcpy(mve_rsrc_mem_cpu_map_page(fw->text[i]),
		       blob->data + (i << 12),
		       min_t(size_t, PAGE_SIZE, blob->size - (i << 12)));
		mve_rsrc_mem_clean_cache_range(fw->text[i], PAGE_SIZE);
	}

	for (i = 0; i < text; i++)
		table[1 + i] = PTE(fw->text[i], PTE_EXEC);
	for (core = 0, b = 0; core < ncores; core++) {
		for (i = 0, sh = 0; i < h->bss_bitmap_size; i++) {
			switch (bss_kind(h, i)) {
			case BSS_SHARED:
				table[bss + i] = PTE(fw->shared[sh++], PTE_RW);
				break;
			case BSS_CORE:
				table[bss + i] = PTE(fw->bss[b++], PTE_RW);
				break;
			}
		}
		if (mve_mmu_map_table(mmu, mve_regions[fw_region[core]].start,
				      table))
			goto fail;
		fw->mapped++;
	}
	kfree(table);
	release_firmware(blob);
	return fw;
bad:
	pr_err("mve: firmware " FW_NAME " is not a protocol 2 image\n");
fail:
	mve_fw_unload(mmu, fw);
	kfree(table);
	release_firmware(blob);
	return NULL;
}

/* Event queues */
void mve_queue_init(struct mve_event_queue *q, u32 max)
{
	INIT_LIST_HEAD(&q->list);
	mutex_init(&q->lock);
	init_waitqueue_head(&q->wait);
	q->bytes = 0;
	q->max = max;
	q->interrupted = false;
}

void mve_queue_clear(struct mve_event_queue *q)
{
	struct mve_event *ev, *tmp;

	list_for_each_entry_safe(ev, tmp, &q->list, node) {
		list_del(&ev->node);
		kfree(ev);
	}
	q->bytes = 0;
}

void mve_queue_flush(struct mve_event_queue *q)
{
	mutex_lock(&q->lock);
	mve_queue_clear(q);
	mutex_unlock(&q->lock);
}

static u32 event_bytes(u16 size)
{
	return 4 + ALIGN(size, 4);
}

void mve_queue_add(struct mve_event_queue *q, u16 code, u16 size,
		   const void *data)
{
	struct mve_event *ev =
		kmalloc(sizeof(*ev) + ALIGN(size, 4), GFP_KERNEL);

	if (!ev)
		return;
	ev->e.code = code;
	ev->e.size = size;
	if (size)
		memcpy(ev->e.data, data, size);
	mutex_lock(&q->lock);
	if (q->bytes + event_bytes(size) > q->max) {
		mutex_unlock(&q->lock);
		kfree(ev);
		return;
	}
	q->bytes += event_bytes(size);
	list_add_tail(&ev->node, &q->list);
	mutex_unlock(&q->lock);
	wake_up_interruptible(&q->wait);
}

struct mve_event *mve_queue_get(struct mve_event_queue *q, u32 timeout)
{
	struct mve_event *ev = NULL;

	if (timeout) {
		q->interrupted = false;
		if (wait_event_interruptible_timeout(
			    q->wait, !list_empty(&q->list) || q->interrupted,
			    msecs_to_jiffies(timeout)) <= 0)
			return NULL;
	}
	mutex_lock(&q->lock);
	if (!list_empty(&q->list)) {
		ev = list_first_entry(&q->list, struct mve_event, node);
		list_del(&ev->node);
		q->bytes -= event_bytes(ev->e.size);
	}
	mutex_unlock(&q->lock);
	return ev;
}

void mve_queue_interrupt(struct mve_event_queue *q)
{
	q->interrupted = true;
	wake_up_interruptible(&q->wait);
}
