// SPDX-License-Identifier: GPL-2.0
/*
 * xm_mmz: the media memory zone of the XM510/XM530/XM550, a drop-in
 * replacement for the vendor's mmz.ko (built as open_mmz.ko).
 *
 * The SDK and the video drivers take their DMA buffers from one physically
 * contiguous region kept out of Linux by mem= on the kernel command line
 * (mmz_start, mmz_size). Each buffer is a named block; a name exists once, and
 * a driver finds a buffer another one made by asking for the name.
 *
 * The vendor allocator, kept here exactly:
 *  - blocks are laid one after another from mmz_start, each on a page and
 *    followed by a 16-byte marker ("MEMORY BARRIERS") in the space up to the
 *    next one, so /proc/umap/mmz can tell an overrun;
 *  - a block is never freed on its own: free succeeds and does nothing, and
 *    only a reset (_IO('M', 0)) forgets every block, so whoever starts the
 *    media pipeline resets first;
 *  - "cached" is a flag a block carries for mmap, which maps it write-back
 *    rather than uncached.
 *
 * User space reaches it through /dev/mmz with a record per call: 44 bytes on
 * XM530/XM550, 40 on XM510, which has no user address in it.
 *
 * The XM510's mmz.ko (3.0 kernel) is an older one, and on CONFIG_ARCH_XM510
 * this does what it does: blocks are laid on the next page with no marker,
 * and /proc/umap/mmz has its layout, without the overrun flag and the free
 * space. Its interface is a subset of this one (ioctls 0-3, mmz_alloc and
 * mmz_free) and it has no mmap: user space maps the zone through /dev/mem.
 *
 * allocator=cma (XM530/XM550 on a kernel with CONFIG_CMA) takes the blocks
 * from the kernel's CMA area instead, with dma_alloc_coherent(), so memory the
 * media pipeline does not use stays Linux's. Kernel blocks named in zone_names
 * still come from the zone: xm530_rgn ioremap()s its block, and ARM refuses to
 * ioremap RAM. A reset forgets the CMA blocks but frees them only cma_hold_ms
 * later, and only once nothing maps them: the previous pipeline's DMA may
 * still be running into them until the next one sets the hardware up, and
 * Linux must not have the pages by then. A name asked for again before that
 * gets its block back, at the address it had, as from the zone.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/namei.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/ratelimit.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <asm/cacheflush.h>
#include <asm/outercache.h>

#define MMZ_NAME_LEN 32
static const char mmz_marker[16] = "MEMORY BARRIERS";

/* The record every /dev/mmz call takes, as the vendor SDK lays it out */
struct mmz_rec {
	char name[MMZ_NAME_LEN];
	u32 size;
	u32 phys;
	u32 vaddr; /* user address of a mapping, for the cache flush */
};

#define MMZ_IOC_RESET _IO('M', 0)
#define MMZ_IOC_ALLOC _IOWR('M', 1, struct mmz_rec)
#define MMZ_IOC_FREE _IOW('M', 2, struct mmz_rec)
#define MMZ_IOC_GET _IOR('M', 3, struct mmz_rec) /* read in, written back */
#define MMZ_IOC_ALLOC_CACHED _IOWR('M', 4, struct mmz_rec)
#define MMZ_IOC_FLUSH _IOW('M', 5, struct mmz_rec)
#define MMZ_IOC_CACHED _IOW('M', 6, struct mmz_rec)
#define MMZ_IOC_UNCACHED _IOW('M', 7, struct mmz_rec)

struct mmz_block {
	struct list_head list; /* in address order */
	char name[MMZ_NAME_LEN];
	u32 phys, size;
	bool cached;
	bool marked; /* the overrun marker was written */
	/* CMA blocks only */
	void *vaddr; /* the kernel's mapping */
	u32 room; /* what was allocated: guarded(size) when it was */
	bool stale; /* forgotten by a reset, not yet freed */
	unsigned long stale_at; /* jiffies of that reset */
	int maps; /* user mappings */
};

static unsigned int mmz_start;
static unsigned int mmz_size;
module_param(mmz_start, uint, 0444);
MODULE_PARM_DESC(mmz_start, "memalloc start addr");
module_param(mmz_size, uint, 0444);
MODULE_PARM_DESC(mmz_size, "memalloc size");

static char *allocator = "xm";
module_param(allocator, charp, 0444);
MODULE_PARM_DESC(allocator, "xm: blocks from the zone; cma: from the CMA area");
static unsigned int cma_size;
module_param(cma_size, uint, 0444);
MODULE_PARM_DESC(cma_size, "most the CMA blocks may take, 0 for no limit");
static char *zone_names = "rgn";
module_param(zone_names, charp, 0444);
MODULE_PARM_DESC(zone_names,
		 "with allocator=cma, kernel blocks kept in the zone (a,b,...)");
static unsigned int cma_hold_ms = 5000;
module_param(cma_hold_ms, uint, 0644);
MODULE_PARM_DESC(cma_hold_ms, "how long a reset keeps the CMA blocks");

static LIST_HEAD(blocks);
static LIST_HEAD(cma_blocks); /* in address order */
static DEFINE_SPINLOCK(mmz_lock);
static bool use_cma;
static u32 cma_used; /* the room of every CMA block, stale ones included */
static struct platform_device *mmz_pdev;
/* Held while CMA blocks are freed, and by /proc while it reads them */
static DEFINE_MUTEX(reap_lock);
static void reap_stale(bool all);
static void reap_work_fn(struct work_struct *work);
static DECLARE_DELAYED_WORK(reap_work, reap_work_fn);
static unsigned long reset_at; /* jiffies of the last reset */

#ifdef CONFIG_ARCH_XM510
#define MMZ_MARKER 0 /* the XM510 module lays blocks with no marker */
#else
#define MMZ_MARKER 1
#endif

/* The space a block takes: its size and the marker, to the next page */
static u32 guarded(u32 size)
{
	return ALIGN(size + (MMZ_MARKER ? sizeof(mmz_marker) : 0), PAGE_SIZE);
}

static struct mmz_block *find_name(const char *name)
{
	struct mmz_block *b;

	list_for_each_entry (b, &blocks, list)
		if (!strncmp(b->name, name, MMZ_NAME_LEN))
			return b;
	list_for_each_entry (b, &cma_blocks, list)
		if (!b->stale && !strncmp(b->name, name, MMZ_NAME_LEN))
			return b;
	return NULL;
}

/* The CMA block holding phys, a stale one included */
static struct mmz_block *find_cma(u32 phys)
{
	struct mmz_block *b;

	list_for_each_entry (b, &cma_blocks, list)
		if (phys >= b->phys && phys - b->phys < b->room)
			return b;
	return NULL;
}

static struct mmz_block *find_phys(u32 phys)
{
	struct mmz_block *b;

	list_for_each_entry (b, &blocks, list)
		if (phys >= b->phys && phys - b->phys < b->size)
			return b;
	return find_cma(phys);
}

/* The marker after a block: write it, or check it */
static bool guard_marker(u32 phys, u32 size, bool write)
{
	void __iomem *p = ioremap(phys + size, sizeof(mmz_marker));
	bool ok = true;

	if (!p)
		return false;
	if (write)
		memcpy_toio(p, mmz_marker, sizeof(mmz_marker));
	else {
		char got[sizeof(mmz_marker)];

		memcpy_fromio(got, p, sizeof(got));
		ok = !memcmp(got, mmz_marker, sizeof(got));
	}
	iounmap(p);
	return ok;
}

static int alloc_block(const char *name, u32 size, u32 *phys, bool cached)
{
	struct mmz_block *b, *last;
	unsigned long flags;
	u32 at;

	if (!size)
		return -1;
	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b) {
		pr_err("mmz: kmalloc failed\n");
		return -1;
	}
	strncpy(b->name, name, MMZ_NAME_LEN - 1);
	b->size = size;
	b->cached = cached;

	spin_lock_irqsave(&mmz_lock, flags);
	if (find_name(b->name)) {
		spin_unlock_irqrestore(&mmz_lock, flags);
		pr_err("mmz: name[%s] already exists\n", b->name);
		kfree(b);
		return -1;
	}
	at = mmz_start;
	if (!list_empty(&blocks)) {
		last = list_entry(blocks.prev, struct mmz_block, list);
		at = last->phys + guarded(last->size);
	}
	/* Each term against what is left, so a huge size cannot wrap */
	if (at < mmz_start || at - mmz_start > mmz_size ||
	    size > mmz_size - (at - mmz_start) ||
	    guarded(size) > mmz_size - (at - mmz_start)) {
		spin_unlock_irqrestore(&mmz_lock, flags);
		pr_err("mmz: out of memory\n");
		kfree(b);
		return -1;
	}
	b->phys = at;
	list_add_tail(&b->list, &blocks);
	spin_unlock_irqrestore(&mmz_lock, flags);

	/* From locals: once the lock is dropped a reset may free b */
	if (!MMZ_MARKER) {
		/* nothing to write */
	} else if (guard_marker(at, size, true)) {
		spin_lock_irqsave(&mmz_lock, flags);
		list_for_each_entry (b, &blocks, list)
			if (b->phys == at) {
				b->marked = true;
				break;
			}
		spin_unlock_irqrestore(&mmz_lock, flags);
	} else {
		pr_warn("mmz: no overrun marker for name[%s]\n", name);
	}
	*phys = at;
	return 0;
}

static int alloc_cma(const char *name, u32 size, u32 *phys, bool cached);

/* A kernel caller's block: the zone's for a name in zone_names, or without
 * CMA */
static int alloc_kernel(const char *name, u32 size, u32 *phys, bool cached)
{
	const char *p = zone_names;
	size_t n = strnlen(name, MMZ_NAME_LEN);

	while (use_cma && p) {
		const char *end = strchr(p, ',');
		size_t len = end ? end - p : strlen(p);

		if (len == n && !strncmp(p, name, n))
			break;
		p = end ? end + 1 : NULL;
	}
	if (use_cma && !p)
		return alloc_cma(name, size, phys, cached);
	return alloc_block(name, size, phys, cached);
}

int mmz_alloc(const char *name, u32 size, u32 *phys)
{
	return alloc_kernel(name, size, phys, false);
}
EXPORT_SYMBOL(mmz_alloc);

int mmz_alloc_cached(const char *name, u32 size, u32 *phys)
{
	return alloc_kernel(name, size, phys, true);
}
EXPORT_SYMBOL(mmz_alloc_cached);

int media_mem_alloc(const char *name, u32 size, u32 *phys, int cached)
{
	return alloc_kernel(name, size, phys, cached != 0);
}
EXPORT_SYMBOL(media_mem_alloc);

/* A user block from the CMA area: a stale one of the name if it is big
 * enough, a new one otherwise */
static int try_alloc_cma(const char *name, u32 size, u32 *phys, bool cached,
			 bool quiet)
{
	struct mmz_block *b, *pos;
	unsigned long flags;
	dma_addr_t dma;
	const u32 room = guarded(size);

	if (!size || size > room)
		return -1;
	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b) {
		pr_err("mmz: kmalloc failed\n");
		return -1;
	}
	strncpy(b->name, name, MMZ_NAME_LEN - 1);
	b->size = size;
	b->cached = cached;
	b->room = room;

	spin_lock_irqsave(&mmz_lock, flags);
	if (find_name(b->name))
		goto exists;
	list_for_each_entry (pos, &cma_blocks, list)
		if (pos->stale && pos->room >= room &&
		    !strncmp(pos->name, b->name, MMZ_NAME_LEN)) {
			pos->stale = false;
			pos->size = size;
			pos->cached = cached;
			memcpy(pos->vaddr + size, mmz_marker, sizeof(mmz_marker));
			*phys = pos->phys;
			spin_unlock_irqrestore(&mmz_lock, flags);
			kfree(b);
			return 0;
		}
	if (cma_size && room > cma_size - cma_used) {
		spin_unlock_irqrestore(&mmz_lock, flags);
		if (!quiet)
			pr_err("mmz: out of memory for name[%s] size %#x\n",
			       b->name, size);
		kfree(b);
		return -ENOMEM;
	}
	cma_used += room;
	spin_unlock_irqrestore(&mmz_lock, flags);

	b->vaddr = dma_alloc_coherent(&mmz_pdev->dev, room, &dma, GFP_KERNEL);
	if (!b->vaddr) {
		spin_lock_irqsave(&mmz_lock, flags);
		cma_used -= room;
		spin_unlock_irqrestore(&mmz_lock, flags);
		if (!quiet)
			pr_err("mmz: out of memory for name[%s] size %#x\n",
			       b->name, size);
		kfree(b);
		return -ENOMEM;
	}
	b->phys = dma;
	memcpy(b->vaddr + size, mmz_marker, sizeof(mmz_marker));
	b->marked = true;

	spin_lock_irqsave(&mmz_lock, flags);
	if (find_name(b->name)) {
		/* the same name, allocated meanwhile */
		cma_used -= room;
		spin_unlock_irqrestore(&mmz_lock, flags);
		dma_free_coherent(&mmz_pdev->dev, room, b->vaddr, dma);
		pr_err("mmz: name[%s] already exists\n", b->name);
		kfree(b);
		return -1;
	}
	list_for_each_entry (pos, &cma_blocks, list)
		if (pos->phys > b->phys)
			break;
	list_add_tail(&b->list, &pos->list);
	/* under the lock: a reset and the reaper may free b once it is out */
	*phys = b->phys;
	spin_unlock_irqrestore(&mmz_lock, flags);
	return 0;

exists:
	spin_unlock_irqrestore(&mmz_lock, flags);
	pr_err("mmz: name[%s] already exists\n", b->name);
	kfree(b);
	return -1;
}

static bool any_stale(void)
{
	struct mmz_block *b;
	unsigned long flags;
	bool stale = false;

	spin_lock_irqsave(&mmz_lock, flags);
	list_for_each_entry (b, &cma_blocks, list)
		stale |= b->stale && !b->maps;
	spin_unlock_irqrestore(&mmz_lock, flags);
	return stale;
}

/*
 * Out of memory with blocks a reset forgot still held, as when a restart asks
 * for bigger buffers than the run before: wait for the hold to end, free them
 * and try again.
 */
static int alloc_cma(const char *name, u32 size, u32 *phys, bool cached)
{
	unsigned long until;
	int ret = try_alloc_cma(name, size, phys, cached, true);

	if (ret != -ENOMEM)
		return ret ? -1 : 0;
	if (any_stale()) {
		/* a signal cuts the wait short, not the hold: the reaper frees
		 * only blocks whose hold is over */
		until = reset_at + msecs_to_jiffies(cma_hold_ms);
		if (time_before(jiffies, until))
			msleep_interruptible(jiffies_to_msecs(until - jiffies));
		reap_stale(false);
	}
	return try_alloc_cma(name, size, phys, cached, false) ? -1 : 0;
}

/*
 * Free the stale CMA blocks nothing maps whose hold is over, or all of them;
 * come back for the next one whose hold is not
 */
static void reap_stale(bool all)
{
	const unsigned long hold = msecs_to_jiffies(cma_hold_ms);
	unsigned long flags, next = 0;
	struct mmz_block *b, *n;
	bool again = false;
	LIST_HEAD(gone);

	mutex_lock(&reap_lock);
	spin_lock_irqsave(&mmz_lock, flags);
	list_for_each_entry_safe (b, n, &cma_blocks, list) {
		if (!b->stale || b->maps)
			continue;
		if (all || !time_before(jiffies, b->stale_at + hold)) {
			list_move(&b->list, &gone);
			cma_used -= b->room;
		} else if (!again || time_before(b->stale_at + hold, next)) {
			next = b->stale_at + hold;
			again = true;
		}
	}
	spin_unlock_irqrestore(&mmz_lock, flags);
	list_for_each_entry_safe (b, n, &gone, list) {
		dma_free_coherent(&mmz_pdev->dev, b->room, b->vaddr, b->phys);
		kfree(b);
	}
	mutex_unlock(&reap_lock);
	if (again)
		schedule_delayed_work(&reap_work, time_before(jiffies, next) ?
							  next - jiffies :
							  0);
}

static void reap_work_fn(struct work_struct *work)
{
	reap_stale(false);
}

/* As the vendor's: blocks are released only by a reset */
int mmz_free(void)
{
	return 0;
}
EXPORT_SYMBOL(mmz_free);

int mmz_get_addr(const char *name, u32 *size, u32 *phys)
{
	struct mmz_block *b;
	unsigned long flags;

	spin_lock_irqsave(&mmz_lock, flags);
	b = find_name(name);
	if (b) {
		*size = b->size;
		*phys = b->phys;
	}
	spin_unlock_irqrestore(&mmz_lock, flags);
	if (!b) {
		pr_err("mmz: name[%s] does not exist\n", name);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(mmz_get_addr);

static void reset_blocks(void)
{
	struct mmz_block *b, *n;
	unsigned long flags;
	LIST_HEAD(gone);

	spin_lock_irqsave(&mmz_lock, flags);
	list_splice_init(&blocks, &gone);
	reset_at = jiffies;
	/* a block forgotten before is held again: whatever ran meanwhile may
	 * have left the hardware writing to it */
	list_for_each_entry (b, &cma_blocks, list) {
		b->stale = true;
		b->stale_at = reset_at;
	}
	spin_unlock_irqrestore(&mmz_lock, flags);
	if (use_cma)
		schedule_delayed_work(&reap_work,
				      msecs_to_jiffies(cma_hold_ms));
	list_for_each_entry_safe (b, n, &gone, list)
		kfree(b);
}

static void set_cached(u32 phys, bool cached)
{
	struct mmz_block *b;
	unsigned long flags;

	spin_lock_irqsave(&mmz_lock, flags);
	b = find_phys(phys);
	if (b)
		b->cached = cached;
	spin_unlock_irqrestore(&mmz_lock, flags);
}

static long mmz_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct mmz_rec r;
	size_t len;
	int ret;

	if (cmd == MMZ_IOC_RESET) {
		reset_blocks();
		return 0;
	}
	len = _IOC_SIZE(cmd);
	if (_IOC_TYPE(cmd) != 'M' ||
	    (len != sizeof(r) && len != offsetof(struct mmz_rec, vaddr)) ||
	    _IOC_NR(cmd) < 1 || _IOC_NR(cmd) > 7) {
		pr_err("mmz: ioctl cmd error\n");
		return 0;
	}
	/* the command numbers carry the size, so compare them without it */
	cmd &= ~(_IOC_SIZEMASK << _IOC_SIZESHIFT);
	cmd |= sizeof(r) << _IOC_SIZESHIFT;
	memset(&r, 0, sizeof(r));
	if (copy_from_user(&r, uarg, len)) {
		pr_err("mmz: ioctl copy_from_user error.\n");
		return -1;
	}
	r.name[MMZ_NAME_LEN - 1] = 0;

	switch (cmd) {
	case MMZ_IOC_ALLOC:
	case MMZ_IOC_ALLOC_CACHED:
		ret = (use_cma ? alloc_cma : alloc_block)(
			r.name, r.size, &r.phys, cmd == MMZ_IOC_ALLOC_CACHED);
		break;
	case MMZ_IOC_GET:
		ret = mmz_get_addr(r.name, &r.size, &r.phys);
		break;
	case MMZ_IOC_FREE:
		return 0;
	case MMZ_IOC_FLUSH:
		/* the caller's mapping, then the outer cache by address */
		if (len == sizeof(r) && r.vaddr)
			__cpuc_flush_dcache_area((void *)(uintptr_t)r.vaddr,
						 r.size);
		outer_flush_range(r.phys, r.phys + r.size);
		set_cached(r.phys, true);
		return 0;
	case MMZ_IOC_CACHED:
	case MMZ_IOC_UNCACHED:
		set_cached(r.phys, cmd == MMZ_IOC_CACHED);
		return 0;
	default:
		return 0;
	}
	/* The record goes back whether or not the call succeeded, as the
	 * vendor's does: a failed one carries phys 0, which is what callers
	 * test. */
	if (copy_to_user(uarg, &r, len)) {
		pr_err("mmz: ioctl copy_to_user error.\n");
		return -1;
	}
	return ret;
}

/*
 * mmap at the physical address in the offset: write-back for a cached block,
 * uncached otherwise. The vendor maps any physical address it is given; this
 * does too, for now, and says so the first time, so whatever relies on it can
 * be found before it is refused.
 */
static void cma_vm_open(struct vm_area_struct *vma)
{
	struct mmz_block *b = vma->vm_private_data;
	unsigned long flags;

	spin_lock_irqsave(&mmz_lock, flags);
	b->maps++;
	spin_unlock_irqrestore(&mmz_lock, flags);
}

static void cma_vm_close(struct vm_area_struct *vma)
{
	struct mmz_block *b = vma->vm_private_data;
	unsigned long flags;
	bool reap;

	spin_lock_irqsave(&mmz_lock, flags);
	reap = !--b->maps && b->stale;
	spin_unlock_irqrestore(&mmz_lock, flags);
	if (reap)
		schedule_delayed_work(&reap_work,
				      msecs_to_jiffies(cma_hold_ms));
}

static const struct vm_operations_struct cma_vm_ops = {
	.open = cma_vm_open,
	.close = cma_vm_close,
};

/*
 * A CMA block is Linux's memory once it is freed, so a mapping must stay
 * inside one block and keeps it until the mapping goes. Uncached is the
 * attribute the kernel's own mapping of it has.
 */
static int cma_mmap(struct vm_area_struct *vma, u32 phys, unsigned long len)
{
	struct mmz_block *b;
	unsigned long flags;

	spin_lock_irqsave(&mmz_lock, flags);
	b = find_cma(phys);
	if (!b || len > b->room - (phys - b->phys)) {
		spin_unlock_irqrestore(&mmz_lock, flags);
		pr_warn_ratelimited(
			"mmz: mmap of %#x+%#lx is no CMA block (by %s)\n", phys,
			len, current->comm);
		return -EINVAL;
	}
	b->maps++;
	if (!b->cached)
		vma->vm_page_prot = pgprot_dmacoherent(vma->vm_page_prot);
	spin_unlock_irqrestore(&mmz_lock, flags);

	vma->vm_private_data = b;
	vma->vm_ops = &cma_vm_ops;
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, len,
			    vma->vm_page_prot)) {
		cma_vm_close(vma);
		return -EAGAIN;
	}
	return 0;
}

static bool no_ram(unsigned long pfn, unsigned long pages)
{
	while (pages--)
		if (pfn_valid(pfn++))
			return false;
	return true;
}

static int mmz_mmap(struct file *file, struct vm_area_struct *vma)
{
	const u32 phys = vma->vm_pgoff << PAGE_SHIFT;
	const unsigned long len = vma->vm_end - vma->vm_start;
	struct mmz_block *b;
	unsigned long flags;
	bool cached = false, cma;

	spin_lock_irqsave(&mmz_lock, flags);
	cma = find_cma(phys) != NULL;
	b = find_phys(phys);
	if (b)
		cached = b->cached;
	spin_unlock_irqrestore(&mmz_lock, flags);
	if (cma)
		return cma_mmap(vma, phys, len);
	/* Linux's own memory, a CMA block freed since among it, anywhere in the
	 * range */
	if (use_cma && !no_ram(vma->vm_pgoff, len >> PAGE_SHIFT)) {
		pr_warn_ratelimited(
			"mmz: mmap of %#x+%#lx is no CMA block (by %s)\n", phys,
			len, current->comm);
		return -EINVAL;
	}

	if (phys < mmz_start || phys - mmz_start >= mmz_size ||
	    len > mmz_size - (phys - mmz_start))
		pr_warn_once("mmz: mmap of %#x+%#lx outside the zone (by %s)\n",
			     phys, len, current->comm);
	if (!cached)
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, len,
			    vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

/* The XM510 module's layout: start, size and name, nothing else */
static int xm510_seq_show(struct seq_file *s)
{
	struct mmz_block *b;
	unsigned long flags;

	seq_puts(s, "--START-------------------SIZE--------------------NAME"
		    "-------\n");
	/* seq_printf() does not sleep, and there is no marker to map */
	spin_lock_irqsave(&mmz_lock, flags);
	list_for_each_entry (b, &blocks, list)
		seq_printf(s, "%#-10x\t\t%#-10x\t\t%s\n", b->phys, b->size,
			   b->name);
	spin_unlock_irqrestore(&mmz_lock, flags);
	seq_puts(s, "-------------------------------------------------------"
		    "------\n");
	return 0;
}

static int mmz_seq_show(struct seq_file *s, void *unused)
{
	struct mmz_block *b, *copy;
	unsigned long flags;
	int n = 0, i, max;
	u32 used = 0, cma;

	seq_puts(s, "[MMZ] openxiongmai xm_mmz\n\n");
	seq_printf(s, "[%#x--------%#x]\n", mmz_start, mmz_start + mmz_size);
	if (!MMZ_MARKER)
		return xm510_seq_show(s);
	seq_puts(s, "--START-----------SIZE----------------OVERFLOW----NAME"
		    "---------------\n");

	/* The marker check maps each guard page, which cannot happen under the
	 * spinlock, so the records are copied first; the CMA blocks are not
	 * freed until it is done. */
	mutex_lock(&reap_lock);
	spin_lock_irqsave(&mmz_lock, flags);
	list_for_each_entry (b, &blocks, list)
		n++;
	list_for_each_entry (b, &cma_blocks, list)
		n++;
	spin_unlock_irqrestore(&mmz_lock, flags);
	max = n + 8; /* room for blocks added meanwhile */
	copy = kcalloc(max, sizeof(*copy), GFP_KERNEL);
	if (!copy) {
		mutex_unlock(&reap_lock);
		return -ENOMEM;
	}
	n = 0;
	spin_lock_irqsave(&mmz_lock, flags);
	list_for_each_entry (b, &blocks, list)
		if (n < max)
			copy[n++] = *b;
	list_for_each_entry (b, &cma_blocks, list)
		if (n < max && !b->stale)
			copy[n++] = *b;
	cma = cma_used;
	spin_unlock_irqrestore(&mmz_lock, flags);

	for (i = 0; i < n; i++) {
		bool over = false;

		if (copy[i].vaddr)
			over = memcmp(copy[i].vaddr + copy[i].size, mmz_marker,
				      sizeof(mmz_marker));
		else if (copy[i].marked)
			over = !guard_marker(copy[i].phys, copy[i].size, false);
		seq_printf(s, "%#-10x\t%#-10x(%5dk)\t%d\t%s\n", copy[i].phys,
			   copy[i].size,
			   (int)(ALIGN(copy[i].size, PAGE_SIZE) >> 10), over,
			   copy[i].name);
		if (!copy[i].vaddr)
			used += ALIGN(copy[i].size, PAGE_SIZE);
	}
	mutex_unlock(&reap_lock);
	kfree(copy);
	seq_puts(s, "-------------------------------------------------------"
		    "--------------\n");
	seq_printf(s, "FREE: %#-10x(%5dk)\n", mmz_size - used,
		   (mmz_size - used) >> 10);
	/* An addition to the vendor's layout, after it: what CMA holds,
	 * blocks a reset forgot included */
	if (use_cma)
		seq_printf(s, "CMA: %#-10x(%5dk)\n", cma, cma >> 10);
	return 0;
}

static int mmz_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmz_seq_show, NULL);
}

static const struct file_operations mmz_proc_fops = {
	.owner = THIS_MODULE,
	.open = mmz_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations mmz_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = mmz_ioctl,
	.mmap = mmz_mmap,
};

/* /proc/umap: shared with the other media drivers (hx280enc's venc and h264
 * do not need this module), so it is made once and never removed */
static void umap_mkdir(void)
{
	struct path p;

	if (!kern_path("/proc/umap", LOOKUP_DIRECTORY, &p)) {
		path_put(&p);
		return;
	}
	proc_mkdir("umap", NULL);
}

static struct miscdevice mmz_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mmz",
	.fops = &mmz_fops,
};

static int __init xm_mmz_init(void)
{
	int ret;

	if (!strcmp(allocator, "cma")) {
#ifndef CONFIG_CMA
		pr_err("mmz: allocator=cma needs a kernel with CMA\n");
		return -EINVAL;
#endif
		use_cma = true;
	} else if (strcmp(allocator, "xm")) {
		pr_err("mmz: allocator is xm or cma\n");
		return -EINVAL;
	}
	if (!mmz_size && !use_cma) {
		pr_err("mmz: mmz_start and mmz_size are required\n");
		return -EINVAL;
	}
	if (use_cma) {
		/* the device dma_alloc_coherent() allocates for */
		mmz_pdev = platform_device_register_simple("open_mmz", -1,
							   NULL, 0);
		if (IS_ERR(mmz_pdev))
			return PTR_ERR(mmz_pdev);
		mmz_pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
		mmz_pdev->dev.dma_mask = &mmz_pdev->dev.coherent_dma_mask;
	}
	umap_mkdir();
	if (!proc_create("umap/mmz", 0444, NULL, &mmz_proc_fops))
		pr_err("mmz: create mmz proc fail!\n");
	ret = misc_register(&mmz_misc);
	if (ret) {
		pr_err("mmz: register device failed!\n");
		remove_proc_entry("umap/mmz", NULL);
		if (mmz_pdev)
			platform_device_unregister(mmz_pdev);
		return ret;
	}
	pr_info("mmz: openxiongmai xm_mmz init ok start=%#x, size=%#x%s\n",
		mmz_start, mmz_size, use_cma ? ", user blocks from CMA" : "");
	return 0;
}

static void __exit xm_mmz_exit(void)
{
	misc_deregister(&mmz_misc);
	remove_proc_entry("umap/mmz", NULL);
	reset_blocks();
	if (use_cma) {
		/* nothing maps a block now: a mapping holds /dev/mmz open */
		cancel_delayed_work_sync(&reap_work);
		reap_stale(true);
		platform_device_unregister(mmz_pdev);
	}
}

module_init(xm_mmz_init);
module_exit(xm_mmz_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM5x0 media memory zone, compatible with mmz.ko");
