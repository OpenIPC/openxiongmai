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
 */

#include <linux/fs.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
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
};

static unsigned int mmz_start;
static unsigned int mmz_size;
module_param(mmz_start, uint, 0444);
MODULE_PARM_DESC(mmz_start, "memalloc start addr");
module_param(mmz_size, uint, 0444);
MODULE_PARM_DESC(mmz_size, "memalloc size");

static LIST_HEAD(blocks);
static DEFINE_SPINLOCK(mmz_lock);

/* The space a block takes: its size and the marker, to the next page */
static u32 guarded(u32 size)
{
	return ALIGN(size + sizeof(mmz_marker), PAGE_SIZE);
}

static struct mmz_block *find_name(const char *name)
{
	struct mmz_block *b;

	list_for_each_entry (b, &blocks, list)
		if (!strncmp(b->name, name, MMZ_NAME_LEN))
			return b;
	return NULL;
}

static struct mmz_block *find_phys(u32 phys)
{
	struct mmz_block *b;

	list_for_each_entry (b, &blocks, list)
		if (phys >= b->phys && phys - b->phys < b->size)
			return b;
	return NULL;
}

/* The marker after a block: write it, or check it */
static bool guard_marker(const struct mmz_block *b, bool write)
{
	void __iomem *p = ioremap(b->phys + b->size, sizeof(mmz_marker));
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

	guard_marker(b, true);
	*phys = at;
	return 0;
}

int mmz_alloc(const char *name, u32 size, u32 *phys)
{
	return alloc_block(name, size, phys, false);
}
EXPORT_SYMBOL(mmz_alloc);

int mmz_alloc_cached(const char *name, u32 size, u32 *phys)
{
	return alloc_block(name, size, phys, true);
}
EXPORT_SYMBOL(mmz_alloc_cached);

int media_mem_alloc(const char *name, u32 size, u32 *phys, int cached)
{
	return alloc_block(name, size, phys, cached != 0);
}
EXPORT_SYMBOL(media_mem_alloc);

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
	spin_unlock_irqrestore(&mmz_lock, flags);
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
		ret = alloc_block(r.name, r.size, &r.phys,
				  cmd == MMZ_IOC_ALLOC_CACHED);
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
static int mmz_mmap(struct file *file, struct vm_area_struct *vma)
{
	const u32 phys = vma->vm_pgoff << PAGE_SHIFT;
	const unsigned long len = vma->vm_end - vma->vm_start;
	struct mmz_block *b;
	unsigned long flags;
	bool cached = false;

	spin_lock_irqsave(&mmz_lock, flags);
	b = find_phys(phys);
	if (b)
		cached = b->cached;
	spin_unlock_irqrestore(&mmz_lock, flags);

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

static int mmz_seq_show(struct seq_file *s, void *unused)
{
	struct mmz_block *b, *copy;
	unsigned long flags;
	int n = 0, i, max;
	u32 used = 0;

	seq_puts(s, "[MMZ] openxiongmai xm_mmz\n\n");
	seq_printf(s, "[%#x--------%#x]\n", mmz_start, mmz_start + mmz_size);
	seq_puts(s, "--START-----------SIZE----------------OVERFLOW----NAME"
		    "---------------\n");

	/* The marker check maps each guard page, which cannot happen under the
	 * spinlock, so the records are copied first. */
	spin_lock_irqsave(&mmz_lock, flags);
	list_for_each_entry (b, &blocks, list)
		n++;
	spin_unlock_irqrestore(&mmz_lock, flags);
	max = n + 8; /* room for blocks added meanwhile */
	copy = kcalloc(max, sizeof(*copy), GFP_KERNEL);
	if (!copy)
		return -ENOMEM;
	n = 0;
	spin_lock_irqsave(&mmz_lock, flags);
	list_for_each_entry (b, &blocks, list)
		if (n < max)
			copy[n++] = *b;
	spin_unlock_irqrestore(&mmz_lock, flags);

	for (i = 0; i < n; i++) {
		seq_printf(s, "%#-10x\t%#-10x(%5dk)\t%d\t%s\n", copy[i].phys,
			   copy[i].size,
			   (int)(ALIGN(copy[i].size, PAGE_SIZE) >> 10),
			   !guard_marker(&copy[i], false), copy[i].name);
		used += ALIGN(copy[i].size, PAGE_SIZE);
	}
	kfree(copy);
	seq_puts(s, "-------------------------------------------------------"
		    "--------------\n");
	seq_printf(s, "FREE: %#-10x(%5dk)\n", mmz_size - used,
		   (mmz_size - used) >> 10);
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

static struct miscdevice mmz_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mmz",
	.fops = &mmz_fops,
};

static int __init xm_mmz_init(void)
{
	int ret;

	if (!mmz_size) {
		pr_err("mmz: mmz_start and mmz_size are required\n");
		return -EINVAL;
	}
	proc_mkdir("umap", NULL);
	if (!proc_create("umap/mmz", 0444, NULL, &mmz_proc_fops))
		pr_err("mmz: create mmz proc fail!\n");
	ret = misc_register(&mmz_misc);
	if (ret) {
		pr_err("mmz: register device failed!\n");
		remove_proc_entry("umap/mmz", NULL);
		return ret;
	}
	pr_info("mmz: openxiongmai xm_mmz init ok start=%#x, size=%#x\n",
		mmz_start, mmz_size);
	return 0;
}

static void __exit xm_mmz_exit(void)
{
	misc_deregister(&mmz_misc);
	remove_proc_entry("umap/mmz", NULL);
	reset_blocks();
}

module_init(xm_mmz_init);
module_exit(xm_mmz_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM5x0 media memory zone, compatible with mmz.ko");
