// SPDX-License-Identifier: GPL-2.0
/*
 * The XM533's encoder memory book-keeping (/dev/venc_mem): a drop-in for
 * the vendor venc_mem.ko. User space allocates one block of MMZ for each
 * VENC channel and has this driver hand out parts of it (4 KB aligned,
 * first fit, freed parts merged). Each part has a 16-byte guard after it,
 * checked in /proc/umap/venc_mem.
 *
 *   0x40045601  {chn}: the channel the next calls are for
 *   0x5602      no channel
 *   0x402c5603  the channel's block: 8 words, size, phys, virt
 *   0x5604      the channel's block gone
 *   0xc02c5605  a part: name[32], size; phys and virt back
 *   0x402c5606  that part back (by virt)
 *   0x5600      every channel's book-keeping cleared
 */

#include <linux/fs.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define N_CHN 32
#define GUARD "MEMORY BARRIERS" /* 16 bytes with its 0 */

#define VM_RESET 0x5600
#define VM_SET_CHN 0x40045601
#define VM_UNSET_CHN 0x5602
#define VM_CREATE 0x402c5603
#define VM_DESTROY 0x5604
#define VM_ALLOC 0xc02c5605
#define VM_FREE 0x402c5606

struct arg {
	u32 w[8]; /* the block's settings, a part's name */
	u32 size, phys, virt;
};

struct part {
	struct list_head node;
	u32 len, size, phys, virt; /* len: aligned, size: asked for */
	char name[32];
	u32 used;
};

static struct {
	u32 created;
	struct arg a;
	struct list_head parts;
} chns[N_CHN];
static int cur = -1;
static DEFINE_MUTEX(vm_lock);

static void chn_clear(int c)
{
	struct part *p, *n;

	list_for_each_entry_safe (p, n, &chns[c].parts, node) {
		if (p->used)
			pr_warn("phy %x vir %x size 0x%x should been free first\n",
				p->phys, p->virt, p->size);
		list_del(&p->node);
		kfree(p);
	}
	chns[c].created = 0;
}

static int part_alloc(struct arg *a)
{
	u32 len = (a->size + 0x100f) & ~0xfffu;
	struct part *p, *rest;
	void __iomem *g;

	/* its rounding would wrap, and the guard land outside the part */
	if (!a->size || a->size > 0x7fffffff)
		return -EINVAL;

	list_for_each_entry (p, &chns[cur].parts, node)
		if (!p->used && p->len >= len)
			goto found;
	pr_err("venc_mem: out of memory chn %d %x\n", cur, len);
	return -ENOMEM;
found:
	g = ioremap(p->phys + a->size, 16);
	if (!g) {
		pr_err("venc_mem: ioremap error\n");
		return -EIO;
	}
	memcpy_toio(g, GUARD, 16);
	iounmap(g);
	if (p->len - len >= 0x1000) {
		rest = kzalloc(sizeof(*rest), GFP_KERNEL);
		if (!rest)
			return -ENOMEM;
		rest->len = p->len - len;
		rest->phys = p->phys + len;
		rest->virt = p->virt + len;
		list_add(&rest->node, &p->node);
		p->len = len;
	}
	p->size = a->size;
	memcpy(p->name, a->w, sizeof(p->name));
	p->name[sizeof(p->name) - 1] = 0;
	p->used = 1;
	a->phys = p->phys;
	a->virt = p->virt;
	return 0;
}

static int part_free(u32 virt)
{
	struct list_head *head = &chns[cur].parts;
	struct part *p, *q;

	list_for_each_entry (p, head, node)
		if (p->used && p->virt == virt)
			goto found;
	pr_err("venc_mem: not find viraddr %x\n", virt);
	return -EINVAL;
found:
	p->used = 0;
	if (p->node.next != head) {
		q = list_entry(p->node.next, struct part, node);
		if (!q->used) {
			p->len += q->len;
			list_del(&q->node);
			kfree(q);
		}
	}
	if (p->node.prev != head) {
		q = list_entry(p->node.prev, struct part, node);
		if (!q->used) {
			q->len += p->len;
			list_del(&p->node);
			kfree(p);
		}
	}
	return 0;
}

static long venc_mem_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	void __user *u = (void __user *)arg;
	struct part *p;
	struct arg a;
	long ret = 0;
	u32 c;
	int i;

	mutex_lock(&vm_lock);
	switch (cmd) {
	case VM_RESET:
		for (i = 0; i < N_CHN; i++)
			if (chns[i].created)
				chn_clear(i);
		cur = -1;
		break;
	case VM_SET_CHN:
		if (get_user(c, (u32 __user *)u)) {
			ret = -EFAULT;
		} else if (c >= N_CHN) {
			pr_err("invalid chn %d\n", c);
			ret = -EINVAL;
		} else if (cur != -1) {
			pr_err("venc_mem:should unregister chn[%d] before reigster new chn[%d]\n",
			       cur, c);
			ret = -EBUSY;
		} else {
			cur = c;
		}
		break;
	case VM_UNSET_CHN:
		if (cur == -1) {
			pr_err("venc_mem:chn[%d] has not been register\n", cur);
			ret = -EINVAL;
		}
		cur = -1;
		break;
	case VM_CREATE:
	case VM_DESTROY:
	case VM_ALLOC:
	case VM_FREE:
		if (cur < 0) {
			pr_err("invalid chn %d\n", cur);
			ret = -EINVAL;
			break;
		}
		if (cmd != VM_DESTROY && copy_from_user(&a, u, sizeof(a))) {
			pr_err("venc_mem: ioctl copy_from_user error.\n");
			ret = -EFAULT;
			break;
		}
		if (cmd == VM_CREATE) {
			if (chns[cur].created) {
				pr_err("chn %d have been created\n", cur);
				ret = -EEXIST;
				break;
			}
			p = kzalloc(sizeof(*p), GFP_KERNEL);
			if (!p) {
				ret = -ENOMEM;
				break;
			}
			p->len = p->size = a.size;
			p->phys = a.phys;
			p->virt = a.virt;
			list_add_tail(&p->node, &chns[cur].parts);
			chns[cur].a = a;
			chns[cur].created = 1;
			break;
		}
		if (!chns[cur].created) {
			pr_err("chn %d has not been created\n", cur);
			ret = -EINVAL;
			break;
		}
		if (cmd == VM_DESTROY)
			chn_clear(cur);
		else if (cmd == VM_FREE)
			ret = part_free(a.virt);
		else {
			ret = part_alloc(&a);
			if (copy_to_user(u, &a, sizeof(a)))
				ret = -EFAULT;
		}
		break;
	default:
		pr_err("venc_mem: ioctl cmd error\n");
	}
	mutex_unlock(&vm_lock);
	return ret;
}

static int venc_mem_show(struct seq_file *s, void *v)
{
	struct part *p;
	int c;

	seq_puts(s, "[VENC_MEM] openxiongmai xm533_venc_mem\n\n");
	seq_puts(s, "CHN--START-----------SIZE----------------OVERFLOW----NAME--------------\n");
	mutex_lock(&vm_lock);
	for (c = 0; c < N_CHN; c++) {
		u32 used = 0;

		if (!chns[c].created)
			continue;
		list_for_each_entry (p, &chns[c].parts, node) {
			void __iomem *g = ioremap(p->phys + p->size, 16);
			char k[16];
			int over = 0;

			if (g) {
				memcpy_fromio(k, g, 16);
				over = memcmp(k, GUARD, 16) != 0;
				iounmap(g);
			}
			seq_printf(s, "%-3d  %#-10x\t    %#-10x(%5dk)\t%d\t%s\n", c, p->phys,
				   p->size, p->len >> 10, over, p->name);
			if (p->used)
				used += p->len;
		}
		seq_puts(s, "---------------------------------------------------------------------\n");
		seq_printf(s, "SIZE: %#x(%dk)  ", chns[c].a.size, chns[c].a.size >> 10);
		seq_printf(s, "USED: %#x(%dk)  ", used, used >> 10);
		seq_printf(s, "FREE: %#x(%dk)\n\n", chns[c].a.size - used,
			   (chns[c].a.size - used) >> 10);
	}
	mutex_unlock(&vm_lock);
	return 0;
}

static int venc_mem_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, venc_mem_show, NULL);
}

static const struct file_operations venc_mem_proc_fops = {
	.owner = THIS_MODULE,
	.open = venc_mem_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations venc_mem_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = venc_mem_ioctl,
};

static struct miscdevice venc_mem_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "venc_mem",
	.fops = &venc_mem_fops,
};

static int __init xm533_venc_mem_init(void)
{
	int c, ret;

	for (c = 0; c < N_CHN; c++)
		INIT_LIST_HEAD(&chns[c].parts);
	proc_create("umap/venc_mem", 0444, NULL, &venc_mem_proc_fops);
	ret = misc_register(&venc_mem_misc);
	if (ret)
		remove_proc_entry("umap/venc_mem", NULL);
	return ret;
}

static void __exit xm533_venc_mem_exit(void)
{
	int c;

	misc_deregister(&venc_mem_misc);
	remove_proc_entry("umap/venc_mem", NULL);
	for (c = 0; c < N_CHN; c++)
		if (chns[c].created)
			chn_clear(c);
}

module_init(xm533_venc_mem_init);
module_exit(xm533_venc_mem_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM533 encoder memory book-keeping, compatible with venc_mem.ko");
