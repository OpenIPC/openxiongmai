// SPDX-License-Identifier: GPL-2.0
/*
 * xm_v200_sys: the V200 series' system module, a drop-in replacement for the
 * vendor's sys.ko (built as open_sys.ko).
 *
 * /dev/sys holds the bind table the media pipeline describes itself with:
 * 64 entries of {src mod, dev, chn, dst mod, dev, chn}, the whole 0x600
 * bytes each call (0x400c5300), shown in /proc/umap/sys. The modules bind
 * through their own calls; the table is for the record.
 *
 * On the XM535 it also reaches the clock block (0x20000000, its first 0x80
 * bytes) for user space and isp.ko: 0x40085301 {register, value} writes one
 * and locks the block again (0 to +0), 0x40085302 reads one;
 * sys_set_clk()/sys_get_clk() are the same for modules.
 */

#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#define SYS_BIND 0x400c5300
#define SYS_SET_CLK 0x40085301
#define SYS_GET_CLK 0x40085302
#define CLK_BASE 0x20000000
#define CLK_SIZE 0x80

#define N_BINDS 64
struct bind {
	u32 src_mod, src_dev, src_chn, dst_mod, dst_dev, dst_chn;
};

static struct bind binds[N_BINDS];
static void __iomem *clk;
static DEFINE_SPINLOCK(clk_lock);

static const char *const mod_names[32] = {
	[0] = "null", [5] = "vdec", [7] = "venc", [8] = "vpss",
	[15] = "vo",  [16] = "vi",  [17] = "var", [21] = "ai",
	[22] = "ao",  [23] = "aenc", [24] = "adec",
};

int sys_set_clk(u32 reg, u32 val)
{
	unsigned long flags;

	if (reg >= CLK_SIZE) {
		pr_err("sys: set_clk error addr %x\n", reg);
		return -1;
	}
	spin_lock_irqsave(&clk_lock, flags);
	writel(val, clk + reg);
	writel(0, clk); /* locked again */
	spin_unlock_irqrestore(&clk_lock, flags);
	return 0;
}
EXPORT_SYMBOL(sys_set_clk);

int sys_get_clk(u32 reg, u32 *val)
{
	if (reg >= CLK_SIZE) {
		pr_err("sys: set_clk error addr %x\n", reg);
		return -1;
	}
	*val = readl(clk + reg);
	return 0;
}
EXPORT_SYMBOL(sys_get_clk);

static long sys_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	u32 a[2];
	int ret;

	switch (cmd) {
	case SYS_BIND:
		if (copy_from_user(binds, uarg, sizeof(binds)))
			break;
		return 0;
	case SYS_SET_CLK:
		if (copy_from_user(a, uarg, sizeof(a)))
			break;
		return sys_set_clk(a[0], a[1]);
	case SYS_GET_CLK:
		if (copy_from_user(a, uarg, sizeof(a)))
			break;
		ret = sys_get_clk(a[0], &a[1]);
		if (copy_to_user(uarg, a, sizeof(a))) {
			pr_err("sys_ioctl: copy_to_user error.\n");
			return -1;
		}
		return ret;
	default:
		pr_err("sys: ioctl cmd error %x\n", cmd);
		return 0;
	}
	pr_err("sys_ioctl: copy_from_user error.\n");
	return -1;
}

static const char *mod_name(u32 m)
{
	return m < ARRAY_SIZE(mod_names) && mod_names[m] ? mod_names[m] : "?";
}

static int sys_show(struct seq_file *s, void *v)
{
	int i;

	seq_puts(s, "[SYS] openxiongmai xm_v200_sys\n\n");
	seq_puts(s, "SrcMod  SrcDev  SrcChn  DstMod  DstDev  DstChn\n");
	for (i = 0; i < N_BINDS; i++)
		if (binds[i].src_mod)
			seq_printf(s, "%-6s  %-6d  %-6d  %-6s  %-6d  %-6d\n",
				   mod_name(binds[i].src_mod), binds[i].src_dev,
				   binds[i].src_chn, mod_name(binds[i].dst_mod),
				   binds[i].dst_dev, binds[i].dst_chn);
	return 0;
}

static int sys_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sys_show, NULL);
}

static const struct file_operations sys_proc_fops = {
	.owner = THIS_MODULE,
	.open = sys_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations sys_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = sys_ioctl,
};

static struct miscdevice sys_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sys",
	.fops = &sys_fops,
};

static int __init xm_v200_sys_init(void)
{
	int ret;

	clk = ioremap(CLK_BASE, CLK_SIZE);
	if (!clk)
		return -ENOMEM;
	/* /proc/umap is made by whichever media module comes first */
	if (!proc_create("umap/sys", 0444, NULL, &sys_proc_fops))
		pr_err("sys: create sys proc fail!\n");
	ret = misc_register(&sys_misc);
	if (ret) {
		pr_err("sys: register device failed!\n");
		remove_proc_entry("umap/sys", NULL);
		iounmap(clk);
		return ret;
	}
	pr_info("sys: openxiongmai xm_v200_sys init ok\n");
	return 0;
}

static void __exit xm_v200_sys_exit(void)
{
	misc_deregister(&sys_misc);
	remove_proc_entry("umap/sys", NULL);
	iounmap(clk);
}

module_init(xm_v200_sys_init);
module_exit(xm_v200_sys_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V200-series system module, compatible with sys.ko");
