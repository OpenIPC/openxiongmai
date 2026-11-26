// SPDX-License-Identifier: GPL-2.0
/*
 * The XM232's DDR bandwidth monitor (/proc/umap/ddr): a drop-in for its
 * vendor ddr.ko. A kernel timer every 100 jiffies stops the counters in
 * the DDR PHY glue (0x2005C000: 0 stops, 1 runs), takes each of the three
 * ports' write and read byte counts (+0x10 + 8 * port, write first) and
 * starts them again. /proc/umap/ddr shows them in MB/s, and as "Total" the
 * DDR's peak from its PLL, as ddr.ko does.
 */

#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/timer.h>

#define GLUE_BASE 0x2005c000
#define SYS_BASE 0x20000000
#define PORTS 3
#define PERIOD 100 /* jiffies */

static void __iomem *glue;
static struct timer_list ddr_timer;
static u64 bytes[PORTS][2]; /* write, read in the last period */
static u32 peak; /* MB/s */

static void ddr_count(unsigned long data)
{
	int i;

	ddr_timer.expires = jiffies + PERIOD;
	add_timer(&ddr_timer);
	writel(0, glue);
	for (i = 0; i < PORTS; i++) {
		bytes[i][0] = readl(glue + 0x10 + 8 * i);
		bytes[i][1] = readl(glue + 0x14 + 8 * i);
	}
	writel(1, glue);
}

static int ddr_show(struct seq_file *s, void *v)
{
	int i;

	seq_puts(s, "[DDR] openxiongmai xm_v200_ddr (xm232)\n\n");
	seq_puts(s, "Port  WMB/s  RMB/s\n");
	for (i = 0; i < PORTS; i++)
		seq_printf(s, "%-4d  %-5llu  %-5llu\n", i, bytes[i][0] >> 20,
			   bytes[i][1] >> 20);
	/* no newline after it, as XM232's ddr.ko */
	seq_printf(s, "Total %u", peak);
	return 0;
}

static int ddr_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ddr_show, NULL);
}

static const struct file_operations ddr_proc_fops = {
	.owner = THIS_MODULE,
	.open = ddr_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init xm232_ddr_init(void)
{
	void __iomem *sys;
	u32 a, b, c;

	glue = ioremap(GLUE_BASE, 0x40);
	sys = ioremap(SYS_BASE, 0x60);
	if (!glue || !sys) {
		if (glue)
			iounmap(glue);
		if (sys)
			iounmap(sys);
		return -ENOMEM;
	}
	if (!proc_create("umap/ddr", 0444, NULL, &ddr_proc_fops))
		pr_err("ddr: create proc failed\n");
	setup_timer(&ddr_timer, ddr_count, 0);
	ddr_timer.expires = jiffies + PERIOD;
	add_timer(&ddr_timer);
	/* the DDR clock from PLL C, two or four bytes a clock, both edges */
	a = readl(sys + 0x18);
	b = readl(sys + 0x1c);
	c = readl(sys + 0x58);
	peak = (((b >> 16) & 0x3ff) * 24 + 0x180) / (((a >> 9) & 0x3f) + 1);
	peak = (peak / ((a & 0xf) ? 4 : 2)) * 8 / ((c & 0xff) + 1);
	iounmap(sys);
	return 0;
}

static void __exit xm232_ddr_exit(void)
{
	remove_proc_entry("umap/ddr", NULL);
	del_timer_sync(&ddr_timer);
	iounmap(glue);
}

module_init(xm232_ddr_init);
module_exit(xm232_ddr_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM232 DDR bandwidth monitor, compatible with its ddr.ko");
