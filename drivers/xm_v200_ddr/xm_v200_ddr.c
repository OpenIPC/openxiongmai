// SPDX-License-Identifier: GPL-2.0
/*
 * The V200 series' DDR bandwidth monitor (/proc/umap/ddr): a drop-in for
 * the vendor ddr.ko. Timer 7 (0x100D2020, IRQ 86) fires every second
 * (12 MHz); then the DDR controller's byte counters (0x2015C010: read,
 * write for each of 4 ports) are taken and restarted (+0x0).
 * /proc/umap/ddr shows each port's MB/s and the DDR's peak, from its PLL.
 */

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define TIMER_BASE 0x100d2020 /* the block's second timer */
#define T_LOAD 0x00
#define T_CTRL 0x08
#define T_INTCLR 0x0c
#define T_MIS 0x14
#define T_BGLOAD 0x18
#define DDRC_BASE 0x2015c000
#define SYS_BASE 0x20000000
#define TIMER_IRQ 86
#define PORTS 4

static void __iomem *timer, *ddrc;
static u64 bytes[PORTS][2]; /* read, write in the last second */
static u32 peak;	    /* MB/s */

static irqreturn_t timer7_interrupt(int irq, void *dev)
{
	int i;

	if (!readl(timer + T_MIS))
		return IRQ_NONE;
	writel(1, timer + T_INTCLR);
#ifdef DDR_V200
	writel(0, ddrc);
#endif
	for (i = 0; i < PORTS; i++) {
		bytes[i][0] = readl(ddrc + 0x10 + 8 * i);
		bytes[i][1] = readl(ddrc + 0x14 + 8 * i);
	}
	writel(1, ddrc);
	return IRQ_HANDLED;
}

static int ddr_show(struct seq_file *s, void *v)
{
	u64 total = 0;
	int i;

	seq_puts(s, "[DDR] openxiongmai xm_v200_ddr\n\n");
	seq_puts(s, "Port  WMB/s  RMB/s\n");
	for (i = 0; i < PORTS; i++) {
		seq_printf(s, "%-4d  %-5llu  %-5llu\n", i, bytes[i][1] >> 19,
			   bytes[i][0] >> 19);
		total += bytes[i][0] + bytes[i][1];
	}
	seq_printf(s, "Total %llu / %u MB/s\n", total >> 19, peak);
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

static int __init xm_v200_ddr_init(void)
{
	void __iomem *sys;
	u32 a, b, c;
	int ret;

	timer = ioremap(TIMER_BASE, 0x20);
	ddrc = ioremap(DDRC_BASE, 0x40);
	sys = ioremap(SYS_BASE, 0x60);
	if (!timer || !ddrc || !sys) {
		ret = -ENOMEM;
		goto unmap;
	}
	/* the DDR clock from its PLL, two or four bytes a clock, both edges */
	a = readl(sys + 0x18);
	b = readl(sys + 0x1c);
	c = readl(sys + 0x58);
	peak = (((b & 0x3ffffff) >> 16) * 24 + 0x180) / (((a & 0x7fff) >> 9) + 1);
	peak = (peak / ((a & 0xf) ? 4 : 2)) * 8 / ((c & 0xff) + 1);
	iounmap(sys);
	sys = NULL;
	if (!proc_create("umap/ddr", 0444, NULL, &ddr_proc_fops))
		pr_err("ddr: create proc failed\n");
	writel(0x62, timer + T_CTRL);
	writel(12000000, timer + T_LOAD);
	writel(12000000, timer + T_BGLOAD);
	writel(0xe2, timer + T_CTRL); /* periodic, interrupt, 32 bits, on */
	ret = request_irq(TIMER_IRQ, timer7_interrupt, 0, "timer7", NULL);
	if (ret) {
		pr_err("request irq %d failed\n", TIMER_IRQ);
		remove_proc_entry("umap/ddr", NULL);
		goto unmap;
	}
	return 0;
unmap:
	if (sys)
		iounmap(sys);
	if (ddrc)
		iounmap(ddrc);
	if (timer)
		iounmap(timer);
	return ret;
}

static void __exit xm_v200_ddr_exit(void)
{
	writel(0, timer + T_CTRL);
	free_irq(TIMER_IRQ, NULL);
	remove_proc_entry("umap/ddr", NULL);
	iounmap(ddrc);
	iounmap(timer);
}

module_init(xm_v200_ddr_init);
module_exit(xm_v200_ddr_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V200-series DDR bandwidth monitor, compatible with ddr.ko");
