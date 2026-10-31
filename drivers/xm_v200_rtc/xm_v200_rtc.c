// SPDX-License-Identifier: GPL-2.0
/*
 * The V200 series' real-time clock (/dev/rtc): a drop-in for the vendor
 * rtc.ko.
 *
 * The clock is a 64-bit count of 1/32768 s at 0x100A0080: writing 2 to
 * the command register latches the count into +0xC (low) and +0x10
 * (high), and status (+0x14) bit 1 says it is there; writing 1 loads the
 * count from +4 (low) and +8 (high), status bit 0 saying it is done. The
 * kernel clock is set from it when the module loads.
 *
 * The ioctls are Linux's RTC_RD_TIME and RTC_SET_TIME on the misc RTC
 * minor, which busybox hwclock uses.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/time.h>
#include <linux/uaccess.h>

#define RTC_BASE	0x100a0080
#define RTC_CMD		0x00
#define RTC_SET_LO	0x04
#define RTC_SET_HI	0x08
#define RTC_GET_LO	0x0c
#define RTC_GET_HI	0x10
#define RTC_STATUS	0x14
#define RTC_CMD_SET	1
#define RTC_CMD_GET	2

/* next to the ADC: the clock's power and its enable, as rtc.ko sets them */
#define RTC_ANA_BASE	0x10020cec

static void __iomem *regs;
static DEFINE_MUTEX(rtc_lock);

static int rtc_wait(u32 bit)
{
	int n;

	for (n = 0; n < 10000; n++) {
		if (readl(regs + RTC_STATUS) & bit)
			return 0;
		udelay(10);
	}
	return -ETIMEDOUT;
}

static int rtc_get(unsigned long *secs)
{
	u32 lo, hi;
	int ret;

	mutex_lock(&rtc_lock);
	writel(RTC_CMD_GET, regs + RTC_CMD);
	ret = rtc_wait(BIT(1));
	hi = readl(regs + RTC_GET_HI);
	lo = readl(regs + RTC_GET_LO);
	mutex_unlock(&rtc_lock);
	*secs = hi << 17 | lo >> 15;
	return ret;
}

static int rtc_set(unsigned long secs)
{
	int ret;

	mutex_lock(&rtc_lock);
	writel(secs >> 17, regs + RTC_SET_HI);
	writel(secs << 15, regs + RTC_SET_LO);
	writel(RTC_CMD_SET, regs + RTC_CMD);
	ret = rtc_wait(BIT(0));
	mutex_unlock(&rtc_lock);
	return ret;
}

static long rtc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rtc_time tm;
	unsigned long secs;
	int ret;

	switch (cmd) {
	case RTC_RD_TIME:
		ret = rtc_get(&secs);
		if (ret)
			return ret;
		memset(&tm, 0, sizeof(tm));
		rtc_time_to_tm(secs, &tm);
		return copy_to_user((void __user *)arg, &tm, sizeof(tm)) ?
			-EFAULT : 0;
	case RTC_SET_TIME:
		if (!capable(CAP_SYS_TIME))
			return -EACCES;
		if (copy_from_user(&tm, (void __user *)arg, sizeof(tm)))
			return -EFAULT;
		if (rtc_valid_tm(&tm))
			return -EINVAL;
		rtc_tm_to_time(&tm, &secs);
		return rtc_set(secs);
	}
	return -ENOTTY;
}

static const struct file_operations rtc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = rtc_ioctl,
};

static struct miscdevice rtc_misc = {
	.minor = RTC_MINOR,
	.name = "rtc",
	.fops = &rtc_fops,
};

static int __init xm_v200_rtc_init(void)
{
	struct timespec ts = { 0 };
	void __iomem *ana;
	unsigned long secs;
	int ret;

	regs = ioremap(RTC_BASE, 0x18);
	if (!regs)
		return -ENOMEM;
	ana = ioremap(RTC_ANA_BASE, 8);
	if (ana) {
		writel(3, ana + 4);
		writel(1, ana);
		iounmap(ana);
	}
	ret = misc_register(&rtc_misc);
	if (ret) {
		pr_err("xm_rtc init error!\n");
		iounmap(regs);
		return ret;
	}
	if (!rtc_get(&secs)) {
		ts.tv_sec = secs;
		do_settimeofday(&ts);
	} else {
		pr_warn("xm_rtc: the clock does not answer\n");
	}
	return 0;
}

static void __exit xm_v200_rtc_exit(void)
{
	misc_deregister(&rtc_misc);
	iounmap(regs);
}

module_init(xm_v200_rtc_init);
module_exit(xm_v200_rtc_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V200-series real-time clock, compatible with rtc.ko");
