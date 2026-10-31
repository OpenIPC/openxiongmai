// SPDX-License-Identifier: GPL-2.0
/*
 * The V200 series' USB 2.0 host: a drop-in for the vendor xm_usb0.ko.
 *
 * It takes the USB PHY out of reset in the system controller (0x20000000:
 * +0x00 the register lock, +0x6c bit 15, +0x114 the PHY's control) and
 * registers the "dwc2" platform device (the controller at 0x50300000,
 * IRQ 76) that dwc2.ko (open_dwc2.ko) binds to. The XM530V200's also
 * writes 0 and then 0xf to the block at 0x100e0000. USB0_CHIP=xm535 at
 * build time for the XM535D1/D2/W1's sequence; the XM530V200's otherwise.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#define SYS_BASE 0x20000000
#define SYS_LOCK 0x000
#define SYS_USB 0x06c
#define SYS_USB_PHY 0x114
#define USB_CTRL_BASE 0x100e0000

#define DWC2_BASE 0x50300000
#define DWC2_IRQ 76

static struct resource xm_usb0_resource[] = {
	{
		.start = DWC2_BASE,
		.end = DWC2_BASE + 0xfffff,
		.name = "usb_addr",
		.flags = IORESOURCE_MEM,
	},
	{
		.start = DWC2_IRQ,
		.end = DWC2_IRQ,
		.name = "usb_irq",
		.flags = IORESOURCE_IRQ,
	},
};

static void xm_usb0_release(struct device *dev)
{
}

static struct platform_device xm_usb0_dev = {
	.name = "dwc2",
	.id = 0,
	.num_resources = ARRAY_SIZE(xm_usb0_resource),
	.resource = xm_usb0_resource,
	.dev = {
		.release = xm_usb0_release,
	},
};

static int __init xm_usb0_load(void)
{
	void __iomem *sys;
#ifndef USB0_XM535
	void __iomem *ctrl;
#endif

	sys = ioremap(SYS_BASE, 0x1000);
	if (!sys)
		return -ENOMEM;
	writel(1, sys + SYS_LOCK);
	writel(0x8000, sys + SYS_USB);
#ifdef USB0_XM535
	udelay(100);
	writel(9, sys + SYS_USB_PHY);
	mdelay(1);
#else
	mdelay(10);
	writel(0x11, sys + SYS_USB_PHY);
	mdelay(10);
#endif
	writel(0, sys + SYS_LOCK);
	iounmap(sys);

#ifndef USB0_XM535
	ctrl = ioremap(USB_CTRL_BASE, 0x1000);
	if (!ctrl)
		return -ENOMEM;
	writel(0, ctrl);
	writel(0xf, ctrl + 4);
	iounmap(ctrl);
#endif

	return platform_device_register(&xm_usb0_dev);
}

static void __exit xm_usb0_unload(void)
{
	platform_device_unregister(&xm_usb0_dev);
}

module_init(xm_usb0_load);
module_exit(xm_usb0_unload);
MODULE_LICENSE("GPL");
