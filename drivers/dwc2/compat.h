/* SPDX-License-Identifier: GPL-2.0 */
/*
 * What Linux 4.19's DWC2 host driver uses and 3.10 lacks. There is no
 * device tree, generic PHY, reset controller or regulator here: xm_usb0.ko
 * sets the PHY up and registers the "dwc2" platform device, so each of
 * these is absent and the driver takes its defaults.
 */
#ifndef DWC2_COMPAT_H
#define DWC2_COMPAT_H

#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/usb/ch9.h>

#ifndef GENMASK
#define GENMASK(h, l) \
	(((~0UL) << (l)) & (~0UL >> (BITS_PER_LONG - 1 - (h))))
#endif

/* 4.15's timer API, on 3.10's: the callback gets the timer itself */
#define timer_setup(timer, fn, flags) \
	setup_timer(timer, (void (*)(unsigned long))(fn), (unsigned long)(timer))
#define from_timer(var, t, field) container_of(t, typeof(*var), field)

/* the packet size without 4.19's high-bandwidth bits */
#define usb_endpoint_maxp(epd) (le16_to_cpu((epd)->wMaxPacketSize) & 0x7ff)
#define usb_endpoint_maxp_mult(epd) \
	(1 + ((le16_to_cpu((epd)->wMaxPacketSize) >> 11) & 3))

/* 3.12's HCD flag for giving URBs back in a tasklet */
#define HCD_BH 0

enum usb_dr_mode {
	USB_DR_MODE_UNKNOWN,
	USB_DR_MODE_HOST,
	USB_DR_MODE_PERIPHERAL,
	USB_DR_MODE_OTG,
};
#define usb_get_dr_mode(dev) USB_DR_MODE_UNKNOWN

#define dwc2_hsotg_plat s3c_hsotg_plat

static inline int dwc2_compat_einval(void) { return -EINVAL; }
static inline int dwc2_compat_zero(void) { return 0; }

#define device_property_read_u32(dev, name, val) dwc2_compat_einval()
#define device_property_read_u32_array(dev, name, val, n) dwc2_compat_einval()

#define devm_reset_control_get_optional(dev, id) NULL
#define reset_control_assert(r) dwc2_compat_zero()
#define reset_control_deassert(r) dwc2_compat_zero()

#define phy_get_bus_width(phy) 0

/* 3.10's oopses on NULL */
#define kmem_cache_destroy(c) \
	do { if (c) kmem_cache_destroy(c); } while (0)

#define devm_regulator_get_optional(dev, id) ERR_PTR(-ENODEV)

#endif
