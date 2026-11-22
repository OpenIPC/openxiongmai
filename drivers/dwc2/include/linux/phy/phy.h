/* SPDX-License-Identifier: GPL-2.0 */
/* 3.10 has no generic PHY framework; XM's USB PHY is set up by xm_usb0.ko */
#ifndef DWC2_COMPAT_PHY_H
#define DWC2_COMPAT_PHY_H
#include <linux/err.h>
struct phy;
#define devm_phy_get(dev, name) ERR_PTR(-ENODEV)
static inline int phy_init(struct phy *phy) { return 0; }
static inline int phy_exit(struct phy *phy) { return 0; }
static inline int phy_power_on(struct phy *phy) { return 0; }
static inline int phy_power_off(struct phy *phy) { return 0; }
#endif
