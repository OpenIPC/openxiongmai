# dwc2

`dwc2` is the V200 series' USB 2.0 host driver, and builds as `open_dwc2.ko`. It replaces the vendor `dwc2.ko` on the XM530V200 and the XM535D1/D2/W1. Like that module, it binds to the `dwc2` platform device that `xm_usb0.ko` (`open_xm_usb0.ko`, from `drivers/xm_v200_usb0`) registers: the Synopsys DesignWare HS OTG core at 0x50300000, IRQ 76.

The vendor `dwc2.ko` is mainline Linux's DWC2 driver (`drivers/usb/dwc2`, core and platform glue in one module), built host-only and backported to the 3.10 kernel. This directory is the same thing from Linux 4.19.125's files, unchanged except for:

- **`compat.h`:** what 4.19's driver uses and 3.10 lacks: the timer API, `GENMASK`, the high-bandwidth bits of `wMaxPacketSize`, `enum usb_dr_mode`, and empty device-tree, generic-PHY, reset-controller and regulator lookups. `include/` has empty `linux/phy/phy.h` and `linux/usb/of.h`. Without a device tree, `params.c`'s defaults hold, read from the core's hardware configuration.
- **TT bookkeeping:** 3.10's `struct usb_tt` has no `hcpriv`, so `hcd.c` keeps its `dwc2_tt`s in its own list.
- **URB giveback:** 3.10 has no `HCD_BH`, and 4.19's `dwc2_host_complete()` gives URBs back with `hsotg->lock` held. As in the vendor module, completed URBs go to a list and a tasklet gives them back without the lock.
- **The vendor's `dma_desc` parameter:** `dma_desc=1` turns on descriptor DMA in host mode (`dma_desc_enable`). It is off by default, as in `dwc2.ko`.

Build with `make KDIR=...`. `-DCONFIG_USB_DWC2_HOST` is set in the Makefile.

In qemu-bench (its `dwc2-usb` model), the XM530V200 and XM535W1 images give the same results with `open_dwc2.ko` and `open_xm_usb0.ko` as with the vendor modules. The QEMU hub and a `usb-storage` disk behind it enumerate, and a usbfs Bulk-Only test passes: INQUIRY, READ(10), a WRITE(10)/READ(10) round trip and a 64 KB read. The module unloads cleanly.

Differences from `dwc2.ko`:

- The defaults and parameter checks are 4.19.125's. The vendor module's parameter block is laid out differently, and some of its host defaults differ. At probe this driver logs that the core has no LPM (`Invalid parameter lpm=1` and the like) and turns it off.
- `kmem_cache_destroy()` is not called on caches that were never created (3.10's oopses on NULL), so unloading is safe.
