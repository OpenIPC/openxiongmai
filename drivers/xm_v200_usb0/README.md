# xm_v200_usb0

`xm_v200_usb0` replaces the vendor `xm_usb0.ko` of the XM530V200, XM535D1/D2/W1 and XM232, and builds as `open_xm_usb0.ko`. It takes the USB PHY out of reset and registers the `dwc2` platform device (id 0, "usb_addr" 0x50300000-0x503fffff, "usb_irq" 76) for `dwc2.ko` (`open_dwc2.ko`, from `drivers/dwc2`).

The PHY is set up in the system controller at 0x20000000, between unlocking and relocking it (+0x00):

| | XM530V200 | XM535D1/D2/W1 (`USB0_CHIP=xm535`) |
|-|-----------|------------------|
| +0x6c | 0x8000, then 10 ms | 0x8000, then 100 us |
| +0x114 | 0x11, then 10 ms | 9, then 1 ms |
| 0x100e0000 | 0, then 0xf at +4 | not written |

The XM232 (`USB0_CHIP=xm232`) has its controller at 0x50200000, IRQ 55, and sets its PHY up its own way: +0x70 = 0x800, then +0x180 = 7, 3, 1 and 9, 10 ms apart, then the lock; then 0 and 0xf (at +4) to the block at 0x10018000, as the XM530V200 does at 0x100e0000, and 10 ms more.

The sequences and the device are `xm_usb0.ko`'s, from its disassembly. Unloading unregisters the device.
