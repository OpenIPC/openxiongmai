# xm_v200_rtc

`xm_v200_rtc` is the V200 series' real-time clock, at `/dev/rtc` (misc minor 135), and builds as `open_rtc.ko`. It replaces the vendor `rtc.ko`.

The clock counts 1/32768 s in 64 bits at 0x100A0080: command 2 latches the count into +0xC/+0x10 (status +0x14 bit 1), command 1 loads it from +4/+8 (status bit 0). The ioctls are Linux's `RTC_RD_TIME` and `RTC_SET_TIME`, so `hwclock -r`/`hwclock -w` work; loading the module sets the system clock from it, as `rtc.ko` does.

Only the XM530V200 boards load `rtc.ko` in their vendor firmware; the XM533 and XM535 ones do not.
