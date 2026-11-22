# xm_v200_ddr

`xm_v200_ddr` is the V200 series' DDR bandwidth monitor, at `/proc/umap/ddr`, and builds as `open_ddr.ko`. It replaces the vendor `ddr.ko`.

Timer 7 (0x100D2020, IRQ 86) fires every second, and the DDR controller's byte counters (0x2015C010: read and write for 4 ports) are taken and restarted. `/proc/umap/ddr` shows each port's MB/s, and the DDR's peak computed from its PLL. `DDR_CHIP=v200` stops the counters while they are read, as the XM530V200's `ddr.ko` does.

If its interrupt cannot be had, the timer it started is stopped again before the module fails to load.

## XM232 (`DDR_CHIP=xm232`, `xm232_ddr.c`)

The XM232's `ddr.ko` has no timer interrupt: a kernel timer every 100 jiffies stops the counters in the DDR PHY glue (`0x2005c000`: 0 stops, 1 runs), takes three ports' write and read byte counts (`+0x10 + 8 * port`, write first) and starts them again. `/proc/umap/ddr` shows them in MB/s (bytes >> 20), and as `Total` the DDR's peak, from PLL C as on the other chips.
