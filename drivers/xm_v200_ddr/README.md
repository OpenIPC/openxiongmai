# xm_v200_ddr

`xm_v200_ddr` is the V200 series' DDR bandwidth monitor, at `/proc/umap/ddr`, and builds as `open_ddr.ko`. It replaces the vendor `ddr.ko`.

Timer 7 (0x100D2020, IRQ 86) fires every second, and the DDR controller's byte counters (0x2015C010: read and write for 4 ports) are taken and restarted. `/proc/umap/ddr` shows each port's MB/s, and the DDR's peak computed from its PLL. `DDR_CHIP=v200` stops the counters while they are read, as the XM530V200's `ddr.ko` does.

If its interrupt cannot be had, the timer it started is stopped again before the module fails to load.
