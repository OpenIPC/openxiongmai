# xm232_isp

`xm232_isp` is the XM232's ISP driver, at `/dev/isp`, and builds as `open_isp.ko`. It replaces the XM232's vendor `isp.ko` ("XM210V200_ISP"): the same module parameter (`mode`), ioctls and record sizes, `read()`/`write()` (the AWB statistics and their channel), `mmap()` of 16 shared pages, `/proc/umap/isp`, and exports (`isp_register_ops`, `isp_set_next_ch`, `isp_set_complete`, `ib_get_buf`, `ib_put_buf`, `vpss_isp_init`, `vpss_isp_set_attr`, `vpss_isp_set_val`, `vpss_isp_post_deal`). It needs `sys.ko` (`sys_get_clk`/`sys_set_clk`) and `mmz.ko` (`mmz_alloc`), and `vi_mu.ko` (`drivers/xm232_vi_mu`) loads after it.

The XM232 has one VICU whose input is switched between up to three sensors. The mux timer (IRQ 90) marks each sensor's frame start. In the mode the X6S-WEQ-H runs, BPU mode 1, `vi_mu.ko` picks the next sensor every frame through `isp_set_next_ch()`, which:

- takes the finished channel's AWB statistics (512 zones of R, G, B sums, each divided by a quarter of the zone's pixels);
- writes the next channel's BPU blocks;
- writes its VPSS side's blocks.

The parameter blocks come from the sensor library through the ioctls (`iw.c`, `vpss_isp.c`) and are kept at `isp.ko`'s offsets (`isp.h`). Online (mode 0) and offline (mode 3, raw frames through ISP buffers in MMZ, `ib.c`) BPU modes are there as `isp.ko` has them.

| File | |
|------|-|
| `isp.c` | the device, the main ioctls, the mux timer, `isp_set_next_ch`, the sensor power requests (0x4915, 0x4982/83) and `isp_suspend`, `/proc/umap/isp` |
| `bpu.c` | the BPU's defaults and per-frame register writes, the AWB statistics, the offline queue and its tasklet, the BPU interrupt and thread |
| `iw.c` | the BPU parameter blocks (0x4964..0x4980) |
| `vicu.c` | the VICU's channel set-up, its frame interrupt and rates, the raw-frame dump, the offline store interrupts |
| `vpss_isp.c` | the VPSS side's blocks (0x49d2..0x49ec) and their register image each frame |
| `ib.c` | the ISP buffer pools |
| `suspend_regs.h` | the registers `isp.ko` saves and restores around a sensor power-down, in its order |

In qemu-bench's `xm232` with the vendor X6S-WEQ-H firmware (Sofia, two SC2336 sensors), the driver behaves as `isp.ko` does:

- **Register writes:** each frame's program is the vendor's, 818–819 writes from the BPU block to the sensor select. It is identical except for the statistics buffers' addresses and the values the ISP daemon computes from the statistics. The start-up writes match too, but for where `vi_mu.ko`'s latch writes fall.
- **Streams:** RTSP main and sub give 120 H.264 frames each in 10 s with 0 decode errors, and the JPEG snapshot decodes.
- **System:** the ISP daemon keeps running, and the AWB buffer pair alternates. The person detector's DMA keeps copying, with no reboot past 120 s.

Differences from `isp.ko`:

- **Bounds:** channel numbers are checked where `isp.ko` doesn't (0x4961, 0x49db, `read()`'s channel), and `mmap()` maps at most the 16 pages.
- **0x49e3:** it gives back the VPSS lock; `isp.ko` returns holding it.
- **BPU interrupt:** with only run bits above channel 2 set, it stops where `isp.ko` would index past its tables.

Not tested in qemu-bench, which the X6S-WEQ-H's Sofia doesn't exercise:

- the online and offline BPU modes (0 and 3);
- the offline store interrupts (`mode=1`);
- the raw-frame dump (0x495a);
- the sensor power-down save and restore (0x495e).

They follow `isp.ko`'s code.
