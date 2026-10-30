# xm530v200_isp

`xm530v200_isp` is the XM530V200's ISP driver, at `/dev/isp`, and builds as `open_isp.ko`. It replaces the vendor `isp.ko`, and exports what `vpss.ko` (`open_vpss.ko`) calls: `vpss_isp_init`, `vpss_isp_set_attr`, `vpss_isp_set_val` and `vpss_isp_post_deal`.

User space (the sensor library, libxmv200 here) sets the ISP up and hands it parameter blocks. The driver keeps them in one parameter block at `isp.ko`'s offsets (`isp.h`). It handles up to three channels on two VICUs (VICU 1 can be time-shared between sensors). The BPU works one of two ways:

- **Online:** the VICU feeds the BPU. The VICU's frame interrupt (IRQ 87) wakes the BPU thread, which writes the channel's blocks to the registers.
- **Offline:** the VICU stores raw frames in ISP buffers (IRQ 88). The BPU reads them back, one channel at a time, and its own interrupt (IRQ 63) starts the next channel. Its output can go into VB buffers that `vpss.ko` takes. In memory-saver mode a snapshot of each channel's blocks is taken when its frame is stored.

The VPSS side's blocks go to the registers when `vpss.ko` calls `vpss_isp_set_val` once a frame. The ioctls are listed at the top of `isp.c`. `read()` returns the AWB statistics and `write()` picks their channel. `mmap()` maps the pages user space shares.

| File | |
|------|-|
| `isp.c` | the device, register access (four windows from 0x30000000, and the PLL block at 0x20000000), the main ioctls, `/proc/umap/isp` |
| `bpu.c` | the BPU's defaults, AWB/histogram/AF statistics, the register writes, offline scheduling, the BPU interrupt and thread |
| `iw.c` | the BPU's parameter blocks (0x4964..0x497f) |
| `vicu.c` | the two VICU interrupts (frame, store), the time-shared VICU, the frame rate, the raw-frame dump |
| `vpss_isp.c` | the VPSS side's blocks (0x49d2..0x49f0) and its register image: noise reduction, sharpening, gamma, the LDC table, the limit-up scaler and its repair |
| `ib.c` | the ISP buffer pools in MMZ |

In qemu-bench (its xm-isp2 model), the XM530V200 board x6c-weq streams with it in offline memory-saver mode. On the VICU, BPU and VPSS register blocks it writes the same registers with the same values as `isp.ko`, and in the same order frame by frame. The VPSS buffer addresses differ, and so do how many frames run and when the AE values change, both of which depend on timing.

Differences from `isp.ko`:

- Channel and group numbers are checked (`isp.ko` writes past its blocks for some commands).
- 0x49d2 unmaps the LDC tables it mapped (`isp.ko` clears their pointers first and leaks the mappings).
- The online BPU thread clears requests for channels other than 0, which `isp.ko` would spin on.
