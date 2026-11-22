# xm533_isp

`xm533_isp` is the XM533's ISP driver, at `/dev/isp`, and builds as `open_isp.ko`. It replaces the vendor `isp.ko`, and exports what `vi.ko` (`open_vi.ko`) and other modules call: `vpss_isp_init`, `vpss_isp_set_attr`, `vpss_isp_set_val`, `vpss_isp_post_deal`, `ib_get_buf` and `ib_put_buf`.

User space, the sensor library (libxmv200 here), sets the ISP up and hands it parameter blocks. The driver keeps them in one parameter block, at `isp.ko`'s offsets (`isp.h`). Each frame, the VICU's interrupt (IRQ 57) takes the AWB statistics and wakes the BPU thread, which writes the blocks to the ISP's registers: `bpu.c` for the Bayer side, `vpss_isp.c` for the VPSS side. The ioctls are listed at the top of `isp.c`. Besides them, `read()` gives the AWB statistics, `write()` picks their channel, and `mmap()` maps 8 pages that user space shares.

| File | |
|------|-|
| `isp.c` | the device, register access (four windows from 0x30000000, the PLL block at 0x20000000), the main ioctls, `/proc/umap/isp` |
| `bpu.c` | the BPU's defaults, the AWB statistics, the frame thread and its register writes |
| `iw.c` | the BPU's parameter blocks (0x4964..0x497f) |
| `vicu.c` | the frame interrupt, the frame rate, the raw-frame dump (0x495a) |
| `vpss_isp.c` | the VPSS side's blocks (0x49d2..0x49f0), its register image, the limit-up scaler and its repair |
| `ib.c` | the ISP buffer pool in MMZ |

In qemu-bench (its xm-isp2 model) the XM533 streams with it, and its register writes match `isp.ko`'s in value and order frame by frame; only the AE/AWB results, which depend on timing, differ.

Unlike `isp.ko`, command 0x4977 releases its lock (`isp.ko` returns holding it, and every parameter ioctl after that hangs). Channel numbers are checked: `isp.ko` writes past its blocks for channels it does not have. `mmap()` maps no more than its 8 pages.
