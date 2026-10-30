# xm535w1_isp

`xm535w1_isp` is the XM535W1's ISP driver, at `/dev/isp`, and builds as `open_isp.ko`. It replaces the vendor `isp.ko`, and exports what that module does: `vpss_isp_init`, `vpss_isp_set_attr`, `vpss_isp_set_val` and `vpss_isp_post_deal`, which do nothing on the W1. The W1 has no VPSS, so `vi.ko` (`open_vi.ko`, from `drivers/xm535w1_vi`) scales. When the BPU's output goes straight to `vi.ko`, this driver drives it through `vpss_thread_set_val`, `vpss_thread_middle_deal` and `vpss_thread_post_deal`, so it loads after `vi.ko`.

The W1's `isp.ko` is a mix of the other two V200-series modules:

- **From the XM530V200's (`drivers/xm530v200_isp`):**
  - the ioctls, and the front of its parameter block (4 bytes lower);
  - the VICU frame work, which runs in a tasklet from IRQ 87, with the second VICU time-shared between sensors (0x4912);
  - offline scheduling of the BPU, and its interrupt (IRQ 62) and thread.
- **From the XM535D1/D2's (`drivers/xm535_isp`):**
  - the BPU itself: its defaults, its parameter blocks and how they go to the registers;
  - the VICU blocks (here 0x2f4 bytes a channel);
  - the VPSS side.

The ioctls are listed at the top of `isp.c`. `read()` returns the AWB statistics, `write()` picks their channel, and `mmap()` maps 8 pages that user space shares.

| File | |
|------|-|
| `isp.c` | the device, register access (four windows from 0x30000000, the PLL block), the main ioctls, `/proc/umap/isp` |
| `bpu.c` | the BPU's defaults, the AWB and histogram statistics, the register writes, offline scheduling, the BPU interrupt and thread |
| `iw.c` | the parameter blocks (0x4964..0x49e8), into the BPU groups and the VICU blocks |
| `vicu.c` | the VICU interrupts and the frame tasklet, the time-shared VICU, the frame rate, the raw-frame dump |
| `vpss_isp.c` | the VPSS side's blocks (0x49d2..0x49f0), their copy taken with the BPU's snapshot, and the register image |
| `ib.c` | the ISP buffer pools in MMZ |

In qemu-bench (its xm-isp2 model), the XM535W1 X2W-WNG streams with it, with one sensor and the BPU online. On the VICU, BPU and VPSS register blocks it writes the same registers with the same values as `isp.ko`. The only difference in order is one extra frame at the start of the open run, which depends on timing. The offline paths were written from the decompile and are not exercised in QEMU.

Differences from `isp.ko`:

- Channel numbers are checked.
- Online, the BPU thread clears requests for channels other than 0, which `isp.ko` would spin on.
- On unload all 8 mmap pages are freed (`isp.ko` frees only the first).
