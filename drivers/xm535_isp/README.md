# xm535_isp

`xm535_isp` is the XM535D1/D2's ISP driver, at `/dev/isp`, and builds as `open_isp.ko`. The two chips ship the same vendor `isp.ko`, and this replaces it. Like `isp.ko`, it exports `vpss_isp_init`, `vpss_isp_set_attr`, `vpss_isp_set_val` and `vpss_isp_post_deal`, which do nothing on the XM535. It drives `vpss.ko` (`open_vpss.ko`, from `drivers/xm535_vpss`) through `vpss_thread_set_val`, `vpss_thread_middle_deal` and `vpss_thread_post_deal`, so it loads after it.

User space (the sensor library, libxmv200 here) sets the ISP up and hands it parameter blocks. The driver keeps them in one parameter block at `isp.ko`'s offsets (`isp.h`). It handles up to three channels on two VICUs. With three sensors, VICU 0's receiver alternates between two of them, and timer 4/5 (IRQ 38) runs the frame work once the switched sensor's frame is in.

The BPU works one of two ways:

- **Online:** the VICU feeds the BPU, and the BPU thread writes channel 0's blocks when the VICU's frame interrupt (IRQ 87) wakes it.
- **Offline:** the VICU stores raw frames in ISP buffers, and its store interrupt (IRQ 88) queues each stored channel. A tasklet then starts the BPU on the next channel in the queue, from a snapshot of that channel's blocks. When the BPU finishes (IRQ 62), the next channel starts. The BPU writes its output straight into `vpss.ko`'s buffers, and the VPSS side's blocks go to the registers with each BPU frame.

The ioctls are listed at the top of `isp.c`. `read()` returns the AWB statistics, `write()` picks their channel, and `mmap()` maps 16 pages that user space shares.

| File | |
|------|-|
| `isp.c` | the device, register access (four windows from 0x30000000, the PLL block, timer 4/5), the main ioctls, `/proc/umap/isp` |
| `bpu.c` | the BPU's defaults and per-frame reset, the AWB and histogram statistics, the register writes, the offline queue and its tasklet, the BPU interrupt and thread |
| `iw.c` | the parameter blocks (0x4964..0x49e8), into the BPU groups and the VICU blocks |
| `vicu.c` | the VICU interrupts and their tasklets, the time-shared VICU and timer 4/5, the frame rate, the raw-frame dump |
| `vpss_isp.c` | the VPSS side's blocks (0x49d2..0x49f0), their copy taken with the BPU's snapshot, and the register image |
| `ib.c` | the ISP buffer pools in MMZ |

In qemu-bench (its xm-isp2 model), both chips stream with it:

- the XM535D1 X6E-WEQ: two sensors on their own VICUs, spliced;
- the XM535D2 X9-WE2Q: three sensors, two of them time-sharing VICU 0.

On the VICU, BPU and VPSS register blocks it writes the same registers with the same values as `isp.ko`, in the same order frame by frame. The differences are the buffer addresses, AE values, the point where the BPU-done acknowledgement lands, and which VICU gets the first ISP buffer. All of them depend on timing.

Differences from `isp.ko`:

- Channel and group numbers are checked (`isp.ko` doesn't for 0x49db).
- On unload all 16 mmap pages are freed (`isp.ko` frees only the first).
- The 3D noise reduction's interrupt is only acknowledged; `isp.ko` also prints a test line for each count it reads.
