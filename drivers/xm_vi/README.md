# xm_vi

`xm_vi` is the video input (VI) driver for the XM530/XM550, built as `open_vi.ko` (module `open_vi`, the name OpenIPC firmware gives the open modules that replace vendor ones). It replaces the vendor's `xm530_vi.ko` without changes to applications that use it. Load it after `mmz.ko`, in place of the vendor module:

```sh
rmmod xm530_vi
insmod open_vi.ko buffer=2      # buffers per way, 1-4 (otherwise 4); one value per way
```

Issue #34 records how the vendor module works and the register map behind this driver.

## Ways

VI writes the ISP's picture into memory on up to six *ways*:
- **Way 0** is written at the capture size, or upscaled when it is set wider than the capture.
- **Ways 1–5** are scaled down from the capture.
- **Ways 0–3** have a store engine and an interrupt. Way 3 is luma only.
- **Ways 4 and 5** are motion-detection channels that the vendor driver produced in software. Here their setup succeeds, but they never deliver a frame.

Each way's buffers come from one MMZ block named `vi_way[N]`, sized buffers × w × h × 1.5 (NV12 layout: Y, then interleaved chroma). The block survives VI init and module reloads, and the driver reuses it if it is large enough. With `open_mmz allocator=cma` the block is RAM from the CMA area, and the driver fills it through the kernel's own (uncached) mapping rather than `ioremap()`.

## ioctls on `/dev/vi` (type `'V'`)

| Nr | Argument | Does |
|----|----------|------|
| 0 `SET_DEV` | `{x, y, w, h}` | Sets the capture window. |
| 1 `SET_CHN` | `{way, w, h}` | Sets up a way: allocates or reuses its buffers, fills them with grey, then sets strides, buffer addresses, the scaler and the interrupt. Fails if w×h exceeds the MMZ block, if w or h is 0 or above 8192, or if VI cannot shrink that far. The scaler limit is 16× across and 4× down on way 0, and 64× and 16× on the others. |
| 2 / 3 `ENABLE` / `DISABLE` | `way` | Starts or stops the way's store engine. Fails if the way has not been set up. |
| 4 `GET_FRAME` | frame record, in/out | Waits up to 2 s for a filled buffer and hands out the oldest one. On a descriptor opened `O_NONBLOCK` it returns `EAGAIN` at once instead of waiting. |
| 5 `RELEASE_FRAME` | frame record | Gives the buffer back to the hardware. Fails unless the record is a frame that is currently handed out. |
| 6 `INIT` | none | Stops the store engines and 3D noise reduction, waits 200 ms for their DMA to finish, then resets VI and forgets every way. |
| 7 `SET_RATE` | `{way, rate, keep_mask}` | Sets the store engine's frame rate and keep mask, then does what `SET_MODE` does with the same argument (the vendor driver does this too). |
| 8 `SET_MODE` | `{way, mode, val}` | Mode 2 passes each filled buffer straight to the engine `val` selects (0 or 1) instead of to `GET_FRAME`. This path is untested. |

Errors return −1 (`EPERM`), as the vendor driver does, except `EAGAIN` from a non-blocking `GET_FRAME`. Unknown requests log `vi_ioctl: cmd error` and return 0.

The frame record is 32 bytes: `{way, seq, phys_y, phys_c, w, h, u64 pts}`. `pts` is the interrupt time in microseconds. `seq` counts the frames handed out on that way, and `RELEASE_FRAME` matches it. Other kernel modules can use the same calls through the exported `vi_get_frame()` and `vi_release_frame()`.

`poll()` on `/dev/vi` reports it readable while a way has a filled buffer waiting to be handed out. It watches the ways that descriptor has called `GET_FRAME` on, or every way before its first `GET_FRAME`. The vendor driver has no `poll`.

## Buffers and interrupts

A buffer belongs to the hardware until it is filled. The store engine then sets the buffer's bit in its buffer-full register, and the interrupt queues the buffer for `GET_FRAME`. Software frees the buffer by writing the complement of that bit back, which `RELEASE_FRAME` does.

If more than one buffer filled between two interrupts, the order is unknown. The driver frees those buffers without delivering them and counts them as dropped.

When a process closes `/dev/vi`, or exits, the driver releases every frame that descriptor still holds. The vendor driver keeps such frames until the next `INIT`. Frames taken through the kernel exports are not tied to a descriptor.

## `/proc/umap/vi`

`/proc/umap/vi` shows:
- the capture window;
- per way: size, buffers, interrupt and store enables, rate and keep mask, the time between the last two frames in µs, frames received, handed out and dropped, and the mask of buffers handed out.

It shows the same counters as the vendor file, but the layout differs.

## Differences from the vendor driver

- **No vmalloc leak.** The vendor driver maps each buffer into the kernel at `SET_CHN` and loses the mappings at `INIT`, about 7 MB per application start. This driver keeps no per-buffer mapping.
- **Stricter release.** A release must name a frame that is actually handed out, so a stale or repeated release cannot free a buffer that something else is reading.
- **Release on close.** Frames a process still holds when it closes `/dev/vi` go back to the hardware.
- **`poll` and `O_NONBLOCK`.** The vendor driver supports neither.
- **No freeze at `INIT`.** The vendor driver resets VI together with the ISP blocks on the same reset line while 3D noise reduction may still be doing DMA, which can freeze the board (`tools/xm-vi-hang`). This driver turns 3D-NR off and waits for the DMA to drain first. Whoever sets the ISP up afterwards turns 3D-NR back on, as a normal start does.
- **Missing features:**
  - the motion-detection statistics on way 3;
  - frames on ways 4 and 5.
