# xm_vi

`xm_vi` is the video input (VI) driver for the XM530/XM550. It replaces the vendor's `xm530_vi.ko` without changes to applications that use it. Load it after `mmz.ko`, in place of the vendor module:

```sh
rmmod xm530_vi
insmod xm_vi.ko buffer=2      # buffers per way, 1-4 (otherwise 4); one value per way
```

Issue #34 records how the vendor module works and the register map behind this driver.

## Ways

VI writes the ISP's picture into memory on up to six *ways*:
- **Way 0** is written at the capture size, or upscaled when it is set wider than the capture.
- **Ways 1–5** are scaled down from the capture.
- **Ways 0–3** have a store engine and an interrupt. Way 3 is luma only.
- **Ways 4 and 5** are motion-detection channels that the vendor driver produced in software. Here their setup succeeds, but they never deliver a frame.

Each way's buffers come from one MMZ block named `vi_way[N]`, sized buffers × w × h × 1.5 (NV12 layout: Y, then interleaved chroma). The block survives VI init and module reloads, and the driver reuses it if it is large enough.

## ioctls on `/dev/vi` (type `'V'`)

| Nr | Argument | Does |
|----|----------|------|
| 0 `SET_DEV` | `{x, y, w, h}` | Sets the capture window. |
| 1 `SET_CHN` | `{way, w, h}` | Sets up a way: allocates or reuses its buffers, fills them with grey, then sets strides, buffer addresses, the scaler and the interrupt. Fails if w×h exceeds the MMZ block, if w or h is 0 or above 8192, or if VI cannot shrink that far. The scaler limit is 16× across and 4× down on way 0, and 64× and 16× on the others. |
| 2 / 3 `ENABLE` / `DISABLE` | `way` | Starts or stops the way's store engine. Fails if the way has not been set up. |
| 4 `GET_FRAME` | frame record, in/out | Waits up to 2 s for a filled buffer and hands out the oldest one. |
| 5 `RELEASE_FRAME` | frame record | Gives the buffer back to the hardware. Fails unless the record is a frame that is currently handed out. |
| 6 `INIT` | none | Resets VI and forgets every way. |
| 7 `SET_RATE` | `{way, rate, keep_mask}` | Sets the store engine's frame rate and keep mask, then does what `SET_MODE` does with the same argument (the vendor driver does this too). |
| 8 `SET_MODE` | `{way, mode, val}` | Mode 2 passes each filled buffer straight to the engine `val` selects (0 or 1) instead of to `GET_FRAME`. This path is untested. |

Errors return −1 (`EPERM`), as the vendor driver does. Unknown requests log `vi_ioctl: cmd error` and return 0.

The frame record is 32 bytes: `{way, seq, phys_y, phys_c, w, h, u64 pts}`. `pts` is the interrupt time in microseconds. `seq` counts the frames handed out on that way, and `RELEASE_FRAME` matches it. Other kernel modules can use the same calls through the exported `vi_get_frame()` and `vi_release_frame()`.

## Buffers and interrupts

A buffer belongs to the hardware until it is filled. The store engine then sets the buffer's bit in its buffer-full register, and the interrupt queues the buffer for `GET_FRAME`. Software frees the buffer by writing the complement of that bit back, which `RELEASE_FRAME` does.

If more than one buffer filled between two interrupts, the order is unknown. The driver frees those buffers without delivering them and counts them as dropped.

## `/proc/umap/vi`

`/proc/umap/vi` shows:
- the capture window;
- per way: size, buffers, interrupt and store enables, rate and keep mask, the time between the last two frames in µs, frames received, handed out and dropped, and the mask of buffers handed out.

It shows the same counters as the vendor file, but the layout differs.

## Differences from the vendor driver

- **No vmalloc leak.** The vendor driver maps each buffer into the kernel at `SET_CHN` and loses the mappings at `INIT`, about 7 MB per application start. This driver keeps no per-buffer mapping.
- **Stricter release.** A release must name a frame that is actually handed out, so a stale or repeated release cannot free a buffer that something else is reading.
- **Missing features:**
  - the motion-detection statistics on way 3;
  - frames on ways 4 and 5.
- **Same reset hazard as the vendor driver.** `INIT` resets VI together with the ISP blocks on the same reset line, and it does not stop DMA first. If 3D noise reduction is running, that can freeze the board. Stop it before `INIT`: `tools/xm-vi-hang -N` shows how.
