# xm510_vi

`xm510_vi` is the video input (VI) driver for the XM510, built as `open_vi.ko` (module `open_vi`, the name OpenIPC firmware gives the open modules that replace vendor ones). It replaces the vendor's `xm510_vi.ko` without changes to the applications that use it. The XM530/XM550 VI is a different block, with its own driver in `drivers/xm_vi`.

Load it after `mmz.ko` (or `open_mmz`), in place of the vendor module. It finds `mmz_alloc` (and `mmz_get_addr`, where there is one) when it needs them, not by linking, so it has no module dependency for depmod to resolve: the vendor `mmz.ko` and `open_mmz` export the same symbols, and `modprobe open_vi` would otherwise load one under the other.

```sh
insmod open_vi.ko buffer=3   # buffers per channel, 2-4 (anything else: 4)
```

## Hardware

VI takes the picture inside a capture window and writes it, as NV12, into memory on three channels:
- **Channel 0:** cropped to its size.
- **Channel 1:** scaled down from channel 0.
- **Channel 2:** at its own size.

Each channel has a store engine (registers at `0x40081000 + chn × 0x1000`) that fills a ring of buffers. Two interrupts cover all channels at once: IRQ 16 marks the start of a frame and IRQ 18 its end.

Each channel's buffers come from one MMZ block named `vi_way[N]`, sized buffers × stride × h × 1.5, where the stride is the width rounded up to 128. A buffer is free, ready (filled, not yet taken) or held (taken, not yet released). At the end of a frame, the buffer the engine finished becomes ready. If the next buffer in the ring isn't free, the engine is held, so it never writes over a frame that hasn't been read; a later frame end lets it go once that buffer is free. This is the vendor driver's protocol, kept as it is.

## `/dev/vi`

| ioctl | Argument | Does |
|---|---|---|
| 0 `SET_DEV` | `{x0, y0, x1, y1}` | Capture window, and the frame interrupts on |
| 1 `SET_CHN` | `{chn, w, h}` | Channel size: allocates its MMZ block the first time after VI init, lays out its buffers, sets channel 0's crop or channel 1's scaler. Fails on a bad channel or size, a block too small for it, a channel with frames held, or channel 1 before channel 0 has a size. An enabled store engine is paused while its layout changes. |
| 2 `ENABLE` / 3 `DISABLE` | `chn` | Store engine on/off. Enable fails on a channel with no buffers. |
| 4 `GET_FRAME` | `{chn, idx, y, c, stride, h}` (24 bytes) | Takes the oldest ready buffer. It waits up to 2 s, and fails with −1 if no frame arrives in that time. |
| 5 `RELEASE_FRAME` | the same record | Frees buffer `idx` of `chn`. Fails, with the vendor's "is not using" message, if it isn't held, or was taken through another descriptor. |
| 6 `INIT` | – | Stops the store engines, waits 200 ms for their DMA to drain, resets VI through the system controller (`0x2000007c`), forgets every channel. |
| 7 `SET_RATE` | `{chn, rate, mask}` | Frame rate control: `rate` is input frames − 1, `mask` the frames of those to keep. |

An unknown command logs `vi_ioctl: cmd error` and returns 0, as in the vendor module. `vi_get_frame()` and `vi_release_frame()` are exported for kernel callers, with the same record.

What differs from the vendor driver:
- **Release on close:** frames a descriptor still holds are released when it is closed. A consumer that dies no longer leaves the store engine held until the next VI init.
- **`poll`:** a descriptor is readable while a channel it has taken frames from (any channel, before its first `GET_FRAME`) has a ready buffer. On an `O_NONBLOCK` descriptor `GET_FRAME` returns `-EAGAIN` instead of waiting.
- **Errors:** a signal during the wait interrupts it the usual way, and a timeout fails with −1. The vendor module returns −512 to user space in both cases.
- **VI init:** quiesces the store engines before the reset, which the vendor driver does with them still writing.
- **Layout:** a channel's buffers are laid out again from its block whenever its size changes, instead of keeping the first layout.
- **Ready buffers:** a buffer a reader holds is never marked ready again.
- **Releases:** only the descriptor that took a frame can release it (kernel callers release what kernel callers took).
- **Live resizing:** `SET_CHN` refuses a channel with frames held, pauses a running engine while it rewrites the layout, and sets channel 1's scaler again whenever channel 0 or channel 1 changes size.
- **Block reuse:** a block made before VI init (by an earlier load) is found by name when the MMZ module can look names up (`open_mmz`), and a block MMZ has since forgotten (an MMZ reset) is allocated again. The vendor XM510 `mmz.ko` can't look names up, and then the block must be new, after an MMZ reset.

## `/proc/umap/vi`

It has the vendor's layout: the capture window and channel 0's size, then channels 1 and 2, then per channel the frame time and rate, frames taken (SendCnt), frame ends with the engine held (HoldCnt), short frames (OverCnt), the state of each buffer (−1 unset, 0 free, 1 ready, 2 held) and how many are ready (UsableCnt).

## Tested

On an XM510 + JX-H62 with majestic (1280×720 main stream on channel 0, 704×576 configured on channel 1):
- **`/proc`:** same window, sizes, rates and MMZ addresses as with the vendor driver.
- **Video:** H.264 decodes to the same picture and frame count as with the vendor driver over the same 6 s.
- **Frame calls:** a resize of channel 1 while it runs gives frames at the new size and back; on channel 1, `poll` reports a ready frame; non-blocking `GET_FRAME` returns the three ready buffers oldest first, then `EAGAIN`; a released buffer is filled again within 200 ms; a second release is refused; a process that exits holding frames has them released, and the channel fills again.
- **Restarts:** ten majestic restarts and fifteen `kill -9` with video each time, but for one `kill -9` after which the leftover vendor `isp` daemon ignored SIGTERM and the next majestic could not start the ISP. That has not come back in fourteen more.
- **Not this driver:** unloading the media modules logs `BUG: Bad page state` from the vendor `xm510_isp.ko`'s exit, with the vendor VI as well.
