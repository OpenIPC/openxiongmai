# xm535w1_vi

`xm535w1_vi` is the XM535W1's video input, at `/dev/vi`, and builds as `open_vi.ko`. It replaces the W1's vendor `vi.ko` without changes to what uses it: libxmv200, `isp.ko`, `rgn.ko`/`open_rgn` and Sofia.

The W1 has no VPSS module. Its VI drives the XM535's VPSS block (0x34000000) the way the XM533's VI drives its own: each channel's frame start and end interrupts (IRQ 57, 58, 67, 68) take and fill `vb.ko` buffers. At frame start, a channel that takes the frame gets the buffer for the next one; source and output rates decide which frames, `dst` of every `src`, spread evenly. At frame end, the filled buffer goes to VENC (vb list 2) or to the channel's queue for user space.

| ioctl | |
|-------|-|
| `0x5600` | init: the VI reset (`rst=1`) and every channel cleared |
| `0x40505602` | a channel's attributes: capture x, y, w, h, format (2 = SP420), output w, h, mirror, flip, source and output frame rates |
| `0x40045603` / `0x40045604` | enable / disable a channel |
| `0x40505605` | bind: module (7 VENC), dev, chn |
| `0x40505606` | the depth of the channel's user-space queue |
| `0x40505607` / `0x40505608` | a picture from that queue / that picture back (only one handed out) |
| `0x4a18560e` / `0x4a18560f` | OSD background windows: up to 256 rectangles a channel, two measured for luminance each frame |

It exports `vi_set_chn_win` (covers) and `vi_set_chn_line` (channel 0's lines) for rgn. The `vpss_thread_*`, `vpss_set_chn_win` and `vpss_set_grp_line` hooks that the W1's `isp.ko` and `rgn.ko` link against are empty, as in `vi.ko`.

Not here: the vendor's `line=N` low-latency mode (load_xiongmai uses `line=0`).

```sh
make -C drivers/xm535w1_vi KDIR=... ARCH=arm CROSS_COMPILE=...
insmod open_vi.ko   # after vb, before isp
```

On the XM535D1/D2 (the same `vi.ko`), it scales an online BPU's picture for the tunings captured with one ISP. There `open_vpss` (`drivers/xm535_vpss`, `WITH_VI=1`) loads beside it and has the VPSS hooks, so build this one with `WITH_VPSS=1`, which leaves them out.

