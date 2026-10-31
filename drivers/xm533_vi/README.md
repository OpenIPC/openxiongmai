# xm533_vi

`xm533_vi` is the XM533's video input, at `/dev/vi`, and builds as `open_vi.ko`. It replaces the vendor `vi.ko` without changes to what uses it: libxmv200, `rgn.ko`/`open_rgn` (`vi_set_chn_win`) and Sofia.

The XM533 has no VPSS: VI stores up to three channels itself, scaled down from the ISP's picture, into `vb.ko`'s buffers. When a frame starts (IRQ 59/60), each channel that takes that frame gets a free buffer for the next one. Which frames a channel takes is set by its source and output rates (`dst` of every `src`, spread evenly). When the frame ends, the filled buffer goes to what the channel is bound to (VENC: vb list 2) or to the channel's queue for user space.

| ioctl | |
|-------|-|
| `0x5600` | init: the VI reset (`rst=1`) and every channel cleared |
| `0x40505602` | a channel's attributes: capture x, y, w, h, format, output w, h, mirror, flip, source and output frame rates (words 4..14) |
| `0x40045603` / `0x40045604` | enable / disable a channel |
| `0x40505605` | bind: module (7 VENC), dev, chn |
| `0x40505606` | the depth of the channel's user-space queue |
| `0x40505607` / `0x40505608` | a picture from that queue (waiting word 2 ms) / that picture back (only one handed out) |
| `0x40505609` | crop |
| `0x4050560e` | the last picture sent |

Channel 0's crop goes through the ISP (`isp.ko`'s `vpss_isp_set_attr`); channels 1 and 2 have their own. `/proc/umap/vi` shows the channels and their counters.

Not here: the vendor's `nrt` mode, a second store interrupt handler that is off by default.

```sh
make -C drivers/xm533_vi KDIR=... ARCH=arm CROSS_COMPILE=...
insmod open_vi.ko   # after vb and isp
```

A crop is checked without its sums wrapping.
