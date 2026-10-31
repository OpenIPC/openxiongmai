# xm535_vpss

`xm535_vpss` is the XM535D1/D2's VPSS, at `/dev/vpss`, and builds as `open_vpss.ko`. It replaces the vendor `vpss.ko` without changes to what uses it: `isp.ko`, which drives it a frame at a time, libxmv200, `rgn.ko`/`open_rgn` and Sofia.

VPSS scales each group's picture (one group per sensor, up to 4) into three channels, and stores them into `vb.ko`'s buffers. It has no interrupt of its own. For each group and frame, `isp.ko` calls:
- `vpss_thread_set_val` when the frame starts: every channel that takes it gets a buffer, and the store, scaler, crop, cover and line registers are written;
- `vpss_thread_post_deal` when the frame is done: the buffers go to what each channel is bound to (VENC: vb list 2) or to the channel's queue for user space.

Splicing puts channels of up to three groups into one picture: halves top/bottom (modes 1/2) or left/right (3/4), or thirds (5/6/7). They share a buffer, which goes on once every part is stored, as the channel of the first part.

| ioctl (128-byte record: group, channel, words 4..) | |
|-------|-|
| `0x5000` / `0x5010` | init / every group destroyed |
| `0x40805001` / `0x40805002` | create a group / its attributes |
| `0x40045008` / `0x40045009` | start / stop a group |
| `0x40805004` | a channel's format (2 = SP420; channels 0 and 1 only that), w, h, source and output frame rates, mirror, flip |
| `0x40805006` / `0x40805007` | enable / disable a channel |
| `0x4080500a` | bind: module (7 VENC), dev, chn |
| `0x4080500b` / `0x4080500c` | the group's crop (kept) / a channel's crop |
| `0x4080500d` | the depth of the channel's user-space queue |
| `0x4080500e` / `0x4080500f` | a picture from that queue / that picture back (only one handed out) |
| `0x40805012` | splice: mode, id |
| `0x420c5011` | debug register writes after each frame's |

It exports `vpss_set_chn_win` and `vpss_set_grp_line` for rgn's covers and lines. `vi_set_chn_win` and `vi_set_chn_line` are empty, as in `vpss.ko`. Frame-rate control passes `dst` of every `src` frames.

Not here: the vendor's `line=N` low-latency mode. load_xiongmai loads it with `line=0`.

```sh
make -C drivers/xm535_vpss KDIR=... ARCH=arm CROSS_COMPILE=...
insmod open_vpss.ko   # after vb, before isp
```

**Online BPU (one ISP).** A tuning captured with the BPU online (one ISP, the vendor's FunExt 0: SC3338, SC4336, SC500AI, MIS4001, MIS40C1 on the XM535D1) never reaches the VPSS: `isp.ko` hands it frames only from its offline BPU. The vendor scales the online BPU's picture in `vi.ko`, as on the XM535W1, and libxmv200 does the same for such a tuning. Built with `WITH_VI=1`, this module leaves VI's cover and line hooks to `open_vi` (`drivers/xm535w1_vi` built with `WITH_VPSS=1`) so that both load on the XM535D1/D2; `open_vi` has to load before `open_rgn`.

