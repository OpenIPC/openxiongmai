# xm530v200_vpss

`xm530v200_vpss` is the XM530V200's VPSS, at `/dev/vpss`, and builds as `open_vpss.ko`. It replaces the vendor `vpss.ko` without changes to what uses it: libxmv200, `isp.ko`, `rgn.ko`/`open_rgn`, `vo.ko` and Sofia.

On the XM530V200, VPSS is a memory-to-memory engine. The ISP stores each group's (sensor's) picture into a `vb.ko` buffer on list 1. This driver's thread then, for each picture:
- takes it from list 1 and programs the engine (0x34000000) to scale it into the group's three channels;
- starts the engine and waits for its interrupt (IRQ 72);
- hands each channel's buffer to what the channel is bound to (VENC: list 2; VPSS: list 1; VO: `vo.ko`), or to the channel's queue for user space.

`isp.ko` takes part through `vpss_isp_set_val` and `vpss_isp_post_deal`.

The engine scales in two stages. First the input, cropped by the group, goes to a middle picture as large as the largest channel or the input (at most 2640 wide). Then each channel is scaled from that. A channel the size of an uncropped input takes the input buffer itself. Splicing puts two groups' channels into one picture: modes 1/2 top/bottom, 3/4 left/right.

| ioctl (128-byte record: group, channel, words 4..) | |
|-------|-|
| `0x5000` / `0x5010` | init / every group destroyed |
| `0x40805001` | create a group (max w, h, ...; word 10 = 1 also passes them to isp.ko) |
| `0x40805002` / `0xc0805003` | its attributes |
| `0x40045008` / `0x40045009` | start / stop a group |
| `0x40805004` / `0xc0805005` | a channel's format (2 = SP420; channels 0 and 1 only that), w, h, source and output frame rates, mirror, flip |
| `0x40805006` / `0x40805007` | enable / disable a channel |
| `0x4080500a` | bind: module, dev, chn (channel 100: where the group's input goes after use) |
| `0x4080500b` / `0x4080500c` | the group's crop / a channel's crop |
| `0x4080500d` | the depth of a channel's user-space queue |
| `0x4080500e` / `0x4080500f` | a picture from that queue / that picture back (only one handed out) |
| `0x40805012` | splice: mode, id |
| `0x420c5011` | debug register writes after each picture's |

It exports `vpss_set_chn_win` for rgn's covers, and `vpss_set_chn_osd`, whose regions it keeps but doesn't draw; no vendor module uses it. `vi_set_chn_win` is empty, as in `vpss.ko`. `rst=1` (the default) resets the engine before each picture, as `vpss.ko` does. The vendor's `spec=N` memory modes aren't supported.

```sh
make -C drivers/xm530v200_vpss KDIR=... ARCH=arm CROSS_COMPILE=...
insmod open_vpss.ko   # after vb, vo and isp
```
