# xm_v200_venc

`xm_v200_venc` is the V200 series' VENC frame queue (XM530V200, XM533, XM535D1/D2/W1), `/dev/venc_info`, built as `open_venc.ko`. It replaces the vendor `venc.ko` without changes to what uses it: libxmv200 (`xmv200_frame_get`) or Sofia.

VPSS (or VI) hands the pictures bound to VENC to vb.ko's VENC list. A thread takes them from there and has `rgn.ko` draw the OSD regions on them (`osd_process`). It queues them up to `fifo_depth` (default 4); past that a picture goes back to the free list. The thread waits from the first open of the device until `0x4900`.

| ioctl | |
|-------|-|
| `0xc0284902` | the next picture, waiting up to 2 s: `{VENC chn, phys, width, height, stride, 0, pts lo, pts hi, ...}`, with the buffer's descriptor address at word 8 (word 9 on the XM535, which has a word of the descriptor's own at 8) |
| `0x40284903` | that record back: the picture goes to the free list (only a picture this driver handed out is taken back) |
| `0x4900` | start, with the queue empty |
| `0x41044901` (`0x40c44901` XM533, `0x40c04901` XM535W1) | a channel's settings, shown in `/proc/umap/venc` |
| `0x40104904` | 16 bytes kept |
| `0x40284905` | XM535: the OSD regions composed on a channel's picture (`comp_osd_process`) |

Like `open_vb`, it has to know vb.ko's buffer descriptors: `VENC_LAYOUT=v200` (XM530V200, XM533) or `xm535` at build time. It needs `vb_get_buf`/`vb_put_buf` (open_vb or the vendor's) and `rgn.ko`'s `osd_process` (and `comp_osd_process` on the XM535).

Not here yet: the XM535's `venc.ko` also carries the vendor encoder's memory manager (`/dev/venc_mem`), which only the vendor's Sofia uses.

```sh
make -C drivers/xm_v200_venc KDIR=... ARCH=arm CROSS_COMPILE=... VENC_LAYOUT=xm535
insmod open_venc.ko   # after vb and rgn
```
