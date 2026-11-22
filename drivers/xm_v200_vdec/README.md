# xm_v200_vdec

`xm_v200_vdec` is the XM530V200's video decoder bridge, at `/dev/vdec`, and builds as `open_vdec.ko`. It replaces the vendor `vdec.ko`. User space does the decoding; the module gives it frame buffers from the VB pools (`open_vb`) and the VO channels (`open_vo`), passes decoded frames on, keeps the decoder's state table for `/proc/umap/vdec`, and provides a microsecond delay.

The ioctls take a 36-byte record: words 0 and 1 are the buffer's physical address and size (for a VB get, word 1 is the size wanted); 2..4 stride, width, height; 5 the bind id (VB put: 7 VENC, 8 VPSS, 17 var, anything else the free list; VO put: 15 shows the frame); 6 and 7 the bind dev and chn, or the VO dev and chn; 8 the buffer.

| ioctl | |
|-------|-|
| `0xc0244402` | a VO channel's free buffer |
| `0x40244403` | a VO buffer back, shown or dropped |
| `0xc0244404` | a free VB buffer at least that big, without waiting |
| `0x40244405` | a VB buffer on to the module it is bound to |
| `0x40040144` | `{us}`: sleep on timer 5 (0x100D1020, IRQ 38), at most a second |
| `0x40e80044` | the 8 x 232-byte state table that `/proc/umap/vdec` shows |

Buffers handed back are checked to be VB's (`vb_find_buf`) or the VO channel's; `vdec.ko` takes them on trust. The timer runs one-shot, where `vdec.ko` runs it free and stops it in the interrupt; both count 24 ticks a microsecond, as `vdec.ko` does.

A VB put whose NV12 picture does not fit the buffer is refused (`-EINVAL`).
