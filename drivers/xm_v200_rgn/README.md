# xm_v200_rgn

`xm_v200_rgn` handles regions on the V200 series (XM530V200, XM533, XM535D1/D2/W1), at `/dev/rgn`, and builds as `open_rgn.ko`. It replaces the vendor `rgn.ko` without changes to anything using it: venc.ko (vendor or `open_venc`) calls `osd_process` (and `comp_osd_process` on the XM535), and user space keeps its ioctls.

There are three kinds of region, each with handles 0..255:

| kind | what | drawn by |
|------|------|----------|
| 0 overlay | a bitmap blended into the pictures of the VENC channel it is attached to | the OSD engine, before venc.ko queues each picture; on the XM535, formats 4 and 5 by the encoder's own OSD |
| 1 cover | a filled or framed rectangle | VI or VPSS (`vi_set_chn_win`, `vpss_set_chn_win`); on the XM535 also the encoder |
| 2 line | (XM535) a line | VI or VPSS (`vi_set_chn_line`, `vpss_set_grp_line`), from its direction, length and a 32.32 step |

Every ioctl passes the same 15 words: the handle, the kind, then the arguments.

| ioctl | |
|-------|-|
| `0x403c5201` | create. An overlay's words 2..14 are format, x, y, canvas width and height, shown width and height, 4 colours (AYUV), alpha and scale; a cover's are x, y, w, h, colour, type and thickness; a line's are x0, y0, x1, y1, colour and thickness |
| `0x403c5202` | destroy (only once detached) |
| `0x403c5203` | set the attributes (an overlay keeps its format and canvas) |
| `0x403c5204` | an overlay's bitmap: word 2 is the user address, word 3 the length |
| `0x403c5207` / `0x403c5208` | attach / detach. Words 2..4 are module (7 VENC, 8 VPSS, 0x10 VI), dev and chn; overlays attach to VENC only |
| `0x403c5209` | the engine's YUV420SP to RGB888 planar conversion: width, height, source, destination |
| `0x403c520a` | one overlay blended into a picture of user space's: stride, height, source, destination |
| `0x5200` | everything destroyed (bitmaps freed) and the engine reset |

Overlay formats: 0 ARGB1555, 1 ARGB8888, 2 1 bit, 3 2 bits (4 colours), and 4 and 5 for the XM535's encoder (1 and 2 bits, kept in 16x16 tiles). Each VENC channel takes 8 overlays, or 4 on the XM535. The engine draws up to four a run, and `/proc/umap/rgn` counts process calls, engine runs and timeouts.

Build with `RGN_CHIP=v200` (default), `xm533`, `xm535` or `xm232`. The chips differ in:
- the engine's address: 0x21300000 on the XM530V200, 0x21200000 on the others;
- what draws covers: VI and VPSS on the V200, VI only on the XM533, and VI, VPSS and the encoder on the XM535;
- the XM535's lines and encoder OSD;
- the XM535's order for 2-bit colours 1 and 2, and its 4x scale code.

It needs `mmz_alloc`/`mmz_free`, plus the cover and line hooks: vpss.ko exports them on the XM530V200 and XM535, vi.ko on the XM533.

## XM232

The XM232 has no OSD engine and no VI or VPSS covers: `RGN_CHIP=xm232` builds its own driver, `xm232_rgn.c`, for its vendor `rgn.ko`. The encoder at 0x21000000 draws both kinds of region, and venc.ko (or Sofia through it) calls `comp_osd_process()` for each picture; `comp_osd_process_once()` and `comp_win_process_once()` are exported as by the vendor's.

- Overlays (kind 0): formats 0 (ARGB1555), 4 (1 bit) and 5 (2 bits), up to 8 a VENC channel. Their bitmaps are kept in 16x16 tiles, ARGB1555 pixels byte-swapped. Region n at +0x2A8 + 0x24n (enable mask +0x2A4): first and last 16x16 block (column << 10 | row), format (4: 0, 5: 1, 0: 3), scale (a shift: 0 1x, 1 2x, 2 4x; `down | across << 3`), four colours, bitmap.
- Covers (kind 1): up to 16 a VENC channel; cover n at +0x1A4 + 0x10n (mask +0x1A0): thickness, `y | x << 14`, `(y + h) | (x + w) << 14`, colour.
- The ioctls are the other chips' create, destroy, set, bitmap, attach, detach and `0x5200` (no line regions, conversion or user-space blend).

```sh
make -C drivers/xm_v200_rgn KDIR=... ARCH=arm CROSS_COMPILE=... RGN_CHIP=xm535
insmod open_rgn.ko   # after mmz and vpss (vi on the XM533), before venc
```

Unlike `rgn.ko`, the conversion and the user-space blend take only pictures in physical memory outside the kernel's RAM (MMZ), of the size they declare, and canvases are at most 8192 a side (their size would otherwise wrap).
