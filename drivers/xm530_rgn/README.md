# xm530_rgn

`xm530_rgn` is the region (OSD) device of the XM530/XM550, built as `open_rgn.ko` (the name OpenIPC firmware gives the open modules that replace vendor ones). It replaces the vendor's `xm530_rgn.ko` without changes to the vendor library that uses it.

VI draws the regions over the picture of each of its three channels, so they appear in every stream encoded from that channel and in its snapshots. On majestic, channel 0 is the main stream and channel 1 the stream snapshots come from. There are two kinds:
- **Overlays:** up to 8 bitmaps a channel. Each is coloured through the channel's 16-entry YUV palette and a colour map of its own.
- **Covers:** up to 32 rectangles a channel, each of one YUV colour, solid or outlined.

```sh
insmod open_rgn.ko   # after MMZ (for /proc/umap), any time before the streamer
```

The module needs no other module: it keeps its bitmaps in its own memory. `tools/xm-rgn-test` draws regions from the shell.

## `/dev/rgn`

All calls are `ioctl`s of type `'R'`, with records of 32-bit words copied in; nothing is copied back.

| nr | Record | Does |
|---|---|---|
| 1 | none | Removes every region of every channel. |
| 2 | `{chn, handle, w, h, format}` | Creates overlay `handle` (0-7): `w` × `h` pixels (1-4095), format 0, 1 or 2 (1, 2 or 4 bits a pixel). |
| 3 | `{chn, handle, -, -, -}` | Destroys it. |
| 4 | `{chn, handle, x, y, colormap, effect}` | Places it at `x`, `y` (0-4095) and sets its colours and effects. |
| 5 | `{chn, handle, data, size}` | Loads its bitmap: `size` bytes at `data`, at most what `w` × `h` × bits takes. |
| 6 | `{chn, handle, on}` | Shows it (`on` 1) or hides it. |
| 7 | `{chn, handle, w, h, x, y, color, effect}` | Creates cover `handle` (0-31). |
| 8 | `{chn, handle, ...}` (8 words) | Destroys it. |
| 9 | as 7 | Places it, with `color` 0x00YYUUVV; `effect` 1 is solid, anything else an outline. |
| 10 | `{chn, handle, on}` | Shows it (`on` 1) or hides it. |
| 11 | `{chn, color[16]}` | Loads the channel's palette, 0x00YYUUVV each. |

`chn` is 0-2. A wrong channel, handle, size or format fails with `EINVAL`. A region that exists already gives `EEXIST`, and one that doesn't `ENOENT`. An overlay that doesn't fit in what is left of the channel's OSD memory gives `ENOSPC`. The vendor library creates an overlay with ioctls 2 and 4, and a cover with 7 and 9.

### Bitmaps and colours

A bitmap is a run of pixels, row after row with no padding, each `1 << format` bits, MSB first. The channel's bitmaps share its OSD memory: 8 KiB for channels 0 and 1, 2 KiB for channel 2. Each overlay takes its size rounded up to 32 pixels' worth.

`colormap` holds palette indexes: bits 7-4 the first foreground colour, 11-8 the second. What a pixel draws, as seen on an XM550 with `effect` 0:
- **Format 0:** 0 transparent, 1 the first foreground colour.
- **Format 1:** 0 transparent, 1 the first foreground colour, 2 and 3 the second.
- **Format 2:** every non-zero pixel draws black. What it needs is not known yet.

`effect` and the rest of `colormap` go to the hardware as they are (the register table shows where); what their bits do has not been tried.

The palette resets to 16 colours from black (index 0) to white (index 15).

`/proc/umap/rgn` lists the regions in the vendor's layout. `Offect` is the overlay's place in the bitmap memory of all three channels, channel 1 starting at `0x2000` and channel 2 at `0x4000`.

## Registers

Each channel has its block at `0x30079000 + chn * 0x1000`:

| Offset | Holds |
|---|---|
| `0x000` | Bit `h` shows overlay `h`; bits `8 + h` and `16 + h` its format (1: bit 8; 2: bit 16); bit 31 is set when any is shown |
| `0x004` | The bitmap memory load: the driver may load when bit 0 equals bit 1, then toggles bit 0 |
| `0x010` | The palette, 16 words |
| `0x080 + 16 * h` | Overlay `h`: `x \| y << 16`, `w \| h << 16`, the bitmap offset (bits 13-0) with colour map bits 7-0 (23-16) and effect bits 19-12 (31-24), then effect bits 11-0, colour map bits 11-8 (15-12) and effect bits 23-20 (19-16) |
| `0x200` | Bit `h` shows cover `h` |
| `0x204` | Bit `h` makes cover `h` solid |
| `0x210 + 12 * h` | Cover `h`: `x0 \| y0 << 16`, `x1 \| y1 << 16`, its colour |

The bitmap memory of the channels is at `0x30090000`, `0x30092000` and `0x30094000`. The driver loads a channel's whole bitmap memory from its copy, each 32-bit word byte-swapped, after any bitmap change.

## What differs from the vendor module

- **Checks:** channel, handle, size, position and format are checked. The vendor module indexes its lists and its registers with them as they come.
- **Errors:** errnos. The vendor module returns -1 for every error.
- **Memory:** the bitmap copy is the module's own memory. The vendor module allocates an MMZ block named `rgn` on every ioctl 1, maps it, and leaks the earlier mapping.
- **Reset and unload:** both hide every region. The vendor module leaves them on the picture with nothing left to remove them, so they stay until a reboot.
- **Missing regions:** showing or placing one fails. The vendor module shows whatever the registers hold, and placing a cover that doesn't exist dereferences NULL or moves another cover.
- **Destroying an overlay:** the bitmaps after it move down with `memmove`. The vendor module uses an overlapping `memcpy`.
- **Palette:** ioctl 11 takes the lock and checks the channel. The vendor module writes past its registers for a channel over 2.
- **`/proc/umap/rgn`:** a seq_file that frees what it allocates. Each read of the vendor's leaks some memory.

## Tested

On an XM550 + IMX335 with majestic, which opens `/dev/rgn` and resets it but draws nothing:
- **Drawing:** overlays in formats 0 and 1, and solid and outlined covers, appear in the main stream (H.265, channel 0) and in snapshots (channel 1), as with the vendor module from the same calls.
- **Bitmaps:** MSB first with unpadded rows (a 100-pixel-wide border draws straight).
- **Destroying:** after the first overlay of a channel is destroyed, the two after it still draw correctly at their new offsets.
- **Errors:**
  - a channel of 3, a handle of 8, a format of 3 or a cover handle of 32 are refused;
  - a second create gives `EEXIST`;
  - an overlay too large for what is left gives `ENOSPC`;
  - showing or placing a missing region gives `ENOENT`.
- **Restarts:** a majestic restart removes every region, and unloading the module hides them.
