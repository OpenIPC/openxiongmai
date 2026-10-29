# xm510_rgn

`xm510_rgn` is the region (OSD) device of the XM510, built as `open_rgn.ko` (the name OpenIPC firmware gives the open modules that replace vendor ones). It replaces the vendor's `xm510_rgn.ko` without changes to the vendor programs that use it. The XM530/XM550 has its own driver, `drivers/xm530_rgn`, for a larger block with a different ABI.

VI draws the regions over the picture, so they appear in every stream encoded from it. There are two kinds:
- **Overlays:** up to 16 bitmaps on each of VI channels 0 and 1, 1 bit a pixel. Each has a foreground and a background colour from the channel's 16-entry YUV palette.
- **Covers:** up to 8 rectangles, all of one YUV colour.

```sh
insmod open_rgn.ko   # after MMZ (for /proc/umap), any time before the streamer
```

The module needs no other module: it keeps its bitmaps in its own memory. `tools/xm-rgn-test -510` draws regions from the shell.

## `/dev/rgn`

All calls are `ioctl`s of type `'R'`, with records of 32-bit words copied in; nothing is copied back.

| nr | Record | Does |
|---|---|---|
| 1 | none | Removes every region. |
| 2 | `{chn, handle, w, h}` | Creates overlay `handle` (0-15) of channel `chn` (0-1): `w` (1-2047) × `h` (1-1023) pixels. |
| 3 | `{chn, handle, -, -}` | Destroys it. |
| 4 | `{chn, handle, x, y, fg, bg, effect}` | Places it at `x` (0-2047), `y` (0-1023) with colours `fg` (bitmap 1) and `bg` (bitmap 0). |
| 5 | `{chn, handle, data, size}` | Loads its bitmap: `size` bytes at `data`. Bytes past `w` × `h` / 8 are ignored, as the vendor module ignores them. |
| 6 | `{chn, handle, on}` | Shows it (`on` 1) or hides it. |
| 7 | `{handle, w, h, x, y, color}` | Creates cover `handle` (0-7). |
| 8 | `{handle, ...}` (6 words) | Destroys it. |
| 9 | as 7 | Places it, and sets the colour of all covers to `color` (0x00YYUUVV): there is one colour register. |
| 10 | `{-, handle, on}` | Shows cover `handle` (`on` 1) or hides it. |

A wrong channel, handle or size fails with `EINVAL`. A region that exists already gives `EEXIST`, and one that doesn't `ENOENT`. An overlay that doesn't fit in what is left of the channel's OSD memory gives `ENOSPC`. A new overlay's bitmap is cleared in the OSD memory before it can be shown, so create, like a bitmap load, waits for the hardware and can fail with `ETIMEDOUT` or `EINTR`. When a destroy's load fails, the remaining overlays keep their old place until the next load succeeds. A cover must end within 2047 in both directions. The vendor library creates an overlay with ioctls 2 and 4, and a cover with 7 and 9.

### Bitmaps and colours

A bitmap is a run of pixels, row after row with no padding, 1 bit each, MSB first. The channel's bitmaps share its OSD memory: 12 KiB for channel 0, 8 KiB for channel 1. Each overlay takes its size rounded up to 32 pixels.

`fg` and `bg` are palette entries: bits 27-24 are the entry and bits 23-0 the colour, 0x00YYUUVV. Ioctl 4 writes both into the palette, so overlays that share an entry share its colour. With `effect` 0 the background is transparent. `effect` goes to the hardware as it is; what its bits do has not been tried.

Channel 1 is scaled from channel 0's picture. Its `x` goes to the hardware converted to channel 0 pixels through VI's channel 1 scaling ratio, as the vendor module converts it; `y` is not converted.

`/proc/umap/rgn` lists the regions in the vendor's layout. `Offect` is the overlay's place in the bitmap memory of both channels, channel 1 starting at `0x3000`.

## Registers

Overlays, per channel at `0x40079000 + chn * 0x1000`:

| Offset | Holds |
|---|---|
| `0x000` | Bit `h` shows overlay `h`; bit 31 the channel's overlays |
| `0x004` | The bitmap memory load: the driver may load when bit 0 equals bit 1, then toggles bit 0 |
| `0x010` | The palette, 16 words of 0x00YYUUVV |
| `0x080 + 16 * h` | Overlay `h`: `x \| y << 16`, `w \| h << 16`, the bitmap offset (bits 13-0) with the background entry (19-16), the foreground entry (23-20) and effect bits 19-12 (31-24), then effect bits 11-0 |

Covers, at `0x40077000`:

| Offset | Holds |
|---|---|
| `0x004` | Bit `h` shows cover `h` |
| `0x010` | The colour of every cover |
| `0x040 + 16 * h` | Cover `h`: `x0`, `x1`, `y0`, `y1` |
| `0x1000` | VI channel 1's horizontal scaling ratio, `(src << 10) / dst` (the VI driver's) |

The bitmap memory of the channels is at `0x40090000` and `0x40094000`. The driver loads a channel's bitmap memory from its copy, each 32-bit word byte-swapped, after any bitmap change.

## What differs from the vendor module

- **Checks:** channel, handle and size are checked. The vendor module indexes its lists and its registers with them as they come.
- **Errors:** errnos. The vendor module returns -1 for every error.
- **Destroying an overlay:**
  - It waits for the bitmap memory as a bitmap load does, sleeping, with a timeout. The vendor module spins for it with its lock held, forever if VI is stopped.
  - The bitmaps after the overlay move down with `memmove`; the vendor module uses an overlapping `memcpy`.
  - The channel's bit 31 is left as it was; the vendor module turns it on.
- **Memory:** the bitmap copy is the module's own memory. The vendor module allocates an MMZ block named `rgn` on every ioctl 1, maps it, and leaks the earlier mapping.
- **Reset and unload:** both hide every region. The vendor module leaves them on the picture with nothing left to remove them.
- **Missing regions:** showing one fails. The vendor module shows whatever the registers hold.
- **Channel 1 positions:** a position that converts to below 0 is 0, and one past 2047 is 2047. The vendor module writes -1, the far right of the picture, when VI channel 1 isn't set up, and lets a large ratio wrap.
- **`/proc/umap/rgn`:** a seq_file that frees what it allocates. Each read of the vendor's leaks some memory.

## Tested

On an XM510 + JX-H62 with majestic, whose vendor `isp` daemon opens `/dev/rgn` and resets it but draws nothing:
- **Drawing:** overlays (a border, and a solid one in palette entry 1) and a cover appear in the main stream, as with the vendor module from the same calls.
- **Destroying:** after the first overlay is destroyed, the two after it still draw correctly at their new offsets, and bit 31 stays on.
- **Errors:**
  - a channel of 2, a handle of 16 or a cover handle of 8 are refused;
  - a second create gives `EEXIST`;
  - an overlay too large gives `ENOSPC`;
  - showing or placing a missing region gives `ENOENT`.
- **Restarts:** a majestic restart removes every region, and unloading the module hides them. The vendor module, unloaded, leaves its regions showing.
