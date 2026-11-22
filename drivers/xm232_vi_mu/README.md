# xm232_vi_mu

`xm232_vi_mu` is the XM232's video input, at `/dev/vi`, and builds as `open_vi_mu.ko`. It replaces the vendor `vi_mu.ko` without changes to what uses it: Sofia, `isp.ko` (`isp_register_ops`, `isp_set_next_ch`, `isp_set_complete`), `vb.ko` and `mmz.ko`. It exports `vi_get_sensor_type`.

The XM232 has one capture path, time-multiplexed between up to two sensors. Every frame period `isp.ko` switches the VICU to the next sensor, and the store writes the VPSS's three outputs of the sensor just captured into `vb.ko`'s buffers. A kernel thread runs each period:

- `vi_sel` (IRQ 89; raw events at +0x10020, masked status at +0x10028 through the mask at +0x10024) wakes it. On a VICU end event it waits for the VICU's end flag at +0x7104C, one jiffy at a time, at most `vic_timeout_num` jiffies.
- The buffers of the frame just stored are done. Each goes on to what its channel is bound to (`vb.ko` list 2 for VENC, list 0 otherwise) or to the channel's queue for user space. In the split modes the sensors share one buffer, one above the other, and it goes on once every sensor's part is in (each buffer's +0x4C bits, set from the store's done flag at +0x73090).
- The next sensor goes to `isp.ko`. Each channel that takes the next frame, by its source and output rates (`frtc.h`, the vendor's table), gets a buffer, and the register image is written to the store and latched at +0x10014.

| Parameter | |
|-----------|-|
| `mode` | 0: each sensor its own picture; 1: split; 2: channel 0 into the line buffer; 3: split, with channel 0 into the line buffer |
| `sensor_max` | the sensors, 2 by default |
| `line` | the line buffer's lines (online VENC) |
| `memcomp` | the line buffer from `mmz.ko`'s compressed blocks (`mmz_alloc_comp`), 1 by default |
| `rst` | reset the VI at init (SYS +0x11C), 1 by default |

| ioctl | |
|-------|-|
| `0x5600` | init |
| `0x5632`, `0x5633` | accepted, nothing done |
| `0x40045603` / `0x40045604` | enable / disable a channel |
| `0x40505601` | a device's attributes (ignored, as `vi_mu.ko` does) |
| `0x40505602` | a channel's attributes: capture x, y, w, h, format, output w, h, mirror, flip, source and output frame rates (words 4..14) |
| `0x40505605` | bind: words 4..6, copied to each of the channel's buffers |
| `0x40505606` | the depth of the channel's user-space queue |
| `0x40505607` / `0x40505608` | a picture from that queue (waiting word 2 ms) / that picture back |
| `0x4a18560e` / `0x4a18560f` | a channel's OSD background (0xa18 bytes) / read back |
| `0x80245610` | the version (0x2321001) |

The state is kept at `vi_mu.ko`'s offsets: a 0xe50-byte block with a 0xf8-byte record for each channel (sensor × 3 + output), the store's register image and the thread's state. `/proc/umap/vi` prints the same tables.

Differences from `vi_mu.ko`:

- `0x40505608` takes back only a picture that `0x40505607` handed out; `vi_mu.ko` frees whatever pointer it is given.
- Channel numbers are checked.
- `sensor_max` must be 1..4: its tables have room for four sensors.

In qemu-bench's `xm232` machine, with the vendor X6S-WEQ-H firmware (Sofia, mode 3, two SC2336), the store's registers get the same writes in the same order as with `vi_mu.ko`, frame by frame, apart from the buffer addresses. Sofia streams both RTSP streams (2304×2592 and 640×720 H.264), with both sensors' halves in the main picture, and takes JPEG snapshots.

```sh
make -C drivers/xm232_vi_mu KDIR=... ARCH=arm CROSS_COMPILE=...
insmod open_vi_mu.ko sensor_max=2 mode=3 line=2160   # after vb and isp
```
