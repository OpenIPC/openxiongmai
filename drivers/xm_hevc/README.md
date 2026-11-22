# xm_hevc

`xm_hevc` is the H.265 encoder engine of the V200 series (XM530V200, XM533, XM535D1/D2/W1, XM232), built as `open_hevc.ko`. It replaces the vendor `hevc.ko` without changes to what uses it: `/dev/h265`, the same ioctls and records.

User space works out every register of a picture itself (libxmv200's `venc.c`, or Sofia). The driver runs the jobs:

| ioctl | |
|-------|-|
| `0x4803` / `0x4804` | take and give back the engine (a closing holder gives it back too) |
| `0x40044806` | reset, with the engine's key: `0x10002` (v3), `0x04000000` (v1) |
| job | a list of `{value, write}` pairs, entry n for the register at 4n; the entries with write set are kept, written, and the engine started |
| wait | sleeps until the picture is done and gives the job's record back, its word 11 the bytes written; after a failed picture every wait fails until a reset, as the vendor's |

The two engines (`layout=`, or `HEVC_LAYOUT=` at build time):

| | v3 (XM530V200, XM533) | v1 (XM535D1/D2/W1) |
|-|-|-|
| job / wait | `0x40384805` (56 bytes, list at word 13) / `0xc0344802` | `0x40484805` (72 bytes, list at word 17) / `0xc0444802` |
| list | 189 entries; 2..188 written, but 10..12 | 350 entries; 2..349 written |
| start | register 1 | register 0 |
| status, length | `0x58`, `0x1d8` | `0x74`, `0xf8` |
| reset | the engine's reset in the clock block (`0x20000120`): 5, then 7 | halt and wait, then 6, then 7 |

v1 jobs can also run in line mode (word 13 the line count, mode 1 in word 14: the line block at `0x21010000` paces the engine with the capture) or go on into a next output buffer when one fills (mode 4, words 15 and 16), as the vendor's do.

## XM232

The XM232's engine (H.264, H.265 and JPEG snapshots, SPI 24) has its own driver, `xm232_hevc.c`, built with `HEVC_LAYOUT=xm232`:

| | |
|-|-|
| job / wait | `0x40784805` (120 bytes: word 0 the channel, 4 the line count for line mode, 5 the work type, 29 the list) / `0xc0744802` (the first 29 words back, word 18 the stream's bytes, 26 the JPEG snapshot's) |
| list | 402 entries; 2..401 written; 1 to register 0 starts it |
| reset | key `0x5000508`: halt and wait, then the engine's own reset at `0x21f00000` (at load: the clock block's, `0x20000120`) |
| status | `+0xa0` aux halt, `+0xa4` errors, `+0xb0` begun, `+0xb4` halted, `+0xb8` end (2 done), each written back to `+0x10`..`+0x28`; lengths `+0x174` and `+0x614` |

With `joint_sensor_num` (2 on the X6S-WEQ-H: two sensors spliced into one picture), line mode splits the picture's lines between the sensors in the line block (`+0x90`..`+0x98`), and giving the engine back sets them up again. Work type 4 goes on into the next output buffers when one fills, adding up both lengths. An error (`+0xa4`, an aux halt, the line block's `+0xa8`) stops the engine and the line block and fails every wait until a reset.

```sh
make -C drivers/xm_hevc KDIR=... ARCH=arm CROSS_COMPILE=... HEVC_LAYOUT=v1
insmod open_hevc.ko
```

Only the client holding the engine can unlock it (`-EPERM` otherwise), so a stray unlock cannot let two encoders in.
