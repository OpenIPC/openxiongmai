# xm_v200_acodec

`xm_v200_acodec` is the V200 series' audio codec, at `/dev/acodec`, and builds as `open_acodec.ko`. It replaces the vendor `acodec.ko`.

| ioctl | |
|-------|-|
| `0x40044101` | output volume 0..100 (0 mutes) |
| `0x40044103` / `0x80044104` | set / get the output mute |
| `0x40044105` | input gain 0..100 (10 or less mutes) |

The codec at 0x10068000 comes in two designs: the XM530V200/XM533's and the XM535s'. Each has its own power-up sequence (from the vendor module's disassembly) and its own mute and gain registers. `alc=1` turns on the input's automatic level and `aomute=1` starts muted (XM530V200/XM533). Build with `ACODEC_CHIP=v200`, `xm533`, `xm535` or `xm535w1`. It exports `acodec_adc_reset`, which is empty on these chips, as in `acodec.ko`.
