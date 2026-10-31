# xm530v200_jpeg

`xm530v200_jpeg` is the XM530V200's JPEG encoder, at `/dev/jpeg`, and builds as `open_jpeg.ko`. It replaces the vendor `jpeg.ko`.

| ioctl | |
|-------|-|
| `0x4a04` | take the encoder (waits for it); with `switch_en=1` its clock moves to the faster PLL |
| `0x4a05` | give it back |
| `0x4a06` | reset it (system controller 0x20000120) |
| `0x400c4a03` | a table of 136 register writes `{offset, value, changed}`: the changed ones are written, entry 0 (the start) last |
| `0x80044a02` | wait up to 1 s for the picture: the status back, bit 0 done, bit 1 an error (bits 2..6 which), bit 7 timed out |

The encoder is at 0x21100000, IRQ 78.

Only the owner can give the encoder back and gate its clock (`-EPERM` otherwise).
