# xm_v200_i2s

`xm_v200_i2s` is the V200 series' audio input and output, at `/dev/ai` and `/dev/ao`, and builds as `open_i2s.ko`. It replaces the vendor `i2s.ko`.

The codec talks I2S to three blocks: common setup at 0x10065000, input at 0x10066000 (IRQ 48) and output at 0x10067000 (IRQ 49). Each interrupt moves 160 16-bit samples between a block's FIFO and a FIFO in memory: 5 KB of input read from `/dev/ai`, and 2.5 KB of output (5 KB at 16 kHz) written to `/dev/ao`. Reads and writes wait unless `O_NONBLOCK`.

| ioctl | |
|-------|-|
| ao `0x80044105` | the bytes in the output FIFO |
| ai `0x40044107` | the 3A state (AEC, ANS, AGC), shown in `/proc/umap/audio` |
| ai `0x80844109` | the capture settings `/proc/umap/audio` takes (`save_chose=`, `save_path=`) |
| ai `0x40044110` | XM535D1/D2, XM232: the sample rate, the clock set for it |
| ai `0x80044108` | XM232: the sample rate |

`rate=` (8000, 16000, 32000) sets the I2S clock to 256 fs from its PLL. `gpio=N` drives that GPIO high while `/dev/ao` is open, for a speaker amplifier. Build with `I2S_CHIP=v200` (XM530V200, XM533), `xm535`, `xm535w1` or `xm232`.

XM232 (`I2S_CHIP=xm232`): the input and output blocks only (no common block at 0x10065000 to set up), on IRQs 45 and 46. Its I2S clock is PLL C or D (SYS +0x30 bits 17-18, +0x18/+0x1c or +0x20/+0x24) over SYS +0x64's top and bottom bytes; at load and on `0x40044110` it is set for the rate as its `i2s.ko` does (2.048 MHz for 8 kHz, 4.096 MHz for 16 kHz, a clock other than those back to `0x19191918`), and the rate is then taken from the clock. It needs the audio codec: without `get_acodec_status()` (`open_acodec`, `ACODEC_CHIP=xm232`) it does not load.
