# libxmisp

An open ISP for Xiongmai SoCs (MIT). It is written clean-room: from traces
of the vendor daemon (`tools/xm-isp-trace`), the register map and the
hardware's behaviour, never from the vendor's code. `tools/xmispd` is the
daemon on top of it.

XM510 only so far:
- `xmisp_start()` brings VI, the sensor and the ISP up from a sensor script
  and applies that script's fixed 3A state;
- `xmisp_frame()`, once a frame, runs auto-exposure (M2) and auto white
  balance (M3).

## Layers

- **`ae.c`: auto-exposure, independent of the SoC and the sensor.** It
  takes a frame's mean luma and aims lines × gain at target / mean, by the
  square root of that ratio a step (at most ×8). It uses whole lines before
  gain; the fraction of a line goes to gain, so a bright scene wanting 1.5
  lines gets 1 line at 1.5×. A proposal that rounds to the exposure already
  set isn't written. Its state changes only with what the sensor took,
  which after a failed write is what it reads back. After each change it
  waits `interval` frames for the change to show in the statistics. `make test` runs it on the host against a simulated sensor:
  convergence from far off either way, steadiness, and the limits.
- **`awb.c`: auto white balance, independent of the SoC.** It does
  grey-world over the pixels that look grey under the gains already set. The
  SoC gives their count and mean Cb and Cr; R is scaled by exp(−Cr / step)
  and B by exp(−Cb / step), within the sensor's range. The grey window stays
  wide while the gains are far off and narrows once near, so only grey
  pixels count. With too few grey pixels the gains hold. `make test` also
  runs it on the host.
- **`sensor_*.c`: what a sensor's 3A needs.** That's the exposure range,
  the gain ceiling, the white balance gain range, and reading and setting
  exposure and gain in its register encoding.
- **`xm510.c`: the SoC.** Device access, the script interpreter, and the
  AE statistics:
  - a 32-bin luma histogram at `0x40033080`, whose total is the pixels
    counted;
  - 7×7 zone sums from `0x40033100`, rows in pairs `0x40` apart, the second
    of a pair `0x1c` after the first.

  Zone total over histogram total is the frame's 8-bit mean, linear (before
  gamma). The vendor daemon holds it at about 37.5 in daylight and indoors
  alike, and so does `xmisp`.

  The AWB statistics, from experiments on the lab XM510:
  - they're of the raw picture times candidate gains, four channels each
    with its own R and B gains (`0x40010000`.., `0x40010010`..,
    `0x100` = 1). None of these reach the picture, and the picture's gains
    (`AWB_FRONTGAIN` R, G, B at `0x40015000`) don't reach the statistics;
  - per channel, four regions of {count, Cb sum, Cr sum} from
    `0x400140c0`, `0x40` apart, for the pixels inside a grey window
    (`0x40014048`/`4c`: Cb ± Cr limits, signed bytes min << 8 | max) and a
    luma range (`0x40014054`: max << 8 | min). Summed, they are the grey
    pixels' count and mean Cb and Cr, BT.601 with 9-bit signed
    coefficients at `0x4001300c`;
  - restricted to the colour checker's grey row (the window registers at
    `0x40014000`), grey-world on them lands within 2% of the gains that make
    those patches neutral in the picture.

  Over a whole room of wood, beige and an orange chair, the dim pixels that
  pass as grey pull grey-world to far too blue gains; the vendor daemon's
  AWB lands there too, and its picture is cyan. Counting only pixels of luma
  0x80 and above lands within 5% of the chart's grey, so `xmisp` does that,
  and falls back to the vendor's 0x40 when too few pixels are bright.

## Sensor scripts

A script (`script.h`) is the start-up for one sensor and mode as a list of
steps:
- MMZ, VI and region resets;
- system controller writes (the sensor PLL);
- sensor register writes, and the chip ID it must read;
- VI channel setup;
- the "Isp" MMZ block;
- ISP register writes, plus delays between them.

Registers that hold addresses in the MMZ block are written relative to
wherever MMZ puts it, in 8-byte units from DDR (`0x80000000`).
`xmisp_start()` runs each script in `xmisp_scripts` until one sensor answers
with its chip ID. A sensor that doesn't answer at all before its ID is read
counts as a miss too.

Before the first script, `xmisp_start()` turns 3D noise reduction off and
waits two frames. A daemon killed without its way out leaves 3D-NR writing
into the "Isp" block, and the MMZ and VI resets would hand that memory on. A
start that fails part way turns 3D-NR off again.

Generated scripts are clang-format (LLVM) output, so run clang-format after
`extract.py`.

`tools/xm-isp-trace/extract.py` generates a script from a trace:

```sh
extract.py trace.bin h62_720p "JX-H62 720p25" --fixed 500-510 \
    --sensor h62 -m libraries/isp/include/xm/xm510_isp.h > h62_720p.c
```

| Script | Sensor | Mode | Trace |
|---|---|---|---|
| `h62_720p.c` | JX-H62 (I2C `0x30`, ID `0xa062`), `sensor_h62.c` | 1280x720, 25 fps | lab XM510, vendor daemon under majestic |

The fixed state is one converged frame of the vendor's 3A in the light of
the capture, and AE and AWB start from it.

## JX-H62

These were measured on the lab XM510 with `xmispd` holding the rest fixed:
- **Exposure:** lines in registers `0x02` (high) and `0x01` (low). The
  statistics grow linearly with it (160 lines gave a mean of 12.7, 320 gave
  26.1).
- **Gain:** register `0x00` gives 2^[6:4] × (1 + [3:0]/16). Each step of
  bits 6:4 doubled the mean, and `0x7x` measured the same as `0x6x`.
- **Frame:** the 720p25 script sets 750 lines (`0x23`/`0x22`). AE uses up to
  746 lines, and gain up to 16 for now.
- **White balance:** in daylight the chart's grey needs R 1.56 and B 2.19
  relative to G. AWB keeps to the range the vendor daemon's AWB searches,
  R 0.75–2.8 and B 1.1–3.6.
