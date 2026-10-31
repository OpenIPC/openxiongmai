# libxmisp

An open ISP for Xiongmai SoCs (MIT). It is written clean-room: from traces
of the vendor daemon (`tools/xm-isp-trace`), the register map and the
hardware's behaviour, never from the vendor's code. `tools/xmispd` is the
daemon on top of it.

XM510 only so far:
- `xmisp_start()` brings VI, the sensor and the ISP up from a sensor script
  and applies that script's fixed 3A state;
- `xmisp_frame()`, once a frame, runs auto-exposure (M2) and auto white
  balance (M3);
- the colour matrix is the sensor's own calibration, with Cb/Cr gain 1
  (M4);
- noise reduction and sharpening follow the gain (M4).

## Layers

- **`ae.c`: auto-exposure, independent of the SoC and the sensor.** It
  takes a frame's mean luma and aims lines × gain at target / mean, by the
  square root of that ratio a step (at most ×8). It uses whole lines before
  gain; the fraction of a line goes to gain, so a bright scene wanting 1.5
  lines gets 1 line at 1.5×. A proposal that rounds to the exposure already
  set isn't written. Its state changes only with what the sensor took,
  which after a failed write is what it reads back. After each change it
  waits `interval` frames for the change to show in the statistics.
  With anti-flicker on (`xmisp_set_antiflicker()`, 50 or 60 Hz), the lines
  are whole flicker periods (10 or 8.33 ms), at least one, and gain makes
  up the rest. Light too bright for one period is overexposed, not banded.
  Under a 200 W LED lamp, with the lines left free under one period,
  88 lines (1.6 ms) gave rolling bands of 8.6% at 186.4 rows; at one period
  the rows varied by 0.15%, at noise level, and the picture was 1.45× the
  target brightness. `make test` runs it on the host against a simulated sensor:
  convergence from far off either way, steadiness, and the limits.
- **`awb.c`: auto white balance, independent of the SoC.** It does
  grey-world over the pixels that look grey under the gains already set. The
  SoC gives their count and mean Cb and Cr; R is scaled by exp(−Cr / step)
  and B by exp(−Cb / step), within the sensor's range. The grey window stays
  wide while the gains are far off and narrows once near, so only grey
  pixels count; it widens again only when well off (twice the error that
  narrows it) or when the narrow window finds nothing. With too few grey
  pixels the gains hold.

  Grey-world can settle where some surfaces only look grey, so a search
  chooses where it starts from: a 5×5 grid of gains, log-spaced over the
  sensor's range, measured three at a time in the spare statistics
  channels with the narrow window. The point with the most grey pixels
  wins if it has 1.25× the count at the gains set. It runs at start, every
  250 frames, when the narrow window twice in a row finds nothing, and
  when the wide one does. That also ends a stall between two populations,
  where the wide window's mean looks grey and the narrow one is empty.
  `make test` also runs it on the host, with the scene the camera measured.
- **`tune.c`: tuning by gain, independent of the SoC.** It gives a strength
  from 0 to 1, log-linear in gain between two gains, in steps, and
  interpolates register values by it. `make test` runs it on the host.
- **`sensors/<vendor>_<model>/sensor.c`: what a sensor's 3A needs.** That's the exposure range,
  the gain ceiling, the white balance gain range, its colour matrix, and
  reading and setting exposure and gain in its register encoding.
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
  0x80 and above lands within 5% of the chart's grey in daylight. Under
  4000 K bulbs, at 0x80 the warm surfaces still counted nearly as many
  (3869) at their too-blue gains as the real grey (4164), and a start in
  the dark settled there both times tried (greys R/G 0.41, B/G 1.84). At
  0xa0 the real grey has a single peak (2143 to 40), so `xmisp` counts luma
  0xa0 and above, and the search finds that peak. When too few pixels are
  bright, it holds its gains: in the first frames, before AE has the
  picture bright, and in a dark scene. Falling back to 0x40 there settled
  in the cyan trap on the camera.

## Sensor scripts

The XM510 start-up is the same list of steps for every sensor the vendor
daemon supports, with the sensor's parts in five places. `xm510_start.c`
is that list, the SoC's template, with an `XMISP_SENSOR` slot for each
part:

| Step | Whose |
|---|---|
| MMZ, VI and region resets; the first PLL setup | SoC |
| chip ID probe | sensor: `probe` |
| the sensor's clock (system controller) | sensor: `pll` |
| VI store setup | SoC |
| VI channel ioctls and the VI writes after them | sensor: `vi` |
| sensor register writes | sensor: `init` |
| ISP writes, the "Isp" MMZ block | SoC |
| sensor register writes | sensor: `post` |
| ISP writes | SoC |

A script (`script.h`) is one sensor and mode:
- its I2C bus: the address, and 1 or 2 bytes of register and data;
- its five parts;
- the ISP values it needs other than the template's, mostly frame size,
  black level and tuning, along with the "Isp" block's size if it differs;
- a fixed 3A state applied once after the start-up;
- its 3A driver (`struct xmisp_sensor`), if it has one.

An override replaces the register's last write in the template. Where the
template writes a register twice, the first write is a default that is the
same for every sensor. `xmisp_expand()` puts a script's parts and values
into the template. `xmisp_start()` runs that for each script in
`xmisp_scripts` (`scripts.c`) until a probe's chip ID matches. A sensor that
doesn't answer at all before its ID is read counts as a miss too.

Registers that hold addresses in the MMZ block are written relative to
wherever MMZ puts it, in 8-byte units from DDR (`0x80000000`).

Before the first script, `xmisp_start()` turns 3D noise reduction off and
waits two frames. A daemon killed without its way out leaves 3D-NR writing
into the "Isp" block, and the MMZ and VI resets would hand that memory on. A
start that fails part way turns 3D-NR off again.

`tools/xm-isp-trace/extract.py` generates the template and the scripts
from traces of the vendor daemon, and checks every value: the template with
a script's parts and values must give that trace's start-up. Run
clang-format (LLVM) on what it writes. `make test` checks the result: every
script expands and probes a chip ID, and the JX-H62's start-up is step for
step the one its trace gave before the template was split out
(`test/h62_720p_flat.c`). `sensors/README.md` covers adding a sensor.

| Script | Sensor (I2C, ID) | Mode | 3A driver | Source |
|---|---|---|---|---|
| `sensors/soi_jxh62/jxh62_720p.c` | JX-H62 (`0x30`, `0xa062`) | 1280x720 | yes | lab XM510 |
| `sensors/soi_jxh42/h42_720p.c` | JX-H42 (`0x30`, `0xa042`) | 1280x720 | from the trace | emulated |
| `sensors/soi_jxh65/h65_720p.c` | JX-H65 (`0x30`, `0x0a65`) | 1280x720 | from the trace | emulated |
| `sensors/smartsens_sc1235/sc1235_720p.c` | SC1235 (`0x30`, `0x1235`) | 1280x720 | from the trace | emulated |
| `sensors/smartsens_sc2145h/sc2145h_720p.c` | SC2145H (`0x30`, `0x1245`) | 1280x720 | from the trace | emulated |
| `sensors/smartsens_sc2235/sc2235_1080p.c` | SC2235 (`0x30`, `0x2235`) | 1136x1080 | from the trace | emulated |
| `sensors/onsemi_ar0130/ar0130_720p.c` | AR0130 (`0x10`, `0x2402`) | 1280x720 | yes | 53X13 XM510 board |
| `sensors/omnivision_ov9750/ov9750_720p.c` | OV9750 (`0x36`, `0x9750` at `0x300b`) | 1280x720 | from the trace | emulated |
| `sensors/omnivision_ov9732/ov9732_720p.c` | OV9732 (`0x36`, `0x9732` at `0x300a`, `0x302a` = `0xa0`) | 1280x720 | from the trace | emulated |
| `sensors/imagedesign_mis1002/mis1002_720p.c` | MIS1002 (`0x30`, `0x2f` = `0x02`) | 1280x720 | no | emulated |
| `sensors/smartsens_sc1135/sc1135_720p.c`, `sc1135_960p.c` | SC1135 (`0x30`, `0x3008`/`0x3005` = `0x60`/`0x22`) | 1280x720, 1280x960 | from the trace | emulated |
| `sensors/smartsens_sc1235/sc1235_960p.c` | SC1235 | 1280x960 | from the trace | emulated |
| `sensors/soi_jxh65/h65_960p.c` | JX-H65 | 1280x960 | from the trace | emulated |

A sensor's modes are separate scripts, each with its picture size
(`width`, `height`: VI channel 0). `xmisp_start_mode()` tries the scripts
of the lines asked for first and the rest after, so a sensor without that
mode starts in its other one. The vendor daemon takes the mode from
`Profile=960` in its own configuration, `/mnt/mtd/Config/Json/format`
(`KEY=VALUE` lines; it starts SC1135, SC1235 and JX-H65 in 960P with it,
OV9750 stays 720P), which majestic writes from `video0.size`; xmispd reads
the same file.

The mode is VI channel 0's size; the JX-H62's runs at 25 fps, and the
others' frame rate hasn't been measured. The fixed state
is one frame of the vendor's 3A in the light of the capture, and AE and AWB
start from it.

"Emulated" means traced from the vendor daemon in qemu-bench (`xm510`
machine, a lab camera's flash dump, the generic `xm-sensor` showing the
sensor's chip ID). None of the emulated ones has run on hardware yet, and without a
3A driver (MIS1002) xmispd holds the fixed state. Two kinds of value can't be traced
there:
- the ISP values the daemon computes from what it reads of the hardware
  (`0x4005104c`, `0x40040148`), which keep the JX-H62's;
- sensor writes computed from a sensor read, which the model answers with
  what was written, or 0.

## 3A drivers from emulated traces

Every emulated sensor but the MIS1002 has a 3A driver taken from its
trace, untested on hardware:
- **Gain encoding.** Emulated, the statistics read 0, and the vendor's AE
  steps exposure to its longest and gain up by about 1.37x every other
  frame. On the two sensors whose gain was measured it passes through the
  same gains (JX-H62 `0x06`, `0x0e`, `0x14` … `0x4f`: 1.375x, 1.875x, 2.5x …
  31x; AR0130 within 3%), so a sensor's ramp gives its codes for known
  gains, and an encoding has to reproduce them. JX-H42 and JX-H65 write the
  JX-H62's codes; SC1235, SC2145H and SC2235 a coarse doubling in `0x3e08`
  (`0x03`, `0x07`, `0x0f`, `0x1f`) times `0x3e09`/16, exactly; OV9732
  `0x350b`/16, exactly; OV9750 `0x3508`/`0x3509` /128 (its ramp started at
  1.05x, and goes on 1.37x a step); SC1135 bits 6:4 of `0x3e09` as
  doublings times 1 + [3:0]/16 (three of its codes are the known gains,
  the rest within a fine step). `test/sensor_test.c` checks every driver
  against its ramp. The MIS1002's `0x02`/`0x03` values don't rise with the
  gain in any reading tried, so it has none.
- **Gain ceiling:** where the vendor's analogue gain ends (15.5x or 16x);
  past it the vendor used digital gain, which the drivers don't.
- **Settings by gain.** The SmartSens sensors and the JX-H65 get sensor
  writes with the gain (SC1235: `0x3631`, `0x3301`, `0x3633`, and `0x5781`,
  `0x5785`; JX-H65: `0x1f`), different per mode for the SC1235. They're
  known at the ramp's gains only: they change with the coarse step, or a
  range starts at the lowest gain that showed them.
- **Exposure:** the vendor's longest, two or four lines short of the frame
  the script sets (1/16 lines on the SmartSens and OmniVision parts). The
  line time assumes 25 fps, which the daemon logs for SC2145H and SC2235.
  Where the script leaves the frame length at the chip's (SC1235 960p,
  SC2235) it's the longest exposure plus four.
- **White balance range:** the one the vendor's AWB searches under
  emulation. On the JX-H62 and AR0130 that is wider than on the hardware,
  and a range too wide let the AR0130's AWB settle on an orange chair, so
  these are the first thing to narrow on a camera.
- **Not set:** colour matrix (the script's stays), AE target (the SoC's),
  noise reduction (the JX-H62's schedule).

Run in qemu-bench under majestic with the AE statistics preset to a dark
picture, xmispd starts each of them with AE and AWB on, and AE takes
exposure and gain to the ceiling with the vendor's codes.

## JX-H62

These were measured on the lab XM510 with `xmispd` holding the rest fixed:
- **Exposure:** lines in registers `0x02` (high) and `0x01` (low). The
  statistics grow linearly with it (160 lines gave a mean of 12.7, 320 gave
  26.1).
- **Gain:** register `0x00` gives 2^[6:4] × (1 + [3:0]/16). Each step of
  bits 6:4 doubled the mean, and `0x7x` measured the same as `0x6x`.
- **Frame:** the 720p25 script sets 750 lines (`0x23`/`0x22`); VI counts 750
  too (`VI_VH_ALL` `0x02ee0780`), at 25 fps, so a line is 53.33 µs. AE uses
  up to 746 lines, and gain up to 16 for now. Under anti-flicker that's 3
  periods of 10 ms (563 lines) or 4 of 8.33 ms (625).
- **White balance:** in daylight the chart's grey needs R 1.56 and B 2.19
  relative to G. Under RGBCCT bulbs it needs R 1.17 / B 2.45 at 3000 K, up to
  R 1.78 / B 1.82 at 6000 K. AWB tracks the red gain to within 2% over that
  range, and the blue 5–10% high (see `tools/xmispd/README.md`). It keeps to
  the range the vendor daemon's AWB searches, R 0.75–2.8 and B 1.1–3.6.
- **Colour matrix:** calibrated against the colour checker in daylight with
  `tools/xm-isp-tune/colorchecker.py`, scoring mean dE 10.4–10.8. The
  vendor daemon's matrix and Cb/Cr gain (2.05×), traced in the same light,
  score 16.9 and clip three patches. The
  matrix is written to both CSM banks (`0x40061040`, `0x40061080`, switched
  at `0x40061000`), and Cb/Cr gain (`0x40076004`/`08`) is set to 1,
  `0x100`. It's one illuminant's: a warm-light calibration to interpolate
  by colour temperature, and easing it at high gain, where it amplifies
  noise, are still to do.

## Noise reduction and sharpening (XM510)

The vendor daemon moves eight registers with gain (traced under the lab's
dimmable bulbs): 2D NR `BASE`, `THDH`, `COEF`, `DELTA_THD` (`0x40020000`..),
3D NR `YCTRL_ERR70` (`0x400400d8`) and sharpening `KD`, `KM0`, `KH0`
(`0x40075018`, `0x4007514c`, `0x40075174`). One at a time, at 16× gain, with
the daemon frozen:
- **2D NR:** `BASE` and `THDH` set its strength. `COEF` and `DELTA` alone
  change nothing.
- **3D NR:** bits 3:0 of `YCTRL_ERR70` are a strength, 0 off. 6 halves the
  noise; 10 and up cost detail for little more.
- **Sharpening:** it amplifies the noise, and halving it halves the noise.

`libxmisp` keeps the script's values up to 4× gain and moves to a high-gain
set by 16×, log-linear in eighths. The high set was chosen by measurement:
noise in the chart's grey patches against the detail of the chair's mesh,
with the noise subtracted from the detail.

| Gain | Strength | Temporal noise | Mesh detail |
|---|---|---|---|
| 3× | 0 | 2.1 (unchanged) | 26.5 (unchanged) |
| 8.5× | 0.5 | 1.9 (was 5.0) | 19.8 (was 25.7) |
| 16× | 1 | 1.5 (was 12.6) | 2.3 |

At 3× the high set would over-smooth (detail 26 to 15), which is why the
strength stays 0 there.

Chroma noise comes from the colour matrix and the Cb/Cr gain, which amplify
it. The XM510 has no separate chroma filter; turning `SHARPEN_CBCR_ENABLE`
off made chroma noise worse (5.1 to 7.7). So the same strength also eases
the matrix up to a quarter of the way towards identity and lowers Cb/Cr to
0.8. At 15× gain, with AWB converged under dimmed bulbs:

| At 15× | Chroma noise | Mean dE |
|---|---|---|
| matrix as calibrated, Cb/Cr 1 | 5.4 | 20.6 |
| Cb/Cr 0.7 alone | 3.1 | 23.2 |
| **matrix eased ¼, Cb/Cr 0.8** | **2.8** | **23.3** |
| matrix eased ½, Cb/Cr 0.8 | 2.1 | 25.3 |
| identity, Cb/Cr 1 | 1.7 | 27.7 |

In the running daemon at 15× the result is chroma noise 2.9 (was 5.1) and dE
23.5. At 3× the matrix and Cb/Cr are untouched.

## Black level (XM510)

The black level offsets (`BLACKLEVEL_OFFSET_R/G/B`, `0x40004000`/`04`/`08`)
are subtracted from the sensor data in 12-bit units, before the white
balance gains: a step moves the AE statistics' 8-bit mean by about 0.08.
The sensor's dark level was measured in daylight: with the daemon frozen,
the exposure at 2 and 4 lines (53 µs a line, so almost no light) and the
offset swept, the mean falls linearly until the offset passes the dark
level, and the two exposures extrapolate to no light.

| Sensor | Dark level | Script's offset |
|---|---|---|
| JX-H62 | `0x3b` at 1×, `0x32` at 16× | `0x45` |
| AR0130 | `0xa4`, `0xa3`, `0xa2` at 1×, 8×, 32× | `0xaa` |

The scripts crush the darkest level or so, which hides noise at low gain
and loses the shadows of a dark scene. A sensor driver's `black_high`
(the dark level at the high gain) is where the noise schedule's strength
takes the offset, from the script's. On the JX-H62 at 16×, in a night-like
picture (the exposure cut to 13 lines), the chart's darkest patches came up
3-4 levels (black Y 37 to 41) with the noise unchanged. The vendor daemon
does the same: `0x45` up to 12.5×, `0x30` at 23×.

## Tone curve (XM510)

The RGB gamma (`GAMMA60`, `0x40062040`.., two banks and a switch at
`0x40062000`, as the colour matrix) is flatter than sRGB in the midtones;
the luma gamma after it (`YGAMMA`, `0x40072040`..) is the identity, and the
vendor daemon only ever writes it so. With each curve given its own colour
matrix, fitted to the lab chart, the vendor's curve, sRGB and power curves
of 1/1.6 to 1/2.2 all give a mean dE of 9.8-10.5: the chart's light, uneven
across it, and the lens's fall-off outweigh the curve, so `libxmisp` keeps
the script's. Choosing a contrast needs a flat-lit chart, or a preference.

