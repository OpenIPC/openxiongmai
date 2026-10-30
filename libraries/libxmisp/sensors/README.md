# Sensors for the open XM510 ISP

One directory per sensor, `<vendor>_<model>/`, containing:
- a script per mode, `<model>_<mode>.c`. It's generated: the sensor's
  parts of the XM510 start-up and its ISP values (see `../README.md`,
  "Sensor scripts");
- optionally `sensor.c`, its 3A driver (`struct xmisp_sensor` in
  `../script.h`). That's the exposure range and gain ceiling, how its
  registers encode exposure and gain, its white balance range and colour
  matrix. Without one, xmispd starts the sensor and holds the script's
  fixed 3A state.

| Directory | Modes | 3A driver | Tested on hardware |
|---|---|---|---|
| `soi_jxh62` | 720p, 25 fps | yes | yes (lab XM510) |
| `soi_jxh42` | 720p | no | no |
| `soi_jxh65` | 720p | no | no |
| `smartsens_sc1235` | 720p | no | no |
| `smartsens_sc1245` | 720p | no | no |
| `smartsens_sc2235` | 1080p (1136x1080) | no | no |
| `onsemi_ar0130` | 720p | no | no |
| `omnivision_ov9750` | 720p | no | no |
| `omnivision_ov9732` | 720p | no | no |
| `imagedesign_mis1002` | 720p | no | no |

## Clean room

The rules are the same as for the rest of `libxmisp`:
- every value comes from a trace of the vendor daemon, from a measurement
  on the hardware, or from a public datasheet;
- no vendor code, and no sensor driver from an SDK or from another
  project, including GPL ones such as openhisilicon's;
- chip IDs are hardware facts, so ipctool's table can be used to find
  them.

## Adding a sensor

### 1. Trace the vendor daemon

The best source is a camera running the vendor daemon with the sensor,
under majestic. `tools/xm-isp-trace/README.md` shows how to bind-mount the
tracer over `/usr/bin/isp`. Let it run for a few hundred frames in steady
light.

Without the camera, if the XM510 vendor daemon supports the sensor, trace
it in qemu-bench instead:
- boot an XM510 flash dump, from any camera with a set-up OpenIPC, on the
  `xm510` machine;
- show the daemon the sensor's chip ID with the generic `xm-sensor` model
  (`sensor=xm-sensor,sensor-addr=…`, `-global xm-sensor.regs=…`; see
  qemu-bench's `docs/xiongmai.md`).

The daemon logs `[X1x]<name> sensor init OK` when it takes the sensor. With
`sensor=none` its I2C traffic in the trace shows the probes it tries. The
sensors it knows are named in its log strings (`strings /usr/bin/isp`);
their chip IDs come from public tables (ipctool) or a sweep of the probe in
QEMU, and a probe that reads one more register once the ID matches (OV9732:
`0x302a`) shows that in the `xm-sensor` log.

### 2. Generate the script

Generate it against the template, `xm510_start.c` (or the lab JX-H62
trace it was made from):

```sh
extract.py -m libraries/isp/include/xm/xm510_isp.h \
    libraries/libxmisp/xm510_start.c trace.bin \
    sc2235_1080p "SC2235 1080p" --fixed 300-310 \
    [--ignore 0x4005104c,0x40040148]   # for an emulated trace
clang-format -i --style=LLVM libraries/libxmisp/sensors/smartsens_sc2235/sc2235_1080p.c
```

It fails if the trace doesn't fit the template: a step out of order, a
register write other than the last that differs, sensor writes on two
buses. That's a sensor or mode the template doesn't cover yet. Send the
trace with the issue.

### 3. Register it

Add the object to `SENSORS` in `../Makefile` and the script to
`../scripts.c`, in probe order:
- sensors at the same address with 8-bit registers come before those with
  16-bit registers, as the vendor probes them;
- the export check in `.github/workflows/build.yml` needs the new symbol.

Then run `make -C libraries/libxmisp test`.

### 4. 3A driver

Measure these on the hardware, or take them from the datasheet:
- the exposure registers and the line range;
- the gain register's encoding and its steps. The JX-H62's was found by
  stepping the register and reading the AE statistics (`../README.md`,
  "JX-H62");
- the line time of the mode (`line_ns`), for anti-flicker;
- the white balance range, and a colour matrix from
  `tools/xm-isp-tune/colorchecker.py`.

## What an emulated trace can't give

- **Values the daemon computes from hardware reads.** The emulated ISP and
  sensor answer 0, or what was written. `0x4005104c` and `0x40040148` are
  known cases, and `--ignore` keeps the reference's values for them. A
  sensor write computed from a sensor read is replayed with the emulated
  value.
- **Probe reads that came back 0.** In the probe they are replayed
  unchecked, so only the chip ID registers are checked.
- **The fixed 3A state.** It is the vendor's 3A on an emulated test
  picture, a starting point only.

A sensor traced this way is untested until someone runs it on a board.
Please report what you see, good or bad.
