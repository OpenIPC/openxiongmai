# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Open-source reimplementations of Xiongmai (XM) IP camera SoC userspace libraries and kernel drivers, mostly XM530 (some XM510). The code runs on ARM camera boards. It cannot be run or tested on the host, and the repo has no test suite.

## Building

Every component is a standalone Makefile. Build from the component's directory with an ARM cross toolchain. The README gives the OpenIPC SDK recipe:

```sh
export PATH=/path/to/openipc_sdk/bin:$PATH
make CROSS_COMPILE=arm-openipc-linux-musleabi-
```

The default `CROSS_COMPILE` differs per Makefile, so pass it explicitly:
- `libraries/libsns_X50_XM530`: `arm-openipc-linux-musleabi-`
- `libraries/libsns_X123_XM530`, `libraries/libispapp_XM530`: `arm-xm-linux-`
- `libraries/libmpi`: `arm-linux-`, and the Makefile hard-sets `PATH` to `/opt/vtcs_toolchain/...`

Build outputs:
- The `libsns_*` and `libispapp_*` libraries build stripped `.so` files with `-fPIC`.
- `libmpi` builds the static `libopenmpi.a`.
- The kernel modules in `drivers/` (`hx280enc`, `xm_i2c`) are out-of-tree kbuild modules. `KDIR` defaults to `$(HOME)/projects/cameras/sdk/XM510/os/kernel/linux-xm510`, so override it: `make KDIR=/path/to/kernel`.

`libraries/Makefile` builds the libraries OpenIPC firmware ships for one SoC family: `make -C libraries CHIPARCH=xm530` builds `libsns_X50_XM530`, `libsns_X123_XM530` and `libispapp_XM530`, and any other `CHIPARCH` is an error. Firmware's `xiongmai-opensdk` package calls it with `$(TARGET_CONFIGURE_OPTS)`, which puts `CC`, `STRIP` and `CFLAGS` on the command line. That's why the library Makefiles use `override CFLAGS +=`. A plain `CFLAGS =` would be ignored, and the build would lose its include paths and defines. A library change reaches cameras only after the firmware package's pinned commit is bumped.

Most Makefiles build with `-Werror -Wall -std=c99`. `libsns_X50_XM530` dropped `-Wall` because the sources still contain unused variables. A new source file builds only after you add its `.o` to the Makefile's explicit `OBJECTS` list.

## CI

`.github/workflows/build.yml` runs two jobs. The first builds all four libraries with the OpenIPC `toolchain.xiongmai-xm530` (gcc 13, musl) and asserts the exported symbols and `NEEDED` entries. The kernel job is a matrix over xm510 (3.0.101) and xm530 (3.10.103). Each entry builds `xm_i2c` and `hx280enc` against the pinned `openipc/linux@xiongmai-<soc>` commit, using the firmware's `<soc>.generic.config` and its `general/package/all-patches/linux` patches. Without those patches, gcc 13 can't build the 3.0 kernel. All jobs are required status checks on `main`. Kernel, firmware and toolchain inputs are pinned in `build.yml`. OpenIPC rebuilds the toolchain tarballs in place, so a sha256 mismatch means the checksum needs updating. To reproduce CI locally, download the same pinned inputs. Add any new library or exported entry point to the workflow's checks.

## Architecture

### Two header families

- `libraries/isp/include/{isp,mpi,xm}`: vendor-style SDK headers. These cover the ISP/AE/AWB MPI, sensor callback structs, `Camera.h` with the `XM_SENSOR_CHIP` enum (`SENSOR_CHIP_*`), and per-SoC `xm5x0_isp.h` register maps. The sensor libraries and `libispapp` compile against these.
- `libraries/include/re_*.h`: reverse-engineered MPI headers (the `re_` prefix). Only `libmpi` uses them.

### `libmpi`: the MPI layer reimplemented

`libmpi` reimplements `XM_MPI_SYS_*`, `XM_MPI_VI_*`, `XM_MPI_VENC_*` and `XM_MPI_RGN_*` as raw `ioctl`s on vendor device nodes such as `/dev/mmz`. The ioctl numbers are magic constants recovered from the vendor blobs. Treat them as ABI and don't change them without evidence from the original binaries.

### Sensor libraries (`libsns_<family>_XM530`)

Each library supports several sensors and follows one pattern:
- **`<family>_cmos.c`** (`XAx_cmos.c`, `X50_cmos.c`): the dispatcher.
  - It holds the global state (`gSensorChip`, `gSnsDevAddr`, gain and shutter globals, `pfn_gainLogic`/`pfn_shutLogic`).
  - `sensor_register_callback()` switches on `gSensorChip`. Each case calls that sensor's `cmos_init_*_exp_function_<sensor>()`, points the AWB calibration, CCM, AGC table and AE defaults at the sensor's tables, and sets `pfn_sensor_getlist`.
  - It then registers the callbacks with `XM_MPI_ISP/AE/AWB_SensorRegCallBack`.
  - In `XAx_cmos.c`, `sensor_get_chip_in()` probes the I2C chip-ID registers and maps each ID to a `SENSOR_CHIP_*` value.
- **`<family>_sensor_ctl.c`**: I2C register read and write. It fills `I2C_DATA_S` and calls `XM_I2C_Ioctl`, using the globals `gSnsDevAddr`, `gSnsRegAddrByte` and `gSnsRegDataByte`.
- **`<sensor>_cmos.c`**: the per-sensor ISP/AE/AWB callbacks, gain and shutter logic, and tuning tables.
- **`<sensor>_sensor_ctl.c`** (present for some sensors): the init register table, which `sensor_getlist_<sensor>()` returns.

To add a sensor, make these changes:
1. Add its `_cmos.c` and, if it needs one, its `_sensor_ctl.c`.
2. Add a `case` to the dispatcher's `sensor_register_callback`.
3. Add its chip ID to the probe.
4. Add a `SENSOR_CHIP_*` value in `Camera.h` if it doesn't exist yet.
5. Handle it in the `switch` statements in `libispapp_XM530/isp_sample.c` that cover the sensor (for SC3335, see around lines 1793 and 2341).
6. Add its objects to the Makefile.

Some of the per-sensor code sits inside `#if (defined SOC_SYSTEM) || (defined SOC_ALIOS)`. The Makefiles define `SOC_SYSTEM`, `CHIPID_XM530` and `AWB_ALGO_V2`.

### `libispapp_XM530`

`libispapp_XM530` is the ISP application layer. It handles PLL and VI window setup per resolution, product info, the ISP run loop (`ISP_Run_SocSystem`), the UART/MCU link and VDA. It contains the I2C implementation (`i2c.c`) that the sensor libraries call.

### `tools/xmvenc-test`

`tools/xmvenc-test` is a static test program for the H.264 encoder, a Hantro 8290 laid out like Rockchip VEPU1 with the differences recorded in #13. It encodes IDR and P frames (CAVLC or CABAC, fixed QP) from a synthetic picture through `/dev/h264` and `/dev/mmz`, with majestic stopped. With `-v <chn>` it encodes straight from VI capture buffers. Majestic disables frame delivery (`_IOW('V', 2/3)` enable/disable) when it exits, while capture keeps running. In JPEG mode (`-j`) software writes the header up to SOS and the hardware writes the scan and EOI. The quantisation tables at `0x100`/`0x140` take effect only if `0x038` already selects JPEG when they are written, because other formats' tables share those offsets. Reconstruction and reference buffers ping-pong, and the CABAC context table (`cabac_tbl.c`, Apache-2.0, from Rockchip MPP) goes in MMZ at `0x0cc`. `mmz.ko` takes name/size/phys records: 44 bytes on XM530 (`0xC02C4D01` alloc, `0x802C4D03` get) and 40 bytes on XM510 (`0xC0284D01`, `0x80284D03`). Get-by-name fills size before phys on both. The module returns 0 even for commands it rejects, so a zero address means failure. `libmpi` detects the record size on the first call. The VI frame record has the same split: 32 bytes on XM530 (`0xC0205604` get, `0x40205605` release, with a 64-bit PTS) and 24 on XM510 (`0xC0185604`, `0x40185605`). `libmpi` detects that on the first get that returns a frame. On XM510 majestic runs the vendor `isp` daemon, and VI stops producing frames when it stops.

### Kernel drivers

- `drivers/xm_i2c`: a misc-device stub. Its read, write and ioctl handlers are placeholders that only print "not supported".
- `drivers/hx280enc`: the Hantro 6280/7280/8270/8290 video encoder driver. It implements the `/dev/h264` ioctl ABI of the vendor `xm530_h264.ko`, including the Xiongmai `'k'` 9-12 extensions (`HX280ENC_IOCWAITDONE` and the `/proc` statistics records). The decompiled `H264ProcType` and `VencInfo` structs don't match the vendor record sizes, so the driver stores those records as raw bytes.

## Conventions

- Source encoding and line endings are mixed. Many vendor-derived `.c` files use CRLF, and some contain GBK/ISO-8859 comments (for example `XAx_cmos.c`). Keep each file's existing line endings. Filter with `tr -d '\r'` when grepping for line-anchored patterns.
- Formatting differs by area:
  - Userspace library code uses clang-format LLVM style (2-space indent).
  - `drivers/*` have a `.clang-format` with kernel style: tabs, 8-wide, 80 columns.
- Commit messages are prefixed with a sensor or component tag when one applies, for example `[SC3335] Fixes`.
