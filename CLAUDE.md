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

### `libxmvenc` and `tools/xmvenc-test`

`libraries/libxmvenc` (static `libxmvenc.a`, MIT, so closed applications such as majestic can link it) drives the H.264/JPEG encoder, a Hantro 8290 laid out like Rockchip VEPU1 with the differences recorded in #13. Keep it MIT: don't copy GPL code (`drivers/`, `libmpi`) into it. `cabac_tbl.c` there stays Apache-2.0. `tools/xmvenc-test` (MIT too) is a static test program on the library. It encodes IDR and P frames (CAVLC or CABAC, fixed QP or frame-level rate control, constant bitrate or, with `attr.vbr`, variable: unspent bits are not banked, so an easy scene settles at `qmin` and the bitrate is a ceiling) from a synthetic picture through `/dev/h264` and `/dev/mmz`, with majestic stopped. Pipes (`pipe.c`, with VI capture in `vi.c`) run a channel from VI on a thread of their own, queueing frames in the stream buffer as a ring. The stream limit register `0x060` counts 8-byte units, and the encoder stops writing exactly there, with "buffer full". The library chooses a chip profile (vendor constants, buffer sizes) by the max picture width in `0x0fc[11:0]`: 1280 on XM510, 2688 on XM550. On XM510 the library polls the status register for completion, because `xm510_h264.ko` has no `'k'` 12 wait, and falls back to majestic's idle MMZ blocks by name. With `-v <chn>` the tool encodes straight from VI capture buffers. Majestic disables frame delivery (`_IOW('V', 2/3)` enable/disable) when it exits, while capture keeps running. In JPEG mode (`-j`) software writes the header up to SOS and the hardware writes the scan and EOI. The quantisation tables at `0x100`/`0x140` take effect only if `0x038` already selects JPEG when they are written, because other formats' tables share those offsets. Reconstruction and reference buffers ping-pong, and the CABAC context table (from `cabac_tbl.c`) goes in MMZ at `0x0cc`. `mmz.ko` takes name/size/phys records: 44 bytes on XM530 (`0xC02C4D01` alloc, `0x802C4D03` get) and 40 bytes on XM510 (`0xC0284D01`, `0x80284D03`). Get-by-name fills size before phys on both. The module returns 0 even for commands it rejects, so a zero address means failure. `libmpi` detects the record size on the first call. The VI frame record has the same split: 32 bytes on XM530 (`0xC0205604` get, `0x40205605` release, with a 64-bit PTS) and 24 on XM510 (`0xC0185604`, `0x40185605`). `libmpi` detects that on the first get that returns a frame. On XM510 majestic runs the vendor `isp` daemon, and VI stops producing frames when it stops.

### H.265 (`libxmvenc/hevc.c`, `tools/xmhevc-test`, #27)

The XM530/XM550 H.265 encoder is an Arm Mali-V550 (`hw_version 0x56550002`, registers at `0x21000000`). It runs firmware `hevcenc.fwb`, which speaks host protocol 2.3, behind the vendor `mve_base.ko`/`mve_rsrc.ko` on `/dev/mv500`. `libxmvenc` drives it as codec `XMVENC_H265` (`hevc.c`), with one session per channel, and `tools/xmhevc-test` is the minimal raw session. `libxmvenc/mvb.h` holds our own definitions of that interface:
- **Commands:** there are two ioctls. `0xC008FB00` takes `{cmd, size}` plus the payload, and the reply `{status[2], size, data}` overwrites the same buffer. `0xC0A0FB01` is a 160-byte statistics poll.
- **Settings:** `SET_PARAMETER`/`SET_CONFIG` carry raw firmware messages: 1010 is the set-option message and 3003 the buffer-param message.
- **Buffers:** a buffer's handle is its user VA sign-extended to 64 bits. Output data starts at the descriptor's `offset`, about 2 KiB in, not at offset 0.
- **Session readiness:** configs fail with status 8 until the session is first switched in (event 1), so take events and retry.
- **Per frame:** the input is mapped by physical address through `/dev/mem`, and the driver resolves the VA. Input and output (a slice of the stream ring) are registered, queued and unregistered. NV12 is one plane, so the chroma must follow the luma.
- **Headers and GOP:** VPS/SPS/PPS come once, as an output with flag `0x80`; the library caches them and puts them before every IDR. An IDR request is a `GOP_RESET` buffer param. The P-frame count is read as 16 bits and `0xffff` is refused, so it is capped at 32767.

The protocol numbers match Arm's GPL-2.0-only `mve_protocol_def_v2.h`. Don't copy that header or `mvx` code in: define only what a trace shows, under our own names.

### `tools/xm-isp-trace` (open XM510 ISP, M0)

`isptrace.so` (MIT) is an `LD_PRELOAD` tracer for the XM510 vendor `isp` daemon, the observation step towards a clean-room open ISP: write the open one from traces, the register map and hardware behaviour only, never from the daemon's code. It logs `/dev/isp` register traffic, every other ioctl with its argument, stores to `/dev/mem` register pages (handed out read-only and trapped), file and queue activity, and the frame-start SIGIO, and `decode.py -m libraries/isp/include/xm/xm510_isp.h` names the registers. What the traces show of the daemon (JX-H62, 720p25):
- **Majestic's side:** the daemon is majestic's child, found on `PATH` (a bind mount over `/usr/bin/isp` takes its place), with its stdout and stderr on a pipe whose lines land in majestic's log; majestic logs "The ISP daemon did not come up within 15s" if it doesn't start, and stopping majestic sends it SIGTERM. Its SysV queues (`/tmp/ispmsg`) carry nothing (`ipcs`: no sender). Killing only the daemon leaves VI capturing (on `open_vi`); majestic gives up ("ISP fail, breaking").
- **Start-up (about 3.4 s):** MMZ reset, VI init, region reset; the sensor PLL through the system controller (`0x2000002c` unlock, `0x2000000c`/`0x10`/`0x14`, lock), twice; chip ID over I2C (`0x30`, registers `0x0a`/`0x0b` = `0xa062`); VI channel setup; about 80 sensor writes; about 495 ISP writes (colour correction, sharpening, gamma, 3D-NR, AWB); an MMZ block "Isp" of `0x270000` mapped through `/dev/mem`.
- **I2C:** `/dev/xm_i2c` command 1 writes and 3 reads `{dev, reg, reg_bytes, data, data_bytes}`; writes leave `dev` as stack garbage, so the driver keeps its own.
- **Per frame (in the SIGIO handler):** reads of the AWB zone statistics (`0x400140c0`.., 4 channels) and 81 words at `0x400330ec`-`0x400331d8` (not in the header: the AE statistics candidate); writes of the AWB front gains (`0x40015000`), Cb/Cr gain, the colour matrix (CSM, double-buffered: the idle bank, then `CSM_BUF_SW`), sharpening, 2D/3D-NR, and exposure and gain over I2C.
- **Exit (SIGTERM):** it drops its SIGIO handler, turns 3D-NR off (`0x40040000` = 0) and closes its devices.
- **Cost:** untraced the daemon takes about 5% CPU, traced about 50%.

### `libraries/libxmisp` and `tools/xmispd` (open XM510 ISP, M1-M4)

`libxmisp` (static `libxmisp.a`, MIT) is the open ISP and `tools/xmispd` (MIT, static) the daemon on it, a drop-in for the vendor `/usr/bin/isp`. Keep both clean-room as above, and keep GPL code and the vendor-derived headers in `libraries/isp/include` out: register names appear as comments only.
- **Sensor scripts (`script.h`):** the start-up for one sensor and mode as steps: MMZ/VI/region resets, system controller writes, sensor writes, the chip ID to expect, VI ioctls, the "Isp" MMZ block, and ISP writes. A script also carries a fixed 3A state applied once after it. `xmisp_start()` tries each script in `xmisp_scripts` until a chip ID matches. Registers holding addresses in the "Isp" block are written relative to its allocation, in 8-byte units from DDR (`0x80000000`).
- **Generated:** `tools/xm-isp-trace/extract.py` writes a script from a trace (gaps over 3 ms become delays; the colour matrix's two banks both get the fixed values). `h62_720p.c` is the JX-H62 at 1280x720, 25 fps. Regenerate scripts rather than editing them by hand. Add a new one to `xmisp_scripts` and to the Makefile.
- **XM510 interfaces:**
  - MMZ alloc is `0xC0284D01`. musl's `_IOWR()` takes a type, so `_IOWR('M', 1, 40)` encodes `sizeof(int)`, and `mmz.ko` answers "ioctl cmd error". Use encoded numbers for size-only commands.
  - The vendor sleeps 100 ms after the sensor reset and 3 × 1 s around the 3D-NR enable, so start-up takes about 3.4 s.
- **AE (M2):** three layers, so the XM530 can reuse the first:
  - `ae.c` knows neither SoC nor sensor: mean luma in, lines and gain out. It moves lines × gain by the square root of target/mean each step, lines before gain, then waits `interval` frames. Test it on the host with `make -C libraries/libxmisp test`; CI runs that.
  - `sensor_h62.c`: exposure in lines at `0x02`/`0x01`; gain at `0x00`, 2^[6:4] × (1 + [3:0]/16), with `0x7x` measuring the same as `0x6x`. Both were measured with the statistics, not taken from the vendor.
  - `xm510.c` reads the statistics: a 32-bin luma histogram at `0x40033080`, and 7×7 zone sums from `0x40033100` (rows in pairs `0x40` apart, the second `0x1c` in). Zone total over histogram total is the 8-bit linear mean. The vendor holds it at about 37.5, and so does `ae_xm510`.
- **AWB (M3):** in the same split as AE.
  - `awb.c`: grey-world on the pixels grey under the gains set. R is scaled by exp(−Cr/step) and B by exp(−Cb/step). The grey window is wide while far off and narrow once near.
  - The sensor driver gives the gain range: for the H62, the range the vendor's AWB searches.
  - `xm510.c`: the AWB statistics are of the raw picture times four candidate R/B gain pairs (`0x40010000`/`10`). Neither those nor the picture's `AWB_FRONTGAIN` (`0x40015000`) reaches the other. Each channel has four regions of {count, Cb sum, Cr sum} from `0x400140c0`, covering the pixels inside the grey window (`0x40014048`/`4c`, Cb ± Cr) and luma range (`0x40014054`).
  - **Luma floor:** only luma ≥ 0x80 counts. In the lab room (wood, beige, an orange chair) dim near-grey pixels pull grey-world, and the vendor's AWB, to a cyan picture. Tested against a colour checker's grey row. With too few bright grey pixels AWB holds, and widens first if it was narrow. Don't add a fallback to 0x40: it caught the first, still-dark frames and settled in the trap.
- **Anti-flicker (M4):** `xmisp_ae_cfg.flicker_lines` keeps the exposure to whole periods, at least one, with gain for the rest. Light too bright for one period is overexposed, not banded. Under the lab's 200 W LED lamp, free lines gave 8.6% bands 186.4 rows apart, which confirms the line time; one period gave 0.15%. `xmisp_set_antiflicker(isp, 50|60|0)` converts it with the sensor's `line_ns` (H62 720p25: 53,333 ns, from 750 lines at 25 fps, as `VI_VH_ALL` counts them). `xmispd` reads majestic's `isp.antiFlicker` from `/etc/majestic.yaml` at start and on SIGHUP. Majestic restarts itself, and so the daemon, when that key changes.
- **Noise reduction (M4):** `xm510.c` interpolates eight registers by gain through `tune.c` (`xmisp_strength`, log-linear in eighths from 4× to 16×): 2D NR `0x40020000/04/08/14`, 3D NR `0x400400d8` (bits 3:0 a strength, 0 off; the upper bits are kept), and sharpening `0x40075018/514c/5174`. Low end = the script's values, read at start. High end = picked by noise/detail measurements on the colour checker at 16× (`0x10, 0x2f, 0x82, 0x70`, strength 6, `0x26, 0x20, 0x17`). Measured: at 16× temporal noise 12.6 → 1.5; at 3× unchanged.
- **Chroma noise (M4):** the NR strength also eases the colour matrix towards identity (up to ¼, `CCM_EASE_HIGH`) and lowers Cb/Cr to 0.8 (`CBCR_HIGH` `0xcd`). The blended matrix goes into both CSM banks with the switch. At 15×: chroma noise 5.4 → 2.8, dE 20.6 → 23.3. `SHARPEN_CBCR_ENABLE` off makes chroma noise worse, so it stays on.
- **Colour (M4):** the sensor driver's `ccm` (rows sum 256) goes into both CSM banks (`0x40061040`/`80`, switch `0x40061000`), with Cb/Cr gain (`0x40076004`/`08`) at `0x100`. `0x100` is unity: tested by a blue-gain change through an identity matrix, which moved only B. The vendor moves its matrix and its Cb/Cr gain (`0x20e`–`0x250`, a 2–2.3× saturation boost) with the light. Calibrate with `tools/xm-isp-tune/colorchecker.py` against a colour checker. It decodes the stream full-range (the XM510 RGB→YCbCr has no offset; the stream doesn't say so), undoes `GAMMA60`, and fits for CIELAB error as an sRGB display shows it. H62 daylight: dE 10.4–10.8, against 16.9 for the vendor's traced matrix and gain in the same light. The script checks the camera's host key, reads the password from `SSHPASS`, verifies each write by reading it back, and uses local ffmpeg or a digest-pinned image. Matrix registers read back as 13-bit two's complement.
- **Daemon:** it prints the vendor daemon's last start-up line, `ispMsgReceive start!`, in the vendor's format (`\033[0;32;32mDEBUG: ...\n\033[m`); majestic starts video after it. It takes signals with `sigwaitinfo()` (SIGIO from `/dev/isp` with `O_ASYNC`) and on SIGTERM turns 3D-NR off. It runs `xmisp_frame()` on every SIGIO.

### `tools/xm-vi-hang`

`tools/xm-vi-hang` (MIT, static) reproduces the XM530/XM550 VI driver hang with raw ioctls only. A VI init (`_IO('V', 6)`) issued while the previous process left VI capturing freezes the board now and then, CPU and IRQs included; it's the "camera hangs at majestic start" problem.
- **Setup:** `isp_writes.h` holds the ISP/MIPI writes an SDK start makes after VI init (IMX335, 1080p), recorded from `/dev/isp` `_IOW('I', 1, 12)`.
- **MMZ reset:** VI channel setup fails with EPERM unless MMZ was reset (`_IO('M', 0)`), because VI takes its buffers from MMZ.
- **Results:** disabling channels doesn't help, and channel 0 alone is enough. Turning 3D noise reduction off (`0x30040000` = 0) and waiting a few frames before the init avoids it (310 runs clean, `-N`). VI init resets the ISP through the system controller (`0x2000011c`), and 3D-NR's DDR DMA is what that reset must not interrupt.
- **Leak limit:** VI init clears the driver's table of channel buffer mappings without unmapping them, which leaks about 7 MB of vmalloc a run, so a boot lasts about 110 runs.
- **Leak workaround (`-S`):** channel setup unmaps the old mappings before it checks the size. So a refused 64×64 setup on disabled channels, before MMZ reset and VI init, leaves only 8–12 KB mappings to leak (72 kB a run). Channel setup looks its buffers up in MMZ by name (`vi_way[N]`), so after an MMZ reset it fails with EPERM until VI init.
- **One core:** a log line written right before a freezing call never leaves the board, so the tool pauses before the init.

### Kernel drivers

- `drivers/xm_i2c`: a misc-device stub. Its read, write and ioctl handlers are placeholders that only print "not supported".
- `drivers/hx280enc`: the Hantro 6280/7280/8270/8290 video encoder driver, built as `open_h264.ko`. It implements the `/dev/h264` ioctl ABI of the vendor `xm530_h264.ko`, including the Xiongmai `'k'` 9-12 extensions (`HX280ENC_IOCWAITDONE` and the `/proc` statistics records). The decompiled `H264ProcType` and `VencInfo` structs don't match the vendor record sizes, so the driver stores those records as raw bytes.
- `drivers/xm_mmz`: the media memory zone, built as `open_mmz.ko`, a drop-in for the vendor `mmz.ko` (`mmz_start`/`mmz_size`, the `'M'` 0-7 ioctls on 44-byte records, 40 on XM510, `mmz_alloc`/`mmz_get_addr`/`mmz_free`/`mmz_alloc_cached`/`media_mem_alloc`, `/proc/umap/mmz`). It keeps the vendor allocator exactly: blocks laid one after another, each followed by a 16-byte overrun marker up to the next page; free does nothing and only a reset (`_IO('M', 0)`) releases. Built for the XM510 (`CONFIG_ARCH_XM510`), it follows that SoC's older module: no marker, and its `/proc` layout. Only `xm530_vi.ko`/`open_vi` and `xm530_rgn.ko` call it from the kernel; everything else allocates through `/dev/mmz`. `allocator=cma` takes blocks from the kernel's CMA area through `dma_alloc_coherent()` instead. That needs OpenIPC/linux#157 and `open_vi`. Kernel blocks named in `zone_names` (default `rgn`, which `xm530_rgn` `ioremap()`s) still come from the `mem=` zone. A reset frees CMA blocks only after `cma_hold_ms`, and a name asked for again comes back at its address.
- `drivers/xm510_isp`: the XM510 ISP device, built as `open_isp.ko`, a drop-in for the vendor `xm510_isp.ko`. The ISP algorithms run in the vendor user-space `isp` daemon; the module gives it raw ioctls `0x101` (write) and `0x103` (read) on `{addr, val}` in the ISP/VI (`0x40000000`) and GPIO (`0x10020000`) 1 MiB blocks, one shared zeroed page by `mmap`, `SIGIO` by `fasync` on the shared VI frame-start IRQ 16 (it never touches the status, and returns `IRQ_NONE`), and `/proc/isp`. It fixes the vendor module's `free_pages()` on a `struct page *` at unload (the `Bad page state`), its mutex left held on a bad address (every later ioctl hangs uninterruptibly), its direct user-pointer access, and its mappings of any length. Builds against the XM510 kernel only.
- `drivers/xm530_isp`: the XM530/XM550 ISP device, built as `open_isp.ko`, a drop-in for the vendor `xm530_isp.ko`, used by the vendor ISP library inside the streamer. It provides:
  - `'I'` ioctls 1 (write), 2 and 3 (read) on `{addr, val, arg}` in the ISP (`0x30000000`, 2 MiB) and GPIO (`0x10020000`) blocks;
  - ioctl 4, a wait for event 0 (Bayer, IRQ 57) or 1 (luma, IRQ 58), and 5, the audio wait (IRQ 62);
  - ioctls 6 and 7, the MIPI lane-delay sweep, with Bayer-H (IRQ 59) watching `0x300c0040`/`0x300c0044`;
  - a shared 16 KiB block by `mmap`, AWB zone statistics by `write`/`read`, and `/proc/umap/isp`.

  It fixes the vendor's unchecked event index, its -512 on timeout, the garbage from an outside read, its leaked sweep threads, its sweep judging each delay by the frames of the one before, its mappings of any length, its unchecked AWB parameters and its short free. The sweep thread stays until ioctl 6 or unload stops it, so ioctl 7 can run the sweep again.
- `drivers/xm530_rgn`: the XM530/XM550 region (OSD) device, built as `open_rgn.ko`, a drop-in for the vendor `xm530_rgn.ko` (`/dev/rgn`, `'R'` ioctls 1-11, `/proc/umap/rgn`). VI draws the regions over each of its three channels (on majestic 0 is the main stream, 1 the snapshot stream), so they reach every encoder:
  - overlays: 8 a channel, bitmaps of 1, 2 or 4 bits a pixel (format 0-2), MSB first, rows unpadded, packed in the channel's OSD SRAM (`0x30090000`/`0x30092000`/`0x30094000`, 8/8/2 KiB), which the driver loads from its own shadow, each word byte-swapped, after a handshake on register `+0x004`; colours from a 16-entry `0x00YYUUVV` palette through a per-overlay colour map;
  - covers: 32 a channel, solid or outlined rectangles of one YUV colour.

  Registers are at `0x30079000 + chn * 0x1000`. It checks channel, handle, size and format, returns errnos, needs no MMZ block, and hides every region on reset and unload. Format 2 reads 4 bits a pixel but draws every non-zero pixel in colour-map bits 11-8, so an overlay has at most three colours plus its background. `tools/xm-rgn-test` (MIT) drives it from the shell.
- `drivers/xm510_rgn`: the XM510 region (OSD) device, also `open_rgn.ko`, a drop-in for the vendor `xm510_rgn.ko` (`'R'` ioctls 1-10, records unlike the XM530's, no palette ioctl). Overlays are 16 a channel on channels 0 and 1, 1 bit a pixel, in SRAM at `0x40090000` (12 KiB) and `0x40094000` (8 KiB), registers at `0x40079000 + chn * 0x1000`; the overlay attr carries `fg` and `bg` palette entries (index in bits 27-24, colour below) and writes them into the palette. Covers are 8, global, at `0x40077000`, sharing one colour register; `0x40078000` is VI channel 1's scaling ratio, through which channel 1 x is converted. `tools/xm-rgn-test -510` drives it.
- `drivers/xm510_vi`: the XM510 video input driver, also built as `open_vi.ko`, a drop-in for the vendor `xm510_vi.ko` (`/dev/vi`, `'V'` 0-7, the 24-byte frame record `{chn, idx, y, c, stride, h}`, `vi_get_frame`/`vi_release_frame`, `buffer=`). Registers at `0x40000000`, IRQ 16 frame start and 18 frame end; three channels (0 cropped, 1 scaled from 0, 2), each a store engine at `0x40081000 + chn * 0x1000` filling 2-4 buffers of the MMZ block `vi_way[N]`. Buffers are free, ready or held, and the frame-end handler holds an engine whose next buffer is not free, as the vendor driver does. It adds release on close, `poll`/`O_NONBLOCK`, and a quiesce before the VI reset (`0x2000007c`). Builds against the XM510 kernel only.
- `drivers/xm_vi`: the XM530/XM550 video input driver, built as `open_vi.ko` (OpenIPC firmware names open replacements `open_*`), a drop-in for the vendor `xm530_vi.ko` (`/dev/vi`, `'V'` 0-8, the 32-byte frame record, `vi_get_frame`/`vi_release_frame` exports, `buffer=` parameter). It writes the same registers as the vendor module (issue #34 has the map) but keeps no per-buffer kernel mappings, so a restart leaks no vmalloc. Its VI init stops the store engines and 3D noise reduction (`0x30040000`) and waits 200 ms before the reset, so the hang `tools/xm-vi-hang` reproduces does not happen on it. Frames a descriptor still holds are released on close, and it adds `poll` and `O_NONBLOCK` GET. It needs `mmz.ko` loaded first and builds against the XM530 kernel only.

## Conventions

- Source encoding and line endings are mixed. Many vendor-derived `.c` files use CRLF, and some contain GBK/ISO-8859 comments (for example `XAx_cmos.c`). Keep each file's existing line endings. Filter with `tr -d '\r'` when grepping for line-anchored patterns.
- Formatting differs by area:
  - Userspace library code uses clang-format LLVM style (2-space indent).
  - `drivers/*` have a `.clang-format` with kernel style: tabs, 8-wide, 80 columns.
- Commit messages are prefixed with a sensor or component tag when one applies, for example `[SC3335] Fixes`.
