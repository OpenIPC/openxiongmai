# OpenXiongmai

Open-source kernel modules and userspace libraries for Xiongmai (XM) IP camera SoCs. Most of the code targets the XM530. The kernel modules are set up to build against the XM510 vendor kernel.

The goal is to replace the proprietary SDK that Xiongmai ships with its camera boards. The code is intended for [OpenIPC](https://openipc.org/) firmware, and you can also use it in your own projects.

## Supported hardware

| SoC | Status | What's here |
|-----|--------|-------------|
| XM530 | Primary target | Sensor libraries, ISP application library, MPI reimplementation |
| XM530V200, XM533, XM535D1/D2/W1 | Streams on the vendor kernel modules | `libxmv200`: sensors, ISP bring-up, VPSS, H.265, AE/AWB |
| XM510 | Kernel modules | `hx280enc` and `xm_i2c` build against the XM510 vendor kernel tree |
| XM322 / XM350 / XM520 / XM540 / XM550 | Headers only | Register maps in `libraries/isp/include/xm/` |

## How the Xiongmai SDK works

This section introduces the main concepts in the codebase for readers who haven't worked with Xiongmai SoCs.

### The vendor SDK

Xiongmai's SDK is a set of closed-source userspace libraries plus kernel modules. The libraries are the MPI (media process interface: system, VI, VENC and region), the ISP, AE and AWB algorithms, and one sensor library for each family of sensors. Applications call `XM_MPI_*` functions. Those functions talk to the kernel through `ioctl`s on device nodes such as `/dev/mmz`, `/dev/vi` and `/dev/rgn`, and through `/dev/mem` mappings of hardware registers. This repository rebuilds those pieces from source so that they can be compiled with current toolchains and C libraries such as musl.

### Key components

**MMZ (Media Memory Zone).** The video pipeline needs large blocks of contiguous physical memory for DMA between hardware blocks: sensor to ISP to encoder. MMZ reserves that memory and hands it out through `/dev/mmz`. `XM_MPI_SYS_MmzAlloc` calls the MMZ allocation ioctl, then maps the returned physical address through `/dev/mem`.

**Sensor libraries.** Each `libsns_<family>_XM530.so` supports a group of image sensors. At startup the library reads the sensor's chip-ID registers over I2C to identify the sensor and sets `gSensorChip` to the matching `SENSOR_CHIP_*` value. `sensor_register_callback()` then registers that sensor's callbacks with the vendor ISP, AE and AWB libraries through `XM_MPI_ISP/AE/AWB_SensorRegCallBack`. The callbacks cover exposure and gain logic, mirror and flip, and the default tables (AWB calibration, colour matrix and AGC).

**ISP application layer.** `libispapp_XM530` contains the code that sits between the sensor library and the main application: PLL and VI-window setup for each resolution, the ISP run loop, product info, UART and MCU communication, VDA, and the I2C access layer (`XM_I2C_Ioctl`) that the sensor libraries call.

### Reverse-engineered MPI

`libmpi` (`libopenmpi.a`) reimplements part of the vendor MPI. It covers SYS/MMZ, VI, VENC and region (OSD). Its headers are in `libraries/include/re_*.h`. The ioctl numbers in this library come from the vendor binaries, so they must match the vendor kernel modules exactly.

## Repository structure

```
├── drivers/                     Kernel modules (out-of-tree kbuild)
│   ├── hx280enc/                Hantro 6280/7280/8270/8290 video encoder driver
│   ├── xm_hevc/                 V200-series H.265 engine (/dev/h265)
│   ├── xm_i2c/                  /dev/xm_i2c misc-device (stub)
│   ├── xm_mmz/                  XM530/XM550 and V200-series media memory zone (/dev/mmz)
│   ├── xm_v200_i2c/             V200-series I2C masters (/dev/i2c0..2)
│   ├── xm_v200_sys/             V200-series system module: bind table, clocks (/dev/sys)
│   ├── xm_v200_vb/              V200-series video buffer pools (/dev/vb)
│   ├── xm_v200_rgn/             V200-series regions: OSD, covers, lines (/dev/rgn)
│   ├── xm533_vi/                XM533 video input: scaling, store, frame rate (/dev/vi)
│   ├── xm535_vpss/              XM535D1/D2 VPSS: scaling, splicing, store (/dev/vpss)
│   ├── xm535w1_vi/              XM535W1 video input on the VPSS block (/dev/vi)
│   ├── xm530v200_vpss/          XM530V200 VPSS: the memory-to-memory scaler (/dev/vpss)
│   ├── xm_v200_acodec/          V200-series audio codec (/dev/acodec)
│   ├── xm_v200_i2s/             V200-series audio in/out (/dev/ai, /dev/ao)
│   ├── xm_v200_adc/             V200-series ADC (/dev/adc)
│   ├── xm_v200_ddr/             V200-series DDR bandwidth monitor (/proc/umap/ddr)
│   ├── xm_v200_log/             V200-series media log (/dev/log)
│   ├── xm533_mipi_io/           XM533 MIPI lanes as GPIOs (/dev/mipi_io)
│   ├── xm_v200_h264/            XM533/XM535 H.264 encoder's interrupt (/dev/h264)
│   ├── xm530v200_jpeg/          XM530V200 JPEG encoder (/dev/jpeg)
│   ├── xm533_venc_mem/          XM533 encoder memory book-keeping (/dev/venc_mem)
│   ├── xm_v200_nn/              V200-series CNN accelerator's command queue (/dev/nn)
│   ├── xm_v200_venc/            V200-series VENC frame queue (/dev/venc_info)
│   └── xm_vi/                   XM530/XM550 video input driver (/dev/vi)
│
└── libraries/                   Userspace libraries
    ├── include/                 Reverse-engineered MPI headers (re_*.h)
    ├── isp/include/             Vendor-style SDK headers
    │   ├── isp/                 ISP / AE / AWB / sensor interfaces
    │   ├── mpi/                 SYS / VI / VO / AIO interfaces
    │   └── xm/                  Camera.h (SENSOR_CHIP_* enum), per-SoC register maps
    ├── libmpi/                  libopenmpi.a: MPI reimplementation
    ├── libxmv200/               libxmv200.a: the XM530V200 pipeline on the vendor modules (MIT)
    ├── libxmvenc/               libxmvenc.a: open H.264/JPEG (Hantro) and H.265 (Mali-V550) encoding (MIT)
    ├── libispapp_XM530/         libispapp_XM530.so: ISP application layer
    ├── libsns_X123_XM530/       libsns_X123_XM530.so: "X123" sensor family
    └── libsns_X50_XM530/        libsns_X50_XM530.so: "X50" sensor family

tools/
├── xm-vi-hang/                  Reproduces the VI driver hang without a camera application (MIT)
├── xmv200-capture/              Makes libxmv200's sensor tunings from a capture of the vendor stack (MIT)
├── xmv200-test/                 Brings a V200 board's pipeline up and encodes its streams (MIT)
├── xmhevc-test/                 Raw /dev/mv500 H.265 session, the interface reference (MIT)
└── xmvenc-test/                 Encodes H.264, H.265 or JPEG with libxmvenc (MIT)
```

## Building

Each component has its own Makefile and is built with an ARM cross toolchain. Using the OpenIPC SDK:

```bash
export PATH=/path/to/openipc_sdk/bin:$PATH

# Userspace libraries
make -C libraries/libsns_X50_XM530  CROSS_COMPILE=arm-openipc-linux-musleabi-
make -C libraries/libsns_X123_XM530 CROSS_COMPILE=arm-openipc-linux-musleabi-
make -C libraries/libispapp_XM530   CROSS_COMPILE=arm-openipc-linux-musleabi-
make -C libraries/libmpi            CROSS_COMPILE=arm-openipc-linux-musleabi-

# Or the libraries OpenIPC firmware ships for one SoC family, in one go
make -C libraries CHIPARCH=xm530 CC=arm-openipc-linux-musleabi-gcc STRIP=arm-openipc-linux-musleabi-strip

# Kernel modules: point KDIR at a configured kernel tree
make -C drivers/hx280enc KDIR=/path/to/linux
make -C drivers/xm_i2c   KDIR=/path/to/linux
make -C drivers/xm_vi    KDIR=/path/to/linux   # XM530 kernel only
make -C drivers/xm_mmz   KDIR=/path/to/linux   # XM530 kernel only; MMZ_ALLOCATOR=list for the V200 series
make -C drivers/xm_hevc  KDIR=/path/to/linux HEVC_LAYOUT=v3   # V200 series: v3 XM530V200/XM533, v1 XM535
make -C drivers/xm_v200_i2c KDIR=/path/to/linux I2C_CHIP=xm530v200   # or xm533, xm535
make -C drivers/xm_v200_sys KDIR=/path/to/linux
make -C drivers/xm_v200_vb  KDIR=/path/to/linux VB_LAYOUT=v200   # or xm535
make -C drivers/xm_v200_venc KDIR=/path/to/linux VENC_LAYOUT=v200 # or xm535
make -C drivers/xm_v200_rgn  KDIR=/path/to/linux RGN_CHIP=v200    # or xm533, xm535
make -C drivers/xm533_vi     KDIR=/path/to/linux                  # XM533 kernel
make -C drivers/xm535_vpss   KDIR=/path/to/linux                  # XM535 kernel
make -C drivers/xm535w1_vi   KDIR=/path/to/linux                  # XM535W1 kernel
make -C drivers/xm530v200_vpss KDIR=/path/to/linux                # XM530V200 kernel
```

The Makefiles set different default toolchains (`arm-xm-linux-`, `arm-linux-` or `arm-openipc-linux-musleabi-`), so pass `CROSS_COMPILE` explicitly. `libmpi` and the kernel module Makefiles also hard-code `PATH` to a vendor toolchain directory. Override it from the command line, for example `make PATH="$PATH" ...`, or give an absolute `CROSS_COMPILE` prefix. Most components build with `-Werror`.

OpenIPC firmware builds these libraries from source: its `xiongmai-opensdk` package fetches this repository at a pinned commit and runs `make -C libraries CHIPARCH=<soc>` with the Buildroot toolchain and `CFLAGS`. The library Makefiles append their own include paths and defines to the `CFLAGS` they are given.

## Supported sensors

| Manufacturer | Sensor | Library |
|---|---|---|
| **Sony** | IMX291 | X123 |
| | IMX307 | X123 |
| | IMX323 | X123 |
| | IMX335 | X50 |
| **SmartSens** | SC1235 | X123 |
| | SC2145 / SC2145H | X123 |
| | SC2235 / SC2235E / SC2235P | X123 |
| | SC2239 | X123 |
| | SC2335 | X123 |
| | SC3035 | X123 |
| | SC307E | X123 |
| | SC3335 | X123 |
| | SC4236 | X123 |
| | SC335E | X50 |
| | SC5235 | X50 |
| | SC5239 | X50 |
| | SC5332 | X50 |
| **OmniVision** | OV9732 | X123 |
| **Silicon Optronics (JX)** | H62 | X123 |
| | H65 | X123 |
| | F37 | X123 |
| | K03 | X50 |
| **Imagedesign** | MIS2003 / MIS2006 | X123 |
| **SuperPix** | SP140A | X123 |
| | SP2305 | X123 |
| **Other** | AUGE | X123 |

To add a sensor:
1. Add a `<sensor>_cmos.c` with the ISP, AE and AWB callbacks and tuning tables. If the sensor needs an init register table, also add a `<sensor>_sensor_ctl.c`.
2. Add the chip-ID probe and a `case` in `sensor_register_callback()` in the family dispatcher (`XAx_cmos.c` / `X50_cmos.c`).
3. Add a `SENSOR_CHIP_*` value in `Camera.h` if needed, and handle the new sensor in `libispapp_XM530/isp_sample.c`.
4. Add the new object files to the Makefile's `OBJECTS` list.

## Kernel modules

| Module | Function | Status |
|--------|----------|--------|
| `hx280enc` | Hantro 6280/7280/8270/8290 hardware video encoder, built as `open_h264.ko` | Same `/dev/h264` ABI as the vendor `xm530_h264.ko`; ran majestic on an XM550 in place of it, and on an XM510 in place of `xm510_h264.ko` |
| `xm_i2c` | Registers `/dev/xm_i2c` and `/dev/xm_i2c1` | Stub: read, write and ioctl are not implemented yet |
| `xm_hevc` | V200-series H.265 encoder engine: runs the register lists user space builds (`/dev/h265`) | Builds `open_hevc.ko`. Same ioctls and records as the vendor `hevc.ko`, both engines (v3: XM530V200/XM533, v1: XM535); ran libxmv200 on all five V200-series chips in qemu-bench in place of it |
| `xm_mmz` | XM530/XM550 and V200-series media memory zone: the named, physically contiguous DMA buffers the SDK and video drivers share (`/dev/mmz`) | Builds `open_mmz.ko`. Same parameters, ioctls, exports and `/proc/umap/mmz` as the vendor `mmz.ko`; ran majestic on an XM550 in place of it, and libxmv200 on all five V200-series chips in qemu-bench (`allocator=list`) |
| `xm_v200_i2c` | V200-series I2C masters, the sensor buses (`/dev/i2c0..2`, `/dev/xm_i2c*` on XM533) | Builds `open_i2c.ko`. Same devices, record and ioctls as the vendor `i2c.ko`, per-chip timing, pins and resets; ran libxmv200 (sensor writes and chip-ID probes) on all five V200-series chips in qemu-bench in place of it |
| `xm_v200_sys` | V200-series system module: the bind table and the XM535's clock calls (`/dev/sys`, `sys_set_clk`/`sys_get_clk`) | Builds `open_sys.ko`. Same ioctls, exports and `/proc/umap/sys` as the vendor `sys.ko`; ran libxmv200 on all five V200-series chips in qemu-bench in place of it, the XM535's `isp.ko` on its clock calls |
| `xm_v200_vb` | V200-series video buffer pools: the frame buffers `isp`, `vpss`/`vi` and `venc` share (`/dev/vb`, `vb_get_buf`/`vb_put_buf`, ...) | Builds `open_vb.ko`. Same descriptors, lists, exports, ioctls and `/proc/umap/vb` as the vendor `vb.ko`; ran libxmv200 on all five V200-series chips in qemu-bench in place of it |
| `xm_v200_acodec` | V200-series audio codec: power-up, volume, mute, input gain (`/dev/acodec`) | Builds `open_acodec.ko`; same ioctls as the vendor `acodec.ko`, per-chip power-up sequences from its disassembly |
| `xm_v200_i2s` | V200-series audio input and output through the I2S blocks' FIFOs (`/dev/ai`, `/dev/ao`, `/proc/umap/audio`) | Builds `open_i2s.ko`; same devices, ioctls and clock setup as the vendor `i2s.ko` |
| `xm_v200_adc` | V200-series ADC (`/dev/adc`) | Builds `open_adc.ko` |
| `xm_v200_ddr` | V200-series DDR bandwidth monitor (`/proc/umap/ddr`) | Builds `open_ddr.ko` |
| `xm_v200_log` | V200-series media log FIFO and log levels (`/dev/log`, `/proc/umap/logmpp`) | Builds `open_log.ko` |
| `xm533_mipi_io` | XM533 MIPI lanes 6..9 as GPIOs (`/dev/mipi_io`) | Builds `open_mipi_io.ko` |
| `xm_v200_h264` | XM533/XM535 H.264 encoder: its clock, registers' address and interrupt (`/dev/h264`) | Builds `open_h264.ko`; same ioctls as the vendor `h264.ko`, answered as it does in qemu-bench |
| `xm530v200_jpeg` | XM530V200 JPEG encoder: ownership, register table, wait for the picture (`/dev/jpeg`) | Builds `open_jpeg.ko`; same ioctls as the vendor `jpeg.ko`, answered as it does in qemu-bench |
| `xm533_venc_mem` | XM533 encoder memory: parts of each VENC channel's MMZ block, with guards (`/dev/venc_mem`, `/proc/umap/venc_mem`) | Builds `open_venc_mem.ko`; same ioctls, allocations and `/proc` as the vendor `venc_mem.ko` in qemu-bench |
| `xm_v200_nn` | V200-series CNN accelerator: the command queue and waits for the detection libraries (`/dev/nn`, `/proc/umap/cnn`) | Builds `open_nn.ko`; same ioctls and answers as the vendor `nn.ko` on all three designs in qemu-bench, without its lock-ups |
| `xm530v200_vpss` | XM530V200 VPSS: the memory-to-memory engine that scales each sensor's picture into three channels, from its own thread, with splicing and covers (`/dev/vpss`) | Builds `open_vpss.ko`. Same ioctls and `isp.ko`/`vo.ko` hooks as the vendor `vpss.ko`; the XM530V200 boards stream with it in qemu-bench, pictures as with the vendor module |
| `xm535w1_vi` | XM535W1 video input: the three channels' scaling and store on the XM535's VPSS block from their frame interrupts, covers, lines, OSD background luminance (`/dev/vi`) | Builds `open_vi.ko`. Same ioctls and hooks as the W1's vendor `vi.ko`; in qemu-bench the W1 streams with it, pictures as with the vendor module |
| `xm535_vpss` | XM535D1/D2 VPSS: per-frame scaling of each sensor's picture into three channels, splicing of up to three sensors, store into VB buffers, covers and lines (`/dev/vpss`) | Builds `open_vpss.ko`. Same ioctls and `isp.ko` hooks as the vendor `vpss.ko`; all D1/D2 streams in qemu-bench, pictures as with the vendor module |
| `xm533_vi` | XM533 video input: the three channels' scaling and store into VB buffers, frame-rate control, covers (`/dev/vi`) | Builds `open_vi.ko`. Same ioctls and records as the vendor `vi.ko`; in qemu-bench its pictures match the vendor module's |
| `xm_v200_rgn` | V200-series regions: overlays the OSD engine blends into VENC pictures (`osd_process`), covers and (XM535) lines through VI/VPSS, the XM535's encoder OSD (`/dev/rgn`) | Builds `open_rgn.ko`. Same ioctls as the vendor `rgn.ko`; overlays checked pixel by pixel in qemu-bench on the XM530V200, XM533 and XM535D1 |
| `xm_v200_venc` | V200-series VENC frame queue: the pictures bound to VENC, with their OSD, for the encoder (`/dev/venc_info`) | Builds `open_venc.ko`. Same ioctls and records as the vendor `venc.ko` (not yet the XM535's `/dev/venc_mem`); ran libxmv200 on all five V200-series chips in qemu-bench in place of it |
| `xm_vi` | XM530/XM550 video input: capture window, scalers and frame buffers (`/dev/vi`) | Builds `open_vi.ko`. Same ABI as the vendor `xm530_vi.ko`, without its vmalloc leak; ran majestic on an XM550 in place of it |

## Open encoder

`libraries/libxmvenc` drives the H.264/JPEG encoder of the XM530/XM550 and XM510 (a Hantro 8290) directly, with no vendor userspace ([#13](https://github.com/OpenIPC/openxiongmai/issues/13)). It is a static library, `libxmvenc.a`, under the MIT license, so that closed applications can link it too. The API is in `xmvenc.h`:
- `xmvenc_open` opens the encoder and identifies the chip.
- `xmvenc_chn_create` sets up a stream: H.264 (CAVLC or CABAC, fixed QP or frame-level rate control, IDR interval), H.265 (see below) or JPEG (quality).
- `xmvenc_encode` encodes one NV12 picture, given by physical address, and returns the software-written headers (SPS and PPS before an IDR, the JPEG header) and the hardware-written slice or scan.
- `xmvenc_chn_request_idr` makes the next frame an IDR.

Encodes on different channels can come from different threads; they take turns on the hardware.

A **pipe** runs a channel from a VI channel, on a thread of its own:
- `xmvenc_pipe_create` sets one up. With `fps` set, VI frames in between are skipped.
- `xmvenc_pipe_start` / `xmvenc_pipe_stop` start and stop it: continuously, or for a set number of pictures, such as one JPEG snapshot.
- `xmvenc_pipe_get` / `xmvenc_pipe_release` take the encoded frames in order: type, QP, headers, data, and a microsecond timestamp on the `CLOCK_MONOTONIC` scale.
- Frames queue up in the channel's stream buffer, which serves as a ring. When the reader falls behind and there is no room, a frame is skipped rather than encoded, so the stream stays decodable.
- The application enables the VI channel; the pipe only takes its frames.

`tools/xmvenc-test` exercises the library, and its output is checked with ffmpeg.
- **Input:** a synthetic NV12 picture, colour bars that move 8 pixels per frame, so P frames carry motion, or a VI channel.
- **Output:** a raw `.h264` file, or concatenated JPEGs.

```bash
make -C tools/xmvenc-test CROSS_COMPILE=arm-openipc-linux-musleabi-
# on the camera: the encoder serves one user, so stop majestic first
/etc/init.d/S95majestic stop
./xmvenc-test -w 1920 -h 1080 -q 30 -n 60 -g 30 -c -o /tmp/test.h264
```

`-p` runs the `-v` channel through a pipe. `-J <quality>` adds JPEG snapshots of VI channel 1 (about one a second, into `<file>.jpg`), `-r <n>` requests an IDR once n packets have been read, `-S <ms>` slows the reader so the queue fills, and `-G <ms>` sets the keyframe interval in capture time instead of `-g` frames. `-B <KiB>` sets the stream buffer size.

`-v 0` (main) or `-v 1` (sub) takes the pictures from a VI channel instead, encoding straight from the capture buffers. With majestic stopped the capture hardware keeps running, but majestic has turned frame delivery off, so the tool turns it on for the run and back off at the end. `-d file.nv12` saves the first input picture, which you can compare against the decoded stream. `-j <quality>` encodes JPEG pictures (baseline, 4:2:0) instead of H.264; with more than one frame, the output is concatenated JPEGs.

The tool encodes IDR and P frames, at a fixed QP by default, with SPS and PPS before every IDR. `-b <kbps>` turns on frame-level rate control at `-f <fps>`, keeping the QP within `-Q min:max`. `-g` sets the IDR interval (`-g 1` gives all IDR), and `-c` selects CABAC instead of CAVLC. It needs `/dev/mmz` and an encoder driver: the vendor `xm530_h264.ko`, `drivers/hx280enc`, or, on XM510, the vendor `xm510_h264.ko`. It runs on XM530/XM550 and XM510. On XM510 the vendor driver has no blocking wait, so the tool polls the status register, and the MMZ zone is too small for the tool's own buffers, so it reuses majestic's idle encoder buffers. VI on XM510 needs the vendor `isp` daemon, which majestic starts and stops with itself. The library's `cabac_tbl.c` holds the H.264 CABAC init tables in the layout from Rockchip MPP, under the Apache License 2.0.

Two cautions about MMZ:
- **Blocks persist after the tool exits.** MMZ has no free, so a later run reuses the named blocks.
- **Never reset MMZ while the capture pipeline is configured.** That includes when majestic is stopped: VI keeps writing into its old buffers.

Restarting majestic resets MMZ cleanly.

### H.265

The XM530/XM550 encodes H.265 on a separate engine, an Arm Mali-V550 with its own firmware (`hevcenc.fwb`), behind the vendor kernel module `mve_base.ko` and its device `/dev/mv500` ([#27](https://github.com/OpenIPC/openxiongmai/issues/27)). `libxmvenc` drives that module without vendor libraries.

**The interface:** `libraries/libxmvenc/mvb.h` holds what the library uses of the module's interface: the command numbers, the buffer descriptor, the events, and the firmware messages that carry the encoder settings. It was recovered from the module and from a trace of the vendor library, and is written from scratch under MIT.

**Using it:** a channel created with `XMVENC_H265` takes the same calls as H.264, `xmvenc_encode` or a pipe.
- **Settings:** IDR interval, fixed QP, or variable bitrate within a QP range; the firmware keeps the GOP and the rate control.
- **Engine:** each channel is a firmware session of its own. The sessions run alongside the H.264 encoder and alongside each other.
- **Headers:** the firmware sends VPS, SPS and PPS once. The library keeps them and puts them before every IDR.
- **Input:** NV12 with the chroma directly after the luma, as VI delivers it.
- **Measured on an XM550:** about 21 ms per 1080p frame, and a 1080p and a 704×576 pipe running together from VI.

`tools/xmvenc-test -H` encodes with it, from the synthetic picture or VI, directly or through a pipe:

```bash
./xmvenc-test -H -v 0 -p -n 250 -G 1000 -b 4096 -o /tmp/main.h265
```

`tools/xmhevc-test` is the minimal session on `/dev/mv500` that the interface was worked out with. It follows the vendor library's order of commands, which makes it the reference for that order.
- **Input:** a synthetic NV12 picture (a ramp with a square that moves 8 pixels per frame).
- **Output:** a raw `.h265` file of parameter sets and IDR/P frames.
- **Settings:** IDR interval (`-g`), frame rate (`-f`), variable bitrate target (`-b`) and QP range (`-Q`).

It writes a synthetic NV12 picture (a ramp with a square that moves 8 pixels per frame) as a raw `.h265` file. The settings are `-g`, `-f`, `-b` and `-Q`:

```bash
make -C tools/xmhevc-test CROSS_COMPILE=arm-openipc-linux-musleabi-
# on the camera, with majestic stopped
./xmhevc-test -w 1920 -h 1080 -n 100 -g 25 -o /tmp/test.h265
```

Both tools take their MMZ through `libxmvenc`, so they also need `/dev/h264`.

### The VI driver hang: `tools/xm-vi-hang`

On the XM530/XM550, re-initialising VI (`_IO('V', 6)` on `/dev/vi`, which an SDK start makes) now and then freezes the whole board, CPU and interrupts included, when the process before it left VI capturing. Camera applications see it as a hard hang at start, about one start in fifteen, that only a watchdog or a power cycle ends.

`tools/xm-vi-hang` reproduces it with no camera application, only raw ioctls. Each run:
1. Resets MMZ and inits VI.
2. Replays the ISP and MIPI writes an SDK start makes after VI init (`isp_writes.h`, recorded on an XM550 + IMX335). Without them, VI init leaves capture stopped.
3. Sets VI up (channel 0, 1920×1080; ext channel 1, 704×576).
4. Takes frames for a few seconds, then exits, leaving VI capturing.

Run it in a loop: the next run's VI init is the one that may freeze. Start the camera application once after boot so the sensor runs, then stop it.

```bash
make -C tools/xm-vi-hang CROSS_COMPILE=arm-openipc-linux-musleabi-
# on the camera, with the camera application stopped; expect to power-cycle
while ./xm-vi-hang; do sleep 1; done
```

Measured on an XM550 + IMX335 (150-round limit per test):

| Run | Result |
|---|---|
| default | 6 freezes, after 4, 4, 9, 16, 19 and 30 runs |
| `-x`: both channels disabled before exit | froze at run 8 |
| `-p`: both channels disabled before the VI init | froze at run 15 |
| `-n`: ext channel 1 enabled, no frames taken | froze at run 4 |
| `-0`: channel 0 only | froze at run 3 |
| `-i`: VI init only, VI never set up | 150 runs clean |
| `-N`: 3D noise reduction off before the init (`0x30040000` = 0, 200 ms) | **310 runs clean**: 110 + 100 with `devmem`, 100 with `-N`, over three boots. The baseline froze at run 7 in between |

So the condition is VI capturing at the time of the init. Disabling the channels doesn't avoid it, and neither does leaving ext channel 1 out. Putting the sensor into standby doesn't stop VI either: its frame interrupts keep coming from the ISP's timing.

**The way around it: turn 3D noise reduction off before the VI init.**
- **Why it works:** VI init pulses a reset (`0x01590000`, then 1, to the system controller at `0x2000011c`) that covers the ISP. 3D-NR is a bus master: it reads and writes reference frames in DDR for as long as it's on. With it off and its DMA given a few frames to stop, the reset no longer freezes the bus.
- **No lasting effect:** the reset puts the 3D-NR control back to its default (`0x45`, on), so the setup that follows is unchanged.
- **Using it:** `-N` does this, through the ISP driver's register write. `-B` and `-A` run commands right before and after the init, for testing other ways around it.
- **Leak limit:** each run leaks about 7 MB of VI driver vmalloc. A boot lasts about 110 runs before setup fails with ENOMEM. Reboot between long tests.

**The VI driver's vmalloc leak.** Every VI init leaks the channels' buffer mappings.
- **Cause:** channel setup (`0x400c5601`) maps each capture buffer twice, for luma and chroma. It unmaps the previous mappings only if their pointers are still stored. VI init clears the driver's state table without unmapping anything.
- **Way around it:** channel setup does the unmapping before it checks the new size. So a setup with a size VI refuses ("too small", here 64×64), on disabled channels, before the MMZ reset and the init, leaves only tiny mappings to leak. `-S` does this.
- **Measured (XM550):**

  | | vmalloc per run |
  |---|---|
  | without `-S` | 7,316 kB |
  | with `-S` | 72 kB |

  That's about 12,700 runs to exhaustion instead of about 110. 200 runs of `-N -S` were clean.

## The V200 series: `libxmv200`

The XM530V200 (and the XM533 and XM535D1/D2/W1 after it) has a different media pipeline from the XM530's. VI, the ISP (BPU) and 3D noise reduction live in `isp.ko`. VPSS scales and splices (on the XM533 and XM535W1, which have no VPSS, `vi.ko` scales). The VENC side is a frame queue (`venc.ko`) in front of an H.265 engine (`hevc.ko`). The vendor ships it as kernel modules and two statically linked programs: `Sofia`, and an `isp` daemon that runs the sensors and the ISP. `libxmv200` replaces both programs and runs on the vendor modules.

A board is a **layout** (`struct xmv200_layout`), which an application can build or read from a file:
- the sensors: model (or probed by chip ID), I2C bus and address, mirror/flip, the lens' crop, the picture size, and how each is wired (its MIPI receiver, and on the XM535D1/D2 its VI device and index there);
- the streams: their size and which sensors each takes (several are spliced top to bottom).

The sensor's mode comes from the tuning libxmv200 has for that sensor on the running chip family (`sns_<sensor>[_<family>]_tuning.c`, generated by `tools/xmv200-capture` from a capture of the vendor stack).

- **Board start** (`board.c`): `xmv200_layout_start()` brings the whole pipeline up in the vendor's order:
  - MMZ, the VB pools;
  - VPSS groups bound to the ISP, their channels spliced and bound to VENC, and the `venc.ko` channels;
  - the sensor and ISP clocks, the sensors, the MIPI receivers, the VI inputs;
  - the ISP;
  - without VPSS, a VI channel per stream.
- **Chips** (`chip.c`): what differs between the families, for the same steps: the kernel modules' struct versions, where VI's blocks are, clocks through `/dev/mem` or `sys.ko`, and the VB and VENC configuration.
- **ISP** (`isp.c`): `isp.ko`'s common setup and its shared parameter page. Then, per channel, the sensor mode's program: `isp.ko`'s parameter blocks and the sensor's register runs in order, with the buffer, window and channel steps between them. Those steps are built from the channel's place in the layout.
- **H.265** (`venc.c`): `xmv200_frame_get` takes the pictures queued for VENC. `xmv200_venc_encode` encodes them on `hevc.ko`'s engine, one job per picture. The parameter sets and slice headers are the library's; the engine writes only the slice data. Each family's engine is a profile (v3 for the XM530V200, v1 for the XM535s, x533 for the XM533).
- **OSD** (`osd.c`): `xmv200_osd_create`, `_bitmap` and `_attach` put a bitmap on a stream: ARGB1555, ARGB8888, or 1 or 2 bits a pixel with up to four AYUV colours (`xmv200_osd_color`), drawn at 1x or 2x. `rgn.ko` (or `open_rgn`) has the OSD engine blend it into each of the stream's pictures before they are encoded. `_set` moves or recolours one, and `_detach` and `_destroy` remove it. A stream takes 8 (4 on the XM535).
- **AE/AWB** (`ae.c`): `xmv200_ae_frame` samples the frames of the streams the sensors are in, one band per sensor. It steers the sensor's exposure and analog gain, `isp.ko`'s digital gain, and gray-world white balance through `isp.ko`'s R/B gains.

Known boards (`xmv200_board_layout()`, or `xmv200-test layout <board>` for the text form):

| Board | Chip | Sensors | Streams (VENC channel order) |
|-------|------|---------|------------------------------|
| `x6c-weq` | XM530V200 | two SC2336, spliced | 2304x2592, 640x720 |
| `x2-wq2` | XM530V200 | one SC2336 | 1920x1080, 640x360 |
| `x6e-weq` | XM535D1 | two SC2336 on their own receivers, spliced | 1920x2160, 640x720 |
| `x9-we2q` | XM535D2 | three SC2336 (bus 2 at 0x30/0x32 sharing a receiver, bus 0 at 0x30) | per camera n: 1920x1080 and 640x360 as streams 2n and 2n + 1 |
| `x2w-wng` | XM535W1 | one SC1346 | 1280x720, 800x448 |
| `x3-wq-b` | XM533 | one SC2337P | 2304x1296, 800x448 |
| `auto` | any | the sensors found at 0x30/0x32 on bus 0 | one picture, or two of a kind spliced; plus 640 wide |

Sensors with a tuning, per chip family (each captured from that chip's vendor firmware; a sensor its firmware has no mode for can't be captured there):

| Family | Sensors |
|--------|---------|
| XM530V200 | SC1336, SC1345, SC1346, SC1A4T, SC200AI, SC223AP, SC231AI, SC2331, SC2336, SC2337P, SC307C, SC3235, SC3335, SC3338, SC401AI, SC430AI, SC4336, SC500AI, SC831HAI |
| XM535D1/D2 | SC1336, SC1345, SC1346, SC1A4T, SC200AI, SC223AP, SC2331, SC2336, SC2336P, SC307C, SC3235, SC3335, SC401AI |
| XM535W1 | SC1346, SC1A4T, SC200AI, SC2336, SC2336P, SC3338, SC500AI |
| XM533 | SC1346, SC1A4T, SC223A, SC231AI, SC2331, SC2337P, SC3338, SC401AI, SC4336, SC500AI |

A layout file (`xmv200_layout_parse()`):

```
chip xm535d1
fps 12
sensor SC2336 bus=0 addr=0x30 x0=1 width=1920 height=1080
sensor auto bus=0 addr=0x32 x0=1 width=1920 height=1080 phy=1 vi_idx=1
stream 1920x1080 sensors=0,1
stream 640x360 sensors=0,1
```

The XM533 and XM535 boards need the vendor's memory split: `mem=` as in the vendor firmware (29M on a 64 MB XM535D1, 70M on a 128 MB XM535D2, 32M on a 64 MB XM533), the rest MMZ.

```sh
make -C tools/xmv200-test CROSS_COMPILE=arm-openipc-linux-musleabi-
xmv200-test run x6c-weq 100       # on the camera, vendor modules loaded: /tmp/stream<n>.h265, one per stream
xmv200-test run /etc/board.conf   # a layout file
xmv200-test run x6c-weq 100 osd   # with a 2-bit test card (red, green, white, clear) at (16, 16) on each stream
```

## CI

Every push and pull request to `main` runs these jobs, each using the OpenIPC musl toolchain for its SoC (`toolchain.xiongmai-xm5x0`).

- **Libraries cross-compile (XM530):** builds all four userspace libraries, checks that the expected entry points are exported (`sensor_register_callback`, `XM_I2C_Ioctl`, `XM_MPI_*`, ...), and checks that the shared objects depend only on libc.
- **Kernel modules (xm510 / xm530):** builds `xm_i2c` and `hx280enc` against the OpenIPC kernels `openipc/linux@xiongmai-xm510` (3.0.101) and `@xiongmai-xm530` (3.10.103), the same way the firmware does: with the firmware's `<soc>.generic.config` and its generic kernel patches.

All external inputs are pinned: the kernel commits, the firmware commit, and the toolchain sha256 checksums. OpenIPC rebuilds its toolchain tarballs in place, so a checksum failure means upstream published a new build and the checksum in `build.yml` needs updating.

[![Build Status](https://github.com/OpenIPC/openxiongmai/actions/workflows/build.yml/badge.svg)](https://github.com/OpenIPC/openxiongmai/actions/workflows/build.yml)

## License

GPL v3. See [LICENSE](LICENSE). The exceptions:
- `libraries/libxmvenc`, `libraries/libxmv200`, `tools/xmvenc-test`, `tools/xmhevc-test`, `tools/xmv200-test`, `tools/xmv200-capture` and `tools/xm-vi-hang` are under the MIT license, so that they can be linked into applications under any license. See their `LICENSE` files.
- `libraries/libxmvenc/cabac_tbl.c` and `cabac_tbl.h` are under the Apache License 2.0, from Rockchip MPP.
