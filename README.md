# OpenXiongmai

Open-source kernel modules and userspace libraries for Xiongmai (XM) IP camera SoCs. Most of the code targets the XM530. The kernel modules are set up to build against the XM510 vendor kernel.

The goal is to replace the proprietary SDK that Xiongmai ships with its camera boards. The code is intended for [OpenIPC](https://openipc.org/) firmware, and you can also use it in your own projects.

## Supported hardware

| SoC | Status | What's here |
|-----|--------|-------------|
| XM530 | Primary target | Sensor libraries, ISP application library, MPI reimplementation |
| XM530V200, XM533, XM535D1/D2/W1 | Streams on the vendor kernel modules | `libxmv200`: sensors, ISP bring-up, VPSS, H.265, AE/AWB |
| XM510 | Kernel modules | `hx280enc`, `xm_mmz`, `xm510_vi`, `xm510_isp`, `xm510_rgn` and `xm_i2c` build against the XM510 kernel |
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
│   ├── xm510_isp/               XM510 ISP device (/dev/isp)
│   ├── xm510_rgn/               XM510 region (OSD) driver (/dev/rgn)
│   ├── xm510_vi/                XM510 video input driver (/dev/vi)
│   ├── xm530_isp/               XM530/XM550 ISP device (/dev/isp)
│   ├── xm530_rgn/               XM530/XM550 region (OSD) driver (/dev/rgn)
│   ├── xm_hevc/                 V200-series H.265 engine (/dev/h265)
│   ├── xm_i2c/                  /dev/xm_i2c misc-device (stub)
│   ├── xm_mmz/                  XM530/XM550/XM510 and V200-series media memory zone (/dev/mmz)
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
│   ├── xm530v200_jpeg/          XM530V200 JPEG encoder (/dev/jpeg)
│   ├── xm533_venc_mem/          XM533 encoder memory book-keeping (/dev/venc_mem)
│   ├── xm_v200_nn/              V200-series CNN accelerator's command queue (/dev/nn)
│   ├── xm533_ive/               XM533 FFT engine's job queue (/dev/ive)
│   ├── xm530v200_vo/            XM530V200 video output: BT656/LCD, layers, channels (/dev/vo)
│   ├── xm533_isp/               XM533 ISP: parameter blocks to registers once a frame (/dev/isp)
│   ├── xm530v200_isp/           XM530V200 ISP: online/offline BPU, VPSS side, LDC (/dev/isp)
│   ├── xm535_isp/               XM535D1/D2 ISP: offline BPU queue, time-shared VICU (/dev/isp)
│   ├── xm535w1_isp/             XM535W1 ISP: XM530V200 control around the XM535 BPU (/dev/isp)
│   ├── xm_v200_venc/            V200-series VENC frame queue (/dev/venc_info)
│   ├── dwc2/                    V200-series USB 2.0 host (Linux 4.19's DWC2 driver)
│   ├── xm_v200_usb0/            V200-series USB PHY and the dwc2 platform device
│   ├── xm_v200_rtc/             V200-series real-time clock (/dev/rtc)
│   ├── xm_v200_vdec/            XM530V200 decoder bridge: VB/VO buffers, delay (/dev/vdec)
│   ├── xm_v200_var/             XM530V200 rotator, 90/270 degrees (/dev/var)
│   ├── xm_v200_dma/             XM530V200 memory-to-memory DMA, PL330 (/dev/dma)
│   ├── xm_v200_motor/           V200-series pan/tilt step motors (/dev/motor)
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
├── xm-rgn-test/                 Draws OSD overlays and covers from the shell, XM530/XM550 and XM510 (MIT)
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
make -C drivers/xm510_vi KDIR=/path/to/linux   # XM510 kernel only
make -C drivers/xm510_isp KDIR=/path/to/linux  # XM510 kernel only
make -C drivers/xm530_isp KDIR=/path/to/linux  # XM530 kernel only
make -C drivers/xm530_rgn KDIR=/path/to/linux  # XM530 kernel only
make -C drivers/xm510_rgn KDIR=/path/to/linux  # XM510 kernel only
make -C drivers/xm_mmz   KDIR=/path/to/linux   # MMZ_ALLOCATOR=list for the V200 series
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
| `hx280enc` | Hantro 6280/7280/8270/8290 hardware video encoder, built as `open_h264.ko`; `H264_CHIP=v200` for the XM533's and XM535s' (0x21100000, IRQ 78, the encoder's clock turned on) | Same `/dev/h264` ABI as the vendor `xm530_h264.ko`; ran majestic on an XM550 in place of it, and on an XM510 in place of `xm510_h264.ko`; with `H264_CHIP=v200` answers as the V200 series' `h264.ko` in qemu-bench |
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
| `xm530v200_jpeg` | XM530V200 JPEG encoder: ownership, register table, wait for the picture (`/dev/jpeg`) | Builds `open_jpeg.ko`; same ioctls as the vendor `jpeg.ko`, answered as it does in qemu-bench |
| `xm533_venc_mem` | XM533 encoder memory: parts of each VENC channel's MMZ block, with guards (`/dev/venc_mem`, `/proc/umap/venc_mem`) | Builds `open_venc_mem.ko`; same ioctls, allocations and `/proc` as the vendor `venc_mem.ko` in qemu-bench |
| `xm_v200_nn` | V200-series CNN accelerator: the command queue and waits for the detection libraries (`/dev/nn`, `/proc/umap/cnn`) | Builds `open_nn.ko`; same ioctls and answers as the vendor `nn.ko` on all three designs in qemu-bench, without its lock-ups |
| `xm533_ive` | XM533 FFT engine: the job queue and waits (`/dev/ive`) | Builds `open_ive.ko`; same ioctls and answers as the vendor `ive.ko` in qemu-bench, without its crash on a second queued job |
| `xm530v200_vo` | XM530V200 video output: BT656/LCD/MIPI timing, the video layer's six channels and their buffers, image and cursor layers, colour (`/dev/vo`, `vo_get_buf`/`vo_put_buf`) | Builds `open_vo.ko`. Same ioctls, exports and `/proc/umap/vo` as the vendor `vo.ko`; in qemu-bench its register writes match `vo.ko`'s, and the XM530V200 streams with it |
| `xm533_isp` | XM533 ISP: the sensor library's parameter blocks written to the ISP once a frame from the VICU interrupt, AWB statistics, register access, raw-frame dump (`/dev/isp`, `vpss_isp_*`, `ib_*`) | Builds `open_isp.ko`. Same ioctls, read/write/mmap and exports as the vendor `isp.ko`; in qemu-bench the XM533 streams with it, its register writes matching `isp.ko`'s frame by frame |
| `xm530v200_isp` | XM530V200 ISP: two VICUs (one time-shared), the BPU online or offline from ISP buffers into VB buffers for the VPSS, the VPSS side's noise reduction, LDC and limit-up scaler, AWB/AF statistics (`/dev/isp`, `vpss_isp_*`) | Builds `open_isp.ko`. Same ioctls, read/write/mmap and exports as the vendor `isp.ko`; in qemu-bench the XM530V200 streams with it, writing the same registers with the same values, in the same order frame by frame, as `isp.ko` |
| `xm535_isp` | XM535D1/D2 ISP: two VICUs (the first time-shared between two sensors, paced by timer 4/5), the offline BPU's queue of stored channels into vpss.ko's buffers, the VPSS side's blocks, AWB statistics (`/dev/isp`, `vpss_isp_*`) | Builds `open_isp.ko`. Same ioctls, read/write/mmap and exports as the vendor `isp.ko`, which both chips share; in qemu-bench the XM535D1 and D2 boards stream with it, writing the same registers with the same values, in the same order frame by frame, as `isp.ko` |
| `xm535w1_isp` | XM535W1 ISP: the XM530V200's control (ioctls, VICU frame tasklet, time-shared VICU, offline scheduling) around the XM535D1/D2's BPU and VPSS side, output into VB buffers or straight to vi.ko, AWB statistics (`/dev/isp`, `vpss_isp_*`) | Builds `open_isp.ko`. Same ioctls, read/write/mmap and exports as the vendor `isp.ko`; in qemu-bench the XM535W1 streams with it, writing the same registers with the same values as `isp.ko` |
| `dwc2` | V200-series USB 2.0 host: Linux 4.19.125's DesignWare HS OTG driver, host-only, on the 3.10 kernel (`dma_desc` parameter as the vendor's) | Builds `open_dwc2.ko`. Binds to the `dwc2` device as the vendor `dwc2.ko` does; in qemu-bench the XM530V200 and XM535W1 enumerate a hub and a USB disk with it and pass a Bulk-Only read/write test, as with the vendor module |
| `xm_v200_usb0` | V200-series USB PHY reset and the `dwc2` platform device (0x50300000, IRQ 76); `USB0_CHIP=xm535` for the XM535s | Builds `open_xm_usb0.ko`. Same register sequence and device as the vendor `xm_usb0.ko` |
| `xm530v200_vpss` | XM530V200 VPSS: the memory-to-memory engine that scales each sensor's picture into three channels, from its own thread, with splicing and covers (`/dev/vpss`) | Builds `open_vpss.ko`. Same ioctls and `isp.ko`/`vo.ko` hooks as the vendor `vpss.ko`; the XM530V200 boards stream with it in qemu-bench, pictures as with the vendor module |
| `xm535w1_vi` | XM535W1 video input: the three channels' scaling and store on the XM535's VPSS block from their frame interrupts, covers, lines, OSD background luminance (`/dev/vi`) | Builds `open_vi.ko`. Same ioctls and hooks as the W1's vendor `vi.ko`; in qemu-bench the W1 streams with it, pictures as with the vendor module |
| `xm535_vpss` | XM535D1/D2 VPSS: per-frame scaling of each sensor's picture into three channels, splicing of up to three sensors, store into VB buffers, covers and lines (`/dev/vpss`) | Builds `open_vpss.ko`. Same ioctls and `isp.ko` hooks as the vendor `vpss.ko`; all D1/D2 streams in qemu-bench, pictures as with the vendor module |
| `xm533_vi` | XM533 video input: the three channels' scaling and store into VB buffers, frame-rate control, covers (`/dev/vi`) | Builds `open_vi.ko`. Same ioctls and records as the vendor `vi.ko`; in qemu-bench its pictures match the vendor module's |
| `xm_v200_rgn` | V200-series regions: overlays the OSD engine blends into VENC pictures (`osd_process`), covers and (XM535) lines through VI/VPSS, the XM535's encoder OSD (`/dev/rgn`) | Builds `open_rgn.ko`. Same ioctls as the vendor `rgn.ko`; overlays checked pixel by pixel in qemu-bench on the XM530V200, XM533 and XM535D1 |
| `xm_v200_venc` | V200-series VENC frame queue: the pictures bound to VENC, with their OSD, for the encoder (`/dev/venc_info`) | Builds `open_venc.ko`. Same ioctls and records as the vendor `venc.ko` (not yet the XM535's `/dev/venc_mem`); ran libxmv200 on all five V200-series chips in qemu-bench in place of it |
| `xm530_rgn` | XM530/XM550 regions (OSD): bitmap overlays and coloured covers that VI draws over each channel (`/dev/rgn`) | Builds `open_rgn.ko`. Same ABI as the vendor `xm530_rgn.ko`, with its inputs checked; drew overlays and covers into an XM550's streams under majestic |
| `xm510_isp` | XM510 ISP device: register access, a shared page and the frame-start signal for the vendor `isp` daemon (`/dev/isp`) | Builds `open_isp.ko`. Same ABI as the vendor `xm510_isp.ko`, without its `Bad page state` on unload or its hang after a bad address; ran majestic on an XM510 in place of it |
| `xm510_rgn` | XM510 regions (OSD): 1-bit bitmap overlays and covers that VI draws over the picture (`/dev/rgn`) | Builds `open_rgn.ko`. Same ABI as the vendor `xm510_rgn.ko`, with its inputs checked and no unbounded spin; drew overlays and covers into an XM510's stream under majestic |
| `xm530_isp` | XM530/XM550 ISP device: register access, interrupt waits, a shared block, AWB statistics and the MIPI delay sweep for the vendor ISP library (`/dev/isp`) | Builds `open_isp.ko`. Same ABI as the vendor `xm530_isp.ko`, without its unchecked event index, AWB parameters and mapping length; ran majestic on an XM550 in place of it |
| `xm510_vi` | XM510 video input: capture window, crop, scaler and frame buffers (`/dev/vi`) | Builds `open_vi.ko`. Same ABI as the vendor `xm510_vi.ko`, plus release on close and `poll`; ran majestic on an XM510 in place of it |
| `xm_vi` | XM530/XM550 video input: capture window, scalers and frame buffers (`/dev/vi`) | Builds `open_vi.ko`. Same ABI as the vendor `xm530_vi.ko`, without its vmalloc leak; ran majestic on an XM550 in place of it |

## Open encoder

`libraries/libxmvenc` drives the H.264/JPEG encoder of the XM530/XM550 and XM510 (a Hantro 8290) directly, with no vendor userspace ([#13](https://github.com/OpenIPC/openxiongmai/issues/13)). It is a static library, `libxmvenc.a`, under the MIT license, so that closed applications can link it too. The API is in `xmvenc.h`:
- `xmvenc_open` opens the encoder and identifies the chip.
- `xmvenc_chn_create` sets up a stream: H.264 (CAVLC or CABAC, fixed QP or frame-level rate control, constant or variable bitrate, IDR interval), H.265 (see below) or JPEG (quality).
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

The tool encodes IDR and P frames, at a fixed QP by default, with SPS and PPS before every IDR. `-b <kbps>` turns on frame-level rate control at `-f <fps>`, keeping the QP within `-Q min:max`. It is constant bitrate: what a frame spends under its budget is spent by the next frames. `-V` makes it variable: unspent bits are not banked, so an easy scene stays at the QP floor and the bitrate is a ceiling. `-g` sets the IDR interval (`-g 1` gives all IDR), and `-c` selects CABAC instead of CAVLC. It needs `/dev/mmz` and an encoder driver: the vendor `xm530_h264.ko`, `drivers/hx280enc`, or, on XM510, the vendor `xm510_h264.ko`. It runs on XM530/XM550 and XM510. On XM510 the vendor driver has no blocking wait, so the tool polls the status register, and the MMZ zone is too small for the tool's own buffers, so it reuses majestic's idle encoder buffers. VI on XM510 needs the vendor `isp` daemon, which majestic starts and stops with itself. The library's `cabac_tbl.c` holds the H.264 CABAC init tables in the layout from Rockchip MPP, under the Apache License 2.0.

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
- **H.264** (`h264.c`, XM533 and XM535s): `xmv200_venc_create_h264` encodes on the Hantro H1 (`/dev/h264`), which the vendor uses only for JPEG there. Baseline (CAVLC) by default, Main (CABAC) with `xmv200_venc_set_cabac`. The H1 is the XM530's 8290, programmed as `libxmvenc` programs that one (its register differences measured on an XM550), with the CABAC tables from `libxmvenc/cabac_tbl.c`.
- **OSD** (`osd.c`): `xmv200_osd_create`, `_bitmap` and `_attach` put a bitmap on a stream: ARGB1555, ARGB8888, or 1 or 2 bits a pixel with up to four AYUV colours (`xmv200_osd_color`), drawn at 1x or 2x. `rgn.ko` (or `open_rgn`) has the OSD engine blend it into each of the stream's pictures before they are encoded. `_set` moves or recolours one, and `_detach` and `_destroy` remove it. A stream takes 8 (4 on the XM535). On the XM535s, `XMV200_OSD_ENC_1BIT` / `_2BIT` overlays are drawn by the encoder itself (rgn.ko's formats 4 and 5, in 16x16 tiles), which libxmv200 asks for with every picture (`VENC_OSD`), as the vendor encoder does.
- **Motion detection** (`md.c`): `xmv200_md_set`/`_get`, per sensor, the vendor isp daemon's way: an 8x8 grid of block brightness compared about every 20 frames, the darker grid scaled to the brighter first so that a change of light is not motion; a block moved when it changed by more than its old value over the level's divisor (levels 1..6: 2..12). Motion is 2% of the watched blocks (a region mask, a byte a row) or more; more than half, or over 1.2x the light, is a change of scene. It samples the frames given to `xmv200_ae_frame`.
- **Blind detection** (`md.c`): `xmv200_bd_set`/`_get`, the daemon's: about every 20 frames the mean of AE's statistics grid is checked against the level's thresholds: covered (dark), blinded (bright), or a sudden change of it. What it saw stays until read.
- **Privacy masks** (`mask.c`): `xmv200_mask_set` puts up to 8 filled rectangles over a sensor's picture, in thousandths of it, so each covers the same thing in every stream that shows the sensor. Each is an `rgn.ko` cover per stream, attached to the VPSS channel that makes the stream (VI's on the XM533 and XM535W1), which draws it before encoding. Setting a mask again moves it; `xmv200_mask_clear` removes it.
- **Image controls** (`image.c`): `xmv200_image_set` takes brightness, contrast, saturation, hue and sharpness (0..100, 50 being the tuning's own), mirror and flip, mono (no colour) and anti-flicker (50/60 Hz). They act on the isp.ko blocks the vendor's isp daemon uses for the same settings: the luma and chroma gains (0x49de), the tone curve (0x49df), the hue shift (0x49e4) and the sharpening strength (0x49f0), each scaled from the block the tuning last sent. Mirror and flip go through the sensor's driver, and anti-flicker keeps AE's exposure a whole number of the mains light's half periods.
- **AE/AWB** (`ae.c`): AE is the vendor isp daemon's: it reads the ISP's 32x16 zone statistics (R, G, B sums, `read()` on `/dev/isp`), weights them to an 8-bit luma (7x7 metering weights, the mode's part of the sorted zones), and steps the exposure towards the target by (ratio - 1) / speed, then splits it into exposure lines, the sensor's analog and digital gain and isp.ko's digital gain. Target, tolerance, speed and weights come from the tuning's shared page (the AE context). Without the statistics it falls back to sampling the frames.
- **AWB** (`awb.c`): the vendor isp daemon's, on the same statistics and the sensor's calibration in the tuning's shared page (its AWB context: the white region, the colour temperature curve, three CCMs, zone weights, speed). Zones inside the white region, neither too dark nor too bright, weighted by how grey they are, give the gains (isp.ko 0x4965, the VICU's white balance), which move towards them at the vendor's speed; the colour temperature on the calibration curve picks the CCM (0x4966) between the three, and on the XM530V200 the saturation by gain. A tuning without the calibration falls back to gray world.

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
| XM530V200 | F37P, GC3003, H63, H63P, IMX307, IMX335, MIS2009, MIS4001, MIS40C1, MIS5001, Q03, Q03P, SC035GS, SC1336, SC1345, SC1346, SC1A4T, SC200AI, SC202CS, SC2210, SC223A, SC223AP, SC231AI, SC2331, SC2335, SC2336, SC2336P, SC2337P, SC307C, SC307E, SC3235, SC3335, SC3338, SC401AI, SC4210, SC430AI, SC4336, SC500AI, SC5239AI, SC831HAI |
| XM535D1/D2 | GC1084, GC2083, H63P, IMX307, MIS2009, MIS4001, MIS40C1, SC1346, SC1A4T, SC200AI, SC223AP, SC2331, SC2336, SC2336P, SC3335, SC3338, SC401AI, SC4336, SC500AI |
| XM535W1 | IMX307, SC1346, SC1A4T, SC200AI, SC2336, SC2336P, SC3338, SC500AI |
| XM533 | GC1084, MIS2009, MIS4001, MIS40C1, OV2735, SC1346, SC1A4T, SC223A, SC231AI, SC2331, SC2336, SC2337P, SC3338, SC401AI, SC4336, SC500AI |

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
XMV200_CODEC=h264main xmv200-test run x3-wq-b 100   # H.264 Main (CABAC) on the XM533's H1; h264 for Baseline
XMV200_MD=4 xmv200-test run x6c-weq 300   # motion detection at level 4, its state every 8 frames
XMV200_MASK=100,100,300,200 xmv200-test run x6c-weq 100   # a black privacy mask (thousandths: x, y, w, h), later moved and cleared
XMV200_IMAGE=60,55,40,50,70,1,0,0,50 xmv200-test run x6c-weq 100   # image controls: brightness, contrast, saturation, hue, sharpness, mirror, flip, mono, anti-flicker
```

## CI

Every push and pull request to `main` runs these jobs, each using the OpenIPC musl toolchain for its SoC (`toolchain.xiongmai-xm5x0`).

- **Libraries cross-compile (XM530):** builds all four userspace libraries and libxmv200 (with `tools/xmv200-test`), checks that the expected entry points are exported (`sensor_register_callback`, `XM_I2C_Ioctl`, `XM_MPI_*`, ...), and checks that the shared objects depend only on libc.
- **Kernel modules (xm510 / xm530):** builds the modules against the OpenIPC kernels `openipc/linux@xiongmai-xm510` (3.0.101) and `@xiongmai-xm530` (3.10.103), the same way the firmware does: with the firmware's `<soc>.generic.config` and its generic kernel patches. `xm_i2c`, `hx280enc`, `xm_mmz` and `xm_hevc` build for both; the XM530/XM550 and V200-series modules (every `xm_v200_*`, `xm53*`, `dwc2`, `hx280enc` with `H264_CHIP=v200`) for the 3.10 kernel, which the V200 series runs. Each module's vermagic is checked.
- **libxmv200 sensor drivers against their tunings:** `tools/xmv200-sns` runs every sensor driver on the build machine against each chip family's tunings; a driver has to make exactly the sensor writes its tuning recorded.

All external inputs are pinned: the kernel commits, the firmware commit, and the toolchain sha256 checksums. OpenIPC rebuilds its toolchain tarballs in place, so a checksum failure means upstream published a new build and the checksum in `build.yml` needs updating.

[![Build Status](https://github.com/OpenIPC/openxiongmai/actions/workflows/build.yml/badge.svg)](https://github.com/OpenIPC/openxiongmai/actions/workflows/build.yml)

## License

GPL v3. See [LICENSE](LICENSE). The exceptions:
- `libraries/libxmvenc`, `libraries/libxmv200`, `tools/xmvenc-test`, `tools/xmhevc-test`, `tools/xmv200-test`, `tools/xmv200-capture`, `tools/xm-vi-hang` and `tools/xm-rgn-test` are under the MIT license, so that they can be linked into applications under any license. See their `LICENSE` files.
- `libraries/libxmvenc/cabac_tbl.c` and `cabac_tbl.h` are under the Apache License 2.0, from Rockchip MPP.
