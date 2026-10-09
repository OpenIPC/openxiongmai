# OpenXiongmai

Open-source kernel modules and userspace libraries for Xiongmai (XM) IP camera SoCs. Most of the code targets the XM530. The kernel modules are set up to build against the XM510 vendor kernel.

The goal is to replace the proprietary SDK that Xiongmai ships with its camera boards. The code is intended for [OpenIPC](https://openipc.org/) firmware, and you can also use it in your own projects.

## Supported hardware

| SoC | Status | What's here |
|-----|--------|-------------|
| XM530 | Primary target | Sensor libraries, ISP application library, MPI reimplementation |
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
│   └── xm_i2c/                  /dev/xm_i2c misc-device (stub)
│
└── libraries/                   Userspace libraries
    ├── include/                 Reverse-engineered MPI headers (re_*.h)
    ├── isp/include/             Vendor-style SDK headers
    │   ├── isp/                 ISP / AE / AWB / sensor interfaces
    │   ├── mpi/                 SYS / VI / VO / AIO interfaces
    │   └── xm/                  Camera.h (SENSOR_CHIP_* enum), per-SoC register maps
    ├── libmpi/                  libopenmpi.a: MPI reimplementation
    ├── libispapp_XM530/         libispapp_XM530.so: ISP application layer
    ├── libsns_X123_XM530/       libsns_X123_XM530.so: "X123" sensor family
    └── libsns_X50_XM530/        libsns_X50_XM530.so: "X50" sensor family
```

## Building

There's no top-level build. Each component has its own Makefile and is built separately with an ARM cross toolchain. Using the OpenIPC SDK:

```bash
export PATH=/path/to/openipc_sdk/bin:$PATH

# Userspace libraries
make -C libraries/libsns_X50_XM530  CROSS_COMPILE=arm-openipc-linux-musleabi-
make -C libraries/libsns_X123_XM530 CROSS_COMPILE=arm-openipc-linux-musleabi-
make -C libraries/libispapp_XM530   CROSS_COMPILE=arm-openipc-linux-musleabi-
make -C libraries/libmpi            CROSS_COMPILE=arm-openipc-linux-musleabi-

# Kernel modules: point KDIR at a configured kernel tree
make -C drivers/hx280enc KDIR=/path/to/linux
make -C drivers/xm_i2c   KDIR=/path/to/linux
```

The Makefiles set different default toolchains (`arm-xm-linux-`, `arm-linux-` or `arm-openipc-linux-musleabi-`), so pass `CROSS_COMPILE` explicitly. `libmpi` and the kernel module Makefiles also hard-code `PATH` to a vendor toolchain directory. Override it from the command line, for example `make PATH="$PATH" ...`, or give an absolute `CROSS_COMPILE` prefix. Most components build with `-Werror`.

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
| `hx280enc` | Hantro 6280/7280/8270/8290 hardware video encoder | Driver source |
| `xm_i2c` | Registers `/dev/xm_i2c` and `/dev/xm_i2c1` | Stub: read, write and ioctl are not implemented yet |

## CI

Every push and pull request to `main` runs these jobs, each using the OpenIPC musl toolchain for its SoC (`toolchain.xiongmai-xm5x0`).

- **Libraries cross-compile (XM530):** builds all four userspace libraries, checks that the expected entry points are exported (`sensor_register_callback`, `XM_I2C_Ioctl`, `XM_MPI_*`, ...), and checks that the shared objects depend only on libc.
- **Kernel modules (xm510 / xm530):** builds `xm_i2c` against the OpenIPC kernels `openipc/linux@xiongmai-xm510` (3.0.101) and `@xiongmai-xm530` (3.10.103), the same way the firmware does: with the firmware's `<soc>.generic.config` and its generic kernel patches. `hx280enc` isn't built yet because its source references `struct JpegProcType`, which is never defined.

All external inputs are pinned: the kernel commits, the firmware commit, and the toolchain sha256 checksums. OpenIPC rebuilds its toolchain tarballs in place, so a checksum failure means upstream published a new build and the checksum in `build.yml` needs updating.

[![Build Status](https://github.com/OpenIPC/openxiongmai/actions/workflows/build.yml/badge.svg)](https://github.com/OpenIPC/openxiongmai/actions/workflows/build.yml)

## License

GPL v3. See [LICENSE](LICENSE).
