# libxmisp

An open ISP for Xiongmai SoCs (MIT). It is written clean-room: from traces
of the vendor daemon (`tools/xm-isp-trace`), the register map and the
hardware's behaviour, never from the vendor's code. `tools/xmispd` is the
daemon on top of it.

M1, XM510 only. `xmisp_start()` does three things:
- brings VI, the sensor and the ISP up from a sensor script;
- applies that script's fixed 3A state;
- leaves exposure and white balance where they are.

AE and AWB come next, in an algorithm layer that doesn't depend on the SoC,
so the XM530 can share it.

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
    -m libraries/isp/include/xm/xm510_isp.h > h62_720p.c
```

| Script | Sensor | Mode | Trace |
|---|---|---|---|
| `h62_720p.c` | JX-H62 (I2C `0x30`, ID `0xa062`) | 1280x720, 25 fps | lab XM510, vendor daemon under majestic |

The fixed state is one converged frame of the vendor's 3A in the light of
the capture. A different light gets the wrong exposure until AE exists.
