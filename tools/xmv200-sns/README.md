# xmv200-sns

How libxmv200's sensor drivers (`libraries/libxmv200/sns_*.c`, interface `sns.h`) are made.

The vendor's sensor library, libsns, is built into its `isp` daemon. For each sensor it has a probe, an init with a register table per mode, and the functions the AE calls: frame rate, frame length, exposure, gain and its tables, mirror and flip. A driver here is those functions rewritten from the daemon's disassembly. The mode tables are the daemon's own data. `extract.py` copies them out of a known build of `isp`, which it checks by MD5:

```sh
./extract.py isp sc2336 > ../../libraries/libxmv200/sns_sc2336_regs.c
```

`xm530v200/sc2336.tab` lists those tables for the XM530V200 build, one per line: the array, its address, the number of pairs the vendor's init loop writes, and what the vendor logs for the mode. A `+ REG VAL` line appends a pair the vendor's init writes after the table. SC223AP's "mipi max" mode, for one, is its MIPI table and then 0x3650/0x3651.

The X6C-WEQ's firmware (V5.04.R02.20241125) has the XM530V200 `isp`. Its libsns:

- **The probe** (at 0x46608) reads the chip ID and fills a 0x38-byte record per channel at 0xb1268:
  - +0: a type code;
  - +5: the I2C address;
  - +0x28: init;
  - +0x30: the AE functions' constructor;
  - +0x2c and +0x34: the ISP and AWB defaults' constructors.
- **`cmos_set_sns_attr`** (0x480ec) sets the record's mode fields: +0x14 MIPI, +0x18 resolution, +0x20 frame rate, +0x21 lanes.
- **The AE functions' table** has: [0] the AE's limits, [1] frame rate, [3] frame length, [4] exposure, [6] gain, [9] and [10] the analog and digital gain's steps.

A driver is checked against the captures (`tools/xmv200-capture`). A tuning's sensor runs are libsns's init, its frame rate and the AE's first exposure and gain. For the tuning's mode, the driver has to write the same registers in the same order. Where a sensor's runs are laid out differently, its driver says so (`n_runs`, `run`). `make check` (or `make check FAMILY=xm530v200`) builds `check.c` on the build machine with every driver and tuning. It replays each tuning's runs through its driver and lists where the writes differ.

Four builds are known, one per chip family, each with its own tables and its own version of some functions. A family's driver is a `family` section in the sensor's `sns_<sensor>.c`, or a file of its own. It takes only that family's tunings.

| Family | `isp` | libsns |
|--------|-------|--------|
| xm530v200 | X6C-WEQ V5.04.R02.20241125 | Nov 20 2024 e89d30f |
| xm533 | X3-WQ-B V5.04.R02.20250828 | Aug 26 2025 b7cb00d |
| xm535 (D1/D2) | X6E-WEQ V5.07.X02.20250906 | Aug 12 2025 d2db8e4 |
| xm535w1 | X2W-WNG V5.00.R02.20240410 | Apr 7 2024 28505 |

## The drivers

Every family's tunings give the same writes through their drivers (`make check`). Each tuned sensor also streams in qemu-bench through its driver on its family's machine:

| Family | Tuned sensors |
|--------|---------------|
| xm530v200 | 33: the SmartSens (with SC202CS, SC2210, SC307E, SC5239AI), IMX307, IMX335, F37P, Q03, Q03P, H63, H63P, GC3003, MIS2009, MIS40C1 |
| xm533 | 16: with GC1084, OV2735, MIS2009, MIS4001, MIS40C1 |
| xm535 | 14: with IMX307, H63P, GC1084, GC2083, MIS2009 (SC1336, SC1345, SC307C and SC3235 have no D1/D2 driver: its daemon only logs them, so there is no tuning for them) |
| xm535w1 | 8: with IMX307 |

The other families add these sensors, unverified:

- XM533: SC2336P, SC2337P, SC307E, MIS2009, MIS4001, MIS40C1, GC1084, OV2735;
- XM535: SC2337P, SC3338, SC4336, SC500AI;
- W1: SC3335, SC223A, SC401AI, IMX307.

The XM530V200's:

| Driver | Vendor | Checked |
|--------|--------|---------|
| SC1336, SC1345, SC1346, SC1A4T, SC200AI, SC223AP, SC231AI, SC2331, SC2336, SC2336P (the X6C-WEQ's "SC2337P"), SC307C, SC3235, SC3335, SC3338, SC401AI, SC430AI, SC4336, SC500AI, SC831HAI | SmartSens | the same writes as their tunings (`make check`); each streams in qemu-bench through its driver |
| SC035GS, SC202CS, SC2210, SC223A, SC2335, SC2337P, SC307E, SC4210, SC5239AI | SmartSens | no tuning: ported, not run |
| GC3003 | GalaxyCore | no tuning |
| IMX307, IMX335 | Sony | no tuning |
| MIS2009, MIS4001, MIS40C1, MIS5001 | MIS | no tuning |
| F37P, Q03, Q03P, H63, H63P | SOI (JX) | no tuning |

GC2083's chip ID is in the probe, but this build has no driver for it.

`board.c` reads a SmartSens chip ID first. Where none it knows answers, it tries the drivers' probes (`probe`, `probe_addr`), each as the vendor's libsns probes that sensor. Such a sensor's tuning is registered under the ID its probe gives: Sony 0xa0b2 (IMX307) and 0x0335 (IMX335), SOI's own ID. GalaxyCore, MIS and OV2735 parts are captured with qemu-bench's generic ID-pair sensor, e.g. `SENSOR_MODEL='galaxycore-sensor;reg16=on;id-reg=0x3f0;id=0x1084@37'`. To capture one: `capture.sh` with `SENSOR_MODEL=hisi-imx335@1a` (qemu-bench's `sensor=` machine property; write a second address after `,,`), then `gen_tuning.py`.

Not in the interface yet:

- the HDR modes (SC200AI, SC223A, SC4210);
- modes picked by product settings rather than by MIPI/DVP, lanes or 720p (SC1336 50 fps, SC500AI 4 lanes, SC5239AI and SC831HAI's other frame rates, IMX307 4 lanes, MIS4001 3M/4M);
- the dual-sensor sync modes.
