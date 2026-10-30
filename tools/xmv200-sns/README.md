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

Only the XM530V200's build is known so far. The XM535s', the W1's and the XM533's `isp` have tables of their own.

## The XM530V200's drivers

| Driver | Vendor | Checked |
|--------|--------|---------|
| SC1336, SC1345, SC1346, SC1A4T, SC200AI, SC223AP, SC231AI, SC2331, SC2336, SC2336P (the X6C-WEQ's "SC2337P"), SC307C, SC3235, SC3335, SC3338, SC401AI, SC430AI, SC4336, SC500AI, SC831HAI | SmartSens | the same writes as their tunings (`make check`); each streams in qemu-bench through its driver |
| SC035GS, SC202CS, SC2210, SC223A, SC2335, SC2337P, SC307E, SC4210, SC5239AI | SmartSens | no tuning: ported, not run |
| GC3003 | GalaxyCore | no tuning |
| IMX307, IMX335 | Sony | no tuning |
| MIS2009, MIS4001, MIS40C1, MIS5001 | MIS | no tuning |
| F37P, Q03, Q03P, H63, H63P | SOI (JX) | no tuning |

GC2083's chip ID is in the probe, but this build has no driver for it. libxmv200 probes only SmartSens IDs (`board.c`). Each other driver's comment says how the vendor probes that sensor: the I2C address, the register widths, and the chip ID's registers.

Not in the interface yet:

- the HDR modes (SC200AI, SC223A, SC4210);
- modes picked by product settings rather than by MIPI/DVP, lanes or 720p (SC1336 50 fps, SC500AI 4 lanes, SC5239AI and SC831HAI's other frame rates, IMX307 4 lanes, MIS4001 3M/4M);
- the dual-sensor sync modes.
