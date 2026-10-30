# xmv200-sns

How libxmv200's sensor drivers (`libraries/libxmv200/sns_*.c`, interface `sns.h`) are made.

The vendor's sensor library, libsns, is built into its `isp` daemon. For each sensor it has a probe, an init with a register table per mode, and the functions the AE calls: frame rate, frame length, exposure, gain and its tables, mirror and flip. A driver here is those functions rewritten from the daemon's disassembly. The mode tables are the daemon's own data. `extract.py` copies them out of a known build of `isp`, which it checks by MD5:

```sh
./extract.py isp sc2336 > ../../libraries/libxmv200/sns_sc2336_regs.c
```

The X6C-WEQ's firmware (V5.04.R02.20241125) has the XM530V200 `isp`. Its libsns:

- **The probe** (at 0x46608) reads the chip ID and fills a 0x38-byte record per channel at 0xb1268:
  - +0: a type code;
  - +5: the I2C address;
  - +0x28: init;
  - +0x30: the AE functions' constructor;
  - +0x2c and +0x34: the ISP and AWB defaults' constructors.
- **`cmos_set_sns_attr`** (0x480ec) sets the record's mode fields: +0x14 MIPI, +0x18 resolution, +0x20 frame rate, +0x21 lanes.
- **The AE functions' table** has: [0] the AE's limits, [1] frame rate, [3] frame length, [4] exposure, [6] gain, [9] and [10] the analog and digital gain's steps.

A driver is checked against the captures (`tools/xmv200-capture`). A tuning's three sensor runs are libsns's init, its frame rate and the AE's first exposure and gain. For the tuning's mode, the driver has to write the same registers in the same order.

Only the XM530V200's build is known so far. The XM535s', the W1's and the XM533's `isp` have tables of their own.
