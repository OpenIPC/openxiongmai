# xm_v200_i2c

`xm_v200_i2c` is the V200 series' I2C masters (XM530V200, XM533, XM535D1/D2/W1), which talk to the sensors. It builds as `open_i2c.ko` and replaces the vendor `i2c.ko` without changes to what uses it.

There are three buses, each a misc device: `/dev/i2c0`..`2`, or on the XM533 `/dev/xm_i2c`, `/dev/xm_i2c1` and `/dev/xm_i2c2`. A call takes the vendor's 20-byte record `{dev, reg, reg_bytes, val, val_bytes}`:

| ioctl | |
|-------|-|
| 1 | write `reg`, then `val` |
| 3 | write `reg`, read `val_bytes` into `val` (the record goes back) |
| 4 | set the bus up again |

Register and value go most significant byte first.

The controller (bus n at `0x10000000 + n * 0x1000`):

| offset | |
|--------|-|
| `+0x00` | control: `0x40`/`0x80` reset the TX/RX FIFO, `0xb` sends the TX FIFO, `0x803` then reads, `1` stops |
| `+0x0c` | device address |
| `+0x10` / `+0x14` | TX / RX FIFO |
| `+0x18` / `+0x1c` | bytes to send / receive |
| `+0x20`..`+0x38` | timing |
| `+0x44` / `+0x48` | interrupt clear / status (`0x10` sent, `0x20` received) |

As the vendor's, it polls the status every 10 µs, about 40 ms in all. Opening a bus sets its pins to I2C in the pin mux at `0x10020000`.

The chips differ (`I2C_CHIP=` at build time):

| | XM530V200 (default) | XM533 | XM535 |
|-|-|-|-|
| devices | `i2c0..2` | `xm_i2c`, `xm_i2c1`, `xm_i2c2` | `i2c0..2` |
| timing | `0x23` ×5, `0x13`, `0x34` | `0xa8` ×5, `0x50`, `0xfc` | as the XM533 |
| pins (bus 0; 1; 2) | `0x80`,`0x88`=`0x82`; `0x28`,`0x30`=`0x84`; `0x2c`,`0x34`=`0x84` | as the XM530V200 | `0x80`,`0x88`=`0x72`; `0x28`,`0x2c`=`0x74`; `0x178`,`0x17c`=`0x72` |
| bus reset at set-up | clock block `0x20000090 + 4n` | none | as the XM530V200 |
| TX FIFO reset before a write | yes | no | yes |

```sh
make -C drivers/xm_v200_i2c KDIR=... ARCH=arm CROSS_COMPILE=... I2C_CHIP=xm535
insmod open_i2c.ko
```

The bus reset (`I2C_IOC_INIT`) takes no record, so a null argument works.
