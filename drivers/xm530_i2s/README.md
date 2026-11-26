# xm530_i2s

`xm530_i2s` is the XM530/XM550's and the XM510's audio input and output, at `/dev/ai` and `/dev/ao`, and builds as `open_i2s.ko`. It replaces the vendor `xm_i2s.ko` of either chip; the kernel's `CONFIG_ARCH_XM510` picks the XM510's layout.

The codec talks I2S to one block, both ways: at 0x10067000 (IRQ 49) on the XM530, at 0x10060000 (IRQ 9) on the XM510. Each interrupt moves 160 16-bit samples between its FIFO and a FIFO in memory. Reads and writes wait unless `O_NONBLOCK`; there are no ioctls. `/proc/umap/audio` counts the interrupts, in each chip's vendor layout.

The XM510 differs in more than the addresses. Its vendor driver muxes the pins through bits of pad register 0 and +0x20 rather than clearing pads 0 to 4, writes 0x60 to I2S +0x0c, holds the reset for 250 ms, and leaves bit 3 of the control register clear. With that bit set, the receiver runs with no codec on the bus, so it is taken to select the I2S clock; the XM510 build leaves it clear as the vendor does.

Both vendor modules open `/proc/umap/audio` with `seq_open()` and never releases it, so each open leaks 4 KiB that not even `rmmod` returns ([OpenIPC/firmware#2956](https://github.com/OpenIPC/firmware/issues/2956)).

```sh
make -C drivers/xm530_i2s KDIR=... ARCH=arm CROSS_COMPILE=...
insmod open_i2s.ko   # then acodec.ko
```
