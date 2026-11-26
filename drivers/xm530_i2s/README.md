# xm530_i2s

`xm530_i2s` is the XM530/XM550's audio input and output, at `/dev/ai` and `/dev/ao`, and builds as `open_i2s.ko`. It replaces the vendor `xm_i2s.ko`.

The codec talks I2S to one block at 0x10067000 (IRQ 49), both ways. Each interrupt moves 160 16-bit samples between its FIFO and a FIFO in memory. Reads and writes wait unless `O_NONBLOCK`; there are no ioctls. `/proc/umap/audio` counts the interrupts.

The vendor module opens `/proc/umap/audio` with `seq_open()` and never releases it, so each open leaks 4 KiB that not even `rmmod` returns ([OpenIPC/firmware#2956](https://github.com/OpenIPC/firmware/issues/2956)).

```sh
make -C drivers/xm530_i2s KDIR=... ARCH=arm CROSS_COMPILE=...
insmod open_i2s.ko   # then acodec.ko
```
