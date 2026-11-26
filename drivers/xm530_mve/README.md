# xm530_mve

`xm530_mve` is the XM530/XM550's H.265 encoder device (an Arm Mali-V550, host protocol 2) at `/dev/mv500`, and builds as `open_mve.ko`. It replaces the vendor `mve_base.ko`, and works on the vendor `mve_rsrc.ko` (scheduler, registers, power) with the vendor firmware `hevcenc.fwb`.

It encodes H.265 from NV12 in MMZ. Decoding, other codecs, secure sessions and other kinds of buffers are refused. What else differs from `mve_base.ko` is listed at the top of `xm530_mve.c`; the reason for it is that the vendor module leaks 4 KiB for each open of `/proc/umap/h265` ([OpenIPC/firmware#2956](https://github.com/OpenIPC/firmware/issues/2956)).

```sh
make -C drivers/xm530_mve KDIR=... ARCH=arm CROSS_COMPILE=...
insmod mve_rsrc.ko; insmod open_mve.ko
```
