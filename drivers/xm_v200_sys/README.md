# xm_v200_sys

`xm_v200_sys` is the V200 series' system module (XM530V200, XM533, XM535D1/D2/W1), built as `open_sys.ko`. It replaces the vendor `sys.ko` without changes to what uses it.

| `/dev/sys` ioctl | |
|------------------|-|
| `0x400c5300` | the bind table: 64 entries of `{src mod, dev, chn, dst mod, dev, chn}`, the whole 0x600 bytes each call; shown in `/proc/umap/sys` (the modules bind through their own calls; the table is for the record) |
| `0x40085301` | `{register, value}`: writes a register of the clock block (`0x20000000`, its first 0x80 bytes), then locks the block again (0 to `+0`) |
| `0x40085302` | `{register, value}`: reads one |

`sys_set_clk()` and `sys_get_clk()` are the same calls for modules; the XM535's `isp.ko` uses them. The older chips' `sys.ko` has only the bind table; the clock calls do no harm there.

The module names in `/proc/umap/sys` are the vendor's: 7 venc, 8 vpss, 15 vo, 16 vi, 21 ai, 23 aenc, ...

```sh
make -C drivers/xm_v200_sys KDIR=... ARCH=arm CROSS_COMPILE=...
insmod open_sys.ko   # after open_mmz, which makes /proc/umap
```
