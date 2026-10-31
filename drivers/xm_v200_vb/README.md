# xm_v200_vb

`xm_v200_vb` is the V200 series' video buffer pools (XM530V200, XM533, XM535D1/D2/W1), built as `open_vb.ko`. It replaces the vendor `vb.ko` without changes to the modules that use it: `isp`, `vpss` or `vi`, and `venc` take their frame buffers from it and hand them on to each other. They read and write a buffer's descriptor directly, so its layout is the vendor's:

| word | |
|------|-|
| 0, 1 | the list it is on |
| 2, 3 | the list of every buffer |
| 4, 5 | physical address, size |
| 6..8 | stride, width, height |
| 9 (10 on the XM535) | reference count |
| 12..14 (14..16) | bind id, dev, chn |

The descriptor is 64 bytes on the XM530V200 and XM533 and 72 on the XM535 (`VB_LAYOUT=v200` or `xm535` at build time).

Exports: `vb_get_buf(list, size, &buf, ms)`, `vb_put_buf(list, buf)`, `vb_add_buf_reference(buf)`, `vb_get_length(list, &n)`, `vb_flush_dcache(phys, virt, size)`, and `vb_find_buf(addr)`: the descriptor at that address if it is one of the pools', for modules that are handed one by user space (vdec).

The lists:
- **0, free:** in size order. A get takes the first buffer at least as big as asked for (mode 1: of that size) and counts a reference. A put returns the buffer once its last reference goes.
- **1, 2, 3:** VPSS's, VENC's and one more queue.

A get waits forever (`ms < 0`), not at all (0) or `ms` milliseconds.

`/dev/vb`:

| ioctl | |
|-------|-|
| `0x41044201` | the pools: `{count, {size, count}[32]}` |
| `0x4200` | make them: an MMZ block `vb[size][n]` per buffer, of size + 0x10 to 32 bytes, with the "MEMORY BARRIERS" marker in its last 16 bytes (the XM535's also puts two empty buffers at the head of the free list) |
| `0x4204` | the free and VENC lists' blocks back to MMZ |
| `0x40044205` | the mode |
| `0xc0504202` / `0x40504203` (`0xc0584202` / `0x40584203` on the XM535) | get and put from user space, with a copy of the descriptor and its address; this driver checks the address is one of its buffers, which the vendor's takes on trust |

`/proc/umap/vb` lists the pools and every list.

```sh
make -C drivers/xm_v200_vb KDIR=... ARCH=arm CROSS_COMPILE=... VB_LAYOUT=xm535
insmod open_vb.ko   # after mmz (open_mmz or the vendor's)
```

Inputs are checked where the vendor's are not: a pool size that would wrap the buffer's room (over 2 GiB) is refused, and a put whose stride x height is larger than the buffer is refused (`-EINVAL`), as the modules after it take those for the picture's extent.
