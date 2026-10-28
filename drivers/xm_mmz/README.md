# xm_mmz

`xm_mmz` is the media memory zone of the XM530/XM550 and the XM510, built as `open_mmz.ko`. It replaces the vendor `mmz.ko` without changes to the drivers or applications that use it.

The SDK and the video drivers take their DMA buffers from one physically contiguous region that Linux does not use. `mem=` on the kernel command line keeps it out, and the module gets it as `mmz_start` and `mmz_size`. `load_xiongmai` passes what lies between `mem=` and the end of DDR:

```sh
insmod open_mmz.ko mmz_start=0x85000000 mmz_size=0x3000000   # XM550: mem=80M of 128 MB
```

Load it before the drivers that use it (`open_vi`/`xm530_vi`, `xm530_rgn`).

## XM510

The XM510 (3.0 kernel) has an older `mmz.ko`, and a build against a kernel with `CONFIG_ARCH_XM510` does what that one does:
- **Layout:** each block starts on the page after the previous one, with no overrun marker.
- **`/proc/umap/mmz`:** the zone, then start, size and name per block, with no overrun flag and no free-space line.
- **Interface:** the old module's is a subset of this one: ioctls 0–3 on 40-byte records, and `mmz_alloc`/`mmz_free`. It has no `mmap`, so XM510 user space maps the zone through `/dev/mem`.
- **No CMA mode:** the XM510 kernel has no CMA, and 32 MB of DDR leaves nothing to lend.

```sh
insmod open_mmz.ko mmz_start=0x81200000 mmz_size=0xe00000   # XM510: mem=18M of 32 MB
```

On an XM510 + JX-H62 with majestic, `open_mmz` laid out the same 12 blocks at the same addresses as the vendor module, and video streamed.

## Blocks

Every buffer is a named block, and each name exists once. A driver finds a buffer another driver made by asking for its name: VI keeps its frames in `vi_way[N]`.

The allocator is the vendor's, kept as it is:
- **Layout:** blocks are laid one after another from `mmz_start`, each starting on a page.
- **Overrun check:** after each block, before the next page, sits a 16-byte marker (`MEMORY BARRIERS`). `/proc/umap/mmz` reports a block whose marker was overwritten.
- **Freeing:** freeing a block succeeds and does nothing. Only a reset releases memory, and it releases every block at once. So whoever starts the media pipeline resets first; majestic and the SDK do.
- **Caching:** "cached" is a flag a block carries for `mmap`. It is mapped write-back instead of uncached.

## CMA mode

With `allocator=cma`, the blocks come from the kernel's CMA area through `dma_alloc_coherent()` instead of from the zone. Memory the media pipeline isn't using stays with Linux, and the pipeline still gets contiguous buffers. It needs an XM530/XM550 kernel with `CONFIG_CMA` and the two OpenIPC/linux changes that keep the vendor modules' ABI (OpenIPC/linux#157), and it needs `open_vi`: the vendor `xm530_vi.ko` `ioremap()`s its buffers.

```sh
# XM550, kernel booted with mem=127M cma=48M: 1 MB zone at the top of DDR
insmod open_mmz.ko allocator=cma mmz_start=0x87f00000 mmz_size=0x100000
```

- **Kernel blocks:** `mmz_alloc()` and its variants take CMA too, except names in `zone_names` (default `rgn`). Those still come from the zone, which therefore stays, but small. `xm530_rgn.ko` `ioremap()`s its 18 KB block, and ARM refuses to `ioremap()` RAM.
- **Reset:** a reset forgets the CMA blocks, but frees them only `cma_hold_ms` (5000) later, because the previous pipeline's DMA may run into them until the next one sets the hardware up (VI keeps capturing after majestic exits). A name allocated again within that time, at the same or a smaller size, gets its block back at the same address. On a camera restart, that is every block.
- **Running out:** an allocation that finds CMA (or `cma_size`, if set) full while blocks are held waits for the hold to end and tries once more. A restart that asks for bigger buffers than the last run can take up to `cma_hold_ms` longer.
- **`mmap`:** a mapping must stay inside one CMA block and keeps that block until it is unmapped. Uncached blocks are mapped with the attributes the kernel's own mapping of them has. Other RAM is refused; register space is mapped as before.
- **`/proc/umap/mmz`:** lists the CMA blocks among the zone's (stale ones left out). It adds one line at the end, `CMA: <bytes>`, with what CMA holds, stale blocks included.

Measured on an XM550 + IMX335 with majestic (1080p H.264 main stream, sub stream, JPEG), booted `mem=127M cma=48M`:
- all blocks but `rgn` came from CMA, 22 MB in 13 blocks, with 73 MB of memory free;
- restarts reused the same 13 blocks at the same addresses;
- after a reset, a 20 MB allocation that didn't fit beside the held blocks succeeded after 5.05 s;
- a mapped block outlived a reset until it was unmapped.

## Kernel interface

| Function | Does |
|---|---|
| `int mmz_alloc(const char *name, u32 size, u32 *phys)` | New uncached block. Fails if the name exists or the zone is full. |
| `int mmz_alloc_cached(const char *name, u32 size, u32 *phys)` | The same, cached |
| `int media_mem_alloc(const char *name, u32 size, u32 *phys, int cached)` | Either |
| `int mmz_get_addr(const char *name, u32 *size, u32 *phys)` | Look a block up by name |
| `int mmz_free(void)` | Does nothing, as in the vendor module |

They return 0 or −1.

## `/dev/mmz`

Every call takes a record `{ char name[32]; u32 size; u32 phys; u32 vaddr; }` (44 bytes). XM510's is 40 bytes, without `vaddr`, and the module accepts either.

| ioctl | Does |
|---|---|
| `_IO('M', 0)` | Reset: release every block |
| `_IOWR('M', 1)` | Allocate `name` and `size`; `phys` comes back, 0 on failure |
| `_IOW('M', 2)` | Free: does nothing |
| `_IOR('M', 3)` | Look up `name`: `size` and `phys` come back |
| `_IOWR('M', 4)` | Allocate, cached |
| `_IOW('M', 5)` | Flush the CPU caches for the mapping at `vaddr` and `size` (and the outer cache by `phys`), then mark the block cached |
| `_IOW('M', 6)` / `_IOW('M', 7)` | Mark the block at `phys` cached / uncached, for later mappings |

The record goes back to the caller whether or not the call succeeded. A failed allocation or lookup returns −1 with `phys` 0. As in the vendor module, a command it does not know (a record of another size included) logs `ioctl cmd error` and returns 0, so callers test `phys`.

`mmap` with the physical address as the offset maps it write-back for a cached block and uncached otherwise. Like the vendor module, it maps any physical address (in CMA mode, see above). It logs the first mapping outside the zone, so anything that relies on that can be found before it is refused.

## `/proc/umap/mmz`

It has the same layout as the vendor's: the zone, then one line per block (start, size, size in KB rounded to pages, overrun flag, name), then the free space.
