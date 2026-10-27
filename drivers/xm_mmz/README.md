# xm_mmz

`xm_mmz` is the media memory zone of the XM530/XM550, built as `open_mmz.ko`. It replaces the vendor `mmz.ko` without changes to the drivers or applications that use it.

The SDK and the video drivers take their DMA buffers from one physically contiguous region that Linux does not use. `mem=` on the kernel command line keeps it out, and the module gets it as `mmz_start` and `mmz_size`. `load_xiongmai` passes what lies between `mem=` and the end of DDR:

```sh
insmod open_mmz.ko mmz_start=0x85000000 mmz_size=0x3000000   # XM550: mem=80M of 128 MB
```

Load it before the drivers that use it (`open_vi`/`xm530_vi`, `xm530_rgn`).

## Blocks

Every buffer is a named block, and each name exists once. A driver finds a buffer another driver made by asking for its name: VI keeps its frames in `vi_way[N]`.

The allocator is the vendor's, kept as it is:
- **Layout:** blocks are laid one after another from `mmz_start`, each starting on a page.
- **Overrun check:** after each block, before the next page, sits a 16-byte marker (`MEMORY BARRIERS`). `/proc/umap/mmz` reports a block whose marker was overwritten.
- **Freeing:** freeing a block succeeds and does nothing. Only a reset releases memory, and it releases every block at once. So whoever starts the media pipeline resets first; majestic and the SDK do.
- **Caching:** "cached" is a flag a block carries for `mmap`. It is mapped write-back instead of uncached.

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

`mmap` with the physical address as the offset maps it write-back for a cached block and uncached otherwise. Like the vendor module, it maps any physical address. It logs the first mapping outside the zone, so anything that relies on that can be found before it is refused.

## `/proc/umap/mmz`

It has the same layout as the vendor's: the zone, then one line per block (start, size, size in KB rounded to pages, overrun flag, name), then the free space.
