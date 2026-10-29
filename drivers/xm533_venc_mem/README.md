# xm533_venc_mem

`xm533_venc_mem` is the XM533's encoder memory book-keeping, at `/dev/venc_mem`, and builds as `open_venc_mem.ko`. It replaces the vendor `venc_mem.ko`.

User space allocates one block of MMZ per VENC channel and has the driver hand out parts of it: 4 KB aligned, first fit, freed neighbours merged. Each part has a 16-byte guard after it ("MEMORY BARRIERS"), which `/proc/umap/venc_mem` checks along with the channel's use.

| ioctl | |
|-------|-|
| `0x40045601` `{chn}` | the channel the next calls are for (0..31) |
| `0x5602` | no channel |
| `0x402c5603` | the channel's block: 8 words of settings, size, physical and virtual address |
| `0x5604` | the channel's block gone |
| `0xc02c5605` | a part: name[32] and size in, its physical and virtual address back |
| `0x402c5606` | a part back, by its virtual address |
| `0x5600` | every channel's book-keeping cleared |
