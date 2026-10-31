# xm_v200_dma

`xm_v200_dma` is the XM530V200's memory-to-memory DMA, at `/dev/dma`, and builds as `open_dma.ko`. It replaces the vendor `xm_dma.ko`. It drives the DMAC, an ARM PL330 at 0x20020000, on channel 0. The channel is started from the debug registers (DMAGO, non-secure) and its DMASEV 0 interrupts on IRQ 50. The kernel's own `dma-pl330` driver binds the controller but has no clients, as with the vendor module.

| ioctl | |
|-------|-|
| `0x400c4401` | `{src, dst, len}`: `len` bytes, a multiple of 8 |
| `0x40144402` | `{src, dst, stride, width, height}`: `height` rows of `width` bytes (a multiple of 8, 32 KiB at most), `stride` bytes apart in both |

The addresses are physical, and neither copy may touch the kernel's own RAM; `xm_dma.ko` does not check this. The microcode is written here: 16-beat bursts of 8 bytes in loops, then the remaining beats. A 2D copy repeats that per row and steps both addresses on by the gap with DMAADDH. A copy that faults or takes over two seconds is killed (DMAKILL) and returns `-EIO`.
