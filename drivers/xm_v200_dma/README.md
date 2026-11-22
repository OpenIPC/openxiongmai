# xm_v200_dma

`xm_v200_dma` is the XM530V200's memory-to-memory DMA, at `/dev/dma`, and builds as `open_dma.ko`. It replaces the vendor `xm_dma.ko`. It drives the DMAC, an ARM PL330 at 0x20020000, on channel 0. The channel is started from the debug registers (DMAGO, non-secure) and its DMASEV 0 interrupts on IRQ 50. The kernel's own `dma-pl330` driver binds the controller but has no clients, as with the vendor module.

| ioctl | |
|-------|-|
| `0x400c4401` | `{src, dst, len}`: `len` bytes, a multiple of 8 |
| `0x40144402` | `{src, dst, stride, width, height}`: `height` rows of `width` bytes (a multiple of 8, 32 KiB at most), `stride` bytes apart in both |

The addresses are physical, and neither copy may touch the kernel's own RAM; `xm_dma.ko` does not check this. The microcode is written here: 16-beat bursts of 8 bytes in loops, then the remaining beats. A 2D copy repeats that per row and steps both addresses on by the gap with DMAADDH. A copy that faults or takes over two seconds is killed (DMAKILL) and returns `-EIO`.

## XM232 (`DMA_CHIP=xm232`)

The XM232's DMA is not a PL330, and its `dma.ko` (`xm232_dma.c` replaces it) has only the 1D copy, `0x400c4401`. The block at 0x20020000 is a one-channel engine laid out like a DesignWare DMAC, interrupting on IRQ 47:

| | |
|-|-|
| `+0x010` | controller: 0 off, 3 on |
| `+0x018` | channel enable: bit 0, bit 8 its write enable |
| `+0x100` / `+0x108` | source / destination, physical |
| `+0x110` | the block's 8-byte transfers - 1 |
| `+0x118` / `+0x11c` / `+0x124` | control: 0x3fc45b00, 0x24, 0x20000 |
| `+0x180` / `+0x190` | 2 / the done mask, 2 |
| `+0x188` | done status, written back to `+0x198` to clear it |

A copy goes as blocks of at most 16 KiB, each started from the interrupt of the one before, within 200 ms. `dma_mem_copy(src, dst, len)` is exported as `dma.ko`'s is. Unlike `dma.ko` it refuses the kernel's own RAM, returns `-EINVAL` for a length that is not a multiple of 8 (`dma.ko`: -1 and "spi lcd: size error") and `-ETIMEDOUT` when the time is up (`dma.ko`: "dma timeout", and 0).
