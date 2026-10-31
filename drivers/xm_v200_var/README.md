# xm_v200_var

`xm_v200_var` is the XM530V200's rotator (VAR), at `/dev/var`, and builds as `open_var.ko`. It replaces the vendor `var.ko`.

A VPSS channel bound to VAR (bind id 17, with the VAR channel as the bind chn) puts its NV12 frames on VB list 3. A kernel thread takes them while `/dev/var` is open and handles each one by its channel's rotation. At 90° or 270° the engine turns the frame into a new buffer. At 0° or 180° the frame goes on as it is. The result goes to the channel's own binding (VENC, VPSS or a VO channel), or to a queue for user space when the channel has a depth.

The engine is at 0x36000000, IRQ 93:

| offset | |
|--------|-|
| 0x00 | direction: 0 clockwise (the source addresses are its last rows), 1 anticlockwise (the destination addresses are its last rows) |
| 0x04, 0x08 | source width (stride), height |
| 0x10, 0x14 | source luma, chroma |
| 0x18, 0x1c | destination luma, chroma |
| 0x30 | 1 starts |
| 0x40 | done |
| 0x44 | 0 acknowledges |
| 0x48, 0x4c | interrupt enable, status |
| 0x408 | engine enable |

The ioctls take a 56-byte record `{chn, ms, a, b, c, d, stride, width, height, -, pts lo, pts hi, buffer, -}`:

| ioctl | |
|-------|-|
| `0x4100` | every channel off and forgotten |
| `0x40384101` | rotation `a`, in quarter turns clockwise |
| `0x40384102` / `0x40384103` | channel on / off |
| `0x40384104` | queue depth `a` for user space (0: none); the queue is emptied |
| `0xc0384105` | a frame from the queue, waiting up to `ms`. It returns its luma and chroma addresses, stride, width, height, pts and the buffer. A bound channel hands user space a copy, and the frame itself goes on to the binding |
| `0x40384106` | that frame back |
| `0x40384107` | bind to `{a id, b dev, c chn}`, while the depth is 0 |

Frames handed back are checked to be VB's (`vb_find_buf`); `var.ko` takes them on trust.
