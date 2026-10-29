# xm530v200_vo

`xm530v200_vo` is the XM530V200's video output, at `/dev/vo`, and builds as `open_vo.ko`. It replaces the vendor `vo.ko`, and exports `vo_get_buf()`/`vo_put_buf()`, through which `vpss.ko` (or `open_vpss.ko`) fills the display's channels.

One display device drives a BT656, LCD or MIPI panel (`/proc/umap/vo` shows it all). Its video layer holds up to six channels, each with three NV12 buffers carved from the layer's MMZ block (`vo_v_layer[0]`). The buffers are V200 vb descriptors: physical address in word 4, size 5, stride 6, height 8, pts 10/11. `vo_get_buf()` hands out the free one with the oldest pts, and `vo_put_buf(..., 1)` marks it filled. The display interrupt (IRQ 59, once a frame) moves each channel on to the older of its next two filled buffers. There is also an image (OSD) layer, a cursor layer, and luma, saturation, contrast and hue controls.

| ioctl | |
|-------|-|
| `0x4f00` | forget everything |
| `0x40404f01` | the device: background, interface (8 BT656, 0x40 LCD, 0x80 MIPI), sync (720P25, 720P30, 1080P25, 1080P30, 480x272, 1024x600, or 1000 with user timing) |
| `0x40404f02` | a channel: x, y, width, height, priority |
| `0x40404f03` / `04` | a channel on / off |
| `0x40404f05` | the video layer: width, height, x, y, display width, height |
| `0x40404f06` / `07` / `08` | the image layer: format, address, size, position, effect; on; off |
| `0x40404f09` / `0a` / `0b` / `0c` | the cursor: format, position, size, effect; on; off; its picture |
| `0x40404f0d` / `0e` | a channel shown / hidden |
| `0x40404f0f` | the layer's channels forgotten (all off and hidden first) |
| `0x40404f10` | a channel's buffers black |
| `0x40404f11` | colour: luma, saturation, contrast, hue (0..100, 50 neutral) |
| `0x40404f12` / `13` | a channel buffer for user space; then shown |

All take a 64-byte block `{dev, chn, ...}`, with dev 0 only. `lcd_rgb=R,G,B` (pins per colour), `lcd_pwm=` and `lcd_disp=` set an LCD's pins at load, as `vo.ko`'s do.

Unlike `vo.ko`, the interrupt compares the pts of the two buffers after the one shown (`vo.ko` always compares buffers 1 and 2), and setting the video layer a second time works (`vo.ko` looks its MMZ block up by an empty name and fails). In qemu-bench (with a register-file model of the block) its register writes and answers match `vo.ko`'s otherwise.
