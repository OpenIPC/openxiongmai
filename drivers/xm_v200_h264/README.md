# xm_v200_h264

`xm_v200_h264` is the XM533's and XM535s' H.264 encoder, at `/dev/h264`, and builds as `open_h264.ko`. It replaces the vendor `h264.ko`, Hantro's hx280enc driver built for these chips (the XM530's is `drivers/hx280enc`, a different build with more ioctls).

User space programs the encoder's registers itself through `/dev/mem`. The driver turns the encoder's clock on (0x20000120), checks its ID, and answers three ioctls: `0x80046b03` the registers' address (0x21100000), `0x80046b04` their size (0x1c4), and `0x6b0c` wait for the encoder's interrupt (IRQ 78), which it acknowledges. `base_port=` and `irq=` as `h264.ko`'s.
