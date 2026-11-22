# xm232_vta

`xm232_vta` is the XM232's VTA neural network accelerator interrupt, at `/dev/vta`, and builds as `open_XmVtaMte.ko`. It replaces the vendor `XmVtaMte.ko`.

The vendor's detection runtime drives the accelerator itself through `/dev/mem`: it loads the network's instructions into MMZ, writes the instruction count less one at +0x018 and their address at +0x020, starts the engine (+0x000 = 1) and polls `/dev/vta` for the interrupt, giving up after 5 s (it then halts the engine through +0x010, waits for +0x218 to read idle and resets it through the system controller, 0x20000124). The driver acknowledges each interrupt by writing 0 to +0x008 and, once the device has been opened, reports it to the next `poll()` as `POLLIN | POLLRDNORM`, once. An open forgets an interrupt not yet reported.

Parameters keep the vendor's names: `vta_ip_base` (0x22000000), `vta_irq_num` (the interrupt; Xiongmai's loader passes 59, SPI 27) and `mte_irq_num`, accepted and unused as in the vendor module. Unlike it, a failed interrupt request leaves no device registered.
