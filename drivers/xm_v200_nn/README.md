# xm_v200_nn

`xm_v200_nn` is the V200 series' CNN accelerator, at `/dev/nn`, and builds as `open_nn.ko`. It replaces the vendor `nn.ko`. The vendor's detection libraries build the network's command lists in MMZ and hand the accelerator one command at a time; the driver queues the commands (`config_list_num=` of them), starts the next on the accelerator's interrupt, and lets user space wait for its own.

| ioctl | |
|-------|-|
| `0x80186300` | XM530V200: submit a command, 6 words, the last its handle |
| `0x80246300` | XM533, XM535: submit a command, 9 words, the last its handle |
| `0xc0046301` | `{wait, handle, done}`: done once the handle's command has finished; with `wait` set, wait for it (50 s on the XM530V200, 5 s on the XM533, 100 s on the XM535) and reset the accelerator if it never finishes |
| `0xc0046302` | drop every queued command (the XM533's `nn.ko` ignores it) |

The accelerator is at 0x21200000 on the XM530V200 and 0x21500000 on the others, IRQ 75; `/proc/umap/cnn` shows its first registers. Build with `NN_CHIP=v200` (XM530V200), `xm533` or `xm535` (XM535D1/D2/W1). The XM533's driver tells waiters apart by process as well as handle.

Unlike `nn.ko`, the driver starts one queued command per interrupt (`nn.ko` starts every queued one, each over the last), works without the reset ioctl first (the XM530V200's and XM535's `nn.ko` oops on a submit before it), and a query without `wait` on the XM533 no longer leaves a waiter behind (the XM533's `nn.ko` locks up on the next interrupt).
