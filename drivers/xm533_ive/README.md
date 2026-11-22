# xm533_ive

`xm533_ive` is the XM533's FFT engine, at `/dev/ive`, and builds as `open_ive.ko`. It replaces the vendor `ive.ko`. User space hands the engine jobs, up to ten queued; the driver starts the next on the engine's interrupt and lets user space wait for one.

| ioctl | |
|-------|-|
| `0x80346308` | a job: 13 words, words 1..7 the engine's registers, word 8 its handle |
| `0xc00c630d` | `{wait, handle, done}`: done once the handle's job has finished; with `wait` set, wait for it (no time limit) |
| `0xc0046312` | drop every queued job |

Each returns 0, as `ive.ko`'s do: a job that does not fit is dropped with `limit 10 error.` in the log. The engine is at 0x21300000, IRQ 95: words 1..5 go to +0x00..+0x10, word 7 to +0x18, word 6 to +0x14 last, which starts the job; +0x1C's low byte reads 9 when it is done, and 0 written to +0x18 clears that.

Unlike `ive.ko`, the driver works without the reset ioctl first, and keeps a queued job's slot taken whatever the job's words 9..12 hold (`ive.ko` copies them over its own book-keeping, so with 0 there a second queued job takes the first's slot and the kernel crashes on the next interrupt).
