# xmv200-capture

How libxmv200's sensor tunings (`libraries/libxmv200/sns_*_tuning.c`) are made.

A tuning holds a sensor mode's data. It doesn't hold code. Every piece of it
is something the vendor's `isp` daemon hands to the kernel modules, recorded
while the stock firmware runs under emulation:

- the sensor's init, mode and starting exposure registers (I2C);
- the mode's clocks, MIPI receiver setup and the structural `isp.ko`
  payloads (picture size, channel init, frame length, lens correction);
- channel 0's ISP program: the parameter blocks `isp.ko` takes, in order,
  with libxmv200's own steps between them;
- the shared parameter page `isp.ko` reads every frame, and the lens
  correction table.

Nothing here runs on a camera, and the vendor's binaries are not
redistributed. A capture records only what the vendor stack writes to the
hardware and passes to its kernel modules.

## Making a reference

The emulator is [qemu-bench](https://github.com/OpenIPC/qemu-bench) with its
Xiongmai V200 machines (`-M xm530v200`), built with the `XM_REGTRACE`
write log. A capture runs the stock firmware dump of a V200 camera, made
bootable as for any qemu-bench run, with a SmartSens sensor of the chosen
chip ID (`sensor-id=cb3a`):

1. QEMU starts halted. `XM_REGTRACE=<trace>` logs every register and sensor
   write, with its virtual time and whether it came from user or kernel mode.
2. gdb attaches to QEMU's gdbstub and puts hardware breakpoints on the vendor
   modules' ioctl handlers. Their addresses are fixed, because the modules
   load in the same order on every boot. For each call it appends a line
   with the command and the argument bytes to the same trace file, and it
   also records the record each MMZ allocation returns. QEMU is stopped
   while gdb writes, so the file holds one exact timeline.
3. When the last channel starts capturing (`isp.ko` `0x495e`), gdb copies the
   shared page out of the `isp` process. It also reads the lens correction
   block through the gdbstub's physical-memory mode.

Nothing changes inside the guest.

## Generating a tuning

```sh
./gen_tuning.py merged.trace page.bin ldc.bin sc2336 > ../../libraries/libxmv200/sns_sc2336_tuning.c
```

The generator takes channel 0 (the sensor at 0x30) and the `isp` process's
calls. It recognizes that process because it is the only user of `/dev/isp`
and the only allocator of the `isp_*` MMZ blocks. Payload words that hold
buffer addresses become patches, which libxmv200 resolves against the
buffers it allocates itself.

## All sensors at once

`gen_all.sh CAPTURES_DIR` regenerates every tuning from a directory of
captures (one subdirectory per chip ID, as `capture.sh` leaves them). It
takes each sensor's name from the vendor ISP's console log, and writes
`sns_registry.c`, the chip ID to tuning table that `xmv200_board_start("auto")`
probes against.

## Board programs (XM535)

The XM535 chips' boards start from the whole vendor start-up as data:

```sh
./gen_board.py merged.trace page.bin x9-we2q > ../../libraries/libxmv200/board_x9_we2q.c
```

It takes everything both vendor processes do before the first encoder job after the last channel starts capturing (`isp.ko` `0x495e`): sensor I2C writes, with the bus from the trace (`bus << 24 | address << 16 | register`), register writes, device ioctls, MMZ blocks and the shared page. It leaves out what libxmv200 does itself: the encoder, the frame queue and the ISP's per-frame updates. The capture needs `CAP_NCH` set to the board's sensor count, and `CAP_H`, `CAP_ISZ` and `CAP_PAGEPTR` for the chip's modules and `isp` binary (see `capture.sh`).
