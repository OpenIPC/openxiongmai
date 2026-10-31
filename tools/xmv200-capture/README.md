# xmv200-capture

How libxmv200's sensor tunings (`libraries/libxmv200/sns_*_tuning.c`) are made.

A tuning holds a sensor mode's data. It doesn't hold code. Every piece of it
is something the vendor's `isp` daemon hands to the kernel modules, recorded
while the stock firmware runs under emulation:

- the sensor's register runs (I2C), in the program's order;
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

The generator takes channel 0 and the `isp` process's calls. It recognizes
that process because it is the only user of `/dev/isp` and the only
allocator of the channel buffers (`*_ch0`, `*_ch1`, ...). Payload words that
hold buffer addresses become patches, which libxmv200 resolves against the
buffers it allocates itself. They are found by value, and checked against
channel 1's payload of the same command where there is one.

## All sensors at once

`gen_all.sh CAPTURES_DIR` regenerates every tuning from a directory of
captures (one subdirectory per chip ID, as `capture.sh` leaves them). It
takes each sensor's name from the vendor ISP's console log, and writes
`sns_registry.c`, the chip ID to tuning table that `xmv200_board_start("auto")`
probes against.

## Other chip families

The same capture and generator work on the XM533 and XM535 firmware. Name the family:

```sh
./gen_tuning.py merged.trace page.bin ldc.bin sc2336_xm535 xm535 > ../../libraries/libxmv200/sns_sc2336_xm535_tuning.c
./gen_all.sh CAPTURES_DIR xm535      # every sensor captured on that family, and the registry
```

The capture needs `CAP_H`, `CAP_ISZ` and `CAP_PAGEPTR` for the chip's modules and `isp` binary (see `capture.sh`), and `CAP_NCH` set to the board's sensor count. The families are `xm530v200`, `xm535` (XM535D1/D2), `xm535w1` and `xm533`. They differ in where VI's blocks are and in their kernel modules' structs; `libraries/libxmv200/chip.c` has the differences.

`CAP_MMAP` (isp.ko's `isp_mmap`) can stand in for `CAP_PAGEPTR`: the capture then takes the shared pages from the mapping isp.ko made, whatever the `isp` binary keeps it in. The handlers, from each family's stock firmware (modules found in module memory by their `.text`, handlers at their `.ko` symbols):

| family (board) | isp | vpss / vi | vb | hevc | mmz | sys | venc | `CAP_MMAP` |
|---|---|---|---|---|---|---|---|---|
| xm535 (X6E-WEQ) | 0xbf029000+0x13b0 | vpss 0xbf01a000+0x18e4 | 0xbf009000+0xb18 | 0xbf066000+0x740 | 0xbf004000+0xb14 | 0xbf00e000+0x158 | 0xbf059000+0x308 | 0xbf029000+0x388 |
| xm533 (X3-WQ-B) | 0xbf01a000+0x510 | vi 0xbf02a000+0x1a20 | 0xbf009000+0x6b4 | 0xbf04b000+0x3cc | 0xbf004000+0xab8 | 0xbf00e000+0x94 | 0xbf040000+0x2e8 | 0xbf01a000+0x34c |
| xm535w1 (X2W-WNG) | 0xbf024000+0x53c | vi 0xbf01a000+0x2084 | 0xbf009000+0xb18 | 0xbf05f000+0x740 | 0xbf004000+0xb14 | 0xbf00e000+0x13c | 0xbf053000+0x308 | 0xbf024000+0x378 |

(`CAP_H` is the dict of those addresses to `'isp'`, `'vpss'`/`'vi'`, ...; XM530V200's are cap.py's defaults, its `isp_mmap` at 0xbf025000+0x3b4.)

A tuning holds the mode, not the board: the program is channel 0's, and libxmv200 builds what depends on the layout from the board's description. That covers the channel index and input, the receiver, the VI device and the windows.

## Other modes

A stock firmware runs its sensor in the mode its product settings choose (`/mnt/custom/CustomConfig/Isp.json`). To capture another mode, `isp_json.py` makes an image with those settings changed:

- without `MipiAttr`, for the 2-lane MIPI modes;
- with `Assign2336p` off, for the real SC2337P;
- on the XM535D1, with `FunExt` 1 instead of 17, for the sensors without frame sync;
- on the XM530V200 X6C-WEQ: `Resolution`/`ResolutionSns` "1M" for SC2335 (its 720p picture is otherwise scaled up, which the dual product's offline BPU refuses), "4M" for MIS4001's 4M mode, and `SensorType` "MIS5001/MIPI" with `MultiSnsNum` 1 for MIS5001 (it shares MIS4001's chip ID);
- on the XM535D1, `FunExt` 0 (one ISP) for the sensors the synced product's 1125-line frame cannot hold (SC3338, SC4336, SC500AI, MIS4001, MIS40C1): those run the BPU online.

The `isp` daemon's messages show why a sensor does not start; the stock kernel prints nothing, but hardware breakpoints on the daemon's `printf`/`fprintf`/`fputs`/`puts` PLT stubs give them, arguments and all.

Name the tuning after its mode (`sc2336_2lane`, `sc2336_xm535_nosync`). The registry's first tuning of a sensor is the default.

```sh
./isp_json.py stock.bin nolane.bin -MipiAttr
IMAGE=nolane.bin ./capture.sh cb3a captures/nolane_cb3a
./gen_tuning.py captures/nolane_cb3a/merged.trace ... sc2336_2lane
```

None of this is read at run time. The streamer picks the mode in its layout (`hdr`, `sync`, `lanes` in `struct xmv200_layout_sensor`), and `xmv200_sensor_modes()` lists the modes there are tunings for.
