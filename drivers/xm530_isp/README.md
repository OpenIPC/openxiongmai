# xm530_isp

`xm530_isp` is the ISP device for the XM530/XM550, built as `open_isp.ko` (the name OpenIPC firmware gives the open modules that replace vendor ones). It replaces the vendor's `xm530_isp.ko` without changes to the vendor ISP library that uses it. The XM510 has its own, smaller driver in `drivers/xm510_isp`.

The ISP's work (exposure, white balance and the rest) is done in user space by the vendor ISP library linked into the streamer. The kernel module gives it register access, waits for the ISP's interrupts, a shared block of memory, AWB zone statistics and a MIPI lane-delay sweep. It has no ISP logic of its own.

```sh
insmod open_isp.ko   # after MMZ and VI, before the region and encoder modules
```

## `/dev/isp`

| Call | Does |
|---|---|
| `ioctl(fd, _IOW('I', 1, 12), {addr, val, -})` | Writes `val` to the register at `addr`. |
| `ioctl(fd, _IOW('I', 2, 12), …)`, `_IOW('I', 3, 12)` | Reads the register at `addr` into `val`, copied back. |
| `ioctl(fd, _IOW('I', 4, 12), {-, -, event})` | Waits up to 2 s for ISP event 0 (Bayer statistics, IRQ 57) or 1 (luma statistics, IRQ 58) since the last wait. |
| `ioctl(fd, _IO('I', 5))` | The same for the audio interrupt (IRQ 62). |
| `ioctl(fd, _IOW('I', 6, 20), {delay, frames, preset0, preset1, go})` | With `go` 1, starts the MIPI lane-delay sweep: `delay` (at most 500) goes to `0x300f0058`, and each lane delay is judged over `frames` frames (at most 500), against `preset0`/`preset1` if both are set. |
| `ioctl(fd, _IO('I', 7))` | Starts the sweep's run from delay 0. |
| `mmap` | Maps the shared block: 16 KiB, zeroed when it is first mapped and kept until the module is unloaded. |
| `write(fd, {base, count, div}, 12)` | Sets the AWB statistics' block address, zone count (at most 512) and divisor. |
| `read(fd, buf, len)` | Reads `count` zones from the statistics block into three u16 arrays of 512 (`r/div`, `(g/div)/2`, `b/div`); copies up to 3072 bytes. |

`addr` must lie in one of two blocks: ISP at `0x30000000` (2 MiB) or GPIO at `0x10020000` (1 MiB). An address outside them logs `The WriteReg Addr is outside` or `The ReadReg Addr is outside` and returns 0, as in the vendor module; a read then gives 0. An unknown command logs `IspKo:Err: command N error!` and returns 0.

The sweep is the vendor module's, step by step. For each lane delay 0–255 it:
1. writes the delay to `0x300c0010`, all four lanes;
2. pulses `0x300c002c`;
3. waits `frames` frames;
4. judges the delay good if the Bayer-H interrupt (IRQ 59) saw the MIPI status (`0x300c0040`/`0x300c0044`) unchanged.

Each run of good delays is logged with its middle. Only a diagnostic mode of the ISP application starts the sweep.

`/proc/umap/isp` shows the rate and count of the two ISP interrupts and the audio one, in the vendor's layout.

## What differs from the vendor module

- **Event numbers:** ioctl 4's event number is checked. The vendor module uses it as an index into its own data, writes there, and waits on whatever it finds.
- **Waits:** a timeout fails with `ETIMEDOUT`, and a signal interrupts the wait the usual way. The vendor module returns −512 to user space in both cases.
- **Reads outside the blocks:** they return 0. The vendor module copies back an uninitialised value.
- **The sweep:** ioctl 6 stops a sweep that's still running before it starts another. The vendor module starts a new thread every time and loses the earlier ones.
- **Mappings:** 16 KiB at most, at offset 0. The vendor module maps whatever length it's asked for, and with it the memory after the block.
- **AWB statistics:** the base must lie in the ISP block, the count must be at most 512, and the divisor must be non-zero, or `read()` fails with `EINVAL`. The vendor module writes to an unchecked address (an unset one included), overruns its arrays with a large count, and divides by zero.
- **Unload:** the block is freed at the order it was allocated. The vendor module frees one page of the four.

## Tested

On an XM550 + IMX335 with majestic:
- **The vendor library:** it runs through it. It makes about 1,000 register reads and 300 writes a second, the same rates as with the vendor module within the variation between samples. It waits on event 0 and maps 8 KiB.
- **Interrupts:** the Bayer interrupt rate and count match the vendor module's.
- **Video:** video and snapshots work.
- **Calls:**
  - a read returns the register's value, and a read outside both blocks returns 0;
  - waiting on event 0 returns within a frame, and event 7 is refused;
  - the audio wait times out with `ETIMEDOUT` when no audio interrupt comes;
  - a 32 KiB `mmap` is refused, and 8 KiB is mapped;
  - AWB `read()` with no parameters, or with a count of 100000 and a zero divisor, is refused.
- **Restarts:** five majestic restarts and two `kill -9`, with video every time.
- **Unload:** unloading logs nothing.

The MIPI sweep (ioctls 6 and 7) has not been run on hardware.
