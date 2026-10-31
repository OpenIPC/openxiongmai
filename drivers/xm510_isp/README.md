# xm510_isp

`xm510_isp` is the ISP device for the XM510, built as `open_isp.ko` (the name OpenIPC firmware gives the open modules that replace vendor ones). It replaces the vendor's `xm510_isp.ko` without changes to the vendor `isp` daemon that uses it.

The ISP's work (exposure, white balance and the rest) is done in user space by the vendor `isp` daemon, which majestic starts. The kernel module gives the daemon register access, a shared page and a frame-start signal; it has no ISP logic of its own.

```sh
insmod open_isp.ko   # after the VI driver (open_vi or xm510_vi), which clears the shared interrupt
```

## `/dev/isp`

| Call | Does |
|---|---|
| `ioctl(fd, 0x101, {addr, val})` | Writes `val` to the register at `addr`. |
| `ioctl(fd, 0x103, {addr, val})` | Reads the register at `addr` into `val`. |
| `mmap` | Maps the shared page: one page, zeroed when it is first mapped and kept until the module is unloaded. |
| `fcntl(F_SETFL, O_ASYNC)` | `SIGIO` at the start of every frame. |

The record is `{ u32 addr; u32 val; }`, and the command numbers are the raw values the daemon uses, not `_IO()`-encoded. `addr` must lie in one of two 1 MiB blocks: ISP/VI at `0x40000000` or GPIO at `0x10020000`. An unknown command logs `IspKo:Err: command error!`, and an address outside both blocks logs `IspKo:Err: u32RegAddr :N Err!`; both return 0, as in the vendor module.

The frame-start signal comes from IRQ 16, the VI frame-start interrupt. It's shared with the VI driver, which clears its status; this module only listens, and reports it as not handled, as the vendor module does.

`/proc/isp` shows the frame rate and a frame count (`FrameRate`, `FrameCnt`), in the vendor's layout.

## What differs from the vendor module

- **Unload:** the shared page is freed as a page. The vendor module passes the page's `struct page *` to `free_pages()`, which expects an address. It frees the reserved page holding that part of `mem_map` instead, logs `BUG: Bad page state` on every unload and leaks the shared page.
- **Bad addresses:** an address outside both blocks, or not word-aligned, releases the lock. The vendor module returns with its mutex still held, so every later ioctl, the daemon's included, waits forever in uninterruptible sleep. The process then can't be killed or made to exit.
- **User memory:** the record is copied from and to user space; the vendor module dereferences the user pointer directly. A bad pointer gives `EFAULT`.
- **Mappings:** one page at most, at offset 0. The vendor module maps whatever length it's asked for, and with it the memory after the page.
- **Frame rate:** `/proc/isp`'s rate comes from the monotonic clock, so setting the time doesn't disturb it.

## Tested

On an XM510 + JX-H62 with majestic, the vendor `isp` daemon, `open_mmz`, `open_vi` and `open_h264`:
- **Register traffic:** the daemon does about 3,300 register reads and 1,200 writes a second through `/dev/isp`, measured with a counting build.
- **Video:** it streams, with the same mean brightness as with the vendor module over the same frames, in a dark room.
- **`/proc/isp`:** 25 fps.
- **The daemon:** maps one page and gets `SIGIO`.
- **Calls:**
  - a read through `0x103` returns the register's value (channel 0's width, 1280);
  - an address outside both blocks is refused, and the next ioctl works;
  - an unaligned address is refused, and a bad pointer gives `EFAULT`;
  - a two-page `mmap` is refused, and a one-page one works.
  - With the vendor module, the same out-of-block read leaves the caller, and then one of the daemon's threads, in `D` state for good, through `kill -9`.
- **Unload:** unloading `open_isp` logs no `Bad page state`; unloading the vendor module logs one each time.
- **Restarts:** five majestic restarts and three `kill -9`, with video every time and no daemon thread left in `D` state.
