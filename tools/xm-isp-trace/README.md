# xm-isp-trace

`isptrace.so` is an `LD_PRELOAD` tracer for a closed ISP daemon, written for
the XM510 vendor `/usr/bin/isp`. It records what the daemon does to the
hardware, so an open ISP can be written from observed behaviour:

- every `/dev/isp` register write and read (raw commands `0x101`/`0x103` on
  `{addr, val}`; on XM530/XM550 `_IOW('I', 1/2/3, 12)` on `{addr, val, arg}`);
- every store to a register page mapped through `/dev/mem`. The page is
  handed out read-only, and a fault handler logs each store, performs it
  through a writable alias and steps over the instruction. The XM510 daemon
  programs the system controller (`0x20000000`) this way;
- every other ioctl, with its argument before and after (`/dev/xm_i2c`,
  `/dev/vi`, `/dev/mmz`, `/dev/rgn`);
- the files it opens, its mappings, its message queue traffic and its reads;
- the frame start, so each record carries the frame it belongs to: the
  SIGIO on the XM510; on the XM530/XM550, which has none, the return of the
  wait for the Bayer frame event (`_IOW('I', 4, 12)`, `arg` 0).

`decode.py` turns the binary log into text, naming registers from a
register map header. `extract.py` turns logs into `libxmisp` start-up
sources:
- `extract.py --soc REF.bin NAME` writes the SoC's start-up template, with
  a slot for each of the sensor's parts;
- `extract.py REF TRACE.bin NAME DESC --fixed A-B` writes one sensor
  and mode: its parts, the ISP values that differ from REF's, and one 3A
  state. REF is the reference trace or the template it made. It checks that the template with those reproduces TRACE's
  start-up.

The vendor daemon's start-up has the same steps for every sensor it
supports, so the split is exact. `libraries/libxmisp/sensors/README.md`
covers adding a sensor, including tracing one in qemu-bench.

## Build

```sh
make CROSS_COMPILE=arm-openipc-linux-musleabi-
```

The library is built with the musl toolchain that runs the vendor binaries
through `libwrapper`, the shim the firmware's `xiongmai-opensdk-xm510`
package ships for them.

## Use

Majestic starts the daemon as `isp` from `PATH`. A wrapper bind-mounted
over `/usr/bin/isp` preloads the tracer and streams the log to a host. Bind
mounts follow the inode, so after editing the wrapper, mount it again.

```sh
# host
nc -l -p 40510 > trace.bin

# camera
cp /usr/bin/isp /tmp/isp.real
cat > /tmp/isp.wrap <<'EOF'
#!/bin/sh
[ -p /tmp/isptrace.fifo ] || mkfifo /tmp/isptrace.fifo
# a subshell: ash opens a redirection before it forks
(nc HOST 40510 < /tmp/isptrace.fifo) &
export LD_PRELOAD=/tmp/isptrace.so ISPTRACE=/tmp/isptrace.fifo
exec /tmp/isp.real "$@"
EOF
chmod +x /tmp/isp.wrap
mount --bind /tmp/isp.wrap /usr/bin/isp
/etc/init.d/S95majestic restart

# afterwards
umount /usr/bin/isp
/etc/init.d/S95majestic restart
```

`ISPTRACE` names the output (a file or a FIFO). `ISPTRACE_MAX` caps it in
bytes. `/tmp` is RAM on these boards, so stream long captures off the board.

```sh
./decode.py trace.bin -m ../../libraries/isp/include/xm/xm510_isp.h
./decode.py trace.bin -r -f 300-300 -m ...   # registers of frame 300
```

### XM530/XM550

There the vendor ISP is a library, `libisp_XM530.so`, inside majestic,
which is dynamically linked, so the tracer goes on majestic itself:

```sh
/etc/init.d/S95majestic stop
LD_PRELOAD=/tmp/isptrace.so ISPTRACE=/tmp/trace.bin ISPTRACE_MAX=40000000 \
    majestic > /tmp/mj.log 2>&1 &
# ...
killall majestic; /etc/init.d/S95majestic start
```

The tracer also stands in for the 59 functions of `libisp_XM530.so` that
majestic, `libispapp` and the sensor library call, and logs each call on the
way in and out with r0-r3, the module that called it, and 512 bytes at each
argument that points at readable memory, so a register write can be put
down to the call that made it. The register helpers (`Read_IspReg`,
`Write_IspReg`, `ispMemnCpy`), thousands of calls a second for the AWB
statistics, are logged only with `ISPTRACE_CALLS=all`. `decode.py -c` lists
the calls and frames alone, without the snapshots.

Majestic's own traffic (`/dev/h264`, `/dev/mv500`, `/dev/vi`, `/dev/mmz`)
comes along; `decode.py -m ../../libraries/isp/include/xm/xm530_isp.h`
names the ISP registers. 45 s of an XM550 with an IMX335 (602 frames)
came to 26 MB.

## Behaviour to know

- The tracer initialises on its first call, not in a constructor. musl runs
  constructors from `__libc_start_main`, and a uClibc binary entering
  through libwrapper's `__uClibc_main` never calls it.
- The XM510 daemon runs its 3A in the SIGIO handler. Its ioctls re-enter the
  tracer, so the tracer takes its locks with signals blocked.
- The output is non-blocking. A reader that falls a second behind, or goes
  away, ends the trace, and the daemon runs on untraced. The SIGPIPE from a
  closed FIFO is taken by the tracer, not the daemon.
- Records are written out every 100 ms. Once one of the daemon's own signal
  handlers runs (SIGINT, SIGTERM: its exit path), every record is written out
  at once.
- Tracing costs CPU: the XM510 daemon goes from about 5% to about 50%, and
  still keeps up with 25 frames a second.
- Only ARM stores (`STR`, `STRB`, `STRH`) to `/dev/mem` register pages are
  decoded. Anything else (Thumb, `STM`) makes that page writable and
  untraced, and is logged once as such, as is unmapping part of a page.
  Faults outside the traced pages go to the daemon's own SIGSEGV handler.

## Log format

`isptrace.h`: a 16-byte header, then records of a 16-byte head and a padded
payload, little-endian.
