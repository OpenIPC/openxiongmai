# xm-isp-trace

`isptrace.so` is an `LD_PRELOAD` tracer for a closed ISP daemon, written for
the XM510 vendor `/usr/bin/isp`. It records what the daemon does to the
hardware, so an open ISP can be written from observed behaviour:

- every `/dev/isp` register write and read (raw commands `0x101`/`0x103` on
  `{addr, val}`);
- every store to a register page mapped through `/dev/mem`. The page is
  handed out read-only, and a fault handler logs each store, performs it
  through a writable alias and steps over the instruction. The XM510 daemon
  programs the system controller (`0x20000000`) this way;
- every other ioctl, with its argument before and after (`/dev/xm_i2c`,
  `/dev/vi`, `/dev/mmz`, `/dev/rgn`);
- the files it opens, its mappings, its message queue traffic and its reads;
- the frame-start SIGIO, so each record carries the frame it belongs to.

`decode.py` turns the binary log into text, naming registers from a
register map header. `extract.py` turns a log into a `libxmisp` sensor
script: the start-up in order, and one converged 3A state.

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
