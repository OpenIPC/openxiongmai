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

### The 3A interface on XM530/XM550: `3aprobe.so`

`libisp_XM530.so` takes its AE and AWB from the 3A side (inside majestic)
during `XM_MPI_ISP_MemInit`, through `XM_MPI_ISP_AELibRegCallBack` and
`XM_MPI_ISP_AWBLibRegCallBack`. It keeps them in two tables it exports,
`gAlgAeFun` and `gAlgAwbFun`, each of four functions (init, run, ctrl,
exit). Both are zero before `MemInit` and filled after it. Its frame loop
then calls `ae_run` and `awb_run` every frame.

`3aprobe.so` (MIT, no SDK headers) stands in for `XM_MPI_ISP_MemInit`.
Once the real one has filled the tables, it puts a logging wrapper in each
slot. Each wrapped call is logged with its five arguments (`ae_run` takes
five) and the words at each pointer argument, one level of pointers deeper
(AWB's result points at its gains and matrix), before and after the call,
then passed on. Memory is read through a pipe, so a value that isn't a
pointer is skipped rather than faulting majestic. The log (`PROBE_LOG`,
default `/tmp/3aprobe.txt`) is opened without following symlinks and used
only if it is a regular file of the camera process's user. Use it in place
of the tracer: both stand in for `MemInit`.

```sh
LD_PRELOAD=/tmp/3aprobe.so PROBE_WORDS=48 PROBE_EVERY=100 majestic
# /tmp/3aprobe.txt (PROBE_LOG): the first three calls of each, then one in 100
```

On the lab XM550 (IMX335, 2600×1954):
- In `MemInit`, before the probe's wrappers are in, the library calls
  `ae_ctrl(0, handle, 1, …)` and `awb_ctrl(handle, 1, …)` (a GOT patch of the
  two `LibRegCallBack` imports shows it). The handles are buffers the
  library gives the 3A side for its state; their inits refuse to run first.
- `ae_init(0, handle, param, …)` and `awb_init(handle, param, 2600, 1954)` run
  once each. `awb_init` returns the AWB statistics window in its parameter
  (`stWin`, H 4-2564, V 0-1952), which the library spreads over 32 × 16
  zones.
- Every frame comes `awb_run(0, info, result, iso)`, `iso` from the last AE
  result. It reads the AWB statistics itself, through the library's
  `Read_IspReg`. `result` holds the R/G/B gains (`0x186`/`0x100`/`0x1b3`) and
  a colour matrix (`0x149`, `0x6`, …), which the library writes to the AWB
  gains and the CSM.
- Then comes `ae_run(0, 0, info, result, n & 1 ? 0 : 1)`. `info` holds the
  frame count, the frame rate (25), 16 histogram bins of 0x80, a 7 × 7 grid
  and a 16 × 32 one. The 16 × 32 grid is each AWB zone's BT.601 luma with the
  AWB gains applied (16 times the mean per pixel of a colour); the 7 × 7 one
  takes means of 4 × 2 of those, the last column and row the rest. The
  result's low 16 bits are the ISP gain (8 fractional bits).


### Start-up for any sensor: `start530.py` and `compare530.py`

`start530.py` writes `libraries/libisp_XM530/arch/xm530/xm530_start.c`, the
open library's start-up for every sensor, from `decode.py -t` texts of
traces of the vendor library: the first (the lab IMX335's) is the template,
the rest each another sensor. It checks that every trace is the template
with the picture's geometry, the sensor library's defaults, and the
overrides it writes for that sensor, and fails naming what isn't.

Most sensors aren't in the lab, so trace them in qemu-bench: boot a dump of
an XM530/XM550 camera's flash (`-M xm550,flash-file=dump.bin -smp 2 -m
128M`, user networking with an SSH port forward), show the vendor library
the sensor with `sensor=xm-sensor,sensor-addr=…` and
`-global xm-sensor.regs=…` (the chip ID registers the X50/X123 probes in
`libraries/libsns_*` read), point `/usr/lib/libsns.so` at the sensor's
library, and trace majestic. A trace of the emulated IMX335 gives the lab
trace's tables exactly.

```sh
./start530.py imx335_lab.txt imx307.txt sc2335.txt ... \
    > ../../libraries/libisp_XM530/arch/xm530/xm530_start.c
```

`compare530.py VENDOR OPEN` compares two libraries' start-up: each
register's value after `Init` and the first `Run` pass, "Isp" addresses as
offsets, the gain-tuned registers left out. Bind-mount the open library over
the vendor's and trace majestic on each in turn.

### Start-up tables of one trace: `extract530.py`

`extract530.py` turns a `decode.py -t` text of a trace into the tables
`libraries/libisp_XM530` starts a sensor and mode with: the ISP writes the
vendor library made itself in `XM_MPI_ISP_Init` (before the 3A inits, and
after them less the AWB window it computes), the first pass of its
`XM_MPI_ISP_Run` loop, and each attribute `Init` set itself, from the
setter's argument snapshot. Writes of an address in the library's "Isp" MMZ
block become offsets into it.

```sh
./decode.py -t trace.bin > trace.txt
./extract530.py trace.txt xm530_imx335 "IMX335, 2600x1954 at 25 fps, lab XM550" \
    > ../../libraries/libisp_XM530/arch/xm530/xm530_imx335.c
```

`nr3d530.py` writes the 3D noise reduction schedule from several such
traces, each with its `3aprobe.so` log (`PROBE_WORDS=8 PROBE_EVERY=1`, for
the AE result's ISO every frame): the vendor library's 3D NR state at each
pass after 14 frames of a steady gain, per ISO, stopping if two traces
disagree.

```sh
./nr3d530.py xm530_imx335 "IMX335, 2600x1954, lab XM550" \
    t100.txt:p100.txt t30.txt:p30.txt ... \
    > ../../libraries/libisp_XM530/arch/xm530/xm530_imx335_nr3d.c
```

### Holding the gain in emulation: `statfake.so`

In qemu-bench the statistics read 0, so AE ramps the gain to its ceiling,
two frames a step: the Run loop's tuning by gain is never settled.
`statfake.so` (MIT) makes the picture brighten with the gain and nothing
else, so AE settles where one chooses: each AWB zone sum the library reads
through `/dev/isp` becomes `STATFAKE_L x ISO / 100` (the ISO of the AE
library's last result, by wrapping its run function in `gAlgAeFun`), and
the 64-bin luma histogram (`0x30073000`) one bin (`STATFAKE_BIN`, 24).
Zone sums are millions: on the IMX335 AE settles at about ISO 1.8e8 / L.
It stacks with `isptrace.so` and `3aprobe.so`:

```sh
STATFAKE_L=155000 LD_PRELOAD='/tmp/isptrace.so /tmp/statfake.so /tmp/3aprobe.so' \
    ISPTRACE=/tmp/m.bin ISPTRACE_CALLS=none PROBE_WORDS=8 PROBE_EVERY=1 majestic
```

Held on the emulated IMX335 at ISO 285 and 1164, every 3D NR register is
what the lab camera settled at, at those ISOs. `nr3d530.py --sensors`
turns such traces of each group of sensors into
`libraries/libisp_XM530/arch/xm530/xm530_nr3d.c`.

`chromasweep.so` sets each colour strength of `XM_MPI_ISP_SetChromaAttr` to
every value in turn (the others neutral) and logs the hue LUT the library
writes, read back (`SWEEP_LOG`, default `/tmp/chromasweep.txt`); `chroma530.py`
turns the log into `libraries/libisp_XM530/arch/xm530/xm530_chroma.c`.

```sh
LD_PRELOAD=/tmp/chromasweep.so majestic     # about 30 s after start
./chroma530.py chromasweep.txt > ../../libraries/libisp_XM530/arch/xm530/xm530_chroma.c
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
