#!/bin/bash
# SPDX-License-Identifier: MIT
#
# hold530.sh NAME LIB ADDR ADDR_BYTES REGS L...
#
# Traces the XM530/XM550 vendor ISP library's tuning by gain for one
# sensor, the gain held at chosen levels, in qemu-bench: one boot of a
# camera's flash dump on the xm550 machine, the sensor shown to the
# library by xm-sensor (its chip ID registers, as the X50/X123 sensor
# libraries' probes read them), and for each L a majestic start with
# statfake.so holding AE where L x ISO meets its target, traced by
# isptrace.so (registers only) and 3aprobe.so (every AE result).
#
#   NAME        output prefix: NAME.hold-L.bin and NAME.hold-L.probe
#   LIB         X50_XM530 or X123_XM530, the sensor library to link
#   ADDR        the sensor's 7-bit I2C address
#   ADDR_BYTES  1 or 2, its register address width
#   REGS        xm-sensor power-on values, e.g. 0x3107:0xcb,0x3108:0x14
#   L           zone sums at ISO 100 (STATFAKE_L): about 1.8e8 / the ISO
#
# Environment: DUMP (the flash dump), QEMU_BENCH (default
# ~/git/qemu-bench), SSHPASS (the image's root password), OUT (default .),
# PORT (the forwarded SSH port, default 2222). The dump is copied, not
# written. A sensor whose initialisation overwrites its chip ID registers
# in the model (SP2305) wants one L a run.
set -u
[ $# -ge 6 ] || { sed -n '4,28p' "$0"; exit 2; }
NAME=$1 LIB=$2 ADDR=$3 AB=$4 REGS=$5
shift 5
: "${DUMP:?set DUMP to a camera flash dump}" "${SSHPASS:?set SSHPASS}"
Q=${QEMU_BENCH:-$HOME/git/qemu-bench}
OUT=${OUT:-.}
PORT=${PORT:-2222}
T=$(cd "$(dirname "$0")" && pwd)
export SSHPASS
O=(-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null
   -o ConnectTimeout=10 -o LogLevel=ERROR)
run() { sshpass -e ssh -p "$PORT" "${O[@]}" root@127.0.0.1 "$@" < /dev/null; }
get() { sshpass -e scp -O -q -P "$PORT" "${O[@]}" "root@127.0.0.1:$1" "$2"; }

FLASH=$(mktemp "$OUT/hold-$NAME.XXXXXX.bin")
cp "$DUMP" "$FLASH"
setsid "$Q/qemu-src/build/qemu-system-arm" \
  -M xm550,flash-file="$FLASH",sensor=xm-sensor,sensor-addr="$ADDR" \
  -global xm-sensor.addr-bytes="$AB" -global xm-sensor.regs="$REGS" \
  -smp 2 -m 128M -display none -serial null -monitor none \
  -netdev user,id=n,hostfwd=tcp:127.0.0.1:"$PORT"-:22 -net nic,netdev=n \
  > /dev/null 2>&1 &
QPID=$!
trap 'kill $QPID 2>/dev/null; rm -f "$FLASH"' EXIT
for i in $(seq 1 40); do sleep 5; run true 2>/dev/null && break; done
sshpass -e scp -O -q -P "$PORT" "${O[@]}" "$T/isptrace.so" "$T/3aprobe.so" \
  "$T/statfake.so" root@127.0.0.1:/tmp/
run "/etc/init.d/S95majestic stop >/dev/null 2>&1; killall majestic 2>/dev/null
sleep 3; ln -sf /usr/lib/sensors/libsns_$LIB.so /usr/lib/libsns.so
cli -s .video0.codec h264; cli -d .video0.size 2>/dev/null; true"
for L in "$@"; do
  run "R(){ killall majestic 2>/dev/null; for i in 1 2 3 4 5 6 7 8 9 10; do
pidof majestic >/dev/null || break; sleep 1; done
pidof majestic >/dev/null && kill -9 \$(pidof majestic); }
R; cd /tmp; rm -f m.bin probe.txt
(STATFAKE_L=$L LD_PRELOAD='/tmp/isptrace.so /tmp/statfake.so /tmp/3aprobe.so' \
 ISPTRACE=/tmp/m.bin ISPTRACE_MAX=30000000 ISPTRACE_CALLS=none \
 PROBE_LOG=/tmp/probe.txt PROBE_WORDS=8 PROBE_EVERY=1 majestic >/dev/null 2>&1 &)
sleep 30; R"
  get /tmp/m.bin "$OUT/$NAME.hold-$L.bin"
  get /tmp/probe.txt "$OUT/$NAME.hold-$L.probe"
done
