#!/bin/bash
# SPDX-License-Identifier: MIT
# capture.sh CHIP_ID OUTDIR [GDB_PORT]
#   MACHINE  qemu-bench machine (xm530v200); CAP_H / CAP_ISZ for another
#            chip's modules (see cap.py)
#   SENSOR_MODEL  another sensor model than SmartSens, qemu-bench's
#            TYPE@ADDR (e.g. hisi-imx335@1a); CHIP_ID then only names it.
#            hisi-smartsens@ADDR: the SmartSens CHIP_ID at another address
#
# Runs a stock XM530V200 firmware image under qemu-bench with a SmartSens
# sensor of CHIP_ID (e.g. cb3a) and records the reference gen_tuning.py
# takes: OUTDIR/merged.trace, page.bin, ldc.bin, console.log.
#   QEMU   qemu-bench's qemu-system-arm (with XM_REGTRACE)
#   IMAGE  the stock firmware, made bootable for qemu-bench (8 MiB flash)
set -e
ID=$1; O=$2; P=${3:-1234}
: "${QEMU:?set QEMU to the qemu-bench qemu-system-arm}" "${IMAGE:?set IMAGE to the stock firmware}"
D=$(cd "$(dirname "$0")" && pwd)
SNS=sensor-id=$ID
case "$SENSOR_MODEL" in
hisi-smartsens@*) SNS=sensor=$SENSOR_MODEL,sensor-id=$ID ;; # at another address
?*) SNS=sensor=$SENSOR_MODEL ;;
esac
mkdir -p "$O"; rm -f "$O/merged.trace"; cp "$IMAGE" "$O/flash.bin"
XM_REGTRACE="$O/merged.trace" timeout 500 "$QEMU" \
    -M "${MACHINE:-xm530v200}",flash-file="$O/flash.bin","$SNS" -m 128M \
    -nic user,restrict=on -display none -serial file:"$O/console.log" \
    -monitor none -gdb tcp::"$P" -S > "$O/qemu.out" 2>&1 &
Q=$!; sleep 2
CAP_OUT="$O" XM_REGTRACE_FILE="$O/merged.trace" timeout 480 gdb-multiarch -q -batch \
    -ex "set architecture arm" -ex "set pagination off" -ex "target remote :$P" \
    -ex "source $D/cap.py" > "$O/gdb.out" 2>&1 || true
kill $Q 2>/dev/null || true; wait $Q 2>/dev/null || true
echo "$ID: $(grep -a -o '[A-Z0-9]*! Chn:0' "$O/console.log" | head -1) snapshot=$(grep -c SNAPSHOT "$O/merged.trace")"
