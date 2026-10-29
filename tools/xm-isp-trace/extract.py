#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Turns an isptrace.so log into a libxmisp sensor script (C source).

    extract.py trace.bin NAME "DESCRIPTION" --fixed FIRST-LAST [-m HEADER]

The start script is what the daemon did before the first frame, in order:
MMZ and VI resets, system controller and sensor writes, the chip ID it read
(checked at start), VI channel setup, the "Isp" MMZ block and the ISP
register writes. A gap of more than 3 ms before a step becomes a delay. Values
that point into the MMZ block (in 8-byte units from DDR) are written relative
to wherever the block is allocated.

The fixed script holds the last value written to each ISP register, and to
each sensor register, over the frames given: one converged 3A state, applied
once. The colour matrix is double-buffered (CSM_BUF_SW): both banks get the
matrix.

-m adds register names, as comments, from a register map header.
"""
import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decode  # noqa: E402

DDR = 0x80000000
GAP_MS = 3.0
ISP_W, ISP_R, IOCTL, MEMW, FRAME = (decode.ISPW, decode.ISPR, decode.IOCTL,
                                     decode.MEMW, decode.FRAME)
I2C_WRITE, I2C_READ = 1, 3
# The colour matrix banks and the bank switch.
CSM_BANK0, CSM_BANK1, CSM_BANK_LEN, CSM_SW = 0x40061040, 0x40061080, 0x30, 0x40061000


def records(path):
    data = open(path, "rb").read()
    magic, version, t0, _ = decode.HDR.unpack_from(data)
    if magic != decode.MAGIC or version not in decode.VERSIONS:
        sys.exit(f"{path}: not an isptrace log (version 1 or 2)")
    fds, frame, off = {}, 0, decode.HDR.size
    while off + decode.REC.size <= len(data):
        t, aux, _f, typ, fd, n, aux2 = decode.REC.unpack_from(data, off)
        p = data[off + decode.REC.size : off + decode.REC.size + n]
        off += decode.REC.size + ((n + 3) & ~3)
        if typ == FRAME:
            frame = aux
        if typ in (decode.OPEN, decode.CREAT) and decode.s16(aux2) >= 0:
            fds[decode.s16(aux2)] = decode.cstrs(p)[0]
        yield dict(ms=((t - t0) & 0xFFFFFFFF) / 1000, aux=aux, aux2=aux2,
                   type=typ, dev=fds.get(fd, ""), p=p, frame=frame,
                   version=version)


def words(b):
    return list(struct.unpack(f"<{len(b) // 4}I", b[: len(b) // 4 * 4])) if b else []


def ioctl_args(r, need_before=0, need_after=0):
    """The argument words an ioctl record saw before and after the call; at
    least the counts asked for, or the trace can't make a script."""
    _, before, after = decode.ioctl_snapshots(r["p"], r["version"])
    before, after = words(before), words(after)
    if len(before) < need_before or len(after) < need_after:
        sys.exit(f"{r['ms']:.3f} ms: ioctl {decode.ioc(r['aux'])} on {r['dev']} "
                 f"lacks its argument ({len(before)} words before, "
                 f"{len(after)} after)")
    return before, after


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("trace")
    ap.add_argument("name", help="C identifier, e.g. h62_720p")
    ap.add_argument("description")
    ap.add_argument("--fixed", required=True, help="frame range FIRST-LAST")
    ap.add_argument("-m", help="register map header, for comments")
    a = ap.parse_args()
    lo, _, hi = a.fixed.partition("-")
    lo, hi = int(lo), int(hi or lo)
    regname = decode.namer(*decode.load_names(a.m)) if a.m else lambda x: ""

    ops, args = [], []
    buf = None  # (phys, size) of the "Isp" block
    i2c_dev = None
    last_ms = None
    fixed_isp, fixed_i2c = {}, {}

    def op(kind, x=0, y=0, note=""):
        ops.append((kind, x, y, note))

    for r in records(a.trace):
        if r["frame"] == 0:
            if last_ms is not None and r["ms"] - last_ms > GAP_MS:
                is_vi_init = (r["type"] == IOCTL and r["dev"] == "/dev/vi"
                              and r["aux"] == 0x5606)
                if not is_vi_init:
                    op("XMISP_DELAY_MS", round(r["ms"] - last_ms))
            last_ms = r["ms"]
        t, dev = r["type"], r["dev"]
        # A call that failed under the vendor daemon isn't part of the script.
        if t in (IOCTL, ISP_W) and decode.s16(r["aux2"]) < 0:
            continue
        if r["frame"] == 0:
            if t == IOCTL and dev == "/dev/mmz" and r["aux"] == 0x4D00:
                op("XMISP_MMZ_RESET")
            elif t == IOCTL and dev == "/dev/vi" and r["aux"] == 0x5606:
                op("XMISP_VI_INIT")
            elif t == IOCTL and dev == "/dev/rgn" and r["aux"] == 0x5201:
                op("XMISP_RGN_RESET")
            elif t == IOCTL and dev == "/dev/vi":
                size = (r["aux"] >> 16) & 0x3FFF
                before, _ = ioctl_args(r, need_before=size // 4)
                before = before[: size // 4]
                op("XMISP_VI_IOW", r["aux"], len(args), decode.ioc(r["aux"]))
                args.extend([len(before)] + before)
            elif t == IOCTL and dev == "/dev/mmz" and (r["aux"] & 0xFF) == 1:
                _, after = ioctl_args(r, need_after=10)
                size, phys = after[8], after[9]
                name = struct.pack("<8I", *after[:8]).split(b"\0")[0].decode()
                buf = (phys, size)
                op("XMISP_ISP_BUF", size, 0, f'MMZ block "{name}"')
            elif t == IOCTL and dev == "/dev/xm_i2c":
                before, after = ioctl_args(
                    r, need_before=5, need_after=5 if r["aux"] == I2C_READ else 0)
                dev_addr, reg, data = before[0] & 0xFF, before[1], before[3]
                i2c_dev = i2c_dev or dev_addr
                if r["aux"] == I2C_WRITE:
                    op("XMISP_I2C_W", reg, data)
                elif r["aux"] == I2C_READ:
                    op("XMISP_I2C_EXPECT", reg, after[3], "chip ID")
            elif t == MEMW:
                (v,) = struct.unpack("<I", r["p"][:4])
                op("XMISP_SYS_W", r["aux"], v, regname(r["aux"]))
            elif t == ISP_W:
                (v,) = struct.unpack("<I", r["p"][:4])
                if buf and buf[0] <= DDR + v * 8 < buf[0] + buf[1]:
                    op("XMISP_ISP_W_BUF", r["aux"], DDR + v * 8 - buf[0],
                       regname(r["aux"]))
                else:
                    op("XMISP_ISP_W", r["aux"], v, regname(r["aux"]))
        elif lo <= r["frame"] <= hi:
            if t == ISP_W:
                (v,) = struct.unpack("<I", r["p"][:4])
                addr = r["aux"]
                if CSM_BANK1 <= addr < CSM_BANK1 + CSM_BANK_LEN:
                    addr -= CSM_BANK1 - CSM_BANK0
                fixed_isp[addr] = v
            elif t == IOCTL and dev == "/dev/xm_i2c" and r["aux"] == I2C_WRITE:
                before, _ = ioctl_args(r, need_before=5)
                fixed_i2c[before[1]] = before[3]
        elif r["frame"] > hi:
            break

    fixed = []
    for reg, v in fixed_i2c.items():
        fixed.append(("XMISP_I2C_W", reg, v, ""))
    for addr, v in fixed_isp.items():
        if addr == CSM_SW:
            continue
        fixed.append(("XMISP_ISP_W", addr, v, regname(addr)))
        if CSM_BANK0 <= addr < CSM_BANK0 + CSM_BANK_LEN:
            b1 = addr + CSM_BANK1 - CSM_BANK0
            fixed.append(("XMISP_ISP_W", b1, v, regname(b1)))
    if CSM_SW in fixed_isp:
        fixed.append(("XMISP_ISP_W", CSM_SW, fixed_isp[CSM_SW], regname(CSM_SW)))

    def emit(name, table):
        print(f"static const struct xmisp_op {name}[] = {{")
        for kind, x, y, note in table:
            c = f" /* {note} */" if note else ""
            print(f"    {{{kind}, {x:#x}, {y:#x}}},{c}")
        print("    {XMISP_END, 0, 0},\n};\n")

    src = os.path.basename(a.trace)
    print(f"""// SPDX-License-Identifier: MIT
/*
 * {a.description}: generated by tools/xm-isp-trace/extract.py from
 * {src} (start-up, and frames {lo}-{hi} as the fixed 3A state). The values
 * are register writes observed on the hardware under the vendor daemon.
 */
#include "script.h"
""")
    print("static const uint32_t args[] = {")
    for i in range(0, len(args), 6):
        print("    " + ", ".join(f"{w:#x}" for w in args[i : i + 6]) + ",")
    print("};\n")
    emit("start", ops)
    emit("fixed", fixed)
    print(f"""const struct xmisp_script xmisp_{a.name} = {{
    .name = "{a.description}",
    .i2c_dev = {i2c_dev or 0:#x},
    .start = start,
    .fixed = fixed,
    .args = args,
}};""")


if __name__ == "__main__":
    main()
