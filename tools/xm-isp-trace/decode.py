#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Turns an isptrace.so log into text, one record a line.

    decode.py trace.bin [-r] [-f FIRST-LAST] [-m HEADER]

-r prints only the register traffic and frame starts.
-f limits the output to a range of frames.
-m names registers from a register map header such as
   libraries/isp/include/xm/xm510_isp.h: a #define naming the address, or
   the nearest *_BASE below it in the same 4 KiB block, plus the offset.
Times are milliseconds since the tracer started.
"""
import argparse
import re
import struct
import sys

MAGIC = 0x54495358
VERSIONS = (1, 2)
HDR = struct.Struct("<4I")
REC = struct.Struct("<IIHBBHH")

(FRAME, ISPW, ISPR, IOCTL, OPEN, CREAT, FOPEN, ACCESS, CLOSE, READ, MSGSND,
 MSGRCV, XMAP, SIGNAL, MEMW, MEMRW, CALL) = range(1, 18)
CALL_HDR = struct.Struct("<32s32s5I")  # struct isptrace_call
CALL_SNAP = 512


def s16(v):
    return v - 0x10000 if v & 0x8000 else v


def cstrs(p):
    return [x.decode("latin-1") for x in p.split(b"\0") if x]


def ioc(cmd):
    """The _IOC fields of an ioctl command, or the raw number."""
    d, size, typ, nr = cmd >> 30, (cmd >> 16) & 0x3FFF, (cmd >> 8) & 0xFF, cmd & 0xFF
    if d == 0 and size == 0:
        if 0x20 < typ < 0x7F:
            return f"_IO('{chr(typ)}',{nr})"
        return f"{cmd:#x}"
    name = ["_IO", "_IOW", "_IOR", "_IOWR"][d]
    t = f"'{chr(typ)}'" if 0x20 < typ < 0x7F else f"{typ:#x}"
    return f"{name}({t},{nr},{size})"


def load_names(path):
    """Address -> name for the #defines of a register map header."""
    env, names = {}, {}
    text = open(path, "rb").read().decode("latin-1")
    for m in re.finditer(r"^\s*#define\s+(\w+)\s+(.+?)\s*(?://.*|/\*.*)?$", text, re.M):
        name, expr = m.group(1), m.group(2)
        expr = re.sub(r"(0[xX][0-9a-fA-F]+|\d+)[uUlL]+", r"\1", expr)
        if not re.fullmatch(r"[\w\s()+\-*<>|]+", expr):
            continue
        try:
            v = eval(expr, {"__builtins__": {}}, env)
        except Exception:
            continue
        if isinstance(v, int):
            env[name] = v
            if 0x10000000 <= v < 0x80000000:
                names.setdefault(v, name)
    bases = sorted((v, n) for v, n in names.items() if n.endswith("_BASE"))
    return names, bases


def namer(names, bases):
    def name(addr):
        if addr in names:
            return names[addr]
        best = None
        for v, n in bases:
            if v <= addr and addr - v < 0x1000:
                best = (v, n)
        if best:
            return f"{best[1]}+{addr - best[0]:#x}"
        return ""

    return name


def ioctl_snapshots(p, version):
    """An ioctl record's argument value and the bytes at it before and after
    (None if unreadable). Version 1 had one flag for both, the after-image
    present whenever the before-image was."""
    arg, flags = struct.unpack_from("<II", p)
    body = p[8:]
    if version == 1:
        if not flags:
            return arg, None, None
        half = len(body) // 2
        return arg, body[:half], body[half:] or None
    before = after = None
    if flags & 1:
        if flags & 2:
            half = len(body) // 2
            before, after = body[:half], body[half:]
        else:
            before = body
    return arg, before, after


def words(b):
    n = len(b) // 4
    return " ".join(f"{w:08x}" for w in struct.unpack(f"<{n}I", b[: n * 4]))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("trace")
    ap.add_argument("-r", action="store_true", help="registers and frames only")
    ap.add_argument("-c", action="store_true",
                    help="ISP library calls and frames only (XM530), without snapshots")
    ap.add_argument("-f", help="frame range FIRST-LAST")
    ap.add_argument("-m", help="register map header")
    a = ap.parse_args()
    regname = namer(*load_names(a.m)) if a.m else lambda addr: ""
    lo, hi = 0, 1 << 32
    if a.f:
        x, _, y = a.f.partition("-")
        lo, hi = int(x), int(y) if y else int(x)

    data = open(a.trace, "rb").read()
    magic, version, t0, _ = HDR.unpack_from(data)
    if magic != MAGIC or version not in VERSIONS:
        sys.exit(f"{a.trace}: not an isptrace log (version 1 or 2)")
    fds = {}
    frame = 0
    off = HDR.size
    out = sys.stdout
    while off + REC.size <= len(data):
        t, aux, _f16, typ, fd, n, aux2 = REC.unpack_from(data, off)
        p = data[off + REC.size : off + REC.size + n]
        off += REC.size + ((n + 3) & ~3)
        if typ == FRAME:
            frame = aux
        ms = ((t - t0) & 0xFFFFFFFF) / 1000
        if typ in (OPEN, CREAT) and s16(aux2) >= 0:
            fds[s16(aux2)] = cstrs(p)[0]
        if not lo <= frame <= hi:
            continue
        if a.r and typ not in (FRAME, ISPW, ISPR, MEMW, MEMRW, CALL):
            continue
        if a.c and typ not in (FRAME, CALL):
            continue
        dev = fds.get(fd, f"fd{fd}")
        head = f"{ms:10.3f} f{frame:<6}"
        if typ == FRAME:
            line = f"--- frame {aux}"
        elif typ in (ISPW, ISPR):
            (v,) = struct.unpack("<I", p[:4])
            op = "W" if typ == ISPW else "R"
            line = f"{op} {aux:08x} {v:08x}"
            if s16(aux2):
                line += f" -> {s16(aux2)}"
            if regname(aux):
                line += f"  {regname(aux)}"
        elif typ == IOCTL:
            arg, before, after = ioctl_snapshots(p, version)
            line = f"ioctl {dev} {ioc(aux)} arg={arg:#x} -> {s16(aux2)}"
            if before is not None:
                line += f"\n{'':18}in  {words(before)}"
            if after is not None and after != before:
                line += f"\n{'':18}out {words(after)}"
        elif typ in (OPEN, CREAT):
            name = "open" if typ == OPEN else "creat"
            line = f"{name} {(cstrs(p) or [''])[0]} {aux:#o} -> {s16(aux2)}"
        elif typ == FOPEN:
            path, mode = (cstrs(p) + ["", ""])[:2]
            line = f"fopen {path} {mode} -> {'ok' if aux2 else 'fail'}"
        elif typ == ACCESS:
            line = f"access {(cstrs(p) or [''])[0]} {aux} -> {s16(aux2)}"
        elif typ == CLOSE:
            line = f"close {dev}"
            fds.pop(fd, None)
        elif typ == READ:
            line = f"read {dev} {aux} -> {s16(aux2)} {p.hex()}"
        elif typ in (MSGSND, MSGRCV):
            name = "msgsnd" if typ == MSGSND else "msgrcv"
            line = f"{name} q{aux} -> {s16(aux2)} {words(p)}"
        elif typ == XMAP:
            ln, prot, flags, addr = struct.unpack("<4I", p)
            line = f"xmap {dev} off={aux:#x} len={ln:#x} prot={prot} flags={flags:#x} -> {addr:#x}"
            if aux2:
                line += " (stores traced)"
        elif typ == MEMW:
            (v,) = struct.unpack("<I", p[:4])
            line = f"M{aux2} {aux:08x} {v & ((1 << 8 * aux2) - 1):0{2 * aux2}x}"
            if regname(aux):
                line += f"  {regname(aux)}"
        elif typ == MEMRW:
            (insn,) = struct.unpack("<I", p[:4])
            what = "partly unmapped" if insn == 0xFFFFFFFF else f"store {insn:08x}"
            line = f"untraced from here: page {aux:08x}, {what}"
        elif typ == CALL:
            name, frm, a0, a1, a2, a3, mask = CALL_HDR.unpack_from(p)
            name = name.split(b"\0")[0].decode()
            frm = frm.split(b"\0")[0].decode()
            args = ", ".join(f"{v:#x}" for v in (a0, a1, a2, a3))
            line = (f"call {name}({args}) from {frm}" if aux == 0
                    else f"ret  {name} -> {s16(aux2)}")
            at = CALL_HDR.size  # the snapshots, in the payload
            for i in range(4 if not a.c else 0):
                if mask & (1 << i) and at + CALL_SNAP <= len(p):
                    line += f"\n{'':18}arg{i} {words(p[at:at + 64])}"
                    at += CALL_SNAP
        elif typ == SIGNAL:
            line = f"signal {aux}" + {0: "", 1: " (wrapped)", 2: " delivered"}.get(aux2, "")
        else:
            line = f"type {typ}?"
        out.write(f"{head} {line}\n")


if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:
        pass
