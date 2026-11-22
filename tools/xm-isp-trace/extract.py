#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Turns isptrace.so logs into libxmisp start-up sources (C).

    extract.py --soc REF.bin NAME                  the SoC's start-up template
    extract.py REF TRACE.bin NAME "DESCRIPTION" --fixed FIRST-LAST
               [--sensor SENSOR] [--ignore ADDR,...]  one sensor and mode

REF is the reference trace, or the template it made (xm510_start.c).

What the vendor daemon did before the first frame, in order, splits the
same way for every sensor it supports (XM510: JX-H62, JX-H42, JX-H65,
SC1235, SC2145H, SC2235, AR0130, OV9750):

    MMZ, VI and region resets, the first PLL setup     SoC
    the chip ID probe                                  sensor: probe
    the sensor's PLL (system controller)               sensor: pll
    VI store setup                                     SoC
    VI channel ioctls and the VI writes after them     sensor: vi
    sensor register writes                             sensor: init
    ISP writes, the "Isp" MMZ block                    SoC
    sensor register writes                             sensor: post
    ISP writes                                         SoC

The SoC parts write the same ISP registers in the same order for every
sensor; only some values differ (frame size, black level, tuning). --soc
writes the template from a reference trace, with the sensor's parts as
slots. For a sensor, the parts come from its trace, and the ISP values that
differ from the reference become its overrides. Every value is checked: the
template with the parts and overrides must reproduce the trace's start-up.

A gap of more than 3 ms before a step becomes a delay; in the SoC's parts
the reference's delays are kept (a trace under emulation runs at another
speed). Values that point into the MMZ block (in 8-byte units from DDR) are
written relative to wherever the block is allocated. The probe keeps only
the sensor's own bus: a trace taken under emulation also shows the vendor
daemon's probes for the sensors before it, which are dropped with their
delays. In the probe the reads are the
chip ID, checked at start: the last run of reads (the daemon stops once
one matches), or only the registers --id names, and never a read of 0 (under
emulation, a register the sensor model doesn't preset). Earlier reads on
the sensor's bus belong to other probes and are only replayed, as is any
read outside the probe. --id is for a value the probe sequence itself puts
there: a 16-bit register address sent to an 8-bit device writes its low
byte as data, so after the SmartSens probe of 0x3005 an MIS1002 reads 0x05
at 0x30.

--ignore names ISP registers whose value the daemon computes from what it
reads of the hardware: under emulation they read 0, so the reference's
value stays. The fixed script holds the last value written to each ISP
register, and to each sensor register, over the frames given: one 3A state,
applied once. The colour matrix is double-buffered (CSM_BUF_SW): both banks
get the matrix. --sensor names the 3A driver (xmisp_sensor_SENSOR).
"""
import argparse
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decode  # noqa: E402

DDR = 0x80000000
GAP_MS = 3.0
ISP_W, IOCTL, MEMW, FRAME = decode.ISPW, decode.IOCTL, decode.MEMW, decode.FRAME
I2C_WRITE, I2C_READ = 1, 3
CSM_BANK0, CSM_BANK1, CSM_BANK_LEN, CSM_SW = 0x40061040, 0x40061080, 0x30, 0x40061000
PARTS = ("probe", "pll", "vi", "init", "post")
I2C_OPS = ("XMISP_I2C_W", "XMISP_I2C_R")


def records(path):
    data = open(path, "rb").read()
    magic, version, t0, _ = decode.HDR.unpack_from(data)
    if magic != decode.MAGIC or version not in decode.VERSIONS:
        sys.exit(f"{path}: not an isptrace log (version 1 to 3)")
    rs = decode.rec_struct(version)
    fds, frame, off = {}, 0, decode.HDR.size
    while off + rs.size <= len(data):
        t, aux, _f, typ, fd, n, aux2, *tid = rs.unpack_from(data, off)
        p = data[off + rs.size : off + rs.size + n]
        off += rs.size + ((n + 3) & ~3)
        if typ == FRAME:
            frame = aux
        if typ in (decode.OPEN, decode.CREAT) and decode.s16(aux2) >= 0:
            fds[decode.s16(aux2)] = decode.cstrs(p)[0]
        yield dict(ms=((t - t0) & 0xFFFFFFFF) / 1000, aux=aux, aux2=aux2,
                   type=typ, dev=fds.get(fd, ""), p=p, frame=frame,
                   version=version, tid=tid[0] if tid else 0)


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


class Trace:
    """A trace's start-up as steps [kind, a, b, note, bus], its VI ioctl
    arguments, and the fixed state over frames lo-hi. bus is (device,
    register bytes, data bytes) for sensor steps."""

    def __init__(self, path, lo=None, hi=None, regname=lambda x: ""):
        self.path, self.steps, self.args = path, [], []
        fixed_isp, fixed_i2c = {}, {}
        buf, last_ms = None, None

        def step(kind, a=0, b=0, note="", bus=None):
            self.steps.append([kind, a, b, note, bus])

        for r in records(path):
            if r["frame"] == 0:
                if last_ms is not None and r["ms"] - last_ms > GAP_MS:
                    if not (r["type"] == IOCTL and r["dev"] == "/dev/vi"
                            and r["aux"] == 0x5606):
                        step("XMISP_DELAY_MS", round(r["ms"] - last_ms))
                last_ms = r["ms"]
            t, dev = r["type"], r["dev"]
            # A call that failed under the vendor daemon isn't part of it.
            if t in (IOCTL, ISP_W) and decode.s16(r["aux2"]) < 0:
                continue
            if r["frame"] == 0:
                if t == IOCTL and dev == "/dev/mmz" and r["aux"] == 0x4D00:
                    step("XMISP_MMZ_RESET")
                elif t == IOCTL and dev == "/dev/vi" and r["aux"] == 0x5606:
                    step("XMISP_VI_INIT")
                elif t == IOCTL and dev == "/dev/rgn" and r["aux"] == 0x5201:
                    step("XMISP_RGN_RESET")
                elif t == IOCTL and dev == "/dev/vi":
                    size = (r["aux"] >> 16) & 0x3FFF
                    before, _ = ioctl_args(r, need_before=size // 4)
                    before = before[: size // 4]
                    step("XMISP_VI_IOW", r["aux"], len(self.args), decode.ioc(r["aux"]))
                    self.args.extend([len(before)] + before)
                elif t == IOCTL and dev == "/dev/mmz" and (r["aux"] & 0xFF) == 1:
                    _, after = ioctl_args(r, need_after=10)
                    size, phys = after[8], after[9]
                    name = struct.pack("<8I", *after[:8]).split(b"\0")[0].decode()
                    buf = (phys, size)
                    step("XMISP_ISP_BUF", size, 0, f'MMZ block "{name}"')
                elif t == IOCTL and dev == "/dev/xm_i2c":
                    read = r["aux"] == I2C_READ
                    before, after = ioctl_args(r, need_before=5,
                                               need_after=5 if read else 0)
                    bus = (before[0] & 0xFF, before[2], before[4])
                    if read:
                        step("XMISP_I2C_R", before[1], after[3], "", bus)
                    elif r["aux"] == I2C_WRITE:
                        step("XMISP_I2C_W", before[1], before[3], "", bus)
                elif t == MEMW:
                    (v,) = struct.unpack("<I", r["p"][:4])
                    step("XMISP_SYS_W", r["aux"], v, regname(r["aux"]))
                elif t == ISP_W:
                    (v,) = struct.unpack("<I", r["p"][:4])
                    if buf and buf[0] <= DDR + v * 8 < buf[0] + buf[1]:
                        step("XMISP_ISP_W_BUF", r["aux"], DDR + v * 8 - buf[0],
                             regname(r["aux"]))
                    else:
                        step("XMISP_ISP_W", r["aux"], v, regname(r["aux"]))
            elif lo is not None and lo <= r["frame"] <= hi:
                if t == ISP_W:
                    (v,) = struct.unpack("<I", r["p"][:4])
                    addr = r["aux"]
                    if CSM_BANK1 <= addr < CSM_BANK1 + CSM_BANK_LEN:
                        addr -= CSM_BANK1 - CSM_BANK0
                    fixed_isp[addr] = v
                elif t == IOCTL and dev == "/dev/xm_i2c" and r["aux"] == I2C_WRITE:
                    before, _ = ioctl_args(r, need_before=5)
                    fixed_i2c[before[1]] = before[3]
            elif hi is not None and r["frame"] > hi:
                break

        self.fixed = [["XMISP_I2C_W", reg, v, "", None] for reg, v in fixed_i2c.items()]
        for addr, v in fixed_isp.items():
            if addr == CSM_SW:
                continue
            self.fixed.append(["XMISP_ISP_W", addr, v, regname(addr), None])
            if CSM_BANK0 <= addr < CSM_BANK0 + CSM_BANK_LEN:
                b1 = addr + CSM_BANK1 - CSM_BANK0
                self.fixed.append(["XMISP_ISP_W", b1, v, regname(b1), None])
        if CSM_SW in fixed_isp:
            self.fixed.append(["XMISP_ISP_W", CSM_SW, fixed_isp[CSM_SW], regname(CSM_SW), None])
        self.split()

    def fail(self, why):
        sys.exit(f"{self.path}: {why}")

    def split(self):
        """Segments in start-up order: ("soc", steps) or (part, steps). A
        delay goes with the step after it."""
        seq, segs, i = self.steps, [], 0

        def kind(j):
            j = next((k for k in range(j, len(seq)) if seq[k][0] != "XMISP_DELAY_MS"), len(seq))
            return seq[j][0] if j < len(seq) else None, j

        def take(name, pred):
            nonlocal i
            start = i
            while True:
                k, j = kind(i)
                if k is None or not pred(k):
                    break
                i = j + 1
            segs.append((name, seq[start:i]))

        take("soc", lambda k: k not in I2C_OPS)            # resets, first PLL
        take("probe", lambda k: k in I2C_OPS)
        take("pll", lambda k: k == "XMISP_SYS_W")
        take("soc", lambda k: k not in ("XMISP_VI_IOW",) + I2C_OPS)
        take("vi", lambda k: k not in I2C_OPS)
        take("init", lambda k: k in I2C_OPS)
        take("soc", lambda k: k not in I2C_OPS)
        take("post", lambda k: k in I2C_OPS)
        take("soc", lambda k: k not in I2C_OPS)
        # Delays after the last step (before the first frame) go with it.
        if all(s[0] == "XMISP_DELAY_MS" for s in seq[i:]):
            segs[-1][1].extend(seq[i:])
            i = len(seq)
        if i != len(seq):
            self.fail(f"start-up doesn't follow the template: {seq[i][0]} at step {i}")
        for name, steps in segs:
            if name == "vi" and not any(s[0] == "XMISP_VI_IOW" for s in steps):
                self.fail("no VI channel setup")
            if name in ("pll", "init") and not steps:
                self.fail(f"no sensor {name}")
        self.segs = segs

    def frame(self):
        """VI channel 0's size, from its _IOW('V', 1, 12) setup {chn, w, h}:
        the main picture, so the mode."""
        for name, steps in self.segs:
            for s in steps:
                if s[0] == "XMISP_VI_IOW" and s[1] == 0x400C5601:
                    n, chn, w, h = self.args[s[2] : s[2] + 4]
                    if n >= 3 and chn == 0:
                        return w, h
        self.fail("no VI channel 0 setup")

    def bus(self):
        buses = {tuple(s[4]) for name, st in self.segs if name in ("init", "post")
                 for s in st if s[4]}
        if len(buses) != 1:
            self.fail(f"sensor writes on more than one bus: {sorted(buses)}")
        return buses.pop()

    def part(self, name, bus, ids=None):
        steps = [s for n, st in self.segs if n == name for s in st]
        if name == "probe":
            # Only the sensor's own address and register width; a delay goes
            # with the step after it, kept or dropped.
            kept, delays = [], []
            for s in steps:
                if s[0] == "XMISP_DELAY_MS":
                    delays.append(s)
                elif s[0] not in I2C_OPS or s[4][:2] == bus[:2]:
                    kept += delays + [s]
                    delays = []
                else:
                    delays = []
            steps = kept
            while steps and steps[0][0] == "XMISP_DELAY_MS":
                steps.pop(0)
            # The chip ID is the last run of reads: the daemon stops probing
            # once it matches. Earlier reads on the bus are other probes.
            last = len(steps)
            while last and steps[last - 1][0] in ("XMISP_I2C_R", "XMISP_DELAY_MS"):
                last -= 1
            for k, s in enumerate(steps):
                if s[0] == "XMISP_I2C_R" and s[2] and k >= last and (not ids or s[1] in ids):
                    s[0], s[3] = "XMISP_I2C_EXPECT", "chip ID"
        # The interpreter drives every sensor step with the script's widths.
        for s in steps:
            if s[4] and s[0] in I2C_OPS + ("XMISP_I2C_EXPECT",) and s[4][2] != bus[2]:
                self.fail(f"{name}: a {s[4][2]}-byte sensor value on a {bus[2]}-byte bus")
        if not any(s[0] == "XMISP_I2C_EXPECT" for s in steps) and name == "probe":
            self.fail("no chip ID read on the sensor's bus")
        return steps

    def soc(self):
        return [s for n, st in self.segs if n == "soc" for s in st]


class Template:
    """The SoC's template as a reference: the steps of a generated template
    source (xm510_start.c), its slots dropped."""

    STEP = re.compile(r"\{(XMISP_\w+), (\w+), (0x[0-9a-f]+|\d+)\}")

    def __init__(self, path):
        self.path, self.steps = path, []
        src = re.sub(r"/\*.*?\*/", "", open(path).read(), flags=re.S)
        for m in self.STEP.finditer(src):  # clang-format may put two on a line
            if m.group(1) not in ("XMISP_END", "XMISP_SENSOR"):
                self.steps.append([m.group(1), int(m.group(2), 0), int(m.group(3), 0), "", None])
        if not self.steps:
            self.fail("no template steps")

    def soc(self):
        return self.steps

    def fail(self, why):
        sys.exit(f"{self.path}: {why}")


def template(ref):
    """The reference's start-up with its sensor parts as slots."""
    out = []
    for name, steps in ref.segs:
        if name == "soc":
            out.extend(steps)
        else:
            out.append(["XMISP_SENSOR", PARTS.index(name), 0, f"sensor: {name}", None])
    return out


def overrides(ref, tr, ignore):
    """ISP values in tr's SoC parts that differ from ref's, by register, and
    the "Isp" block's size if it differs. The same steps in the same order,
    or the template doesn't fit. An override replaces the register's last
    write in the template: where the daemon writes a register twice, the
    first is a default the same for every sensor."""
    a = [s for s in ref.soc() if s[0] != "XMISP_DELAY_MS"]
    b = [s for s in tr.soc() if s[0] != "XMISP_DELAY_MS"]
    key = lambda s: (s[0], s[1] if s[0] != "XMISP_ISP_BUF" else 0)
    if [key(s) for s in a] != [key(s) for s in b]:
        n = next(k for k, (x, y) in enumerate(zip(a + [None], b + [None]))
                 if x is None or y is None or (x[0], x[1]) != (y[0], y[1]))
        tr.fail(f"SoC step {n} differs from the reference: "
                f"{a[n][:2] if n < len(a) else None} vs {b[n][:2] if n < len(b) else None}")
    last = {x[1]: i for i, x in enumerate(a) if x[0] in ("XMISP_ISP_W", "XMISP_ISP_W_BUF")}
    over, buf = {}, 0
    for i, (x, y) in enumerate(zip(a, b)):
        if x[0] == "XMISP_ISP_BUF":
            buf = y[1] if y[1] != x[1] else 0
            continue
        if x[2] == y[2] or x[1] in ignore:
            continue
        if x[0] not in ("XMISP_ISP_W", "XMISP_ISP_W_BUF"):
            tr.fail(f"{x[0]} {x[1]:#x}: {y[2]:#x}, not {x[2]:#x} as in the reference")
        if last[x[1]] != i:
            tr.fail(f"ISP register {x[1]:#x}: a write other than its last differs "
                    f"({y[2]:#x}, not {x[2]:#x})")
        over[x[1]] = y[2]
    return over, buf


def c_steps(name, steps, static=True):
    out = [f"{'static ' if static else ''}const struct xmisp_op {name}[] = {{"]
    for kind, x, y, note, _bus in steps:
        c = f" /* {note} */" if note else ""
        if kind == "XMISP_SENSOR":
            out.append(f"    {{XMISP_SENSOR, XMISP_PART_{PARTS[x].upper()}, 0}},{c}")
        else:
            out.append(f"    {{{kind}, {x:#x}, {y:#x}}},{c}")
    out.append("    {XMISP_END, 0, 0},\n};\n")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--soc", metavar="REF", help="write the SoC template from REF")
    ap.add_argument("traces", nargs="*", help="REF.bin TRACE.bin")
    ap.add_argument("--fixed", help="frame range FIRST-LAST")
    ap.add_argument("--sensor", help="3A driver, e.g. h62")
    ap.add_argument("--ignore", default="", help="ISP registers to keep from REF")
    ap.add_argument("--id", default="", help="the probe's chip ID registers, if not all its last reads")
    ap.add_argument("-m", help="register map header, for comments")
    a = ap.parse_args()
    regname = decode.namer(*decode.load_names(a.m)) if a.m else lambda x: ""

    if a.soc:
        if len(a.traces) != 1:
            ap.error("--soc REF NAME")
        ref = Trace(a.soc, regname=regname)
        name = a.traces[0]
        print(f"""// SPDX-License-Identifier: MIT
/*
 * The XM510 ISP start-up, generated by tools/xm-isp-trace/extract.py from
 * {os.path.basename(a.soc)}: the vendor daemon's steps, the same for every
 * sensor it supports, with the sensor's parts as XMISP_SENSOR slots. The
 * values are register writes observed on the hardware.
 */
#include "script.h"
""")
        print(c_steps(f"xmisp_{name}", template(ref), static=False))
        return

    if len(a.traces) != 4 or not a.fixed:
        ap.error("REF.bin TRACE.bin NAME DESCRIPTION --fixed FIRST-LAST")
    refpath, path, name, desc = a.traces
    lo, _, hi = a.fixed.partition("-")
    lo, hi = int(lo), int(hi or lo)
    ignore = {int(x, 0) for x in a.ignore.split(",") if x}
    ref = Template(refpath) if refpath.endswith(".c") else Trace(refpath, regname=regname)
    tr = Trace(path, lo, hi, regname)
    bus = tr.bus()
    over, buf = overrides(ref, tr, ignore)
    ids = {int(x, 0) for x in a.id.split(",") if x}
    parts = {p: tr.part(p, bus, ids) for p in PARTS}

    src = os.path.basename(path)
    print(f"""// SPDX-License-Identifier: MIT
/*
 * {desc}: generated by tools/xm-isp-trace/extract.py from {src} (start-up,
 * and frames {lo}-{hi} as the fixed 3A state), as the sensor's parts of the
 * XM510 start-up and the ISP values that differ from
 * {os.path.basename(refpath)}. The values are register writes observed
 * under the vendor daemon.
 */
#include "script.h"
""")
    print("static const uint32_t args[] = {")
    for i in range(0, len(tr.args), 6):
        print("    " + ", ".join(f"{w:#x}" for w in tr.args[i : i + 6]) + ",")
    print("};\n")
    for p in PARTS:
        print(c_steps(p, parts[p]))
    print("static const struct xmisp_isp_val isp[] = {")
    for addr, v in over.items():
        n = regname(addr)
        print(f"    {{{addr:#x}, {v:#x}}}," + (f" /* {n} */" if n else ""))
    print("    {0, 0},\n};\n")
    print(c_steps("fixed", tr.fixed))
    sensor = f"\n    .sensor = &xmisp_sensor_{a.sensor}," if a.sensor else ""
    print(f"""const struct xmisp_script xmisp_{name} = {{
    .name = "{desc}",{sensor}
    .width = {tr.frame()[0]},
    .height = {tr.frame()[1]},
    .i2c_dev = {bus[0]:#x},
    .reg_bytes = {bus[1]},
    .data_bytes = {bus[2]},
    .part = {{probe, pll, vi, init, post}},
    .isp = isp,{f"""
    .isp_buf = {buf:#x},""" if buf else ""}
    .fixed = fixed,
    .args = args,
}};""")


if __name__ == "__main__":
    main()
