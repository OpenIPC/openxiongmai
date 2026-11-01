#!/usr/bin/env python3
"""Writes the XM530 ISP library's start-up tables from a trace.

Input: the text decode.py -t writes of an isptrace.so log of majestic on the
vendor libisp_XM530.so. Output: a C file with the ISP writes the library made
itself, in three parts:

  init  inside XM_MPI_ISP_Init, before the AE library's first sensor write
        (the AE library sets the frame rate from its init);
  post  inside XM_MPI_ISP_Init, after the AE and AWB libraries' inits, less
        the AWB statistics window, which the library computes;
  run   the first pass of the XM_MPI_ISP_Run loop;
  attr  each attribute XM_MPI_ISP_Init set itself, as the call's snapshot of
        it holds it: what the library returns until the attribute is set.

Writes of an address inside the "Isp" MMZ block (in 8-byte units from DDR, as
the 3D noise reduction takes them) become offsets into the block, marked
XM530_MMZ. Each part ends with an XM530_END entry.
"""

import argparse
import re
import sys

SNAP = re.compile(r'\s+(arg\d|\+0x[0-9a-f]+)\s+((?:[0-9a-f]{8}\s*)+)$')
# the attribute setters XM_MPI_ISP_Init calls, and the argument holding it
SETTERS = {'Gamma': 0, 'BlackLevel': 0, 'Saturation': 0, 'Sharpen': 0,
           'SharpenV2': 0, 'NR': 0, '3DNr': 0, '3DNrV2': 0,
           'DyDefectPixel': 0, 'CSC': 0, 'AntiFalseColor': 0, 'Chroma': 0,
           'DRC': 1, 'DeFog': 1, 'DCI': 1}
REC = re.compile(r'\s*[\d.]+ f(\d+)\s+t(\d+)\s+(\S+)\s*(.*)')
MMZ_OUT = re.compile(r'\s+out (?:[0-9a-f]{8} ){8}([0-9a-f]{8}) ([0-9a-f]{8})')
DDR = 0x80000000
# The AWB statistics window, computed from the AWB library's init.
AWB_WINDOW = (0x30010010, 0x3001000c)
AWB_GRID = range(0x30011000, 0x30011104, 4)


def parse(path):
    """Records (tid, kind, rest); a call's snapshots go in a 'snap' record
    after it, {argument: words}."""
    recs = []
    with open(path) as f:
        for line in f:
            m = REC.match(line)
            if m:
                recs.append((int(m.group(2)), m.group(3), m.group(4)))
                continue
            if not recs:
                continue
            if recs[-1][1] == 'ioctl':
                o = MMZ_OUT.match(line)
                if o and '/dev/mmz' in recs[-1][2]:
                    recs.append((recs[-1][0], 'mmz', o.groups()))
                continue
            o = SNAP.match(line)
            if o and recs[-1][1] in ('call', 'snap'):
                if recs[-1][1] == 'call':
                    recs.append((recs[-1][0], 'snap', {}))
                snaps = recs[-1][2]
                if o.group(1).startswith('arg'):
                    snaps['cur'] = int(o.group(1)[3:])
                    snaps[snaps['cur']] = []
                snaps[snaps['cur']] += [int(w, 16) for w in o.group(2).split()]
    return recs


def attrs(recs, tid, start, stop):
    """The attributes the library's own setters were given."""
    out = {}
    for i in range(start, stop - 1):
        t, kind, rest = recs[i]
        if t != tid or kind != 'call':
            continue
        m = re.match(r'XM_MPI_ISP_Set(\w+?)(?:Attr?|Att)?\(.*from libisp',
                     rest)
        if not m or m.group(1) not in SETTERS:
            continue
        snap = recs[i + 1]
        arg = SETTERS[m.group(1)]
        if snap[1] == 'snap' and len(snap[2].get(arg, [])) == 128:
            out[m.group(1)] = snap[2][arg]
    return out


def writes(recs, tid, start, stop):
    out = []
    for t, kind, rest in recs[start:stop]:
        if t == tid and kind == 'W':
            a, v = rest.split()[:2]
            out.append((int(a, 16), int(v, 16)))
    return out


def find(recs, pred, start=0):
    for i in range(start, len(recs)):
        if pred(*recs[i]):
            return i
    sys.exit('extract530: the trace has no %s' % pred.__doc__)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('trace', help='decode.py -t output')
    ap.add_argument('name', help='C name prefix, e.g. xm530_imx335')
    ap.add_argument('desc', help='one line for the header comment')
    args = ap.parse_args()
    recs = parse(args.trace)

    def init_call(t, k, r):
        """XM_MPI_ISP_Init"""
        return k == 'call' and r.startswith('XM_MPI_ISP_Init(')

    i0 = find(recs, init_call)
    main_tid = recs[i0][0]

    def init_ret(t, k, r):
        """return from XM_MPI_ISP_Init"""
        return t == main_tid and k == 'ret' and r.startswith('XM_MPI_ISP_Init ')

    i1 = find(recs, init_ret, i0)
    i2c = [i for i in range(i0, i1) if recs[i][0] == main_tid and
           recs[i][1] == 'ioctl' and '/dev/xm_i2c' in recs[i][2]]
    if not i2c:
        sys.exit('extract530: no sensor writes inside XM_MPI_ISP_Init')
    mmz = [r for t, k, r in recs[i0:i1] if k == 'mmz']
    if not mmz:
        sys.exit('extract530: no MMZ allocation inside XM_MPI_ISP_Init')
    size, phys = (int(x, 16) for x in mmz[0])
    lo, hi = (phys - DDR) // 8, (phys - DDR + size) // 8

    def run_call(t, k, r):
        """XM_MPI_ISP_Run"""
        return k == 'call' and r.startswith('XM_MPI_ISP_Run(')

    r0 = find(recs, run_call, i1)
    run_tid = recs[r0][0]
    run = writes(recs, run_tid, r0, len(recs))
    # The loop's first pass ends where it writes its first address again.
    first = None
    for n, (a, v) in enumerate(run):
        if a == 0x30076004:
            if first is not None:
                run = run[:n]
                break
            first = n

    parts = {
        'init': writes(recs, main_tid, i0, i2c[0]),
        'post': [w for w in writes(recs, main_tid, i2c[-1], i1)
                 if w[0] not in AWB_WINDOW and w[0] not in AWB_GRID],
        'run': run,
    }
    attr = attrs(recs, main_tid, i0, i1)
    missing = set(SETTERS) - set(attr)
    if missing:
        sys.exit('extract530: no snapshot of %s' % ', '.join(sorted(missing)))
    o = sys.stdout
    o.write('/* %s.\n * Generated by tools/xm-isp-trace/extract530.py from a '
            'trace of the vendor\n * library: the ISP writes it made itself. '
            'Regenerate rather than edit. */\n\n' % args.desc)
    o.write('#include "xm530_start.h"\n\n')
    o.write('const XM_U32 %s_mmz_size = 0x%x;\n\n' % (args.name, size))
    for part, ws in parts.items():
        o.write('const struct xm530_reg %s_%s[] = {\n' % (args.name, part))
        for a, v in ws:
            if lo <= v < hi:
                o.write('    {0x%08x, 0x%x, XM530_MMZ},\n' % (a, v - lo))
            else:
                o.write('    {0x%08x, 0x%08x, 0},\n' % (a, v))
        o.write('    {0, 0, XM530_END},\n};\n\n')
    o.write('const struct xm530_attr %s_attr[] = {\n' % args.name)
    for name, words in attr.items():
        o.write('    {"%s", {' % name)
        for n, w in enumerate(words):
            o.write('%s0x%08x,' % ('\n         ' if n % 6 == 0 else ' ', w))
        o.write('}},\n')
    o.write('    {NULL, {0}},\n};\n')


if __name__ == '__main__':
    main()
