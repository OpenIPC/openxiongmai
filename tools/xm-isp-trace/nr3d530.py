#!/usr/bin/env python3
"""Writes the XM530 ISP library's 3D noise reduction schedule from traces.

The vendor libisp_XM530.so's Run loop sets 3D NR (NR_3D_BASE, 0x30040000)
from the gain, but by a rule that also depends on how the gain got there:
during a ramp the same ISO gets different settings. Where the gain has
settled, though, an ISO gets the same settings every time. So the schedule
is the settled states of the traces, each a decode.py -t text of isptrace.so
with 3aprobe.so's log beside it (PROBE_WORDS=8 PROBE_EVERY=1, for the AE
result's ISO every frame): the state at each of the loop's passes after 14
frames of a steady gain (within 3%, every frame's ISO known), the latest for each ISO. Traces that
disagree about an ISO stop it.

Usage: nr3d530.py NAME DESC TRACE:PROBE...
"""

import re
import sys

REGS = (0x14, 0x50, 0x54, 0x60, 0x64, 0x68, 0x6c, 0x14c, 0x150, 0x154,
        0x158)
BASE = 0x30040000


def isos(path):
    """frame -> the AE result's ISO, from a 3aprobe.so log"""
    out = {}
    text = open(path).read()
    for e in re.split(r'\n(?=[a-z_]+ #\d+ )', text):
        m = re.match(r'ae_run #(\d+)', e)
        o = re.search(r'out3 [0-9a-f]+:\s+(?:[0-9a-f]{8}\s+){2}([0-9a-f]{8})',
                      e)
        if m and o:
            out[int(m.group(1))] = int(o.group(1), 16)
    return out


def settled(trace, probe):
    """{ISO: the 3D NR registers} at each pass after a steady gain"""
    iso = isos(probe)
    tid = None
    state = {}
    out = {}
    for line in open(trace):
        if 'call XM_MPI_ISP_Run(' in line:
            tid = line.split()[2]
        m = re.match(r'\s*[\d.]+ f(\d+)\s+(t\d+)\s+W (\S+) (\S+)', line)
        if not m or m.group(2) != tid:
            continue
        a = int(m.group(3), 16) - BASE
        if a in REGS:
            state[a] = int(m.group(4), 16)
        # a pass starts at its Cb/Cr gain write
        if int(m.group(3), 16) != 0x30076004 or len(state) < len(REGS):
            continue
        f = int(m.group(1))
        if not all(k in iso for k in range(f - 14, f + 1)):
            continue
        window = [iso[k] for k in range(f - 14, f + 1)]
        if max(window) <= 1.03 * min(window):
            out[iso[f]] = [state[r] for r in REGS]
    return out


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    name, desc = sys.argv[1], sys.argv[2]
    points = {}
    for arg in sys.argv[3:]:
        trace, probe = arg.split(':')
        for iso, regs in settled(trace, probe).items():
            if iso in points and points[iso] != regs:
                sys.exit('nr3d530: two settled states for ISO %d' % iso)
            points[iso] = regs
    o = sys.stdout
    o.write('/* %s.\n * 3D noise reduction where the gain has settled, generated '
            'by\n * tools/xm-isp-trace/nr3d530.py from traces of the vendor '
            'library, at\n * %d settled ISOs. Regenerate rather than edit. */\n\n'
            % (desc, len(points)))
    o.write('#include "xm530_start.h"\n\n')
    o.write('/* ISO, then NR_3D_BASE + %s */\n' %
            ', '.join('0x%x' % r for r in REGS))
    o.write('const struct xm530_nr3d %s_nr3d[] = {\n' % name)
    for iso in sorted(points):
        o.write('    {%d, {%s}},\n' % (iso, ', '.join(
            '0x%08x' % v for v in points[iso])))
    o.write('    {0, {0}},\n};\n')


if __name__ == '__main__':
    main()
