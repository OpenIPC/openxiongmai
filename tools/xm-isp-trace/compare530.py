#!/usr/bin/env python3
"""Compares two XM530 ISP libraries' start-up: each register's value after
XM_MPI_ISP_Init and the first pass of XM_MPI_ISP_Run, from decode.py -t
output of isptrace.so logs of majestic on each. Addresses in the "Isp" MMZ
block are compared as offsets into it, and the registers the Run loop sets
by gain (start530.py TUNED) are left out. Exits 1 if they differ.

Usage: compare530.py VENDOR_TRACE OPEN_TRACE
"""

import sys

from extract530 import DDR, find, parse, writes
from start530 import TUNED


def final_state(path):
    recs = parse(path)

    def init_call(t, k, r):
        """XM_MPI_ISP_Init"""
        return k == 'call' and r.startswith('XM_MPI_ISP_Init(')

    i0 = find(recs, init_call)
    tid = recs[i0][0]

    def init_ret(t, k, r):
        """return from XM_MPI_ISP_Init"""
        return t == tid and k == 'ret' and r.startswith('XM_MPI_ISP_Init ')

    def run_call(t, k, r):
        """XM_MPI_ISP_Run"""
        return k == 'call' and r.startswith('XM_MPI_ISP_Run(')

    i1 = find(recs, init_ret, i0)
    mmz = [r for t, k, r in recs[i0:i1] if k == 'mmz']
    if not mmz:
        sys.exit('compare530: %s: no MMZ block in XM_MPI_ISP_Init' % path)
    size, phys = (int(x, 16) for x in mmz[0])
    lo, hi = (phys - DDR) // 8, (phys - DDR + size) // 8
    r0 = find(recs, run_call, i1)
    run = writes(recs, recs[r0][0], r0, len(recs))
    first = None
    for n, (a, v) in enumerate(run):
        if a == 0x30076004:
            if first is not None:
                run = run[:n]
                break
            first = n
    out = {}
    for a, v in writes(recs, tid, i0, i1) + run:
        if a not in TUNED:
            out[a] = ('+%#x' % (v - lo)) if lo <= v < hi else '%#x' % v
    return out


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    a, b = final_state(sys.argv[1]), final_state(sys.argv[2])
    diff = sorted(r for r in set(a) | set(b) if a.get(r) != b.get(r))
    for r in diff:
        print('%#010x  vendor %-12s open %s' % (r, a.get(r, '-'),
                                                 b.get(r, '-')))
    print('%d registers, %d differ' % (len(set(a) | set(b)), len(diff)))
    sys.exit(1 if diff else 0)


if __name__ == '__main__':
    main()
