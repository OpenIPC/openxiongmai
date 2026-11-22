#!/usr/bin/env python3
"""Writes the XM530 ISP library's start-up for every sensor from traces.

The vendor libisp_XM530.so starts every sensor the same way, but for four
things, and this checks that on each trace given:

  geometry   the 3D noise reduction's buffers and line strides and the
             local tone mapping grid follow the picture size (formulas
             below; XM530_GEO entries);
  defaults   eleven attribute fields are the sensor library's
             ISP_CMOS_DEFAULT_S (its AGC table and gamma), which the library
             gets through pfn_cmos_get_isp_default (MAPPED);
  tuning     the registers the Run loop sets by gain (TUNED), which the open
             library's isp_tune.c sets itself;
  sensor     what the vendor library has built in for some sensors by their
             SENSOR_CHIP id: tables in SharpenV2, 3DNrV2 and DRC, and some
             registers. These become per-sensor overrides.

Everything else is one template: the first trace's writes (the lab
IMX335), each register's final value and each attribute byte being what
most traces have. A register a trace leaves out is one the vendor's setter
skipped because the value was already set; the template writes it.

Input: decode.py -t output of isptrace.so logs of majestic on the vendor
library, the first being the template's. Fails, naming what, unless every
trace is the template with its geometry, defaults and overrides.

Usage: start530.py TRACE... > libraries/libisp_XM530/arch/xm530/xm530_start.c
"""

import collections
import os
import re
import sys

from extract530 import extract

HERE = os.path.dirname(os.path.abspath(__file__))
CAMERA_H = os.path.join(HERE, '../../libraries/isp/include/xm/Camera.h')

# sizeof each attribute (the SDK headers, gcc for ARM): the trace's
# snapshot of a setter's argument runs on into the caller's stack
SIZES = {'Gamma': 204, 'BlackLevel': 40, 'Saturation': 28, 'Sharpen': 64,
         'SharpenV2': 116, 'NR': 28, '3DNr': 52, '3DNrV2': 100,
         'DyDefectPixel': 24, 'CSC': 80, 'AntiFalseColor': 8, 'Chroma': 24,
         'DRC': 72, 'DeFog': 100, 'DCI': 28}
# (offset, length) of the fields the library fills from the sensor's
# defaults; isp_sensor_defaults() in mpi_isp.c, by field name
MAPPED = {'Gamma': [(8, 194)], 'BlackLevel': [(6, 32)],
          'Saturation': [(9, 16)], 'Sharpen': [(13, 48)], 'NR': [(10, 16)],
          '3DNr': [(11, 32)], 'DyDefectPixel': [(5, 16)], 'CSC': [(28, 16)]}
# the registers isp_tune.c and the 3A thread set
TUNED = ({0x30036000, 0x30036004, 0x30036008, 0x30036010, 0x3007500c,
          0x3007508c, 0x3007511c, 0x30004000, 0x30004004, 0x30004008,
          0x30076004, 0x30076008, 0x30035008, 0x3003500c, 0x3000b020,
          0x3000b024, 0x30005000} |
         {0x30072000 + 4 * k for k in range(65)} |
         {0x30040000 + o for o in (0x14, 0x40, 0x44, 0x48, 0x4c, 0x50, 0x54,
                                   0x58, 0x5c, 0x60, 0x64, 0x68, 0x6c, 0x130,
                                   0x134, 0x138, 0x14c, 0x150, 0x154, 0x158)})
# the geometry, by register: XM530_GEO entries, their val one of these
GEO = {0x30040038: 'XM530_GEO_NR3D_MOTION', 0x3004003c: 'XM530_GEO_NR3D_ERR',
       0x300400c8: 'XM530_GEO_NR3D_FRAME_LINE',
       0x300400cc: 'XM530_GEO_NR3D_MOTION_LINE',
       0x300400d0: 'XM530_GEO_NR3D_ERR_LINE',
       0x3000b000: 'XM530_GEO_LTM_NX', 0x3000b004: 'XM530_GEO_LTM_NY',
       0x3000b008: 'XM530_GEO_LTM_STEP_X', 0x3000b00c: 'XM530_GEO_LTM_STEP_Y',
       0x3000b010: 'XM530_GEO_LTM_TL', 0x3000b014: 'XM530_GEO_LTM_TR',
       0x3000b018: 'XM530_GEO_LTM_BL', 0x3000b01c: 'XM530_GEO_LTM_BR'}


def geometry(w, h, mmz8):
    """What the vendor library writes for a w x h picture, its "Isp" MMZ
    block at mmz8 (8-byte units from DDR); xm530_geometry() in C"""
    def cdiv(a, b):
        return -(-a // b)
    frame, motion = cdiv(w, 5) * 8 * h, cdiv(w, 10) * 4 * h
    nx, ny = (w + 16) // 32 - 1, (h + 16) // 32 - 1
    kx, ky = w - 31 * (nx + 1), h - 31 * (ny + 1)
    m = 1 << 20
    return {'XM530_GEO_NR3D_MOTION': mmz8 + frame // 8,
            'XM530_GEO_NR3D_ERR': mmz8 + cdiv(frame + motion, 8),
            'XM530_GEO_NR3D_FRAME_LINE': cdiv(w, 5),
            'XM530_GEO_NR3D_MOTION_LINE': cdiv(w, 10),
            'XM530_GEO_NR3D_ERR_LINE': cdiv(w, 16),
            'XM530_GEO_LTM_NX': nx, 'XM530_GEO_LTM_NY': ny,
            'XM530_GEO_LTM_STEP_X': m // (nx + 1),
            'XM530_GEO_LTM_STEP_Y': m // (ny + 1),
            'XM530_GEO_LTM_TL': m // ((nx + 1) * (ny + 1)),
            'XM530_GEO_LTM_TR': m // (kx * (ny + 1)),
            'XM530_GEO_LTM_BL': m // ((nx + 1) * ky),
            'XM530_GEO_LTM_BR': m // (kx * ky)}


def sensor_names():
    names = {}
    for line in open(CAMERA_H, encoding='latin1'):
        m = re.match(r'\s*(SENSOR_CHIP_\w+)\s*=\s*(\d+)', line)
        if m:
            names[int(m.group(2))] = m.group(1)
    return names


def attr_bytes(words, name):
    return bytes(b for w in words for b in w.to_bytes(4, 'little'))[
        :SIZES[name]]


def mapped(name, i):
    return any(o <= i < o + n for o, n in MAPPED.get(name, []))


def state(t):
    """A trace's registers after the first Run pass, but the tuned ones,
    MMZ-relative values as offsets, and its geometry checked"""
    lo, hi, _ = t['mmz']
    x, y, w, h = t['window']
    geo = geometry(w, h, lo)
    out, where = {}, {}
    for part in ('init', 'post', 'run'):
        for a, v in t['parts'][part]:
            if a in GEO:
                if v != geo[GEO[a]]:
                    sys.exit('start530: %s at %ux%u: %#x is %#x, the geometry '
                             'gives %#x' % (t['name'], w, h, a, v,
                                            geo[GEO[a]]))
                continue
            if a in TUNED:
                continue
            out[a] = ('mmz', v - lo) if lo <= v < hi else ('val', v)
            where[a] = part
    return out, where


def majority(values, first):
    c = collections.Counter(values)
    best = max(c.values())
    return first if c[first] == best else c.most_common(1)[0][0]


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    names = sensor_names()
    traces = []
    for path in sys.argv[1:]:
        t = extract(path)
        t['name'] = '%s (%s)' % (names.get(t['sensor'], t['sensor']),
                                 os.path.basename(path))
        t['state'], t['where'] = state(t)
        t['bytes'] = {k: attr_bytes(v, k) for k, v in t['attr'].items()}
        traces.append(t)
    if len({t['sensor'] for t in traces}) != len(traces):
        sys.exit('start530: two traces of one sensor')
    tmpl = traces[0]

    # each register's and attribute byte's default: the most traces' value
    regs = {}
    for a in sorted({a for t in traces for a in t['state']}):
        have = [t['state'][a] for t in traces if a in t['state']]
        if len(set(have)) == 1:
            regs[a] = have[0]  # unwritten elsewhere: the setter skipped it
        else:
            regs[a] = majority(have, tmpl['state'].get(a, have[0]))
    attr = {}
    for name in SIZES:
        b = bytearray(tmpl['bytes'][name])
        for i in range(SIZES[name]):
            if not mapped(name, i):
                b[i] = majority([t['bytes'][name][i] for t in traces], b[i])
        attr[name] = bytes(b)

    # what each sensor keeps of its own
    sensors = []
    for t in traces:
        rp = {p: [] for p in ('init', 'post', 'run')}
        for a, v in sorted(t['state'].items()):
            if regs.get(a) != v:
                if v[0] != 'val':
                    sys.exit('start530: %s: an MMZ address at %#x' %
                             (t['name'], a))
                part = tmpl['where'].get(a, t['where'][a])
                rp[part].append((a, v[1]))
        ap = []
        for name in SIZES:
            mine, base = t['bytes'][name], attr[name]
            diff = [i for i in range(SIZES[name])
                    if not mapped(name, i) and mine[i] != base[i]]
            runs = []
            for i in diff:
                if runs and i - runs[-1][1] <= 4:
                    runs[-1][1] = i
                else:
                    runs.append([i, i])
            ap += [(name, s, mine[s:e + 1]) for s, e in runs]
        sensors.append((t, rp, ap))

    # the template's tables, with the defaults
    out = sys.stdout
    out.write('/* XM530/XM550 ISP start-up for every sensor: one template, '
              'its geometry\n * computed, its defaults from the sensor '
              'library, and what the vendor\n * library has built in for '
              'some sensors. Generated by\n * tools/xm-isp-trace/start530.py '
              'from traces of the vendor library:\n')
    for t in traces:
        out.write(' *   %s, %ux%u\n' % ((t['name'],) + t['window'][2:]))
    out.write(' * Regenerate rather than edit. */\n\n#include '
              '"xm530_start.h"\n\n')
    lo = tmpl['mmz'][0]
    for part in ('init', 'post', 'run'):
        seq = tmpl['parts'][part]
        last = {a: n for n, (a, v) in enumerate(seq)}
        out.write('const struct xm530_reg xm530_%s[] = {\n' % part)
        for n, (a, v) in enumerate(seq):
            if a in GEO:
                out.write('    {0x%08x, %s, XM530_GEO},\n' % (a, GEO[a]))
                continue
            if a not in TUNED and last[a] == n:
                kind, v = regs[a]
            else:
                kind = 'mmz' if lo <= v < tmpl['mmz'][1] else 'val'
                v = v - lo if kind == 'mmz' else v
            out.write('    {0x%08x, 0x%08x, %s},\n' %
                      (a, v, 'XM530_MMZ' if kind == 'mmz' else '0'))
        if part == 'init':
            # registers only the setters' cold-boot writes set
            for a in sorted(regs):
                if a not in tmpl['state'] and a not in GEO:
                    out.write('    {0x%08x, 0x%08x, 0},\n' % (a, regs[a][1]))
        out.write('    {0, 0, XM530_END},\n};\n\n')
    out.write('/* the attributes the library starts with, before the '
              'sensor\'s defaults */\nconst struct xm530_attr xm530_attr[] '
              '= {\n')
    for name in SIZES:
        b = attr[name] + bytes(-len(attr[name]) % 4)
        words = [int.from_bytes(b[i:i + 4], 'little')
                 for i in range(0, len(b), 4)]
        out.write('    {"%s", {' % name)
        for n, w in enumerate(words):
            out.write('%s0x%08x,' % ('\n         ' if n % 6 == 0 else ' ', w))
        out.write('}},\n')
    out.write('    {NULL, {0}},\n};\n\n')
    for t, rp, ap in sensors:
        if not ap and not any(rp.values()):
            continue
        sid = names[t['sensor']].lower()[len('sensor_chip_'):]
        for part, ws in rp.items():
            if ws:
                out.write('static const struct xm530_reg %s_%s[] = {\n' %
                          (sid, part))
                for a, v in ws:
                    out.write('    {0x%08x, 0x%08x, 0},\n' % (a, v))
                out.write('    {0, 0, XM530_END},\n};\n')
        if ap:
            out.write('static const struct xm530_patch %s_attr[] = {\n' % sid)
            for name, start, data in ap:
                out.write('    {"%s", %u, %u, {%s}},\n' % (
                    name, start, len(data), ', '.join('%u' % x for x in data)))
            out.write('    {NULL, 0, 0, {0}},\n};\n')
        out.write('\n')
    out.write('const struct xm530_sensor xm530_sensors[] = {\n')
    for t, rp, ap in sensors:
        if not ap and not any(rp.values()):
            continue
        sid = names[t['sensor']].lower()[len('sensor_chip_'):]
        out.write('    {%u /* %s */, %s, %s, %s, %s},\n' % (
            t['sensor'], names[t['sensor']],
            *(('%s_%s' % (sid, p)) if rp[p] else 'NULL'
              for p in ('init', 'post', 'run')),
            '%s_attr' % sid if ap else 'NULL'))
    out.write('    {0, NULL, NULL, NULL, NULL},\n};\n')


if __name__ == '__main__':
    main()
