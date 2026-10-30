#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""gen_tuning.py MERGED_TRACE PAGE_BIN LDC_BIN NAME [FAMILY] > sns_NAME_tuning.c

Turns a reference capture of the vendor stack (see README.md) into
libxmv200's tuning for one sensor mode on one chip family:

- channel 0's program: what the vendor isp process does for it, in order.
  That is isp.ko's parameter blocks, the sensor's register runs, the
  channel's buffers, and the steps libxmv200 does itself between them
  (XMV200_OP_*: those depend on the board's layout);
- the mode's clocks: the ISP PLL programs, as written;
- the MIPI receiver's setup;
- isp.ko's structural payloads (picture, channel, window), which
  libxmv200 fills in for each channel;
- the shared parameter page image and the lens-correction table.

FAMILY is the chip family the capture ran on: xm530v200, xm535 (XM535D1/D2),
xm535w1 or xm533. Without it, xm535 if the PLL went through sys.ko, else
xm530v200. Families differ in where VI's blocks are, and in what libxmv200
fills in (its chip table, chip.c).

Block payloads are channel 0's. libxmv200 fills in the channel index
(word 0) and the addresses of the buffers it allocates itself. Those are
found by value: a payload word that points into one of channel 0's
buffers, as (phys - 0x80000000) >> 4 or as phys, becomes a patch with its
offset in the buffer (where channel 1 sent the command too, its payload
has to agree)."""
import re
import struct
import sys

trace, page_bin, ldc_bin, name = sys.argv[1:5]
family = sys.argv[5] if len(sys.argv) > 5 else None

# where each family's VI blocks are, from VI's base (0x30000000): the MIPI
# receiver and the first VICU
VI_LAYOUT = {'xm530v200': (0x4000, 0x40000), 'xm535': (0x4000, 0x40000),
             'xm535w1': (0x4000, 0x40000), 'xm533': (0x0000, 0x10000)}
# isp.ko commands libxmv200 issues from its own code
NAMED = {0x49e9: 'XMV200_OP_LDC', 0x4908: 'XMV200_OP_CHN_ATTR',
         0x4964: 'XMV200_OP_AE_INIT', 0x4965: None,
         0x490e: 'XMV200_OP_CHN_INIT', 0x4980: 'XMV200_OP_CHN_SIZE',
         0x495e: 'XMV200_OP_VICU_RUN'}
# the program starts with channel 0's first command of its own
START = (0x4913, 0x4974)


def chn_buf(name):
    # an ISP channel buffer (isp_nr_alpha_ch0, bpu_defog_ch1, ...): its name
    # without the channel, and the channel
    m = re.match(r'^(\w+_ch)(\d+)$', name)
    return (m.group(1), int(m.group(2))) if m else None


def payload(f):
    return bytes.fromhex(f[5]) if len(f) > 5 and not f[5].startswith('<') else b''


def isp_process(f):
    # the vendor isp process: the only user of /dev/isp, and of the
    # channel buffers
    if f[2] == 'isp':
        return True
    if f[2] == 'mmz' and payload(f):
        return chn_buf(payload(f)[:32].split(b'\0')[0].decode()) is not None
    return False


def arr(cname, data):
    out = 'static const uint8_t %s[%d] = {\n' % (cname, len(data))
    for i in range(0, len(data), 12):
        out += '    ' + ', '.join('0x%02x' % b for b in data[i:i + 12]) + ',\n'
    return out + '};\n'


lines = [ln.split() for ln in open(trace)]

if family is None:
    family = 'xm530v200'
    for f in lines:
        if f and f[0] == 'IOCTL' and f[2] == 'sys' and f[3] == '40085301' \
                and struct.unpack_from('<I', payload(f))[0] == 0x10:
            family = 'xm535'
            break
RX, VICU = VI_LAYOUT[family]
REG_OP = {RX: 'XMV200_OP_MIPI_CTRL', RX + 0x30c: 'XMV200_OP_VI_SYNC',
          VICU + 0x98: 'XMV200_OP_VI_WINDOW'}

# MMZ blocks by name, from the returned records
mmz, mmz_size = {}, {}
for f in lines:
    if len(f) > 4 and f[0] == 'R' and f[1] == 'mmz':
        d = bytes.fromhex(f[4])
        size, phys = struct.unpack_from('<II', d, 32)
        mmz[d[:32].split(b'\0')[0].decode()] = phys
        mmz_size[d[:32].split(b'\0')[0].decode()] = size

# the mode's structural payloads and the receiver: what the isp process
# sets for channel 0 up to its capture start
mode_w = {}
sysw, mipi = [], []
mipi_on = None  # None: before the receiver table, True: in it, False: after
for f in lines:
    if f and f[0] == 'IOCTL':
        cmd, d = int(f[3], 16), payload(f)
        if f[2] == 'sys' and cmd in (0x40085301, 0x40085302):
            # the clocks through sys.ko, {register, value} (get: an end)
            sysw.append(struct.unpack_from('<II', d) + ('sys' if cmd & 1 else 'get',))
            continue
        if f[2] != 'isp':
            continue
        if d[:1] not in (b'\0', b'') and cmd not in (0x4962,):
            continue
        w = [struct.unpack_from('<I', d, i)[0] for i in range(0, len(d) - 3, 4)]
        key = {0x4962: 'comm', 0x490e: 'chn_init', 0x4908: 'chn_attr',
               0x4964: 'dgain', 0x4965: 'gain', 0x49e9: 'ldc', 0x4980: 'size'}.get(cmd)
        if key and key not in mode_w:
            mode_w[key] = w
        if cmd == 0x495e:
            break
    elif len(f) == 5 and f[1] == 'U':
        dev, off, val = f[2], int(f[3], 16), int(f[4], 16)
        if dev == 'sysctl':
            sysw.append((off, val, 'mem'))
            continue
        if dev != 'isp2.vi':
            continue
        # the receiver table: its writes in order, up to the first VI write
        # outside the receiver
        rx = RX <= off < RX + 0x400
        if mipi_on is None and rx:
            mipi_on = True
        elif mipi_on and not rx:
            mipi_on = False
        if mipi_on:
            mipi.append((0x30000000 + off, val))
        if off == RX and 'mipi_ctrl' not in mode_w:
            mode_w['mipi_ctrl'] = [val]
        elif off == RX + 0x30c and mipi_on is False and 'vi_sync' not in mode_w:
            mode_w['vi_sync'] = [val]

# the ISP PLL programs, as written: through /dev/mem, the writes to 0x10..0x54
# between unlocking the clock block (0x0 = 1) and locking it again, where
# they include 0x10; through sys.ko, the writes from 0x10 up to reading it
pll, cur = [], None
for off, val, how in sysw:
    if how == 'mem' and off == 0x0:
        if cur and any(r == 0x10 for r, v in cur):
            pll.append(cur)
        cur = [] if val == 1 else None
    elif how == 'mem' and cur is not None and 0x10 <= off <= 0x54:
        cur.append((off, val))
    elif how == 'mem' and off in (0x74, 0x78):
        mode_w.setdefault('sns_clk', {})[off] = val
    elif how == 'sys' and off == 0x10:
        cur = [(off, val)]
        pll.append(cur)
    elif how == 'sys' and cur is not None and 0x14 <= off <= 0x54:
        cur.append((off, val))
    elif how in ('sys', 'get'):
        cur = None
assert len(pll) >= 2, 'no PLL programs'

# channel 0's buffers, in the order the isp process allocates them (the
# lens table is the LDC step's own)
bufs = []
for f in lines:
    if f and f[0] == 'IOCTL' and f[2] == 'mmz' and isp_process(f):
        n = payload(f)[:32].split(b'\0')[0].decode()
        if chn_buf(n)[1] == 0 and n != 'isp_ldc_ch0' and n not in bufs:
            bufs.append(n)

# the first payload of each command per channel, to tell address fields
# from values that only look like one
first = {}
for f in lines:
    if f and f[0] == 'IOCTL' and f[2] == 'isp' and payload(f):
        first.setdefault((int(f[3], 16), payload(f)[0]), payload(f))


def find_patches(cmd, d):
    # payload words pointing into channel 0's buffers, or just past one's
    # end (where no other starts), checked against channel 1's payload of
    # the command where there is one
    def owner(a, chn):
        names = [chn_buf(n)[0] + str(chn) for n in bufs]
        for i, n in enumerate(names):
            if n in mmz and mmz[n] <= a < mmz[n] + mmz_size[n]:
                return i
        for i, n in enumerate(names):
            if n in mmz and a == mmz[n] + mmz_size[n]:
                return i
        return None

    def addr(w, sh):
        if sh and not 0x100000 <= w < 0x8000000:
            return None
        return (w << 4) + 0x80000000 if sh else w

    d1 = first.get((cmd, 1))
    out = []
    for off in range(4, len(d) - 3, 4):
        w = struct.unpack_from('<I', d, off)[0]
        for sh in (4, 0):
            a = addr(w, sh)
            i = owner(a, 0) if a is not None else None
            if i is None:
                continue
            rel = a - mmz[bufs[i]]
            if d1 is not None and len(d1) == len(d):
                w1 = struct.unpack_from('<I', d1, off)[0]
                a1 = addr(w1, sh)
                n1 = chn_buf(bufs[i])[0] + '1'
                # a constant is the same for both channels; an address isn't
                # (and where channel 1 got that buffer, points as far in it)
                if w1 == w or (mmz.get(n1) and (a1 is None or a1 - mmz[n1] != rel)):
                    continue
            out.append((off, i, sh, rel))
            break
    return out


# channel 0's program
prog, blocks, data, runs = [], [], [], []
on, run, sensor = False, None, None
for f in lines:
    if not f:
        continue
    if f[0] == 'IOCTL':
        if not isp_process(f):
            continue
        dev, cmd, d = f[2], int(f[3], 16), payload(f)
        if dev == 'isp' and cmd in START and d[:1] == b'\0':
            on = True
        if not on:
            continue
        run = None
        if dev == 'mmz':
            n = d[:32].split(b'\0')[0].decode()
            if n in bufs:
                prog.append('XMV200_OP_ALLOC(%d)' % bufs.index(n))
            continue
        if dev != 'isp' or d[:1] not in (b'\0', b''):
            continue
        if cmd == 0x4914:
            prog.append('XMV200_OP_CHN_ENABLE(%d)' % struct.unpack_from('<I', d, 4)[0])
            continue
        if cmd in NAMED:
            if NAMED[cmd]:
                prog.append(NAMED[cmd])
            if cmd == 0x495e:
                break
            continue
        cn = 'p%03d_%04x' % (len(blocks), cmd)
        data.append(arr(cn, d))
        patches = ['{0x%x, %d, %d, 0x%x}' % (off, i + 1, sh, rel)
                   for off, i, sh, rel in find_patches(cmd, d)]
        assert len(patches) <= 16, (hex(cmd), patches)
        prog.append('XMV200_OP_BLOCK(%d)' % len(blocks))
        blocks.append('    {0x%04x, %s, sizeof(%s),\n     {%s}},' %
                      (cmd, cn, cn, ', '.join(patches) or '{0}'))
    elif on and len(f) == 5 and f[1] == 'U':
        o_, v_ = int(f[3], 16), int(f[4], 16)
        op = REG_OP.get(o_) if f[2] == 'isp2.vi' else None
        if f[2] == 'isp2.vi' and o_ == VICU + 0x7050:
            op = 'XMV200_OP_VICU_IRQ(%d)' % v_
        if op and (not prog or prog[-1] != op):
            prog.append(op)
        run = None
    elif on and len(f) == 5 and f[2] == 'sensor':
        # channel 0's sensor: the first one the program writes
        a = int(f[3], 16)
        sensor = sensor or a >> 16
        if a >> 16 != sensor:
            continue
        if run is None:
            run = []
            runs.append(run)
            prog.append('RUN%d' % (len(runs) - 1))
        run.append((a & 0xffff, int(f[4], 16)))

# sensor runs: the board's mirror/flip alone is its own step
rmap, sruns = {}, []
for i, r in enumerate(runs):
    if all(a == 0x3221 for a, v in r):
        rmap['RUN%d' % i] = 'XMV200_OP_SNS_FLIP'
    else:
        rmap['RUN%d' % i] = 'XMV200_OP_SNS_RUN(%d)' % len(sruns)
        sruns.append(r)
prog = [rmap.get(p, p) for p in prog]
# the exposure the program leaves: 0x3e00..0x3e02 hold lines << 4
expo = {}
for r in sruns:
    for a, v in r:
        if 0x3e00 <= a <= 0x3e02:
            expo[a] = v
exp_lines = (expo.get(0x3e00, 0) << 16 | expo.get(0x3e01, 0) << 8 | expo.get(0x3e02, 0)) >> 4
# Sony IMX335: the frame length (VMAX, 0x3030..) less the shutter (SHR0,
# 0x3058..), both in quarter lines
sony = {}
for r in sruns:
    for a, v in r:
        if 0x3030 <= a <= 0x3032 or 0x3058 <= a <= 0x305a:
            sony[a] = v
if not expo and 0x3058 in sony:
    vmax = sony.get(0x3032, 0) << 16 | sony.get(0x3031, 0) << 8 | sony.get(0x3030, 0)
    shr = sony.get(0x305a, 0) << 16 | sony.get(0x3059, 0) << 8 | sony[0x3058]
    exp_lines = (vmax - shr) // 4

o = ['/* SPDX-License-Identifier: MIT */',
     '/* %s tuning for libxmv200, generated by tools/xmv200-capture/gen_tuning.py' % name.upper(),
     ' * from a reference capture of the vendor stack. Do not edit. */',
     '#include "sns_tuning.h"', '']
for i, r in enumerate(sruns):
    o.append('static const struct xmv200_sns_reg run%d[] = {' % i)
    o += ['    {0x%04x, 0x%02x},' % x for x in r]
    o += ['};', '']
o.append('static const struct xmv200_sns_run runs[] = {')
o += ['    {run%d, sizeof(run%d) / sizeof(run%d[0])},' % (i, i, i) for i in range(len(sruns))]
o += ['};', '']
o += data
o.append('static const struct xmv200_isp_block blocks[] = {')
o += blocks
o += ['};', '', 'static const uint16_t program[] = {']
o += ['    %s,' % p for p in prog]
o += ['};', '']
o.append(arr('page', open(page_bin, 'rb').read()))
ldc_data = open(ldc_bin, 'rb').read()
o.append(arr('ldc', ldc_data or b'\0'))


def words(k, n):
    w = (mode_w.get(k) or [0])[:n]
    return '{' + ', '.join('0x%x' % x for x in w) + '}'


o.append('static const struct xmv200_reg mipi[] = {')
o += ['    {0x%08x, 0x%x},' % r for r in mipi]
o += ['};', '']
o.append('static const struct xmv200_isp_buf bufs[] = {')
o += ['    {"%s%%u", 0x%x},' % (chn_buf(n)[0], mmz_size[n]) for n in bufs]
o += ['};', '']
for i in (0, 1):
    p = pll[0] if i == 0 else pll[-1]
    o.append('static const struct xmv200_reg pll%d[] = {' % i)
    o += ['    {0x%x, 0x%x},' % r for r in p]
    o += ['};', '']
sc = mode_w.get('sns_clk', {})
sz = mode_w['size']
# 0x4980: {chn, x0 | y0 << 16, w | h << 16}, or the longer form with the
# window at words 4..7 (XM535)
w_, h_ = (sz[2] & 0xffff, sz[2] >> 16) if len(sz) == 3 else (sz[6], sz[7])
o += ['const struct xmv200_sns_tuning xmv200_tuning_%s = {' % name,
      '    .chip = "%s",' % family,
      '    .width = %d,' % w_,
      '    .height = %d,' % h_,
      '    .sns_clk = {0x%x, 0x%x},' % (sc.get(0x74, 0), sc.get(0x78, 0)),
      '    .pll = {pll0, pll1},',
      '    .n_pll = {sizeof(pll0) / sizeof(pll0[0]), sizeof(pll1) / sizeof(pll1[0])},',
      '    .mipi = mipi,',
      '    .n_mipi = sizeof(mipi) / sizeof(mipi[0]),',
      '    .mipi_ctrl = 0x%x,' % (mode_w.get('mipi_ctrl') or [0])[0],
      '    .vi_sync = 0x%x,' % (mode_w.get('vi_sync') or [0])[0],
      '    .comm = %s,' % words('comm', 14),
      '    .chn_init = %s,' % words('chn_init', 20),
      '    .n_chn_init = %d,' % len(mode_w['chn_init']),
      '    .chn_attr = %s,' % words('chn_attr', 7),
      '    .chn_size = %s,' % words('size', 12),
      '    .n_chn_size = %d,' % len(sz),
      '    .dgain = 0x%x,' % (mode_w.get('dgain') or [0, 0x100])[1],
      '    .gain = %s,' % words('gain', 4),
      '    .ldc = %s,' % words('ldc', 12),
      '    .bufs = bufs,',
      '    .n_bufs = sizeof(bufs) / sizeof(bufs[0]),',
      '    .runs = runs,',
      '    .n_runs = sizeof(runs) / sizeof(runs[0]),',
      '    .exp_lines = %d,' % exp_lines,
      '    .program = program,',
      '    .n_program = sizeof(program) / sizeof(program[0]),',
      '    .blocks = blocks,',
      '    .page = page,',
      '    .page_size = sizeof(page),',
      '    .ldc_table = ldc,',
      '    .ldc_size = %d,' % len(ldc_data),
      '};']
print('\n'.join(o))
