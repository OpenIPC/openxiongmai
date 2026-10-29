#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""gen_tuning.py MERGED_TRACE PAGE_BIN LDC_BIN NAME > sns_NAME_tuning.c

Turns a reference capture of the vendor stack (see README.md) into
libxmv200's tuning for one sensor mode:

- the sensor's init and mode registers (sensor 0x30's writes);
- channel 0's ISP program: the parameter blocks isp.ko takes, in order, and
  the steps libxmv200 does itself between them (XMV200_OP_*);
- the shared parameter page image and the lens-correction table.

Block payloads are channel 0's. libxmv200 fills in the channel index
(word 0) and the addresses of the buffers it allocates itself. Those are
found by value: a payload word that points into one of channel 0's
buffers, as (phys - 0x80000000) >> 4 or as phys, becomes a patch with its
offset in the buffer.

The same capture works for each V200-series chip (XM530V200, XM535...):
what differs between them (struct sizes, clocks through sys.ko) stays in
the payloads, and libxmv200's chip table knows how to fill them in."""
import re
import struct
import sys


def isp_process(f):
    # the vendor isp process: the only user of /dev/isp, and of the isp_*
    # MMZ blocks
    if f[2] == 'isp':
        return True
    if f[2] == 'mmz' and len(f) > 5 and not f[5].startswith('<'):
        return chn_buf(bytes.fromhex(f[5])[:32].split(b'\0')[0].decode()) is not None
    return False


def chn_buf(name):
    # an ISP channel buffer (isp_nr_alpha_ch0, bpu_defog_ch1, ...): its name
    # without the channel, and the channel
    m = re.match(r'^(\w+_ch)(\d+)$', name)
    return (m.group(1), int(m.group(2))) if m else None

trace, page_bin, ldc_bin, name = sys.argv[1:5]

# isp.ko commands libxmv200 issues from its own code
NAMED = {0x49e9: 'XMV200_OP_LDC', 0x4908: 'XMV200_OP_CHN_ATTR',
         0x4964: 'XMV200_OP_AE_INIT', 0x4965: None,
         0x490e: 'XMV200_OP_CHN_INIT', 0x4980: 'XMV200_OP_CHN_SIZE',
         0x495e: 'XMV200_OP_VICU_RUN'}
SNS_OP = {0x0100: 'XMV200_OP_SNS_INIT', 0x301f: 'XMV200_OP_SNS_MODE',
          0x3e02: 'XMV200_OP_SNS_EXPOSURE', 0x3221: 'XMV200_OP_SNS_FLIP'}
REG_OP = {0x4000: 'XMV200_OP_MIPI_CTRL', 0x430c: 'XMV200_OP_VI_SYNC',
          0x40098: 'XMV200_OP_VI_WINDOW'}
# the program starts with channel 0's first command of its own
START = (0x4913, 0x4974)


def arr(cname, data):
    out = 'static const uint8_t %s[%d] = {\n' % (cname, len(data))
    for i in range(0, len(data), 12):
        out += '    ' + ', '.join('0x%02x' % b for b in data[i:i + 12]) + ',\n'
    return out + '};\n'


lines = [ln.split() for ln in open(trace)]

# MMZ blocks by name, from the returned records
mmz, mmz_size = {}, {}
for f in lines:
    if len(f) > 4 and f[0] == 'R' and f[1] == 'mmz':
        d = bytes.fromhex(f[4])
        size, phys = struct.unpack_from('<II', d, 32)
        mmz[d[:32].split(b'\0')[0].decode()] = phys
        mmz_size[d[:32].split(b'\0')[0].decode()] = size

# sensor 0x30: the init table (after the standby write) and the mode
# registers (from 0x301f up to the first exposure write)
init, mode, expo, phase = [], [], [], None
for f in lines:
    if len(f) == 5 and f[2] == 'sensor' and int(f[3], 16) >> 16 == 0x30:
        a, v = int(f[3], 16) & 0xffff, int(f[4], 16)
        if a == 0x0100 and v == 0 and phase is None:
            phase = 'init'
        elif phase == 'init' and a == 0x301f:
            phase = 'mode'
        elif phase == 'mode' and a in (0x3e00, 0x3e01, 0x3e02):
            phase = 'expo'
        elif phase == 'expo' and len(expo) == 6:
            break
        if phase == 'init' and a != 0x0100:
            init.append((a, v))
        elif phase == 'mode':
            mode.append((a, v))
        if phase == 'expo':
            expo.append((a, v))

# the mode: everything the isp process sets that depends on the sensor mode
mode_w = {}
sysw, mipi, pll = [], [], []
mipi_on = None  # None: before the receiver table, True: in it, False: after
for f in lines:
    if f and f[0] == 'IOCTL':
        cmd = int(f[3], 16)
        d = bytes.fromhex(f[5]) if len(f) > 5 and not f[5].startswith('<') else b''
        if f[2] == 'sys' and cmd == 0x40085301:
            # XM535: the clocks through sys.ko, {register, value}
            sysw.append(struct.unpack_from('<II', d) + ('sys',))
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
        # outside it
        rx = 0x4000 <= off < 0x4400
        if mipi_on is None and rx:
            mipi_on = True
        elif mipi_on and not rx:
            mipi_on = False
        if mipi_on:
            mipi.append((0x30000000 + off, val))
        if off == 0x4000 and 'mipi_ctrl' not in mode_w:
            mode_w['mipi_ctrl'] = [val]
        elif off == 0x430c and mipi_on is False and 'vi_sync' not in mode_w:
            mode_w['vi_sync'] = [val]
# the ISP PLL programs: 0x14 (XM530V200, /dev/mem after an unlock) or 0x10
# (XM535, sys.ko), then 0x44, 0x48, 0x50, 0x54
cur, pll_regs = None, (0x14, 0x44, 0x48, 0x50, 0x54)
for off, val, how in sysw:
    if (off, how) in ((0x14, 'mem'), (0x10, 'sys')):
        cur = {}
        pll.append(cur)
        pll_regs = (off, 0x44, 0x48, 0x50, 0x54)
    if cur is not None and off in pll_regs:
        cur[off] = val
    if off in (0x74, 0x78) and how == 'mem':
        mode_w.setdefault('sns_clk', {})[off] = val

# channel 0's buffers, in the order the isp process allocates them (the
# lens table is the LDC step's own)
bufs = []
for f in lines:
    if f and f[0] == 'IOCTL' and f[2] == 'mmz' and isp_process(f):
        n = bytes.fromhex(f[5])[:32].split(b'\0')[0].decode()
        if chn_buf(n)[1] == 0 and n != 'isp_ldc_ch0' and n not in bufs:
            bufs.append(n)


# the first payload of each command per channel, to tell address fields
# from values that only look like one
first = {}
for f in lines:
    if f and f[0] == 'IOCTL' and f[2] == 'isp' and len(f) > 5 and not f[5].startswith('<'):
        d = bytes.fromhex(f[5])
        first.setdefault((int(f[3], 16), d[0]), d)


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
prog, blocks, data = [], [], []
on, in_sens = False, False
for f in lines:
    if not f:
        continue
    if f[0] == 'IOCTL':
        if not isp_process(f):
            continue
        dev, cmd = f[2], int(f[3], 16)
        d = bytes.fromhex(f[5]) if len(f) > 5 and not f[5].startswith('<') else b''
        if dev == 'isp' and cmd in START and d[:1] == b'\0':
            on = True
        if not on:
            continue
        in_sens = False
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
        op = REG_OP.get(o_)
        if o_ == 0x47050:
            op = 'XMV200_OP_VICU_IRQ(%d)' % v_
        if op and (not prog or prog[-1] != op):
            prog.append(op)
        in_sens = False
    elif on and len(f) == 5 and f[2] == 'sensor':
        if not in_sens:
            a = int(f[3], 16) & 0xffff
            prog.append(SNS_OP.get(a, 'XMV200_OP_NOP'))
            in_sens = True

o = ['/* SPDX-License-Identifier: MIT */',
     '/* %s tuning for libxmv200, generated by tools/xmv200-capture/gen_tuning.py' % name.upper(),
     ' * from a reference capture of the vendor stack. Do not edit. */',
     '#include "sns_tuning.h"', '',
     'static const struct xmv200_sns_reg init_regs[] = {']
o += ['    {0x%04x, 0x%02x},' % r for r in init]
o += ['};', '', 'static const struct xmv200_sns_reg mode_regs[] = {']
o += ['    {0x%04x, 0x%02x},' % r for r in mode]
o += ['};', '', 'static const struct xmv200_sns_reg exposure_regs[] = {']
o += ['    {0x%04x, 0x%02x},' % r for r in expo]
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
pl = lambda p: '{' + ', '.join('0x%x' % p.get(k, 0) for k in pll_regs) + '}'
sc = mode_w.get('sns_clk', {})
sz = mode_w['size']
# 0x4980: {chn, x0 | y0 << 16, w | h << 16} (XM530V200) or the longer
# form with the window at words 4..7 (XM535)
w_, h_ = (sz[2] & 0xffff, sz[2] >> 16) if len(sz) == 3 else (sz[6], sz[7])
o += ['const struct xmv200_sns_tuning xmv200_tuning_%s = {' % name,
      '    .chip = "%s",' % ('xm535' if pll_regs[0] == 0x10 else 'xm530v200'),
      '    .width = %d,' % w_,
      '    .height = %d,' % h_,
      '    .sns_clk = {0x%x, 0x%x},' % (sc.get(0x74, 0), sc.get(0x78, 0)),
      '    .pll = {%s, %s},' % (pl(pll[0]), pl(pll[-1])),
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
      '    .init_regs = init_regs,',
      '    .n_init_regs = sizeof(init_regs) / sizeof(init_regs[0]),',
      '    .mode_regs = mode_regs,',
      '    .exposure_regs = exposure_regs,',
      '    .n_exposure_regs = sizeof(exposure_regs) / sizeof(exposure_regs[0]),',
      '    .n_mode_regs = sizeof(mode_regs) / sizeof(mode_regs[0]),',
      '    .program = program,',
      '    .n_program = sizeof(program) / sizeof(program[0]),',
      '    .blocks = blocks,',
      '    .page = page,',
      '    .page_size = sizeof(page),',
      '    .ldc_table = ldc,',
      '    .ldc_size = %d,' % len(ldc_data),
      '};']
print('\n'.join(o))
