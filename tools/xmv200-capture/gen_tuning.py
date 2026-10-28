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
(word 0) and the addresses of the buffers it allocates itself; the
address fields hold (phys - 0x80000000) >> 4 of a buffer plus an offset,
which is kept here."""
import struct
import sys


def isp_process(f):
    # the vendor isp process: the only user of /dev/isp, and of the isp_*
    # MMZ blocks
    if f[2] == 'isp':
        return True
    if f[2] == 'mmz' and len(f) > 5 and not f[5].startswith('<'):
        return bytes.fromhex(f[5])[:4] == b'isp_'
    return False


def mipi_done(m):
    # the receiver setup ends with its sync timeouts (0x430c, 0x4310)
    return any(a == 0x30004310 for a, v in m)

trace, page_bin, ldc_bin, name = sys.argv[1:5]

# isp.ko commands whose payloads hold buffer addresses: (offset, buffer)
ADDR = {
    0x49dc: [(0x14, 'nr_alpha'), (0x18, 'nr_alpha'), (0x2c, 'nr_idx'),
             (0x34, 'nr_idx'), (0x38, 'nr_idx_max'), (0x48, 'nr_fetch'),
             (0x54, 'nr_fetch'), (0x60, 'nr_fetch'), (0x6c, 'nr_fetch')],
    0x497f: [(0x14, 'defog')],
    0x49e3: [(0x14, 'defog')],
    0x49e8: [(0x1c, 'defog')],
}
BUFNAME = {'nr_alpha': 'isp_nr_alpha_ch0', 'nr_idx': 'isp_nr_idx_ch0',
           'nr_idx_max': 'isp_nr_idx_max_ch0', 'nr_fetch': 'isp_nr_fetch_ch0',
           'defog': 'isp_defog_ch0'}
# isp.ko commands libxmv200 issues from its own code
NAMED = {0x49e9: None, 0x4908: 'XMV200_OP_CHN_ATTR',
         0x4964: 'XMV200_OP_AE_INIT', 0x4965: None,
         0x490e: 'XMV200_OP_CHN_INIT', 0x4980: 'XMV200_OP_CHN_SIZE',
         0x495e: 'XMV200_OP_VICU_RUN'}
SNS_OP = {0x0100: 'XMV200_OP_SNS_INIT', 0x301f: 'XMV200_OP_SNS_MODE',
          0x3e02: 'XMV200_OP_SNS_EXPOSURE', 0x3221: 'XMV200_OP_SNS_FLIP'}
REG_OP = {0x4000: 'XMV200_OP_MIPI_CTRL', 0x430c: 'XMV200_OP_VI_SYNC',
          0x40098: 'XMV200_OP_VI_WINDOW'}
MMZ_OP = {'isp_nr_alpha_ch0': 'XMV200_OP_NR_ALLOC',
          'isp_defog_ch0': 'XMV200_OP_DEFOG_ALLOC',
          'isp_ldc_ch0': 'XMV200_OP_LDC'}


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
for f in lines:
    if f and f[0] == 'IOCTL':
        if f[2] != 'isp':
            continue
        cmd = int(f[3], 16)
        d = bytes.fromhex(f[5]) if len(f) > 5 and not f[5].startswith('<') else b''
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
            sysw.append((off, val))
        elif dev == 'isp2.vi' and 0x4000 < off < 0x4400 and not mipi_done(mipi):
            mipi.append((0x30000000 + off, val))
        elif dev == 'isp2.vi' and off == 0x4000 and 'mipi_ctrl' not in mode_w:
            mode_w['mipi_ctrl'] = [val]
        elif dev == 'isp2.vi' and off == 0x430c and mipi and 'vi_sync' not in mode_w and mipi_done(mipi):
            mode_w['vi_sync'] = [val]
# the ISP PLL programs: 0x14, 0x44, 0x48, 0x50, 0x54 after each unlock
cur = None
for off, val in sysw:
    if off == 0x14:
        cur = {}
        pll.append(cur)
    if cur is not None and off in (0x14, 0x44, 0x48, 0x50, 0x54):
        cur[off] = val
    if off in (0x74, 0x78):
        mode_w.setdefault('sns_clk', {})[off] = val

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
        if dev == 'isp' and cmd == 0x4913 and d[:1] == b'\0':
            on = True
        if not on:
            continue
        in_sens = False
        if dev == 'mmz':
            op = MMZ_OP.get(d[:32].split(b'\0')[0].decode())
            if op:
                prog.append(op)
            continue
        if dev != 'isp' or d[:1] not in (b'\0', b''):
            continue
        if cmd in NAMED:
            if NAMED[cmd]:
                prog.append(NAMED[cmd])
            if cmd == 0x495e:
                break
            continue
        cn = 'p%03d_%04x' % (len(blocks), cmd)
        data.append(arr(cn, d))
        patches = []
        for off, buf in ADDR.get(cmd, []):
            w = struct.unpack_from('<I', d, off)[0]
            rel = (w << 4) + 0x80000000 - mmz[BUFNAME[buf]]
            patches.append('{0x%x, XMV200_BUF_%s, 0x%x}' % (off, buf.upper(), rel))
        patches += ['{0}'] * (9 - len(patches))
        prog.append('XMV200_OP_BLOCK(%d)' % len(blocks))
        blocks.append('    {0x%04x, %s, sizeof(%s),\n     {%s}},' %
                      (cmd, cn, cn, ', '.join(patches)))
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
o.append(arr('ldc', open(ldc_bin, 'rb').read()))
def words(k, n):
    w = (mode_w.get(k) or [0])[:n]
    return '{' + ', '.join('0x%x' % x for x in w) + '}'

o.append('static const struct xmv200_reg mipi[] = {')
o += ['    {0x%08x, 0x%x},' % r for r in mipi]
o += ['};', '']
pl = lambda p: '{0x%x, 0x%x, 0x%x, 0x%x, 0x%x}' % tuple(p.get(k, 0) for k in (0x14, 0x44, 0x48, 0x50, 0x54))
sc = mode_w.get('sns_clk', {})
sz = mode_w['size']
o += ['const struct xmv200_sns_tuning xmv200_tuning_%s = {' % name,
      '    .width = %d,' % (sz[2] & 0xffff),
      '    .height = %d,' % (sz[2] >> 16),
      '    .sns_clk = {0x%x, 0x%x},' % (sc.get(0x74, 0), sc.get(0x78, 0)),
      '    .pll = {%s, %s},' % (pl(pll[0]), pl(pll[-1])),
      '    .mipi = mipi,',
      '    .n_mipi = sizeof(mipi) / sizeof(mipi[0]),',
      '    .mipi_ctrl = 0x%x,' % (mode_w.get('mipi_ctrl') or [0])[0],
      '    .vi_sync = 0x%x,' % (mode_w.get('vi_sync') or [0])[0],
      '    .comm = %s,' % words('comm', 14),
      '    .chn_init = %s,' % words('chn_init', 13),
      '    .chn_attr = %s,' % words('chn_attr', 7),
      '    .dgain = 0x%x,' % (mode_w.get('dgain') or [0, 0x100])[1],
      '    .gain = %s,' % words('gain', 4),
      '    .ldc = %s,' % words('ldc', 12),
      '    .buf_size = {0, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x},' % tuple(
          mmz_size[n] for n in ('isp_nr_alpha_ch0', 'isp_nr_idx_ch0', 'isp_nr_idx_max_ch0',
                                'isp_nr_fetch_ch0', 'isp_defog_ch0')),
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
      '    .ldc_size = sizeof(ldc),',
      '};']
print('\n'.join(o))
