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
mmz = {}
for f in lines:
    if len(f) > 4 and f[0] == 'R' and f[1] == 'mmz':
        d = bytes.fromhex(f[4])
        size, phys = struct.unpack_from('<II', d, 32)
        mmz[d[:32].split(b'\0')[0].decode()] = phys

# sensor 0x30: the init table (after the standby write) and the mode
# registers (from 0x301f up to the first exposure write)
init, mode, phase = [], [], None
for f in lines:
    if len(f) == 5 and f[2] == 'sensor' and int(f[3], 16) >> 16 == 0x30:
        a, v = int(f[3], 16) & 0xffff, int(f[4], 16)
        if a == 0x0100 and v == 0 and phase is None:
            phase = 'init'
        elif phase == 'init' and a == 0x301f:
            phase = 'mode'
        elif phase == 'mode' and a in (0x3e00, 0x3e01, 0x3e02):
            break
        if phase == 'init' and a != 0x0100:
            init.append((a, v))
        elif phase == 'mode':
            mode.append((a, v))

# channel 0's program
prog, blocks, data = [], [], []
on, in_sens = False, False
for f in lines:
    if not f:
        continue
    if f[0] == 'IOCTL':
        arg = int(f[4], 16)
        if not ((arg >> 20) == 0xbef or (arg >> 24) == 0xb6):
            continue        # not the vendor isp process
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
o += ['};', '']
o += data
o.append('static const struct xmv200_isp_block blocks[] = {')
o += blocks
o += ['};', '', 'static const uint16_t program[] = {']
o += ['    %s,' % p for p in prog]
o += ['};', '']
o.append(arr('page', open(page_bin, 'rb').read()))
o.append(arr('ldc', open(ldc_bin, 'rb').read()))
o += ['const struct xmv200_sns_tuning xmv200_tuning_%s = {' % name,
      '    .init_regs = init_regs,',
      '    .n_init_regs = sizeof(init_regs) / sizeof(init_regs[0]),',
      '    .mode_regs = mode_regs,',
      '    .n_mode_regs = sizeof(mode_regs) / sizeof(mode_regs[0]),',
      '    .program = program,',
      '    .n_program = sizeof(program) / sizeof(program[0]),',
      '    .blocks = blocks,',
      '    .page = page,',
      '    .page_size = sizeof(page),',
      '    .ldc = ldc,',
      '    .ldc_size = sizeof(ldc),',
      '};']
print('\n'.join(o))
