# SPDX-License-Identifier: MIT
# gdb: merged capture (IOCTL / R lines appended to the XM_REGTRACE file),
# shared page + LDC snapshot at the last vicu_run, then stop.
import gdb,struct,os
OUT=os.environ['CAP_OUT']; NCH=int(os.environ.get('CAP_NCH','2'))
inf=gdb.selected_inferior()
H={0xbf025000+0x594:'isp',0xbf040000+0x2860:'vpss',0xbf00a000+0x814:'vb',0xbf0c2000+0x500:'hevc',
   0xbf005000+0xb44:'mmz',0xbf00f000+0x94:'sys',0xbf0bb000+0x2fc:'venc'}
# the isp.ko payload sizes; at least what isp.ko copies in (0x49e4 0x164 and
# 0x49e8 0xa0, drivers/xm530v200_isp); larger only reads more of the caller's
# stack
ISZ={0x495b:0x24,0x4913:8,0x490a:0xc,0x4908:0x1c,0x4909:4,0x490e:0x34,0x4912:0x1c,0x490b:4,0x4950:0x18,
 0x4932:4,0x4933:4,0x4959:4,0x4962:0x38,0x495e:4,0x495c:0x3c,0x495d:0x28,0x4963:4,0x4980:0xc,0x4961:0x10,
 0x4964:8,0x4965:0x10,0x4966:0x34,0x4967:0x44,0x4968:0x44,0x4969:200,0x496a:0x14,0x496b:0x84,0x496c:0xd0,
 0x496d:0xd0,0x496e:0x214,0x496f:0x6c,0x4970:0xa4,0x4971:0x20c,0x4973:0x28,0x4974:0x10,0x4975:0x20,0x4976:0x20,
 0x4977:0x5c,0x497f:0x18,0x49d2:0x40,0x49d7:0x28,0x49db:0x38,0x49dc:0x13c,0x49de:0x10,0x49df:0x88,0x49e0:0x18,0x49e2:0x18,
 0x49e3:0x164,0x49e4:0x164,0x49e5:0x1d8,0x49e6:0x1d8,0x49e7:0xa0,0x49e8:0xa0,0x49e9:0x30,0x49f0:0x184}
# another chip's modules: their ioctl handlers ({addr: dev}) and the
# commands' sizes (merged into the XM530V200's, the larger kept)
if os.environ.get('CAP_H'):
    H=eval(os.environ['CAP_H'])
for k,v in eval(os.environ.get('CAP_ISZ','{}')).items():
    ISZ[k]=max(ISZ.get(k,0),v)
out=open(os.environ['XM_REGTRACE_FILE'],'a',buffering=1)
for a in H: gdb.execute('hbreak *%#x'%a,to_string=True)
def rd(a,n):
    try: return bytes(inf.read_memory(a,n))
    except Exception: return None
def physread(ph,sz):
    gdb.execute('maint packet Qqemu.PhyMemMode:1',to_string=True)
    try: return bytes(inf.read_memory(ph,sz))
    finally: gdb.execute('maint packet Qqemu.PhyMemMode:0',to_string=True)
pend={}; cnt={}; blocks={}; runs=0; n=0; pages=8; stop_at=None
# the isp binary's pointer to its mapping of the shared pages
PAGEPTR=int(os.environ.get('CAP_PAGEPTR','0xadbb4'),0)
while True:
    try: gdb.execute('continue',to_string=True)
    except Exception as e: out.write('END %s\n'%e); break
    pc=int(gdb.parse_and_eval('$pc'))&0xffffffff; sp=int(gdb.parse_and_eval('$sp'))&0xffffffff
    if sp in pend and pend[sp][0]==pc:
        lr,arg,bp=pend.pop(sp); bp.delete(); d=rd(arg,44)
        if d:
            out.write('R mmz c02c4d01 ret=0 %s\n'%d.hex()); nm=d[:32].split(b'\0')[0].decode(errors='replace')
            blocks[nm]=struct.unpack_from('<II',d,32)
        continue
    dev=H.get(pc)
    if not dev: continue
    if stop_at is not None and n>=stop_at: break
    cmd=int(gdb.parse_and_eval('$r1'))&0xffffffff; arg=int(gdb.parse_and_eval('$r2'))&0xffffffff
    k=(dev,cmd); c=cnt.get(k,0); cnt[k]=c+1; n+=1
    if c<60:
        sz=0x600 if (dev,cmd)==('sys',0x400c5300) else (cmd>>16)&0x3fff or (ISZ.get(cmd,0x40) if dev=='isp' else 0x40)
        d=rd(arg,sz) if 0x1000<arg<0xc0000000 else b''
        out.write('IOCTL %d %s %08x %08x %s\n'%(n,dev,cmd,arg,d.hex() if d is not None else '<unreadable>'))
    if dev=='mmz' and cmd in (0xc02c4d01,0xc02c4d04):
        lr=int(gdb.parse_and_eval('$lr'))&0xffffffff
        pend[sp]=(lr,arg,gdb.Breakpoint('*%#x'%lr,type=gdb.BP_HARDWARE_BREAKPOINT,internal=True))
    if dev=='isp' and cmd==0x400c490c and d:
        pages=struct.unpack_from('<I',d,4)[0] or 8
    if dev=='isp' and cmd==0x495e:
        runs+=1
        if runs==NCH:
            pg=struct.unpack('<I',rd(PAGEPTR,4))[0]
            open(OUT+'/page.bin','wb').write(rd(pg,pages*4096))
            if 'isp_ldc_ch0' in blocks:
                sz,ph=blocks['isp_ldc_ch0']; open(OUT+'/ldc.bin','wb').write(physread(ph,sz))
            else: open(OUT+'/ldc.bin','wb').write(b'')
            out.write('SNAPSHOT\n')
            # CAP_CONTINUE=N: keep logging N more calls (the encoder's)
            stop_at=n+int(os.environ.get('CAP_CONTINUE','0'))
            if stop_at==n: break
gdb.execute('detach',to_string=True)
