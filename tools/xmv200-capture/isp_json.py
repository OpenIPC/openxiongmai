#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""isp_json.py IN.bin OUT.bin KEY=VALUE|-KEY|camera:TEXT ...

A stock firmware image (8 MiB, its custom partition a cramfs at 0x770000)
with CustomConfig/Isp.json changed, to make the vendor's ISP run another
sensor mode under qemu-bench for a reference capture (capture.sh): no
"MipiAttr" for 2-lane MIPI, "Assign2336p" off for the real SC2337P,
"FunExt": 1 on the XM535D1 for its sensors without frame sync, "HDR": 2
for a sensor's HDR mode where the vendor's daemon has one for it (SC4210
always, SC223A; not the XM530V200's SC200AI). VALUE is
JSON ("1LANE_10BIT" quoted, 1 a number); -KEY drops a key; camera:TEXT
puts TEXT into Camera.custom's first Param. libxmv200 reads none of
these files: the streamer chooses the mode (xmv200_layout_sensor).
"""
import os,re,subprocess,sys,tempfile,shutil
src,dst=sys.argv[1],sys.argv[2]
OFF,SIZE=0x770000,0x40000
d=bytearray(open(src,'rb').read())
t=tempfile.mkdtemp()
open(t+'/c','wb').write(d[OFF:OFF+SIZE])
subprocess.run(['fsck.cramfs','--extract='+t+'/x',t+'/c'],check=True,capture_output=True)
p=t+'/x/CustomConfig/Isp.json'; s=open(p).read()
for kv in sys.argv[3:]:
    if kv.startswith('camera:'):
        # text into Camera.custom's first Param
        cp=t+'/x/CustomConfig/Camera.custom'; c=open(cp).read()
        c=re.sub(r'("Param"\s*:\s*\[\s*\{)',lambda m:m.group(1)+'\n'+kv[7:]+',',c,count=1)
        open(cp,'w').write(c); print(c.replace('\t','').replace('\n',' ')); continue
    if kv.startswith('-'):
        k=kv[1:]
        s2=re.sub(r'\n[ \t]*"%s"\s*:\s*("[^"]*"|[-0-9x]+|\[[^\]]*\]|\{[^}]*\})\s*,?'%re.escape(k),'',s)
        assert s2!=s,'no key '+k; s=s2; continue
    k,v=kv.split('=',1)
    if re.search(r'"%s"\s*:'%re.escape(k),s):
        s=re.sub(r'("%s"\s*:\s*)("[^"]*"|[-0-9x]+)'%re.escape(k),lambda m:m.group(1)+v,s)
    else:
        s=re.sub(r'\{','{\n\t"%s": %s,'%(k,v),s,count=1)
# a trailing comma before } is what the vendor's parser takes too, but tidy
open(p,'w').write(s)
subprocess.run(['mkfs.cramfs',t+'/x',t+'/n'],check=True,capture_output=True)
n=open(t+'/n','rb').read(); assert len(n)<=SIZE,len(n)
d[OFF:OFF+SIZE]=n+b'\xff'*(SIZE-len(n))
open(dst,'wb').write(d); print(s.replace('\t','').replace('\n',' ')); shutil.rmtree(t)
