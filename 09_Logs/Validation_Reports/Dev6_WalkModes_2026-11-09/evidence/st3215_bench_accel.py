#!/usr/bin/env python3
"""ST3215 realized acceleration and braking from the bench QC V6.1 RAW captures
(walk modes 2026-11-09). Usage: st3215_bench_accel.py <dir with the *.bin of
the 2026-08-24/25 campaign> (schema: 05_Firmware/ST3215_Bench_Tools/Bench_QC_V6_1/RAW_SCHEMA.md).
Per move (constant goal tuple): PresentSpeed plateau, acceleration 10->80 %
of it, deceleration from the last 80 % sample to the first 10 % one, against
the commanded ACC x 100 step/s^2. No load (bench), bus 11.1 V."""
import struct,sys,glob,collections,statistics as st
def load(p):
    b=open(p,'rb').read();h=struct.unpack_from('<8s7I48s',b,0);n=h[5];out=[]
    for k in range(n):
        o=84+27*k;t,gp,gs,ga,ph,pr,fl=struct.unpack_from('<IHHBBBB',b,o);r=b[o+12:o+27]
        pos=r[0]|r[1]<<8;sp=r[2]|r[3]<<8;sp=-(sp&0x7fff) if sp&0x8000 else sp
        ld=r[4]|r[5]<<8;ld=-(ld&0x3ff) if ld&0x400 else ld
        cur=r[13]|r[14]<<8;cur=-(cur&0x7fff) if cur&0x8000 else cur
        out.append((t,gp,gs,ga,ph,pos,sp,ld,r[6],cur))
    return h,out
# For every move (constant goal tuple), measure: plateau speed, accel ramp (10->80% of plateau) from encoder-independent PresentSpeed,
# and decel at arrival, against the commanded ACC*100 tick/s^2.
res={}
for p in sorted(glob.glob(sys.argv[1]+'/*.bin')):
    name=p.split('/')[-1].split('__')[0]
    if name in('M13','M23') or 'ELR' in name and 'NORM' not in name: pass
    h,d=load(p);i=0;n=len(d)
    while i<n:
        j=i
        while j+1<n and d[j+1][1:5]==d[i][1:5]:j+=1
        seg=d[i:j+1];gp,gs,ga,ph=d[i][1],d[i][2],d[i][3],d[i][4]
        if ga not in(255,) and ga>=10 and len(seg)>60:
            sp=[abs(s[6]) for s in seg];t=[s[0]/1e6 for s in seg];peak=max(sp)
            k80=next(k for k,v in enumerate(sp) if v>=.8*peak)
            k10=next((k for k,v in enumerate(sp) if v>=.1*peak),0)
            if k80>k10+1 and peak>300:
                a=(sp[k80]-sp[k10])/(t[k80]-t[k10])
                # decel: last sample at >=80% peak to first <=10% after it
                kl=max(k for k,v in enumerate(sp) if v>=.8*peak)
                ke=next((k for k in range(kl,len(sp)) if sp[k]<=.1*peak),None)
                dec=(sp[kl]-sp[ke])/(t[ke]-t[kl]) if ke and t[ke]>t[kl] else float('nan')
                vol=st.median(s[8] for s in seg)
                res.setdefault((ph,gs,ga),[]).append((name,peak,a,dec,vol))
        i=j+1
for k,v in sorted(res.items()):
    pk=[x[1] for x in v];a=[x[2] for x in v];dc=[x[3] for x in v if x[3]==x[3]]
    print(f"phase {k[0]:2d} goalSpeed {k[1]:4d} acc {k[2]:3d} (cmd {k[2]*100} t/s2) n={len(v):3d} peak med {st.median(pk):6.0f} [{min(pk)}..{max(pk)}]  accel med {st.median(a):7.0f} p10 {sorted(a)[len(a)//10]:7.0f}  decel med {st.median(dc) if dc else 0:7.0f} p10 {sorted(dc)[len(dc)//10] if dc else 0:7.0f} volt {st.median(x[4] for x in v)}")
