#!/usr/bin/env python3
"""ST3215 braking from the encoder POSITION (not the 50-step quantized PresentSpeed),
bench QC V6.1 RAW (walk modes final 2026-11-09). Usage: brake_pos.py <qc_campaign dir>.
Per move (constant goal tuple, |peak speed| > 300 step/s):
 - v0: speed at the braking onset = least-squares slope of the position over the
   last 20 ms before the speed starts to fall (last sample >= 0.9 peak);
 - d: position travel from that onset to the extreme reached (overshoot included);
 - a_stop = v0^2 / (2 d): the constant deceleration that stops in the same distance
   (the quantity a STOP proof depends on);
 - a_fit: second derivative of a quadratic fitted to the position from the onset to
   the first sample with speed <= 10 % of the peak;
 - overshoot beyond the goal, ticks.
No load, 11.1 V bus (bench). Grouped by phase, goal speed, ACC; per servo id spread."""
import struct,sys,glob,statistics as st
def load(p):
    b=open(p,'rb').read();h=struct.unpack_from('<8s7I48s',b,0);n=h[5];out=[]
    for k in range(n):
        o=84+27*k;t,gp,gs,ga,ph,pr,fl=struct.unpack_from('<IHHBBBB',b,o);r=b[o+12:o+27]
        pos=r[0]|r[1]<<8;sp=r[2]|r[3]<<8;sp=-(sp&0x7fff) if sp&0x8000 else sp
        out.append((t,gp,gs,ga,ph,pos,sp,r[6]))
    return h,out
def slope(ts,xs):
    n=len(ts);mt=sum(ts)/n;mx=sum(xs)/n;den=sum((t-mt)**2 for t in ts)
    return sum((t-mt)*(x-mx) for t,x in zip(ts,xs))/den if den>0 else float('nan')
def quad(ts,xs):
    import numpy as np
    c=np.polyfit(ts,xs,2);return 2*c[0]
res={}
for p in sorted(glob.glob(sys.argv[1]+'/*.bin')):
    name=p.split('/')[-1].split('__')[0];h,d=load(p);sid=h[2];i=0;n=len(d)
    while i<n:
        j=i
        while j+1<n and d[j+1][1:5]==d[i][1:5]:j+=1
        seg=d[i:j+1];gp,gs,ga,ph=seg[0][1],seg[0][2],seg[0][3],seg[0][4];i=j+1
        if ga==255 or ga<10 or len(seg)<60 or gp==0xFFFF:continue
        sp=[abs(s[6]) for s in seg];peak=max(sp)
        if peak<=300:continue
        t=[s[0]/1e6 for s in seg];pos=[s[5] for s in seg]
        kl=max(k for k,v in enumerate(sp) if v>=.9*peak)
        ke=next((k for k in range(kl,len(sp)) if sp[k]<=.1*peak),None)
        if ke is None or kl<10:continue
        w=[k for k in range(len(t)) if t[kl]-.02<=t[k]<=t[kl]]
        v0=abs(slope([t[k] for k in w],[pos[k] for k in w]))
        dirn=1 if pos[ke]>pos[kl] else -1
        ext=max(pos[kl:],key=lambda x:dirn*x);dist=abs(ext-pos[kl])
        if dist<1 or v0<200:continue
        astop=v0*v0/(2*dist)
        try:afit=abs(quad([t[k]-t[kl] for k in range(kl,ke+1)],[pos[k] for k in range(kl,ke+1)])) if ke-kl>=4 else float('nan')
        except Exception:afit=float('nan')
        over=dirn*(ext-gp)
        res.setdefault((ph,gs,ga),[]).append((sid,v0,astop,afit,over,dist))
def pct(x,q):x=sorted(x);return x[int(q*(len(x)-1))]
print("phase goalSpd ACC (cmd t/s2)  n | v0 med  | a_stop med  p10   min  [min/cmd] | a_fit med  p10 | overshoot med max | per-id a_stop min..max of medians")
for k,v in sorted(res.items()):
    a=[x[2] for x in v];af=[x[3] for x in v if x[3]==x[3]];ov=[x[4] for x in v]
    ids={};[ids.setdefault(x[0],[]).append(x[2]) for x in v];meds=[st.median(z) for z in ids.values()]
    cmd=k[2]*100
    print(f"{k[0]:3d} {k[1]:5d} {k[2]:4d} ({cmd:5d}) {len(v):4d} | {st.median(x[1] for x in v):6.0f} | {st.median(a):7.0f} {pct(a,.1):6.0f} {min(a):6.0f} [{min(a)/cmd:4.2f}] | {st.median(af) if af else 0:7.0f} {pct(af,.1) if af else 0:6.0f} | {st.median(ov):5.1f} {max(ov):5.0f} | {len(ids)} ids {min(meds):6.0f}..{max(meds):6.0f}")
