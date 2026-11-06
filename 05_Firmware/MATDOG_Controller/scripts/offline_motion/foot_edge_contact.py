#!/usr/bin/env python3
"""Edge contact of the MATDOG tread on the canonical URDF meshes (dev.15). Offline, read-only.

`mesh`: lowest point of the real foot STL vs the firmware fillet model
(FootContact.h edgeContactLiftM) and lower-leg clearance versus foot-axis tilt.
`frames JSON NAME FILE ...`: joystick_envelope.cpp dumps (body heading frame). For
every 3rd release: static support margin of the URDF COM inside the polygon
of the REAL stance contacts (strip reference within the nominal strip, the
lowest fillet point beyond it), the lowest non-foot mesh point above the
ground, and the lowest foot point (penetration). A dump without WALK frames
is NOT_APPLICABLE (no value). Rigid flat ground; no compliance, friction or
dynamics.
"""
import sys,math,json
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'offline_prone'))
import geometry as G
R,S,RHO,NOMINAL=.0149,.0099,.002,0.034906585
LEGS=('lf','rf','rh','lh')
def frames(q):
    q=dict(zip(G.names,q));fr={'base_link':np.eye(4)};pend=G.x.findall('joint')
    while pend:
        for j in pend[:]:
            p=j.find('parent').get('link')
            if p not in fr:continue
            a=np.eye(4)
            if j.get('type')!='fixed':a[:3,:3]=G.rot(j.find('axis').get('xyz'),q[j.get('name')])
            fr[j.find('child').get('link')]=fr[p]@G.origin(j.find('origin'))@a;pend.remove(j)
    return fr
def contact(F):
    """Strip reference, real lowest tread point and axis tilt for ground normal +Z."""
    c=(F@np.array([0,0,R,1]))[:3];ax=F[:3,:3]@np.array([0,1.,0]);n=np.array([0,0,1.])
    perp=n-ax*(ax@n);d=-perp/np.linalg.norm(perp);ref=c+R*d;tilt=math.asin(min(1,abs(ax@n)))
    if tilt<=NOMINAL:return ref,ref,tilt
    low=ax if ax@n<0 else -ax
    return ref,c+(S/2)*low+(R-RHO)*d-RHO*n,tilt
def lift(t):
    rigid=lambda a:.5*S*math.sin(a)+RHO*(1-math.cos(a))
    return 0 if t<=NOMINAL else rigid(t)-rigid(NOMINAL)
def mesh():
    qs=[0,0.669,-0.193]*2+[0,1.20,-0.40]*2;out=[]
    print('leg tilt_deg mesh_low-ref_z_mm model_z_mm  lowerleg_clear_mm')
    for leg,i in (('lf',0),('rh',6)):
        for hd in (0,2,4,6,8,10,12,15,20,30):
            q=list(qs);q[i]=math.radians(hd);F=frames(q)[leg+'_foot_link'];V,_=G.geometry(q)
            foot=V[leg+'_foot_link'];low=foot[:,2].min();ref,_,t=contact(F)
            model=-(.5*S*math.sin(t)+RHO*(1-math.cos(t)))
            ll=V[leg+'_lower_leg_link'][:,2].min()-low
            print(f'{leg} {hd:5.1f} {1e3*(low-ref[2]):9.3f} {1e3*model:9.3f} {1e3*ll:9.2f}');out.append((leg,hd,1e3*(low-ref[2]),1e3*model,1e3*ll))
    return out
def check(path):
    if Path(path).stat().st_size==0:return None
    a=np.loadtxt(path,ndmin=2);res=dict(support_mm=1e9,support_cycle=0,clearance_mm=1e9,clearance_link='',penetration_mm=0,max_tilt_deg=0)
    for k in range(0,len(a),3):
        row=a[k];bz=row[3];q=row[4:16];ft=row[16:].reshape(4,4)
        fr=frames(q);V,com=G.geometry(q);pts=[];ground=[]
        for l,leg in enumerate(LEGS):
            ref,real,t=contact(fr[leg+'_foot_link'])
            if ft[l,3]>.5:
                pts.append(real[:2]);res['max_tilt_deg']=max(res['max_tilt_deg'],math.degrees(t))
                res['penetration_mm']=max(res['penetration_mm'],-1e3*(V[leg+'_foot_link'][:,2].min()+bz))
        for name,v in V.items():
            if name.endswith('foot_link'):continue
            z=1e3*(v[:,2].min()+bz)
            if z<res['clearance_mm']:res['clearance_mm']=z;res['clearance_link']=name
        if len(pts)>=3:
            h=ConvexHull(np.array(pts));m=float(np.min(-(h.equations[:,:2]@com[:2]+h.equations[:,2])))*1e3
            if m<res['support_mm']:res['support_mm']=m;res['support_cycle']=float(row[0])
    return res
if __name__=='__main__':
    if sys.argv[1]=='mesh':mesh()
    else:
        out=Path(sys.argv[2]);rep={}
        for name,f in zip(sys.argv[3::2],sys.argv[4::2]):
            r=check(f);rep[name]=r
            if r is None:print(f"{name} support NOT_APPLICABLE | non-foot clearance NOT_APPLICABLE (no WALK frame)",flush=True);continue
            print(f"{name} support {r['support_mm']:.2f} mm c{r['support_cycle']:.3f} | non-foot clearance {r['clearance_mm']:.2f} mm ({r['clearance_link']}) | foot penetration {r['penetration_mm']:.3f} mm | stance tilt {r['max_tilt_deg']:.2f} deg",flush=True)
        out.write_text(json.dumps(rep,indent=1)+'\n')
