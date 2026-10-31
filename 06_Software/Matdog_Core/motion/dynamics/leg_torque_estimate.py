#!/usr/bin/env python3
"""Leg joint torque estimate from the URDF (Issue #52, 2026-10-15). STIMATO.

Inputs: the URDF (masses, COM, inertia tensors, joint origins/axes) and a
trajectory dump of the production gait (scripts/tests/run_gait_envelope_sweep.sh
"stride,T,lift,duty": rows joint_slot,t,q,qd,qdd).
Two cases, both explicit approximations:
  FIXTURE  - body held fixed and level, legs in free air: recursive Newton-Euler
             (gravity + inertial torques of hip/upper/lower+foot only).
  GROUND   - quasi-static stance support: each stance foot carries M*g/n_stance
             vertically (n=3, the crawl worst case), tau = J^T F at the stance
             configurations of the same trajectory, plus the FIXTURE leg terms.
             Uneven load sharing, body dynamics, impacts and friction are NOT modelled.
Not modelled anywhere: servo gearbox friction/efficiency, cable drag, battery
and electronics mass not present in the URDF (sensitivity via --extra-mass-kg).
"""
from pathlib import Path
import argparse,math,sys
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[4]
URDF=ROOT/'03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
def rpy(r,p,y):
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
def axis_rot(a,q):
    a=a/np.linalg.norm(a);K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3)+math.sin(q)*K+(1-math.cos(q))*K@K
def vec(s,default='0 0 0'):return np.array([float(x) for x in (s or default).split()])
class Model:
    def __init__(self):
        t=ET.parse(URDF).getroot()
        self.links={};self.joints={}
        for l in t.findall('link'):
            i=l.find('inertial')
            if i is None:continue
            o=i.find('origin');I=i.find('inertia')
            R=rpy(*vec(o.get('rpy') if o is not None else None)) if o is not None else np.eye(3)
            Ic=np.array([[float(I.get('ixx')),float(I.get('ixy')),float(I.get('ixz'))],[float(I.get('ixy')),float(I.get('iyy')),float(I.get('iyz'))],[float(I.get('ixz')),float(I.get('iyz')),float(I.get('izz'))]])
            self.links[l.get('name')]=dict(m=float(i.find('mass').get('value')),c=vec(o.get('xyz') if o is not None else None),I=R@Ic@R.T)
        for j in t.findall('joint'):
            o=j.find('origin');ax=j.find('axis')
            self.joints[j.get('name')]=dict(type=j.get('type'),parent=j.find('parent').get('link'),child=j.find('child').get('link'),
                xyz=vec(o.get('xyz') if o is not None else None),R=rpy(*vec(o.get('rpy') if o is not None else None)),axis=vec(ax.get('xyz') if ax is not None else None,'1 0 0'),
                effort=float(j.find('limit').get('effort')) if j.find('limit') is not None else None)
        self.mass=sum(l['m'] for l in self.links.values())
    def chain(self,leg):
        names=[f'{leg}_hip_joint',f'{leg}_upper_leg_joint',f'{leg}_lower_leg_joint']
        foot=next(n for n,j in self.joints.items() if j['parent']==f'{leg}_lower_leg_link' and j['type']=='fixed')
        return names,foot
    def lower_plus_foot(self,leg,foot):
        a=self.links[f'{leg}_lower_leg_link'];j=self.joints[foot];b=self.links[j['child']]
        cb=j['xyz']+j['R']@b['c'];Ib=j['R']@b['I']@j['R'].T;m=a['m']+b['m'];c=(a['m']*a['c']+b['m']*cb)/m
        def shift(I,mm,d):return I+mm*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
        return dict(m=m,c=c,I=shift(a['I'],a['m'],a['c']-c)+shift(Ib,b['m'],cb-c)),j['xyz']+j['R']@np.zeros(3)
    def rnea(self,leg,q,qd,qdd,g=np.array([0,0,-9.81])):
        names,foot=self.chain(leg);bodies=[self.links[f'{leg}_hip_link'],self.links[f'{leg}_upper_leg_link']]
        lf,_=self.lower_plus_foot(leg,foot);bodies.append(lf)
        # forward: base fixed, frames expressed in each link frame
        w=np.zeros(3);dw=np.zeros(3);a=-g.copy();Rs=[];ps=[];ws=[];dws=[];acs=[]
        Rprev=np.eye(3)
        for k,n in enumerate(names):
            j=self.joints[n];R=j['R']@axis_rot(j['axis'],q[k]);Rt=R.T;z=j['axis']/np.linalg.norm(j['axis'])
            p=j['xyz']
            a=Rt@(a+np.cross(dw,p)+np.cross(w,np.cross(w,p)))
            w_new=Rt@w+qd[k]*z;dw=Rt@dw+qdd[k]*z+np.cross(Rt@w,qd[k]*z);w=w_new
            b=bodies[k];ac=a+np.cross(dw,b['c'])+np.cross(w,np.cross(w,b['c']))
            Rs.append(R);ps.append(p);ws.append(w.copy());dws.append(dw.copy());acs.append(ac)
        f=np.zeros(3);n_=np.zeros(3);tau=[0,0,0]
        for k in (2,1,0):
            b=bodies[k];F=b['m']*acs[k];N=b['I']@dws[k]+np.cross(ws[k],b['I']@ws[k])
            if k<2:R=Rs[k+1];p=ps[k+1];fc=R@f;nc=R@n_+np.cross(p,fc)
            else:fc=np.zeros(3);nc=np.zeros(3)
            f=F+fc;n_=N+nc+np.cross(b['c'],F)
            j=self.joints[names[k]];tau[k]=float(n_@(j['axis']/np.linalg.norm(j['axis'])))
        return tau
    def foot_pos(self,leg,q):
        names,foot=self.chain(leg);T=np.eye(4)
        for k,n in enumerate(names):
            j=self.joints[n];A=np.eye(4);A[:3,:3]=j['R']@axis_rot(j['axis'],q[k]);A[:3,3]=j['xyz'];T=T@A
        jf=self.joints[foot];A=np.eye(4);A[:3,:3]=jf['R'];A[:3,3]=jf['xyz'];T=T@A
        return T[:3,3]
def main():
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dump',type=Path);ap.add_argument('--extra-mass-kg',type=float,default=0.0);ap.add_argument('--stance-feet',type=int,default=3);ap.add_argument('--every',type=int,default=5,help='use every Nth 1 ms sample')
    a=ap.parse_args();m=Model()
    rows=np.loadtxt(a.dump,delimiter=',')
    legs=('lf','rf','rh','lh')   # firmware slot leg index 0..3 = motion::LegId {LF, RF, RH, LH}
    out={}
    for li,leg in enumerate(legs):
        sel=[rows[rows[:,0]==li*3+k][::a.every] for k in range(3)];n=len(sel[0])
        fx=np.zeros((n,3));gr=np.zeros((n,3))
        Mg=(m.mass+a.extra_mass_kg)*9.81/a.stance_feet
        for i in range(n):
            q=[sel[k][i,2] for k in range(3)];qd=[sel[k][i,3] for k in range(3)];qdd=[sel[k][i,4] for k in range(3)]
            fx[i]=m.rnea(leg,q,qd,qdd)
            # quasi-static stance: J^T F, F = (0,0,Mg/n) applied at the foot (ground pushes up)
            J=np.zeros((3,3));p0=m.foot_pos(leg,q)
            for k in range(3):
                dq=list(q);dq[k]+=1e-6;J[:,k]=(m.foot_pos(leg,dq)-p0)/1e-6
            gr[i]=m.rnea(leg,q,[0,0,0],[0,0,0])-J.T@np.array([0,0,Mg])
        out[leg]=(np.abs(fx).max(0),np.sqrt((fx**2).mean(0)),np.abs(gr).max(0))
    eff=m.joints['lf_hip_joint']['effort']
    print(f'URDF mass {m.mass:.3f} kg (+{a.extra_mass_kg} kg), URDF effort limit {eff} N*m per joint, stance feet {a.stance_feet}')
    for leg,(pk,rms,g) in out.items():
        print(f'{leg.upper()} FIXTURE peak hip/upper/lower = {pk[0]:.3f}/{pk[1]:.3f}/{pk[2]:.3f} N*m rms {rms[0]:.3f}/{rms[1]:.3f}/{rms[2]:.3f} | GROUND quasi-static peak {g[0]:.3f}/{g[1]:.3f}/{g[2]:.3f} N*m')
if __name__=='__main__':main()
