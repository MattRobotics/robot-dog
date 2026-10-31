#!/usr/bin/env python3
"""PRONE offline CAD helper, derived from the previous resting-plane study.
Mirror-pair sagittal model; no friction, compliance or dynamic qualification.
"""
from pathlib import Path
import csv,json,math,struct,sys
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial import ConvexHull
from scipy.optimize import linprog
root=Path(__file__).resolve().parents[4];urdf=root/'03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
x=ET.parse(urdf).getroot();legs=('lf','rf','rh','lh');names=[f'{l}_{j}_joint' for l in legs for j in ('hip','upper_leg','lower_leg')]
def vec(s):return np.array([float(v) for v in s.split()])
def rot(axis,q):
 a=vec(axis);K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
 return np.eye(3)+math.sin(q)*K+(1-math.cos(q))*(K@K)
def origin(o):
 t=np.eye(4)
 if o is not None:
  t[:3,3]=vec(o.get('xyz','0 0 0'));assert o.get('rpy','0 0 0')=='0 0 0'
 return t
meshes={};inert=[]
for link in x.findall('link'):
 name=link.get('name');m=link.find('collision/geometry/mesh');raw=(urdf.parent/m.get('filename')).read_bytes();n=struct.unpack_from('<I',raw,80)[0];assert len(raw)==84+50*n
 v=np.frombuffer(raw,dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]),offset=84)['v'].reshape(-1,3)*vec(m.get('scale'))
 # Convex hull vertices preserve every plane minimum exactly.
 v=v[ConvexHull(v).vertices];meshes[name]=(origin(link.find('collision/origin'))@np.c_[v,np.ones(len(v))].T).T
 i=link.find('inertial');inert.append((name,float(i.find('mass').get('value')),np.r_[vec(i.find('origin').get('xyz')),1]))

def geometry(q):
 q=dict(zip(names,q));frames={'base_link':np.eye(4)};pending=x.findall('joint')
 while pending:
  for j in pending[:]:
   parent=j.find('parent').get('link')
   if parent not in frames:continue
   a=np.eye(4)
   if j.get('type')!='fixed':a[:3,:3]=rot(j.find('axis').get('xyz'),q[j.get('name')])
   frames[j.find('child').get('link')]=frames[parent]@origin(j.find('origin'))@a;pending.remove(j)
 return {n:(frames[n]@v.T).T[:,:3] for n,v in meshes.items()},sum(m*(frames[n]@p)[:3] for n,m,p in inert)/sum(m for _,m,_ in inert)

def rest(q,reference=0):
 V,c=geometry(q);pts=np.concatenate(list(V.values()))[:,[0,2]];pts=pts[ConvexHull(pts).vertices];pts=sorted(set(map(tuple,pts)));h=[]
 for p in pts:
  while len(h)>=2 and np.cross(np.array(h[-1])-h[-2],np.array(p)-h[-2])<=0:h.pop()
  h.append(p)
 candidates=[]
 for a,b in zip(h,h[1:]):
  pitch=math.atan2(b[1]-a[1],b[0]-a[0]);R=rot('0 1 0',pitch);cw=R@c
  ax=(R@np.array([a[0],0,a[1]]))[0];bx=(R@np.array([b[0],0,b[1]]))[0]
  if ax<=cw[0]<=bx:candidates.append((abs(pitch-reference),pitch,min(cw[0]-ax,bx-cw[0])))
 assert candidates
 _,pitch,margin=min(candidates);R=rot('0 1 0',pitch);W={n:v@R.T for n,v in V.items()};z=-min(v[:,2].min() for v in W.values());low={n:float(v[:,2].min()+z) for n,v in W.items()}
 return {'pitch':pitch,'z':z,'belly_gap':low['base_link'],'low':low,'contacts':[n for n,v in low.items() if v<1e-5],'sagittal_margin':margin,'com':(R@c+np.array([0,0,z])).tolist()}

def entry_q(hip=0,uf=1.4615259421016524,ur=math.pi/2,lf=-88.5*math.pi/180,lr=-89*math.pi/180):
 return np.array([hip,uf,lf,-hip,uf,lf,-hip,ur,lr,hip,ur,lr])

if __name__=='__main__':
 for h in [0,.05,.10,.15,.18,.19]:
  r=rest(entry_q(h));print(h,json.dumps(r))

def rest3d(q,reference=(0,.09)):
 """Rigid static support plane of all collision meshes, including asymmetric q.
 Select stable hull face nearest the reference gravity. No contact sensor model.
 """
 V,c=geometry(q);pts=np.concatenate(list(V.values()));h=ConvexHull(pts)
 ref=rot('0 1 0',reference[1])@rot('1 0 0',reference[0]);up=ref[2];candidates=[]
 for face,e in zip(h.simplices,h.equations):
  n,d=e[:3],e[3]
  if n[2]>-.7:continue
  projected=c-(n@c+d)*n;a,b,cc=pts[face];uv=np.linalg.lstsq(np.stack([b-a,cc-a],axis=1),projected-a,rcond=None)[0]
  if min(uv)<-1e-8 or sum(uv)>1+1e-8:continue
  candidates.append((np.linalg.norm(-n-up),-n,-d))
 assert candidates
 _,n,z=min(candidates,key=lambda a:a[0]);pitch=math.asin(-n[0]);roll=math.atan2(n[1],n[2]);R=rot('0 1 0',pitch)@rot('1 0 0',roll)
 W={k:v@R.T+np.array([0,0,z]) for k,v in V.items()};low={k:float(v[:,2].min()) for k,v in W.items()};cw=R@c+np.array([0,0,z])
 # 20 micrometres only groups STL roundoff between mirrored coplanar patches.
 contacts={k:v[v[:,2]<2e-5] for k,v in W.items() if low[k]<2e-5}
 cp=np.concatenate(list(contacts.values()));hull=ConvexHull(cp[:,:2]);margin=float(np.min(-(hull.equations[:,:2]@cw[:2]+hull.equations[:,2])))
 A=np.array([np.ones(len(cp)),cp[:,0],cp[:,1]]);b=sum(m for _,m,_ in inert)*9.80665*np.r_[1,cw[:2]]
 belly=np.concatenate([np.ones(len(v)) if k=='base_link' else np.zeros(len(v)) for k,v in contacts.items()]);forces=[]
 for sign in [1,-1]:
  r=linprog(sign*belly,A_eq=A,b_eq=b,bounds=(0,None),method='highs');assert r.success;forces.append(float(sign*r.fun))
 return {'pitch':pitch,'roll':roll,'z':z,'belly_gap':low['base_link'],'belly_max_gap':float(W['base_link'][V['base_link'][:,2]<1e-6,2].max()),'low':low,'contacts':list(contacts),'support_margin':margin,'belly_force_N_minmax':forces,'com':cw.tolist()}
