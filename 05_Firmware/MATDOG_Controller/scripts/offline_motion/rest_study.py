#!/usr/bin/env python3
"""CAD static resting plane, independent of intended body pose. Offline only.
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
rows=list(csv.DictReader(Path(sys.argv[1]).open()));out=Path(sys.argv[2]);results=[]
for ix,r in enumerate(rows):
 if ix%5 and ix+1<len(rows) and rows[ix+1]['group']==r['group'] and rows[ix+1]['route']==r['route'] and float(r['t'])!=0:continue
 q={name:float(r[f'q{i}']) for i,name in enumerate(names)};frames={'base_link':np.eye(4)};pending=x.findall('joint')
 while pending:
  for j in pending[:]:
   parent=j.find('parent').get('link')
   if parent not in frames:continue
   a=np.eye(4)
   if j.get('type')!='fixed':a[:3,:3]=rot(j.find('axis').get('xyz'),q[j.get('name')])
   frames[j.find('child').get('link')]=frames[parent]@origin(j.find('origin'))@a;pending.remove(j)
 V={n:(frames[n]@v.T).T[:,:3] for n,v in meshes.items()};c=sum(m*(frames[n]@p)[:3] for n,m,p in inert)/2.48
 pts=np.concatenate(list(V.values()))[:,[0,2]];pts=pts[ConvexHull(pts).vertices];pts=sorted(set(map(tuple,pts)));h=[]
 for p in pts:
  while len(h)>=2 and np.cross(np.array(h[-1])-h[-2],np.array(p)-h[-2])<=0:h.pop()
  h.append(p)
 candidates=[]
 for a,b in zip(h,h[1:]):
  pitch=math.atan2(b[1]-a[1],b[0]-a[0]);R=rot('0 1 0',pitch);cw=R@c
  ax=(R@np.array([a[0],0,a[1]]))[0];bx=(R@np.array([b[0],0,b[1]]))[0]
  if ax<=cw[0]<=bx:candidates.append((abs(pitch-float(r['pitch'])),pitch))
 assert candidates
 pitch=min(candidates)[1];R=rot('0 1 0',pitch);W={n:v@R.T for n,v in V.items()};z=-min(v[:,2].min() for v in W.values());low={n:float(v[:,2].min()+z) for n,v in W.items()};cw=R@c
 # Contact points: lowest vertex per foot (finite patch underdetermined).
 feet=np.array([W[l+'_foot_link'][np.argmin(W[l+'_foot_link'][:,2])] for l in legs]);A=np.array([np.ones(4),feet[:,0],feet[:,1]]);b=2.8*9.80665*np.array([1,cw[0],cw[1]])
 bounds=[(0,None) if low[l+'_foot_link']<.0005 else (0,0) for l in legs];front=np.array([1,1,0,0]);mn=linprog(front,A_eq=A,b_eq=b,bounds=bounds);mx=linprog(-front,A_eq=A,b_eq=b,bounds=bounds)
 row={'route':r['route'],'group':r['group'],'t':r['t'],'rest_pitch':pitch,'rest_z':z,'delta_pitch':pitch-float(r['pitch']),'delta_z':z-float(r['body_z']),'belly_clearance_m':low['base_link'],'min_nonfoot_clearance_m':min(v for n,v in low.items() if not n.endswith('foot_link')),'contacts':','.join(n for n,v in low.items() if v<.0005),'com_x':cw[0],'com_z':cw[2]+z,'front_N_min':mn.fun if mn.success else None,'front_N_max':-mx.fun if mx.success else None}
 for l in range(4):
  objective=np.eye(4)[l];mn1=linprog(objective,A_eq=A,b_eq=b,bounds=bounds);mx1=linprog(-objective,A_eq=A,b_eq=b,bounds=bounds)
  row[legs[l]+'_N_min']=mn1.fun if mn1.success else None;row[legs[l]+'_N_max']=-mx1.fun if mx1.success else None
 # Extra 0.32 kg at +/-50 mm along base x instead of uniform scaling: sensitivity, not installed COM bounds.
 for dx in [-.05,.05]:
  cc=R@((2.48*c+.32*np.array([dx,0,.02773]))/2.8);bb=2.8*9.80665*np.array([1,cc[0],cc[1]]);s=linprog(front,A_eq=A,b_eq=bb,bounds=bounds);row['front_extra_'+str(dx)+'_N']=s.fun if s.success else None
 results.append(row)
with (out/'rest_analysis.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=results[0]);w.writeheader();w.writerows(results)
s={}
for route in ('current','front_early','reverse'):
 for group in ('0','1'):
  rr=[r for r in results if r['route']==route and r['group']==group];s[route+'_'+group]={'samples':len(rr),'min_nonfoot_clearance_mm':1000*min(r['min_nonfoot_clearance_m'] for r in rr),'belly_clearance_start_end_mm':[1000*rr[i]['belly_clearance_m'] for i in (0,-1)],'max_delta_pitch':max(abs(r['delta_pitch']) for r in rr),'front_load_N_range':[min(r['front_N_min'] for r in rr if r['front_N_min'] is not None),max(r['front_N_max'] for r in rr if r['front_N_max'] is not None)],'contacts':sorted(set(r['contacts'] for r in rr))}
(out/'rest_summary.json').write_text(json.dumps(s,indent=2)+'\n');print(json.dumps(s,indent=2))
