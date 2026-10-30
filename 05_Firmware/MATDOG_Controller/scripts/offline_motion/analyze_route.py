#!/usr/bin/env python3
"""Offline geometry/load STUDY, not physical contact or actuator qualification.
Usage: python3 analyze_route.py PROBE.csv OUTPUT_DIR
Production FK feet in CSV; URDF inertials transformed independently for COM.
"""
from pathlib import Path
import csv, json, math, struct, sys, hashlib
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import linprog
root=Path(__file__).resolve().parents[4]
urdf=root/'03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
x=ET.parse(urdf).getroot(); links=x.findall('link'); joints=x.findall('joint')
names=[f'{l}_{j}_joint' for l in ('lf','rf','rh','lh') for j in ('hip','upper_leg','lower_leg')]
rows=list(csv.DictReader(Path(sys.argv[1]).open()));out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
def xyz(s):return np.array([float(v) for v in s.split()])
def rotation(axis,q):
 a=np.array(axis);a=a/np.linalg.norm(a);K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
 return np.eye(3)+math.sin(q)*K+(1-math.cos(q))*(K@K)
def transform(origin):
 t=np.eye(4)
 if origin is not None:
  t[:3,3]=xyz(origin.get('xyz','0 0 0'));r,p,y=xyz(origin.get('rpy','0 0 0'))
  t[:3,:3]=rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
 return t
mass=sum(float(l.find('inertial/mass').get('value')) for l in links)
mesh=x.find("link[@name='base_link']/collision/geometry/mesh")
meshpath=urdf.parent/mesh.get('filename');raw=meshpath.read_bytes();n=struct.unpack_from('<I',raw,80)[0]
assert len(raw)==84+50*n,'binary STL expected'
v=np.frombuffer(raw, dtype=np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')]),offset=84)['v'].reshape(-1,3)*xyz(mesh.get('scale'))
v=np.unique(v,axis=0);v=np.c_[v,np.ones(len(v))]
collision_origin=transform(x.find("link[@name='base_link']/collision/origin"))
lo=np.array([-.783864,-.914253,-1.598408,-.783864,-.914253,-1.58,-.783864,-.914253,-1.592272,-.783864,-.914253,-1.590738]);hi=np.array([.783864,2.110758,.590583,.783864,2.10002,.581379,.771592,2.136835,.590583,.78233,2.129166,.585981])
results=[]
for r in rows:
 q=np.array([float(r[f'q{i}']) for i in range(12)]);body=np.eye(4);body[:3,:3]=rotation([0,1,0],float(r['pitch']));body[:3,3]=[float(r['body_x']),0,float(r['body_z'])]
 frames={'base_link':body};pending=joints[:]
 while pending:
  for j in pending[:]:
   parent=j.find('parent').get('link')
   if parent not in frames:continue
   a=np.eye(4)
   if j.get('type')!='fixed':a[:3,:3]=rotation(xyz(j.find('axis').get('xyz')),q[names.index(j.get('name'))])
   frames[j.find('child').get('link')]=frames[parent]@transform(j.find('origin'))@a;pending.remove(j)
 c=sum(float(l.find('inertial/mass').get('value'))*(frames[l.get('name')]@np.r_[xyz(l.find('inertial/origin').get('xyz')),1])[:3] for l in links)/mass
 feet=np.array([[float(r[f'f{l}{a}']) for a in 'xyz'] for l in range(4)])
 belly=float((body@collision_origin@v.T)[2].min());touch=np.abs(feet[:,2])<=.001
 # A uniform mass scaling to 2.8 kg preserves CAD COM (explicit assumption).
 # Four feet statics: range rather than invented unique per-leg loads.
 A=np.array([np.ones(4),feet[:,0],feet[:,1]]);b=np.array([2.8*9.80665,2.8*9.80665*c[0],2.8*9.80665*c[1]])
 front=np.array([1,1,0,0]);bounds=[(0,None) if t else (0,0) for t in touch]
 mn=linprog(front,A_eq=A,b_eq=b,bounds=bounds,method='highs');mx=linprog(-front,A_eq=A,b_eq=b,bounds=bounds,method='highs')
 result={**r,'com_x':c[0],'com_y':c[1],'com_z':c[2],'belly_min_z':belly,'ground_feet':''.join(str(i) for i,t in enumerate(touch) if t),'feet_equilibrium':bool(mn.success and mx.success),'front_N_min':mn.fun if mn.success else None,'front_N_max':-mx.fun if mx.success else None,'gen1_margin_rad':float(min((q-lo).min(),(hi-q).min()))}
 results.append(result)
with (out/'route_analysis.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=results[0].keys());w.writeheader();w.writerows(results)
summary={'classification':'OFFLINE_STUDY_NOT_HARDWARE','urdf_mass_kg':mass,'load_mass_kg':2.8,'mass_assumption':'uniform CAD mass scaling; installed extra 0.32 kg location unknown','urdf_sha256':hashlib.sha256(urdf.read_bytes()).hexdigest(),'ground_contact_tolerance_m':.001,'groups':{}}
for route in ('current','front_early','reverse'):
 for group in ('0','1'):
  rr=[r for r in results if r['route']==route and r['group']==group];good=[r for r in rr if r['feet_equilibrium']]
  summary['groups'][route+'_'+group]={'samples':len(rr),'duration_s':float(rr[-1]['t']),'box_margin_rad':min(float(r['box_margin']) for r in rr),'gen1_margin_rad':min(r['gen1_margin_rad'] for r in rr),'min_foot_z_m':min(float(r[f'f{l}z']) for r in rr for l in range(4)),'max_foot_z_m':max(float(r[f'f{l}z']) for r in rr for l in range(4)),'min_belly_z_m':min(r['belly_min_z'] for r in rr),'feet_equilibrium_samples':len(good),'front_N_range': [min(r['front_N_min'] for r in good),max(r['front_N_max'] for r in good)] if good else None,'start':{k:rr[0][k] for k in ('pitch','body_z','belly_min_z','ground_feet')},'end':{k:rr[-1][k] for k in ('pitch','body_z','belly_min_z','ground_feet')}}
(out/'route_summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
