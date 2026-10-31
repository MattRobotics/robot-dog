#!/usr/bin/env python3
"""Initial closed-surface nesting, complementary to continuous surface separation.
No intentional overlap is exempted. Refuse ambiguous rays/open triangulations.
"""
import argparse,json,hashlib,sys
from pathlib import Path
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from paired_prone_model import PairModel,ROOT
from certify_adjacent_axis import coefficients,trig_range,bound

def components(t):
 v,index=np.unique(np.round(t.reshape(-1,3),10),axis=0,return_inverse=True);f=index.reshape(-1,3)
 edge=np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1)
 unique,count=np.unique(edge,axis=0,return_counts=True)
 assert np.all(count==2),'OPEN_OR_NONMANIFOLD_SURFACE'
 graph=coo_matrix((np.ones(len(unique)),(unique[:,0],unique[:,1])),shape=(len(v),len(v)))
 n,labels=connected_components(graph,directed=False)
 # Representatives are original unrounded STL vertices, not the welding keys.
 original=t.reshape(-1,3);first=np.unique(index,return_index=True)[1]
 return original[first[np.unique(labels,return_index=True)[1]]],n

def inside(point,t):
 e1=t[:,1]-t[:,0];e2=t[:,2]-t[:,0];s=point-t[:,0];classification=[]
 for direction in [[1,.314159,.271828],[.161803,1,.414213],[.732051,.236068,1]]:
  d=np.array(direction);h=np.cross(np.broadcast_to(d,e2.shape),e2);det=np.einsum('ij,ij->i',e1,h)
  good=abs(det)>1e-15;inv=np.divide(1.,det,out=np.zeros_like(det),where=good)
  u=inv*np.einsum('ij,ij->i',s,h);q=np.cross(s,e1);v=inv*(q@d);time=inv*np.einsum('ij,ij->i',e2,q)
  hit=good&(time>1e-9)&(u>=0)&(v>=0)&(u+v<=1)
  assert not np.any(hit&((u<1e-10)|(v<1e-10)|(abs(u+v-1)<1e-10))),'AMBIGUOUS_RAY_EDGE'
  classification.append(int(hit.sum())%2)
 assert len(set(classification))==1,'AMBIGUOUS_RAY_PARITY'
 return bool(classification[0])

def self_test():
 lo,hi=trig_range(np.array([.4]),np.array([.3]),np.array([.7]),-1.6,1.6)
 sampled=.4+.3*np.cos(np.linspace(-1.6,1.6,10001))+.7*np.sin(np.linspace(-1.6,1.6,10001))
 assert lo[0]<=sampled.min()+1e-14 and hi[0]>=sampled.max()-1e-14
 A=np.array([[[0,0,0],[1,0,0],[0,1,0]]],dtype=float);Z=np.zeros_like(A)
 assert bound(A,A,Z,Z,0,0)[0]<0 # no false separation of coincident triangles
 assert bound(A,A+np.array([0,0,.01]),Z,Z,0,0)[0]>.009999

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args();self_test()
 source=ROOT/'09_Logs/Validation_Reports/Motion_Night_Run_2026-10-12/continuous_pair_path.json';raw=source.read_bytes();knots=json.loads(raw)['segments'];m=PairModel();s=m.scene
 reps={};topology=[]
 for name in s.model.collision_link_names:
  reps[name],n=components(s.mesh(name).triangles_local);topology.append(dict(link=name,closed=True,components=n))
 rows=[]
 for leg in ('lf','rf','rh','lh'):
  for part,field in [('hip','h'),('upper_leg','uf' if leg in ('lf','rf') else 'ur'),('lower_leg','lf' if leg in ('lf','rf') else 'lr')]:
   joint=s.model.joints[leg+'_'+part+'_joint'];sign=-1 if part=='hip' and leg in ('lf','lh') else 1;q=sign*knots[0]['start_parameters'][field]
   a=s.mesh(joint.parent_link).triangles_local;b=s.mesh(joint.child_link).triangles_local
   c,u,v=coefficients(b,np.array(joint.axis_xyz));transformed=c+u*np.cos(q)+v*np.sin(q)+np.array(joint.origin_xyz)
   c,u,v=coefficients(reps[joint.child_link],np.array(joint.axis_xyz));rb=c+u*np.cos(q)+v*np.sin(q)+np.array(joint.origin_xyz)
   nested_a=[i for i,x in enumerate(reps[joint.parent_link]) if inside(x,transformed)];nested_b=[i for i,x in enumerate(rb) if inside(x,a)]
   rows.append(dict(joint=joint.name if hasattr(joint,'name') else leg+'_'+part+'_joint',nested_parent_components=nested_a,nested_child_components=nested_b,status='PASS_NO_NESTED_SURFACE' if not nested_a and not nested_b else 'BLOCKED_NESTING_REVIEW_REQUIRED'))
 result=dict(schema='MATDOG_INITIAL_ASSEMBLY_NESTING_V1',source_sha256=hashlib.sha256(raw).hexdigest(),topology=topology,rows=rows,all_initial_adjacent_non_nested=all(x['status']=='PASS_NO_NESTED_SURFACE' for x in rows),physical_fit_or_load_qualified=False,
  inference='Combined with adjacent_axis continuous separation, initial closed components cannot newly cross or enter each other along this route. CAD solid semantics remain a physical assembly review requirement.')
 args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':main()
