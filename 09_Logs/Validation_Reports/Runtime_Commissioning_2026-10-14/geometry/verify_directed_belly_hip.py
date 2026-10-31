#!/usr/bin/env python3
"""Offline supplemental proof: 3 mm downward-only ventral extrusion vs adjacent hips.
No connected surface can become nested through a path whose full simplex
boundaries remain separated. Each extrusion prism starts from original base
triangles; initial baseline-CAD no-nesting is checked separately by the entry
certificate. No arbitrary 3 mm lateral expansion of HIP assembly interfaces.
"""
from pathlib import Path
import argparse,sys,math,hashlib,json,time
import numpy as np,fcl
p=argparse.ArgumentParser();p.add_argument('--joint-log',type=Path,required=True);p.add_argument('--repo',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise SystemExit('preserve evidence')
sys.path.insert(0,str(a.repo/'06_Software/Matdog_Core/motion/paired_prone'))
from continuous_pair_path import Continuous
from certify_commissioning_entry import SLOTS,from_log,head
log=a.joint_log
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
c=Continuous();start=dict(zip(SLOTS,from_log(log)));goal=head(-88.5,-89);middle={j:(goal[j] if j.endswith('lower_leg_joint') else start[j]) for j in SLOTS}
t=c.s.mesh('base_link').triangles_local;bottom=t[:,:,2].min();t=t[t[:,:,2].min(axis=1)<=bottom+.015]
b=t+[0,0,-.003];envelope=[t,b]
for i in range(3):
 j=(i+1)%3;envelope.extend((np.stack([t[:,i],t[:,j],b[:,i]],axis=1),np.stack([b[:,i],t[:,j],b[:,j]],axis=1)))
envelope=np.concatenate(envelope)
def model(t):
 v=t.reshape(-1,3);f=np.arange(len(v)).reshape(-1,3);b=fcl.BVHModel();b.beginModel(len(v),len(f));b.addSubModel(v,f);b.endModel();return b
def place(m,n,q):
 tf=c.s.collision_transform(n,q);return fcl.CollisionObject(m,fcl.Transform(np.asarray(tf.rotation),np.asarray(tf.translation)))
base=model(envelope);hips={n:model(c.s.mesh(n).triangles_local) for n in c.s.model.collision_link_names if n.endswith('hip_link')}
rows=[];witness=[];t0=time.monotonic()
for name,qa,qb in [('ENTRY_DISENGAGE',start,middle),('ENTRY_ALIGN',middle,goal)]:
 samples=1 if name=='ENTRY_DISENGAGE' else 100
 for n,m in hips.items():
  worst={tube:math.inf for tube in [.004,.01]};details={}
  for k in range(samples+1):
   u=k/samples;q={j:qa[j]+u*(qb[j]-qa[j]) for j in SLOTS};r=fcl.DistanceResult()
   d=fcl.distance(place(base,'base_link',q),place(m,n,q),fcl.DistanceRequest(),r)
   for tube in worst:
    margin=sum(radius*(abs(qb[j]-qa[j])/(2*samples)+tube) for j,radius in c.radii[n].items())+1e-9
    if d-margin<worst[tube]:worst[tube]=d-margin;details[tube]=dict(u=u,sample_distance_m=d,lipschitz_and_roundoff_m=margin)
  rows.append(dict(stage=name,link=n,samples=samples,bounds=[dict(tube_rad=t,certified_lower_bound_m=worst[t],pass_=worst[t]>0,**details[t]) for t in worst]))
  print(name,n,worst,flush=True)
upper=c.s.mesh('lf_upper_leg_link').triangles_local;lower=c.s.mesh('lf_lower_leg_link').triangles_local
for rad in [start['lf_lower_leg_joint']-.01,-1.60693236053,start['lf_lower_leg_joint']-.004,start['lf_lower_leg_joint']]:
 q=dict(start);q['lf_lower_leg_joint']=rad;A=place(model(upper),'lf_upper_leg_link',q);B=place(model(lower),'lf_lower_leg_link',q)
 cr=fcl.CollisionResult();contacts=fcl.collide(A,B,fcl.CollisionRequest(num_max_contacts=1,enable_contact=True),cr)
 d=fcl.distance(A,B,fcl.DistanceRequest(),fcl.DistanceResult());witness.append(dict(q_lower_rad=rad,q_lower_deg=math.degrees(rad),surface_distance_m=d,sampled_collision=bool(contacts)))
payload=dict(schema='MATDOG_DIRECTED_BELLY_HIP_ENVELOPE_V1',status='PASS_SURFACE_SEPARATION' if all(b['pass_'] for r in rows for b in r['bounds']) else 'BLOCKED',
 hardware_authorized=False,source_sha256=sha(__file__),joint_log_sha256=sha(log),urdf_sha256=sha(c.s.model.urdf_path),
 mesh_sha256={n:sha(c.s.model.collision_mesh_path(n)) for n in ['base_link',*hips]},
 start_rad=[start[j] for j in SLOTS],head_rad=[goal[j] for j in SLOTS],source_ventral_triangles=len(t),extrusion_surface_triangles=len(envelope),
 envelope='Union of all triangular prisms obtained by moving retained ventral triangles continuously from z to z-3 mm; all faces touching bottom+15 mm retained in full. No lateral enlargement.',
 interpretation='Adjacent assembly clearance must remain positive; the 3 mm non-adjacent margin is in the entry certificate. Initial baseline no-nesting is required separately. Actual cover must be measured to lie inside this directed envelope; current photo does not establish that condition.',
 full_symmetric_10mrad_adjacent_lf_lower_collision_witness=witness,rows=rows,elapsed_s=time.monotonic()-t0)
digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(',',':')).encode()).hexdigest()
a.output.write_text(json.dumps(dict(certificate=payload,certificate_sha256=digest),indent=2)+'\n');print(payload['status'],digest)
