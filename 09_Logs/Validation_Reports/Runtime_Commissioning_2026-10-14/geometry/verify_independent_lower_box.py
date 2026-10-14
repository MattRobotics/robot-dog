#!/usr/bin/env python3
"""120-pair continuous proof for independent LOWER progress during PRIME/ENTRY.
The entry's four LOWER joints need not share a progress parameter after PRIME.
HIP/UPPER remain within the full independent tube; LOWER each spans its entire
measured-start-to-route-head interval +/- tube. Lipschitz/FCL pairwise box proof
subdivides only the at-most-two non-common LOWER axes relevant to each pair.
"""
from pathlib import Path
import argparse,sys,math,hashlib,json,time
import numpy as np,fcl
p=argparse.ArgumentParser();p.add_argument('--joint-log',type=Path,required=True);p.add_argument('--repo',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--tube-rad',type=float,default=.004);a=p.parse_args()
if a.output.exists():raise SystemExit('preserve evidence')
sys.path.insert(0,str(a.repo/'06_Software/Matdog_Core/motion/paired_prone'))
from continuous_pair_path import Continuous
from certify_commissioning_entry import SLOTS,from_log,head
from check_assembly_nesting import components,inside
log=a.joint_log
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
c=Continuous();start=dict(zip(SLOTS,from_log(log)));goal=head(-88.5,-89);tube=a.tube_rad
if not 0<tube<=.01:raise SystemExit('tube domain')
lo={j:min(start[j],goal[j])-tube for j in SLOTS if j.endswith('lower_leg_joint')};hi={j:max(start[j],goal[j])+tube for j in lo}
center=dict(start);center.update({j:(lo[j]+hi[j])/2 for j in lo})
def model(t):
 v=t.reshape(-1,3);f=np.arange(len(v)).reshape(-1,3);b=fcl.BVHModel();b.beginModel(len(v),len(f));b.addSubModel(v,f);b.endModel();return b
models={n:model(c.s.mesh(n).triangles_local) for n in c.s.model.collision_link_names}
def place(n,q):
 tf=c.s.collision_transform(n,q);return fcl.CollisionObject(models[n],fcl.Transform(np.asarray(tf.rotation),np.asarray(tf.translation)))
reps={n:components(c.s.mesh(n).triangles_local)[0] for n in models}
rows=[];t0=time.monotonic()
for na,nb in c.s.model.clearance_pairs():
 chaina=c.s.model.relevant_actuated_joints_for_link(na);chainb=c.s.model.relevant_actuated_joints_for_link(nb);common=[]
 for ja,jb in zip(chaina,chainb):
  if ja!=jb:break
  common.append(ja)
 # Initial no nesting check for every complete closed collision component.
 # A positive surface separation over the connected joint box prevents a
 # component that begins outside from entering through another surface.
 ta=c.s.collision_transform(na,center);tb=c.s.collision_transform(nb,center)
 va=ta.apply_points(c.s.mesh(na).triangles_local.reshape(-1,3)).reshape(-1,3,3);vb=tb.apply_points(c.s.mesh(nb).triangles_local.reshape(-1,3)).reshape(-1,3,3)
 if np.any(va.min(axis=(0,1))>vb.max(axis=(0,1))) or np.any(vb.min(axis=(0,1))>va.max(axis=(0,1))):nested_a=[];nested_b=[]
 else:
  nested_a=[int(i) for i,x in enumerate(ta.apply_points(reps[na])) if inside(x,vb)]
  nested_b=[int(i) for i,x in enumerate(tb.apply_points(reps[nb])) if inside(x,va)]
 if nested_a or nested_b:raise SystemExit(f'initial nesting {na} {nb}')
 axes=[j for j in lo if j not in common and (j in c.radii[na] or j in c.radii[nb])]
 stack=[(dict((j,lo[j]) for j in axes),dict((j,hi[j]) for j in axes),0)];checks=0;leaves=0;minimum=math.inf;blocked=None
 while stack:
  left,right,depth=stack.pop();q=dict(center);q.update({j:(left[j]+right[j])/2 for j in axes})
  distance=fcl.distance(place(na,q),place(nb,q),fcl.DistanceRequest(),fcl.DistanceResult());checks+=1
  movement=sum(radius*((right[j]-left[j])/2 if j in axes else tube) for n in (na,nb) for j,radius in c.radii[n].items() if j not in common)
  bound=distance-movement-1e-9
  if bound>=.003:minimum=min(minimum,bound);leaves+=1;continue
  if distance<=0:blocked=dict(reason='MIDPOINT_SURFACE_COLLISION',q_rad=q,distance_m=distance);break
  if not axes or depth>=24:blocked=dict(reason='UNRESOLVED_BOX',lower_rad=left,upper_rad=right,distance_m=distance,movement_bound_m=movement);break
  joint=max(axes,key=lambda j:sum(c.radii[n].get(j,0) for n in (na,nb))*(right[j]-left[j]))
  mid=(left[joint]+right[joint])/2;first_right=dict(right);first_right[joint]=mid;second_left=dict(left);second_left[joint]=mid
  stack.extend(((left,first_right,depth+1),(second_left,right,depth+1)))
 row=dict(pair=[na,nb],pass_=blocked is None,checks=checks,leaves=leaves,axes=axes,minimum_continuous_separation_bound_m=None if math.isinf(minimum) else minimum,blocked=blocked,initial_nested_a=nested_a,initial_nested_b=nested_b);rows.append(row)
 print(len(rows),na,nb,row['pass_'],row['minimum_continuous_separation_bound_m'],flush=True)
payload=dict(schema='MATDOG_INDEPENDENT_LOWER_ENTRY_BOX_V1',status='PASS_INDEPENDENT_LOWER_BOX' if len(rows)==120 and all(x['pass_'] for x in rows) else 'BLOCKED',hardware_authorized=False,
 tube_rad=tube,start_rad=[start[j] for j in SLOTS],head_rad=[goal[j] for j in SLOTS],lower_domains_rad={j:[lo[j],hi[j]] for j in lo},
 hip_upper_domain='each independent +/-tube around measured start',required_nonadjacent_separation_m=.003,roundoff_m=1e-9,
 mesh_sha256={n:sha(c.s.model.collision_mesh_path(n)) for n in models},joint_log_sha256=sha(log),tool_sha256=sha(__file__),
 source_sha256={n:sha(a.repo/'06_Software/Matdog_Core/motion/paired_prone'/n) for n in ['continuous_pair_path.py','certify_commissioning_entry.py','check_assembly_nesting.py']},
 lipschitz_radius_m=c.radii,rows=rows,elapsed_s=time.monotonic()-t0,
 scope='Baseline-CAD 120 non-adjacent pairs. Each LOWER may progress independently anywhere in the full measured-start-to-head range plus tube. Separate entry adjacent and ventral certificates are still required. Physical uncertainty and fixtures are excluded.')
digest=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(',',':')).encode()).hexdigest();a.output.write_text(json.dumps(dict(certificate=payload,certificate_sha256=digest),indent=2)+'\n');print(payload['status'],digest)
