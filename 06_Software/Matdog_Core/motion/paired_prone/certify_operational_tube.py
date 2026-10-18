#!/usr/bin/env python3
"""Continuous fixture route boxes using the existing exact-FCL Region checker.
Every straight polyline segment is enclosed with the full independent joint
error tube. Recursive cells, not discrete poses, establish each PASS. Reuses
meshes/BVH and proves adjacent full ranges once. No physical clearance change.
"""
import argparse,json,math,time,hashlib
from pathlib import Path
from certify_measured_entry_region import Region,PairModel,sha,URDF,ROUTE
from certify_adjacent_axis import certify
from paired_prone_model import LEGS

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--tube-rad',type=float,required=True);a=p.parse_args()
 if a.output.exists():raise SystemExit('preserve previous certificate')
 if not math.isfinite(a.tube_rad) or not 0<a.tube_rad<=.050:raise SystemExit('tube domain')
 source=json.loads(ROUTE.read_text());scene=PairModel().scene;poses=[]
 for i in range(252):
  v=dict(source['segments'][min(i,250)]['end_parameters' if i==251 else 'start_parameters'])
  v['lf']=max(v['lf'],math.radians(-88.5));v['lr']=max(v['lr'],math.radians(-89.))
  q={}
  for l in LEGS:
   q[l+'_hip_joint']=v['h']*(-1 if l in ('lf','lh') else 1)
   q[l+'_upper_leg_joint']=v['uf' if l in ('lf','rf') else 'ur'];q[l+'_lower_leg_joint']=v['lf' if l in ('lf','rf') else 'lr']
  poses.append(q)
 full={j:(min(q[j] for q in poses)-a.tube_rad,max(q[j] for q in poses)+a.tube_rad) for j in poses[0]}
 r=Region(scene,full);rows=[];start=time.monotonic()
 for i,(qa,qb) in enumerate(zip(poses,poses[1:])):
  r.box={j:(min(qa[j],qb[j])-a.tube_rad,max(qa[j],qb[j])+a.tube_rad) for j in qa}
  pairs=[]
  for la,lb in scene.model.clearance_pairs():
   # Same physical requirement as the original fixture-route certificate.
   row=r.certify_pair(la,lb,.003,30);pairs.append(row)
   if row['status']!='PASS':print('BLOCKED',i,row,flush=True);break
  rows.append(dict(segment=i,pass_=len(pairs)==120 and all(x['status']=='PASS' for x in pairs),pairs=pairs))
  print('SEGMENT',i,rows[-1]['pass_'],'calls',r.calls,flush=True)
  if not rows[-1]['pass_']:break
 adj=[]
 if len(rows)==251 and all(x['pass_'] for x in rows):
  for j,(lo,hi) in full.items():
   v=certify(scene,j,lo,hi,180);adj.append(dict(joint=j,domain_rad=[lo,hi],**v));print(j,v['status'],flush=True)
 result=dict(schema='MATDOG_OPERATIONAL_ROUTE_BOX_CERTIFICATE_V1',tube_rad=a.tube_rad,
  all_pass=len(rows)==251 and all(x['pass_'] for x in rows) and len(adj)==12 and all(x['status']=='PASS_CONTINUOUS' for x in adj),
  minimum_required_m=.003,segments=rows,adjacent=adj,elapsed_s=time.monotonic()-start,distance_calls=r.calls,
  identity=dict(urdf_sha256=sha(URDF),route_sha256=sha(ROUTE),tool_sha256=sha(__file__),region_tool_sha256=sha(Path(__file__).with_name('certify_measured_entry_region.py')),scene_sha256=sha(Path(__file__).parents[2]/'calibration/matdog_geometry_scene_v5.py'),meshes={n:sha(scene.model.collision_mesh_path(n)) for n in scene.model.collision_link_names}),
  hardware_authorized=False,not_modelled=['fixture','cables','modified cover','encoder absolute accuracy','loads'])
 a.output.write_text(json.dumps(result,indent=1)+'\n');print('ROUTE_ALL_PASS',result['all_pass'],flush=True)
if __name__=='__main__':main()
