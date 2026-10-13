#!/usr/bin/env python3
"""Named suspended derivative of frozen knots. No loaded/ground proof.
Defect: canonical LOWER -92deg has zero URDF margin and exceeds gen-1
candidate contact insets. Replacing the two LOWER endpoints before quintic
interpolation preserves C2 knot joins; original candidate remains untouched.
"""
from pathlib import Path
import argparse,json,hashlib,math
from continuous_pair_path import Continuous
from paired_prone_model import ROOT,JOINTS
from certify_adjacent_axis import certify
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--tube-rad',type=float,default=.004);p.add_argument('--front-floor-deg',type=float,default=-89);p.add_argument('--rear-floor-deg',type=float,default=-89.5);a=p.parse_args();assert not a.output.exists()
source=ROOT/'09_Logs/Validation_Reports/Motion_Night_Run_2026-10-12/continuous_pair_path.json';raw=source.read_bytes();d=json.loads(raw)
recipe=dict(schema='MATDOG_FIXTURE_LIMIT_COMPATIBLE_V1',canonical_route_sha256=hashlib.sha256(raw).hexdigest(),front_lower_floor_deg=a.front_floor_deg,rear_lower_floor_deg=a.rear_floor_deg,method='floor each LOWER knot, then original quintic; HIP/UPPER/phase/duration unchanged',scope='FIXTURE_ONLY_NO_GROUND_OR_LOAD_CLAIM')
encoded=json.dumps(recipe,sort_keys=True,separators=(',',':')).encode();route_sha=hashlib.sha256(encoded).hexdigest();c=Continuous();poses=[]
for i in range(252):
 par=dict(d['segments'][min(i,250)]['end_parameters' if i==251 else 'start_parameters']);par['lf']=max(par['lf'],math.radians(a.front_floor_deg));par['lr']=max(par['lr'],math.radians(a.rear_floor_deg));poses.append(c.a.q(par))
report=dict(schema='MATDOG_FIXTURE_LIMIT_ROUTE_CERTIFICATE_V1',recipe=recipe,recipe_sha256=route_sha,tube_rad=a.tube_rad,first_knot=0,last_knot=251,segments=[],adjacent=[],status='IN_PROGRESS',hardware_qualified=False,limits_approved=False,ground_certified=False)
a.output.write_text(json.dumps(report,indent=2)+'\n')
for i in range(251):
 ok,r=c.self_proof_q(poses[i],poses[i+1],.003,a.tube_rad);report['segments'].append(dict(segment=i,pass_=ok,certificate=r));a.output.write_text(json.dumps(report,indent=2)+'\n')
 if i%20==0:print('fixture_limit',i,ok,flush=True)
 if not ok:report['status']='BLOCKED_NONADJACENT';break
else:
 for j in JOINTS:
  lo=min(x[j] for x in poses)-a.tube_rad;hi=max(x[j] for x in poses)+a.tube_rad;r=certify(c.s,j,lo,hi,90);report['adjacent'].append(dict(joint=j,domain_rad=[lo,hi],**r));print(j,r['status'],flush=True);a.output.write_text(json.dumps(report,indent=2)+'\n')
 report['status']='PASS_FIXTURE_ROUTE_TUBE' if all(x['status']=='PASS_CONTINUOUS' for x in report['adjacent']) else 'BLOCKED_ADJACENT'
report['joint_target_ranges_rad']=[[min(x[j] for x in poses),max(x[j] for x in poses)] for j in JOINTS]
report['scope']='Rigid self-clearance >=3mm plus independent twelve-joint reviewed tube; canonical adjacent triangles include assembly interfaces. Requires separate fixed-fixture free-volume, gen-1 current limits, mapping/mechanical uncertainty, timing, drive, tracking and operator proofs. Ground/support/load never asserted.'
a.output.write_text(json.dumps(report,indent=2)+'\n');print(report['status'],route_sha,flush=True)
