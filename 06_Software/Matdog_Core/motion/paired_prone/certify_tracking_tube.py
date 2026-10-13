#!/usr/bin/env python3
"""Qualify a frozen route suffix + independent twelve-joint error tube.
No target regeneration, calibrated-limit admission or hardware load claim.
"""
from pathlib import Path
import argparse,json,math,hashlib,os
if "MATDOG_PAIRED_OUTPUT" not in os.environ:raise SystemExit("Set MATDOG_PAIRED_OUTPUT to the preserved host adapter directory")
from pathlib import Path
from continuous_pair_path import Continuous
from paired_prone_model import ROOT,JOINTS
from certify_adjacent_axis import certify
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--tube-rad',type=float,default=.002);args=p.parse_args()
if args.output.exists():raise SystemExit('Preserve existing proof')
base=ROOT/'09_Logs/Validation_Reports/Motion_Night_Run_2026-10-12/continuous_pair_path.json';raw=base.read_bytes();d=json.loads(raw);c=Continuous()
review=json.loads((ROOT/'09_Logs/Validation_Reports/Motion_Hardware_Readiness_2026-10-13/joint_limits_review.json').read_text())
bounds={j['joint']:[math.radians(x) for x in j['proposal_operational_deg']] for j in review['joints']}
# URDF intersection is conservative and remains a proposal; no contact widening.
for j in JOINTS:
 model=c.s.model.joints[j];bounds[j]=[max(bounds[j][0],model.lower_limit_rad)+360/4096*math.pi/180,min(bounds[j][1],model.upper_limit_rad)-360/4096*math.pi/180]
first=None
for i in range(252):
 q=c.a.q(d['segments'][min(i,250)]['end_parameters' if i==251 else 'start_parameters'])
 if all(bounds[j][0]+args.tube_rad<=q[j]<=bounds[j][1]-args.tube_rad for j in JOINTS):first=i;break
report=dict(schema='MATDOG_TRACKING_TUBE_V1',route_sha256=hashlib.sha256(raw).hexdigest(),tube_rad=args.tube_rad,first_knot=first,last_knot=251,limits_approved=False,hardware_qualified=False,segments=[],adjacent=[])
args.output.write_text(json.dumps(report,indent=2)+'\n')
if first is None:raise SystemExit('No compatible entry')
for i,s in enumerate(d['segments'][first:],first):
 ok,r=c.self_proof(s['start_parameters'],s['end_parameters'],.003,args.tube_rad)
 report['segments'].append(dict(segment=i,pass_=ok,certificate=r));args.output.write_text(json.dumps(report,indent=2)+'\n')
 if i%20==0:print('tube',i,ok,flush=True)
 if not ok:report['status']='BLOCKED_NONADJACENT';break
else:
 for j in JOINTS:
  values=[c.a.q(s[k])[j] for s in d['segments'][first:] for k in ('start_parameters','end_parameters')]
  lo=min(values)-args.tube_rad;hi=max(values)+args.tube_rad
  r=certify(c.s,j,lo,hi,90);report['adjacent'].append(dict(joint=j,domain_rad=[lo,hi],**r));print(j,r['status'],flush=True)
  args.output.write_text(json.dumps(report,indent=2)+'\n')
 report['status']='PASS_GEOMETRY_TUBE' if all(x['status']=='PASS_CONTINUOUS' for x in report['adjacent']) else 'BLOCKED_ADJACENT'
report['scope']='Frozen suffix, rigid self-clearance >=3mm, independent +/-joint tube; ground/fixture/load, limits and bus tracking require separate evidence'
args.output.write_text(json.dumps(report,indent=2)+'\n');print(report['status'],first,flush=True)
