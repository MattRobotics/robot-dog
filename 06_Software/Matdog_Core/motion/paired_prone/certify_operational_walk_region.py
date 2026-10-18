#!/usr/bin/env python3
"""Offline collision certificate of the continuous fixture WALK (2026-10-15).

Geometry only, no device access. The firmware admits the walk only if every
commanded target lies in the pinned per-joint union kTargetMinRad..kTargetMaxRad
(src/motion/OperationalWalkProfile.cpp: 20/30/40 mm presets, ramp-in, steady
cycles and coordinated stop, production FeedbackWalk + IK). This tool proves
that the whole BOX [min - tube, max + tube] over all 12 joints, every joint
independent (a superset of every walk trajectory, any timing, any lag inside
the tracking tube), is clear:
  A. the 120 non-adjacent canonical link pairs keep >= --clearance-mm
     (base_link pairs + --ventral-extra-mm), exact BVH distance at box centres
     minus a rigid-motion bound, recursive bisection (certify_measured_entry_region.Region);
  B. every adjacent revolute pair is continuously clear over its joint range
     (certify_adjacent_axis.certify).
The fixture, cables and the reprinted cover are NOT modelled.
"""
from pathlib import Path
import argparse,hashlib,json,math,platform,re,time
import numpy as np
import fcl
from paired_prone_model import PairModel,ROOT,LEGS
from certify_adjacent_axis import certify
from certify_measured_entry_region import Region,sha,URDF

PROFILE=ROOT/'05_Firmware/MATDOG_Controller/src/motion/OperationalWalkProfile.cpp'
PARTS=('hip','upper_leg','lower_leg')

def profile_source(profile):
    return PROFILE.with_name('StandHeightRegion.h') if profile=='height' else PROFILE

def pinned_ranges(profile='standard'):
    text=profile_source(profile).read_text()
    prefix='kHeight' if profile=='height' else 'kFast' if profile=='fast' else 'k'
    def arr(name):
        m=re.search(r'\b'+name+r'\[12\]\s*=\s*\{([^}]*)\}',text)
        if not m:raise SystemExit('pinned '+name+' not found')
        v=[float(x) for x in m.group(1).split(',')]
        if len(v)!=12 or not all(math.isfinite(x) for x in v):raise SystemExit('bad '+name)
        return v
    return arr(prefix+'TargetMinRad'),arr(prefix+'TargetMaxRad')

def main():
    a=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--tube-rad',type=float,default=.006)
    a.add_argument('--clearance-mm',type=float,default=3.);a.add_argument('--ventral-extra-mm',type=float,default=3.)
    a.add_argument('--pair-budget-s',type=float,default=600);a.add_argument('--adjacent-budget-s',type=float,default=180)
    a.add_argument('--output',type=Path,required=True)
    a.add_argument('--profile',choices=('standard','fast','height'),default='standard',help='which pinned box of OperationalWalkProfile.cpp (Issue #52: fast = EXPERIMENTAL 60 mm / 3 s)')
    args=a.parse_args()
    if args.output.exists():raise SystemExit('output exists; certificates are never overwritten')
    lo,hi=pinned_ranges(args.profile);m=PairModel();s=m.scene;box={};adjacent_domain={}
    for k,(leg,part) in enumerate((l,p) for l in LEGS for p in PARTS):
        j=f'{leg}_{part}_joint';joint=s.model.joints[j]
        box[j]=(lo[k]-args.tube_rad,hi[k]+args.tube_rad)
        if box[j][0]<joint.lower_limit_rad or box[j][1]>joint.upper_limit_rad:raise SystemExit(j+' walk box outside URDF limits')
        adjacent_domain[j]=box[j]
    r=Region(s,box);rows=[];t0=time.monotonic()
    for la,lb in s.model.clearance_pairs():
        req=args.clearance_mm*1e-3+(args.ventral_extra_mm*1e-3 if 'base_link' in (la,lb) else 0.)
        row=r.certify_pair(la,lb,req,args.pair_budget_s);rows.append(row)
        print(la,lb,row['status'],round(row.get('minimum_centre_distance_m',row.get('witness_distance_m',row.get('centre_distance_m',float('nan'))))*1e3,3),'mm',flush=True)
    adjacent=[]
    for j,(l,h) in adjacent_domain.items():
        res=certify(s,j,l,h,args.adjacent_budget_s);adjacent.append(dict(joint=j,domain_rad=[l,h],**res));print(j,res['status'],flush=True)
    passed=[x for x in rows if x['status']=='PASS']
    result=dict(schema='MATDOG_STAND_HEIGHT_REGION_CERTIFICATE_V1' if args.profile=='height' else 'MATDOG_OPERATIONAL_WALK_REGION_CERTIFICATE_V1',profile=args.profile,
        scope='whole independent pinned target box +/- tube; height targets guarded in AdaptiveStand' if args.profile=='height' else 'every configuration in the per-joint box of all commanded continuous-WALK targets (20/30/40 mm presets, ramp, cycles, stop) +/- tracking tube',
        box_rad={j:list(v) for j,v in box.items()},tracking_tube_rad=args.tube_rad,clearance_m=args.clearance_mm*1e-3,ventral_extra_m=args.ventral_extra_mm*1e-3,
        nonadjacent_pairs=len(rows),nonadjacent_pass=len(passed),adjacent_pass=sum(x['status']=='PASS_CONTINUOUS' for x in adjacent),
        minimum_centre_distance_m=min(x['minimum_centre_distance_m'] for x in passed) if passed else None,
        all_pass=len(passed)==len(rows) and all(x['status']=='PASS_CONTINUOUS' for x in adjacent),
        distance_calls=r.calls,elapsed_s=round(time.monotonic()-t0,1),
        identity=dict(urdf_sha256=sha(URDF),profile_sha256=sha(profile_source(args.profile)),tool_sha256=sha(Path(__file__)),
                      meshes={n:sha(s.model.collision_mesh_path(n)) for n in s.model.collision_link_names},
                      python=platform.python_version(),numpy=np.__version__),
        not_modelled=['fixture','cables','battery','reprinted cover beyond the ventral allowance','servo horn hardware'],
        hardware_authorized=False,motion_eligible=False,nonadjacent=rows,adjacent=adjacent)
    args.output.write_text(json.dumps(result,indent=1)+'\n')
    print('OPERATIONAL_WALK_REGION',('PASS' if result['all_pass'] else 'FAIL'),f"nonadjacent={result['nonadjacent_pass']}/{len(rows)}",
          f"adjacent={result['adjacent_pass']}/{len(adjacent)}",f"min_mm={result['minimum_centre_distance_m']*1e3 if passed else float('nan'):.2f}",flush=True)

if __name__=='__main__':main()
