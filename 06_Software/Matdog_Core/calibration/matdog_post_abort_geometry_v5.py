#!/usr/bin/env python3
"""Offline CAD check of the post-ABORT reverse paths; no device access.

Reuses Geometry V5 scene and existing sequence poses/corridors. A probed
joint's intended parent/child stop pair is excluded exactly as in the existing
sequence validator; every other obstruction pair is checked at <=0.5 degree.
Sampling is CAD evidence, not a physical clearance qualification.
"""
import argparse
import json
import math
from pathlib import Path
from matdog_full_calibration_sequence_geometry_v5 import (
    Checker, RobotSceneV5, URDF, EXPECTED_URDF_SHA256, sha256,
    leg_joints, joint_limits, active_pair, TICK, FRONT_REAR, V5_REAR_UPPER_PARK_RAD)


def validate(checker, leg, phase, upper_offset=0, park_offset=0, hip_offset=0, lower_offset=0):
    joints=leg_joints(leg)
    hip, upper, lower=(joints[k] for k in ('hip','upper','lower'))
    rear=FRONT_REAR.get(leg)
    base={hip:hip_offset*TICK,upper:0.,lower:lower_offset*TICK}
    if rear: base[f'{rear}_upper_leg_joint']=V5_REAR_UPPER_PARK_RAD+park_offset*TICK
    results=[]
    moving=lower if phase=='LOWER' else upper
    if phase=='LOWER': base[upper]=math.pi/2+upper_offset*TICK
    # Return from EITHER guard to Q0 with the same known prerequisite pose.
    lo,hi=joint_limits(checker.model,moving)
    for side,start in [('MIN',lo-64*TICK),('MAX',hi+64*TICK)]:
        result=checker.sweep(base,moving,start,0.,math.radians(.5),active_pair(checker.model,moving))
        results.append({'leg':leg,'phase':phase,'side':side,
                        'dependency_offsets':{'hip':hip_offset,'upper':upper_offset,
                                              'lower':lower_offset,'park':park_offset},'sweep':result})
    if phase=='LOWER':
        base[lower]=0.
        results.append({'leg':leg,'phase':'RETURN_UPPER','sweep':checker.sweep(base,upper,base[upper],0.,math.radians(.5),active_pair(checker.model,upper))})
    if rear:
        base[upper]=0.
        park=f'{rear}_upper_leg_joint'
        results.append({'leg':leg,'phase':'RESTORE_PARK','sweep':checker.sweep(base,park,base[park],0.,math.radians(.5),None)})
    return results


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--corners',action='store_true');args=ap.parse_args()
    assert sha256(URDF)==EXPECTED_URDF_SHA256
    checker=Checker(RobotSceneV5.from_urdf(URDF));results=[]
    for leg in ['lf','rf','rh','lh']:
        for phase in ['UPPER','LOWER']:
            results+=validate(checker,leg,phase)
            if args.corners:
                for hip_offset in [-10,10]:
                    for dependency_offset in [-10,10]:
                        for park_offset in ([-10,10] if leg in FRONT_REAR else [0]):
                            results+=validate(checker,leg,phase,
                                upper_offset=dependency_offset if phase=='LOWER' else 0,
                                lower_offset=dependency_offset if phase=='UPPER' else 0,
                                park_offset=park_offset,hip_offset=hip_offset)
    # The exact observed RF/RH residual pose, followed by its dependency order.
    base={'rf_hip_joint':0.,'rf_upper_leg_joint':1026*TICK,
          'rf_lower_leg_joint':-351*TICK,'rh_upper_leg_joint':393*TICK}
    for joint in ['rf_lower_leg_joint','rf_upper_leg_joint','rh_upper_leg_joint']:
        result=checker.sweep(base,joint,base[joint],0.,math.radians(.5),
                             None if joint.startswith('rh_') else active_pair(checker.model,joint))
        results.append({'leg':'rf','phase':'OBSERVED_20261003','joint':joint,'sweep':result});base[joint]=0.
    passed=all(r['sweep']['status']=='COLLISION_FREE' for r in results)
    Path(args.output).write_text(json.dumps({'urdf_sha256':EXPECTED_URDF_SHA256,'step_deg':.5,
        'evaluations':checker.evaluations,'passed':passed,'paths':results},indent=2)+'\n')
    print(f'post_abort_geometry: {len(results)} paths, {checker.evaluations} poses, passed={passed}',flush=True)
    return 0 if passed else 1
if __name__=='__main__': raise SystemExit(main())
