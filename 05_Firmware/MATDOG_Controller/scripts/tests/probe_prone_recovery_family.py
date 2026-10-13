#!/usr/bin/env python3
"""Bounded MATDOG anatomical acquisition experiment, not a runtime motion permit.

Three explicit synthetic folds; two prescribed rear-first/coordinated recipes.
There is no 12-DOF optimizer and no identification of an actual physical pose.
Upper horizontal derives u=pi/2. Lower is near its URDF fold boundary, with
three disclosed research margins. Mirrored HIP angles solve actual STL/ground
constraints. Failure stops that candidate and records its exact witness.
"""
from pathlib import Path
from dataclasses import asdict
import argparse
import hashlib
import json
import math
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / '06_Software/Matdog_Core/calibration'))
from matdog_geometry_scene_v5 import RobotSceneV5, _rotation_from_rpy
from render_motion_recovery import anatomy, JOINTS

URDF = ROOT / '03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
LEGS = ('lf', 'rf', 'rh', 'lh')


def minimum_z(scene, q, pitch, height, links):
    up = _rotation_from_rpy((0, pitch, 0))[2]
    return min(float((scene.world_broad_phase(link, q).hull_vertices_world @ up).min())+height for link in links)


def body_height(scene, pitch):
    return -minimum_z(scene, scene.home_pose(), pitch, 0, ('base_link',))


def hip_for_ground(scene, q, leg, pitch, height):
    links = tuple(name for name in scene.model.collision_link_names if name.startswith(leg+'_'))
    key = leg+'_hip_joint';sign = 1 if leg in ('lf', 'lh') else -1
    limit = min(abs(scene.model.joints[key].lower_limit_rad), scene.model.joints[key].upper_limit_rad)
    # Bounded bracketing of this single scalar; no monotonicity assumption.
    previous = 0.
    for value in np.linspace(0, limit, 33):
        q[key] = sign*float(value)
        if minimum_z(scene, q, pitch, height, links) >= -1e-9:
            lo, hi = previous, float(value)
            for _ in range(26):
                mid=(lo+hi)/2;q[key]=sign*mid
                if minimum_z(scene,q,pitch,height,links)>=-1e-9:hi=mid
                else:lo=mid
            q[key]=sign*hi
            return True
        previous=float(value)
    return False


def row(scene, q, pitch, height, frame, phase):
    r = _rotation_from_rpy((0,pitch,0))
    feet={}
    for leg in LEGS:
        transform=scene.collision_transform(leg+'_foot_link',q)
        # Actual cylindrical lowest strip/core point, not an invented load.
        vertices=scene.world_broad_phase(leg+'_foot_link',q).hull_vertices_world
        index=int(np.argmin(vertices@r[2]))
        point=r@vertices[index]+np.array((0,0,height))
        feet[leg.upper()]={'position_m':point.tolist(),'state':'CANDIDATE_CONTACT' if abs(point[2])<=1e-5 else 'SWING'}
    return {'version':1,'now_ms':frame*40,'stand_state':'ACQUIRE_SUPPORT','acquisition_phase':phase,
            'initial_pose_provenance':'SYNTHETIC_GEOMETRIC_CANDIDATE: horizontal upper; URDF near-fold margin; STL-plane HIP root, not measured',
            'telemetry':{'actual_rad':[q[name] for name in JOINTS],'target_rad':[q[name] for name in JOINTS],
              'quaternion':[math.cos(pitch/2),0,math.sin(pitch/2),0],'base_position_m':[0,0,height],
              'base_height_m':height,'feet':feet,'tracking_error_rad':0,'fault':None,'stamp_ms':frame*40},
            'reason':'GEOMETRIC_PROBE_NOT_RUNTIME_APPROVED;CONTACT_LOAD_UNKNOWN'}


def witness(scene, q, pitch, height, frame, phase):
    collision,pair=scene.first_collision_at_pose(q)
    if not collision:return None
    result=scene.check_link_pair(*pair,q,require_distance=True)
    transforms={}
    for link in pair:
        tf=scene.collision_transform(link,q)
        transforms[link]={'rotation_base_from_collision':tf.rotation.tolist(),'translation_base_from_collision_m':tf.translation.tolist()}
    return dict(frame=frame,phase=phase,pitch_rad=pitch,base_link_z_m=height,q_rad=q.copy(),
                pair_result=asdict(result),transforms=transforms,
                signed_penetration_depth_m=None,reason='Canonical V5 intersection; signed depth not estimated')


def ground_witness(scene, q, leg, pitch, height):
    r = _rotation_from_rpy((0, pitch, 0))
    values = {}
    for link in scene.model.collision_link_names:
        if link.startswith(leg+'_'):
            vertices = scene.world_broad_phase(link, q).hull_vertices_world
            index = int(np.argmin(vertices @ r[2]))
            values[link] = dict(minimum_z_m=float(vertices[index] @ r[2]+height),
                                lowest_world_vertex_m=(r @ vertices[index]+[0, 0, height]).tolist())
    return dict(support_plane='world Z=0', hip_at_urdf_limit_rad=q[leg+'_hip_joint'],
                link_plane_distances=values, assessment='FAILED at limiting HIP; not an exhaustive signed-HIP search')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--rock-before-deploy', action='store_true',
                        help='One bounded correction: belly rock precedes unfolding, only fold margin .03')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    scene=RobotSceneV5.from_urdf(URDF)
    reports=[]
    # These are research hypotheses, not operating limits or captured readings.
    for margin in ((.03,) if args.rock_before_deploy else (.03,.08,.13)):
        base={}
        for leg in LEGS:
            base[leg+'_hip_joint']=0.
            base[leg+'_upper_leg_joint']=math.pi/2
            base[leg+'_lower_leg_joint']=scene.model.joints[leg+'_lower_leg_joint'].lower_limit_rad+margin
        initial_height=body_height(scene,0);height=initial_height
        initial=True;failed_leg=None
        for leg in LEGS:
            if not hip_for_ground(scene,base,leg,0,height):initial=False;failed_leg=leg;break
        initial_row=row(scene,base,0,height,0,'C0_BELLY_SUPPORTED')
        initial_collision=witness(scene,base,0,height,0,'C0_BELLY_SUPPORTED') if initial else None
        for recipe in ('rear_first','coordinated'):
            result={'id':f'{recipe}_fold_margin_{margin:.2f}','fold_margin_rad':margin,
                    'initial_angles_rad':initial_row['telemetry']['actual_rad'],'initial_anatomy':anatomy(scene,initial_row),
                    'initial_pose_measured':False,'body_height_from_underside_m':height,'initial_ground_feasible':initial,
                    'contact_load_proven':False,'runtime_approved':False,'trajectory_geometric_pass':False,
                    'rock_precedes_deploy':args.rock_before_deploy}
            result['body_height_from_underside_m']=initial_height
            rows=[initial_row];failure=initial_collision
            if not initial:failure={'phase':'C0_BELLY_SUPPORTED','frame':0,'failed_leg':failed_leg,'reason':'No ground-clear HIP root within URDF range','q_rad':base.copy()}
            if initial and not failure:
                # Two anatomically prescribed clocks, bounded to 51 frames each.
                # Belly rocking raises the lower rear hip from its 20-mm deficit.
                for index in range(1,52):
                    u=index/51;s=u*u*u*(10-15*u+6*u*u)
                    pitch=-.15*min(1.,s*3)
                    deploy=max(0.,(s-1/3)/(2/3)) if args.rock_before_deploy else s
                    height=body_height(scene,pitch);q=base.copy()
                    for leg in LEGS:
                        is_rear=leg in ('rh','lh')
                        clock=deploy if is_rear else (max(0.,(deploy-.65)/.35) if recipe=='rear_first' else max(0.,(deploy-.25)/.75))
                        clock=min(1.,clock)
                        q[leg+'_upper_leg_joint']=math.pi/2+clock*(scene.model.joints[leg+'_upper_leg_joint'].upper_limit_rad-.05-math.pi/2)
                        q[leg+'_lower_leg_joint']=base[leg+'_lower_leg_joint']+clock*(scene.model.joints[leg+'_lower_leg_joint'].upper_limit_rad-.05-base[leg+'_lower_leg_joint'])
                        if not hip_for_ground(scene,q,leg,pitch,height):
                            failure={'frame':index,'phase':'C1_REAR_DEPLOY' if is_rear else 'C2_FRONT_DEPLOY','failed_leg':leg,
                                     'reason':'No outward ground-clear HIP root within URDF range','pitch_rad':pitch,'base_link_z_m':height,'q_rad':q.copy(),
                                     'ground_witness':ground_witness(scene,q,leg,pitch,height)};break
                    phase='C1_REAR_DEPLOY' if s<.25 else 'C2_FRONT_DEPLOY'
                    record=row(scene,q,pitch,height,index,phase)
                    if not failure:failure=witness(scene,q,pitch,height,index,phase)
                    record['geometric_frame_accepted']=not bool(failure)
                    rows.append(record)
                    if failure:break
            result['evaluated_frames']=len(rows);result['failure']=failure
            if not failure:result['failure']={'reason':'Deployment probe completed, but C3 load-transfer/C4 bridge and runtime tilted-acquisition integration are not yet qualified'}
            output=args.output/(result['id']+'.jsonl')
            output.write_text(''.join(json.dumps(r,separators=(',',':'))+'\n' for r in rows))
            result['trace_file']=output.name;result['trace_sha256']=hashlib.sha256(output.read_bytes()).hexdigest()
            reports.append(result)
            print(result['id'],result['evaluated_frames'],result['failure']['reason'],flush=True)
    result={'schema':'MATDOG_PRONE_BOUNDED_GEOMETRIC_FAMILY_V1','hardware_access':False,'physical_readiness':False,
            'search_dimensions':'one disclosed fold margin; mirrored HIP roots; prescribed rear/front clocks and bounded belly pitch',
            'upper_horizontal_constraint_rad':math.pi/2,'recipes':2,'initial_folds':len(reports)//2,'maximum_frames_per_recipe':52,
            'angles_are_measured':False,'candidate_acceptance_requires_ik_rates_continuity_full_collision_and_support':True,
            'candidates':reports}
    (args.output/'bounded_family.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
