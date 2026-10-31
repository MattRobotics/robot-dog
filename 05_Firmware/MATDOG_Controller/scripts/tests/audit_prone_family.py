#!/usr/bin/env python3
"""Independent finite geometry witnesses, not an execution qualification.

Rechecks initial/failure/last-clear poses from bounded diagnostic probes with
all 17 canonical meshes. Records exact or lower-bound distances as labelled by
V5; never substitutes an AABB overlap for a signed mesh penetration depth.
"""
from dataclasses import asdict
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
from render_motion_recovery import RobotSceneV5, URDF, ROOT, pose, body, JOINTS


def evaluate(scene, row):
    q=pose(row);r,t=body(row)
    ground={}
    for link in scene.model.collision_link_names:
        points=scene.world_broad_phase(link,q).hull_vertices_world@r.T+t
        index=int(np.argmin(points[:,2]))
        ground[link]=dict(minimum_z_m=float(points[index,2]),
                          lowest_vertex_world_m=points[index].tolist(),
                          expected_belly_contact=link=='base_link')
    collision,pair=scene.first_collision_at_pose(q)
    a,b,result=scene.worst_pair_at_pose(q)
    return dict(q_rad=q,base_position_m=t.tolist(),rotation_world_from_base=r.tolist(),
                plane_world='Z=0',plane_by_link=ground,mesh_collision=collision,
                first_intersection_pair=pair,nonadjacent_worst_pair=asdict(result),
                signed_penetration_depth_m=None,
                contact_load_unknown=True,continuous_clearance_qualified=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--family',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();scene=RobotSceneV5.from_urdf(URDF)
    family=json.loads(args.family.read_text());reports=[];cache={}
    for candidate in family['candidates']:
        trace=args.family.parent/candidate['trace_file']
        rows=[json.loads(s) for s in trace.read_text().splitlines()]
        poses={}
        for name,index in (('initial',0),('last_before_failure',max(0,len(rows)-2)),('failure',len(rows)-1)):
            row=rows[index];key=json.dumps([row['telemetry'][k] for k in ('actual_rad','quaternion','base_position_m')])
            if key not in cache:cache[key]=evaluate(scene,row)
            poses[name]=dict(frame=index,**cache[key])
        values=np.asarray([row['telemetry']['actual_rad'] for row in rows])
        dt=np.diff([row['now_ms']/1000 for row in rows]);rates=np.diff(values,axis=0)/dt[:,None]
        reports.append(dict(id=candidate['id'],trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
                            evaluated_poses=poses,
                            diagnostic_sample_max_abs_rate_rad_s=np.max(np.abs(rates),axis=0).tolist(),
                            timing_is_approved_execution_timing=False,
                            runtime_ik_branch_and_limits_qualified=False,
                            accepted_complete_prone_to_stand=False))
        print(candidate['id'],poses['failure']['plane_by_link']['rh_lower_leg_link']['minimum_z_m'],flush=True)
    result=dict(schema='MATDOG_PRONE_INDEPENDENT_WITNESSES_V1',hardware_access=False,
                physical_readiness=False,link_count=len(scene.model.links),collision_mesh_count=len(scene.model.collision_link_names),
                nonadjacent_pairs=len(scene.model.clearance_pairs()),
                obstruction_pairs=len(scene.model.path_obstruction_pairs()),
                urdf_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest(),
                meshes={name:dict(path=str(scene.model.collision_mesh_path(name).relative_to(ROOT)),
                                 sha256=hashlib.sha256(scene.model.collision_mesh_path(name).read_bytes()).hexdigest())
                        for name in scene.model.collision_link_names},
                sampling_cannot_prove_continuous_clearance=True, candidates=reports)
    args.output.write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
