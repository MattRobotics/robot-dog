#!/usr/bin/env python3
"""Targeted startup-path qualification; no transport or hardware access.

Non-adjacent pairs: adaptive boxes, mesh clearance + conservative vertex
movement bound cover the continuous box, including all support tolerances.
Common ancestor rotations cancel. Unresolved boxes fail closed.
Revolute stop pairs: existing Geometry V5 joint-domain evidence plus a dense
one-dimensional collision sweep; fixed-adjacent pairs follow the V5 model.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from matdog_full_calibration_sequence_geometry_v5 import (
    Checker, RobotSceneV5, URDF, EXPECTED_URDF_SHA256, sha256, TICK)
from matdog_geometry_mesh_kernel import (
    narrow_phase_check, MeshKernelError, PairCollisionResult,
    _triangle_grid_index, _iter_candidate_batches,
    DEFAULT_MAX_NARROW_PHASE_CANDIDATE_PAIRS)
from matdog_geometry_model_v5 import PAIR_CLASS_REVOLUTE_ADJACENT
from matdog_geometry_safety_policy import REFERENCE_CLEARANCE_THRESHOLD_M

SUPPORT_TICKS = 10
STAGES = [('RF_LOWER', {'rf_lower_leg_joint':(-10,367),
                       'rf_upper_leg_joint':(1014,1034), 'rh_upper_leg_joint':(388,408)}),
          ('RF_UPPER', {'rf_upper_leg_joint':(-10,1034),'rh_upper_leg_joint':(388,408)}),
          ('RH_UPPER', {'rh_upper_leg_joint':(-10,408)})]


def triangle_separation_bounds(a, b):
    """Conservative surface-distance bounds for batches of triangle pairs.

    Every projection gap is a lower bound, even if the chosen axes miss the
    closest direction. No optimiser or approximate distance is trusted.
    """
    gap=np.maximum(np.maximum(a.min(axis=1)-b.max(axis=1),
                              b.min(axis=1)-a.max(axis=1)),0.)
    bound=np.linalg.norm(gap,axis=1)
    ea=np.roll(a,-1,axis=1)-a;eb=np.roll(b,-1,axis=1)-b
    na=np.cross(ea[:,0],ea[:,1]);nb=np.cross(eb[:,0],eb[:,1])
    axes=[na,nb]
    axes += [np.cross(ea[:,i],eb[:,j]) for i in range(3) for j in range(3)]
    axes += [np.cross(n,e[:,i]) for n,e in [(na,ea),(nb,eb)] for i in range(3)]
    for axis in axes:
        length=np.linalg.norm(axis,axis=1)
        unit=np.divide(axis,length[:,None],out=np.zeros_like(axis),where=length[:,None]>1e-15)
        pa=np.einsum('nij,nj->ni',a,unit);pb=np.einsum('nij,nj->ni',b,unit)
        projection=np.maximum(np.maximum(pa.min(axis=1)-pb.max(axis=1),
                                          pb.min(axis=1)-pa.max(axis=1)),0.)
        bound=np.maximum(bound,projection)
    return np.maximum(0.,bound-1e-9)


def mesh_separation_bound(scene,pair,pose,margin):
    triangles=[scene.collision_transform(l,pose).apply_points(
        scene.mesh(l).triangles_local.reshape(-1,3)).reshape(-1,3,3) for l in pair]
    grids=[_triangle_grid_index(t,.010,margin/2) for t in triangles]
    bound=margin
    for ia,ib in _iter_candidate_batches(*grids,DEFAULT_MAX_NARROW_PHASE_CANDIDATE_PAIRS,
                                          'startup '+str(pair),4096):
        bound=min(bound,float(triangle_separation_bounds(triangles[0][ia],triangles[1][ib]).min()))
        if bound==0.:break
    return PairCollisionResult(link_a=pair[0],link_b=pair[1],status='SEPARATED_NARROW',
                               clearance_m=bound,clearance_kind='LOWER_BOUND')


def radius(scene, link, joint):
    chain=scene.model.joint_chain_by_link[link]
    downstream=chain[chain.index(joint)+1:]
    mesh=scene.mesh(link)
    return (float(np.linalg.norm(mesh.hull_vertices_local,axis=1).max()) +
            float(np.linalg.norm(scene.model.collision_geometry(link).origin_xyz)) +
            sum(float(np.linalg.norm(scene.model.joint(j).origin_xyz)) for j in downstream))


class ContinuousCheck:
    def __init__(self, scene):
        self.scene=scene;self.boxes=0;self.refinements=0;self.minimum_certified=float('inf')
    def check(self, pair, bounds, max_boxes=30000):
        relevant=self.scene.model.relevant_actuated_joints_for_pair(*pair)
        radii={j:sum(radius(self.scene,l,j) for l in pair
                     if j in self.scene.model.relevant_actuated_joints_for_link(l)) for j in relevant}
        pending=[({j:bounds[j] for j in relevant},0)];start=self.boxes
        while pending:
            box,depth=pending.pop();self.boxes+=1
            pose=self.scene.full_pose({j:(lo+hi)*TICK/2 for j,(lo,hi) in box.items()})
            # Rotation of a vertex through |dq| moves it at most r*|dq|.
            terms={j:radii[j]*(hi-lo)*TICK/2 for j,(lo,hi) in box.items()}
            movement=sum(terms.values())
            result=self.scene.check_link_pair(*pair,pose,require_distance=False)
            if result.status=='INTERSECTING':
                return {'status':'COLLISION','pair':pair,'ticks':{j:(lo+hi)/2 for j,(lo,hi) in box.items()}}
            clearance=result.clearance_m
            if clearance is None or clearance-movement < REFERENCE_CLEARANCE_THRESHOLD_M:
                # Broad-phase figures can be lower bounds. Refine; do not
                # call a lower bound below threshold an exact collision.
                margin=min(.030,max(.005,movement+REFERENCE_CLEARANCE_THRESHOLD_M+.0001))
                try:
                    refined=mesh_separation_bound(self.scene,pair,pose,margin)
                    if movement < .0001 and refined.clearance_m < REFERENCE_CLEARANCE_THRESHOLD_M:
                        refined=narrow_phase_check(self.scene.mesh(pair[0]),self.scene.collision_transform(pair[0],pose),
                            self.scene.mesh(pair[1]),self.scene.collision_transform(pair[1],pose),margin_m=.005)
                except MeshKernelError:
                    # The unchanged kernel's resource guard is not a collision
                    # result. Subdivide; an exhausted budget remains UNRESOLVED.
                    refined=None
                self.refinements+=1
                if refined is not None: result=refined;clearance=result.clearance_m
                if result.status=='INTERSECTING':
                    return {'status':'COLLISION','pair':pair,'ticks':{j:(lo+hi)/2 for j,(lo,hi) in box.items()}}
            if clearance is not None and clearance-movement >= REFERENCE_CLEARANCE_THRESHOLD_M:
                self.minimum_certified=min(self.minimum_certified,clearance-movement);continue
            if result.clearance_kind=='EXACT' and clearance < REFERENCE_CLEARANCE_THRESHOLD_M:
                return {'status':'CLEARANCE_BELOW_POLICY','pair':pair,'clearance_m':clearance,
                        'ticks':{j:(lo+hi)/2 for j,(lo,hi) in box.items()}}
            if depth>=24 or self.boxes-start>=max_boxes or not terms:
                return {'status':'UNRESOLVED','pair':pair,'clearance_kind':result.clearance_kind,
                        'clearance_m':clearance,'movement_bound_m':movement,'depth':depth}
            joint=max(terms,key=terms.get);lo,hi=box[joint];mid=(lo+hi)/2
            for interval in [(lo,mid),(mid,hi)]:
                child=dict(box);child[joint]=interval;pending.append((child,depth+1))
        return {'status':'PASS_CONTINUOUS_BOX','pair':pair,'boxes':self.boxes-start}


def qualify():
    assert sha256(URDF)==EXPECTED_URDF_SHA256
    scene=RobotSceneV5.from_urdf(URDF);checker=Checker(scene);continuous=ContinuousCheck(scene)
    outcomes=[]
    def result():
        return {'schema':'MATDOG_STARTUP_GEOMETRY_V1','status':'PASS' if len(outcomes)>0 and all(o['status'].startswith('PASS') for o in outcomes) else 'BLOCKED',
                'urdf_sha256':EXPECTED_URDF_SHA256,'clearance_policy_m':REFERENCE_CLEARANCE_THRESHOLD_M,
                'support_tolerance_ticks':SUPPORT_TICKS,'rf_lower_direction':1,
                'continuous_boxes':continuous.boxes,'narrow_refinements':continuous.refinements,
                'minimum_certified_clearance_m':continuous.minimum_certified if math.isfinite(continuous.minimum_certified) else None,
                'stages':STAGES,'outcomes':outcomes,'hardware_io':False}
    for name,overrides in STAGES:
        bounds={j:(-SUPPORT_TICKS,SUPPORT_TICKS) for j in scene.model.actuated_joint_names};bounds.update(overrides)
        for j,(lo,hi) in bounds.items():
            spec=scene.model.joint(j)
            if lo*TICK<spec.lower_limit_rad or hi*TICK>spec.upper_limit_rad:
                outcomes.append({'stage':name,'status':'OUTSIDE_URDF_LIMIT','joint':j});return result()
        pairs=scene.model.path_obstruction_pairs()
        # Resolve the close RF/RH dependency first; unchanged stop pairs need
        # collision booleans, not expensive exact surface-distance searches.
        pairs=sorted(pairs,key=lambda p: 0 if set(p)=={'rf_foot_link','rh_lower_leg_link'} else 1)
        for pair in pairs:
            print(name,pair,'CHECK',flush=True)
            if scene.model.classify_pair(*pair)==PAIR_CLASS_REVOLUTE_ADJACENT:
                relevant=scene.model.relevant_actuated_joints_for_pair(*pair)
                assert len(relevant)==1
                j=relevant[0];lo,hi=bounds[j]
                steps=max(1,math.ceil((hi-lo)*TICK/math.radians(.25)))
                swept={'status':'COLLISION_FREE','samples':steps+1,'step_deg_max':.25}
                for index in range(steps+1):
                    pose=scene.full_pose({j:(lo+(hi-lo)*index/steps)*TICK})
                    if scene.check_link_pair(*pair,pose,require_distance=False).status=='INTERSECTING':
                        swept.update(status='COLLISION',ticks=lo+(hi-lo)*index/steps);break
                outcomes.append({'stage':name,'pair':pair,'scope':'REVOLUTE_STOP_1D_V5','sweep':swept,
                                 'status':'PASS' if swept['status']=='COLLISION_FREE' else 'COLLISION'})
            else:
                checked=continuous.check(pair,bounds)
                outcomes.append(dict(checked,stage=name))
            if not outcomes[-1]['status'].startswith('PASS'):
                return result()
            print(name,pair,outcomes[-1]['status'],continuous.boxes,'boxes',flush=True)
        print(name,'PASS',continuous.boxes,'boxes',flush=True)
    return result()



def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);args=ap.parse_args()
    report=qualify();Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
    print('STARTUP_GEOMETRY='+report['status'],flush=True)
    return 0 if report['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
