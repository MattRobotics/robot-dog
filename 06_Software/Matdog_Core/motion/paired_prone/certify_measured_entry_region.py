#!/usr/bin/env python3
"""Offline collision certificate of the WHOLE measured-start entry region, 4 legs.

Geometry only, no device access. The operational STAND (directive 2026-10-14)
plans its entry from the MEASURED pose (FixtureEntryPlanner::planMeasuredEntry):
stage 0 moves the four LOWER joints to the route head, stage 1 moves HIP and
UPPER with LOWER held. The firmware admits a start only if AdaptiveStand's
PRONE recognition holds: |q - head| <= prone_max_deviation_rad on EVERY joint
(default 10 deg), and every sample stays inside the gen-1 hard limits. Both
stages are straight joint-space segments between the start and the head, so
every commanded configuration lies in the per-joint box

    [head - band, head + band]  (LOWER: from the physical folded stop)

widened by the executor tracking tube. This tool proves, for that whole box
(every admissible start, not one sampled start):

  A. the 120 non-adjacent canonical link pairs keep >= --clearance-mm
     (base_link pairs >= clearance + --ventral-extra-mm, the belly cover
     allowance of certify_commissioning_entry.py). Exact BVH distance
     (python-fcl) at the box centre minus a rigid-motion bound (vertex
     radius about each moving joint x half-width), recursive bisection; a
     box centre closer than required is a FAIL witness, never trimmed.
  B. every adjacent revolute pair is continuously clear over its joint
     domain (certify_adjacent_axis.certify, unchanged). The LOWER domain
     starts at the URDF lower limit: the firmware hard limit never admits a
     command beyond it, and the folded rest beyond it is primed inward.
The fixture, cables and the reprinted cover are NOT in the model.
The certificate is evidence for review, never an approval or a permit.
"""
from pathlib import Path
import argparse,hashlib,json,math,platform,time
import numpy as np
import fcl
from paired_prone_model import PairModel,ROOT,LEGS
from certify_adjacent_axis import certify

URDF=ROOT/'03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
ROUTE=ROOT/'09_Logs/Validation_Reports/Motion_Night_Run_2026-10-12/continuous_pair_path.json'
PARTS=('hip','upper_leg','lower_leg')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def route_head(front_floor_deg,rear_floor_deg):
    """Fixture-variant route head, as pairedProneSample(0,0,true)."""
    p=dict(json.loads(ROUTE.read_bytes())['segments'][0]['start_parameters'])
    p['lf']=max(p['lf'],math.radians(front_floor_deg));p['lr']=max(p['lr'],math.radians(rear_floor_deg))
    q={}
    for leg in LEGS:
        front=leg in ('lf','rf')
        q[leg+'_hip_joint']=p['h']*(-1 if leg in ('lf','lh') else 1)
        q[leg+'_upper_leg_joint']=p['uf' if front else 'ur'];q[leg+'_lower_leg_joint']=p['lf' if front else 'lr']
    return q

class Region:
    def __init__(self,scene,box):
        self.s=scene;self.box=box;self.calls=0;self.bvh={}
        for n in scene.model.collision_link_names:
            tri=np.asarray(scene.mesh(n).triangles_local,dtype=float).reshape(-1,3)
            m=fcl.BVHModel();m.beginModel(len(tri),len(tri)//3)
            m.addSubModel(tri,np.arange(len(tri)).reshape(-1,3));m.endModel();self.bvh[n]=m
        # Radius of every link about each joint of its own chain, valid for ANY
        # joint values: |v - o_j| <= chain offsets from o_j + |v - o_own|.
        self.radius={'base_link':{}}
        for n in scene.model.collision_link_names:
            if n=='base_link':continue
            leg,part=n[:2],n[3:-5]
            v=np.asarray(scene.mesh(n).hull_vertices_local,dtype=float)
            if part=='foot':v=v+np.asarray(scene.model.joints[leg+'_foot_joint'].origin_xyz)
            own=float(np.linalg.norm(v,axis=1).max());J=scene.model.joints;r={}
            if part in ('lower_leg','foot'):
                r[leg+'_lower_leg_joint']=own
                r[leg+'_upper_leg_joint']=own+float(np.linalg.norm(J[leg+'_lower_leg_joint'].origin_xyz))
                r[leg+'_hip_joint']=r[leg+'_upper_leg_joint']+float(np.linalg.norm(J[leg+'_upper_leg_joint'].origin_xyz))
            elif part=='upper_leg':
                r[leg+'_upper_leg_joint']=own
                r[leg+'_hip_joint']=own+float(np.linalg.norm(J[leg+'_upper_leg_joint'].origin_xyz))
            else:r[leg+'_hip_joint']=own
            self.radius[n]=r
    def distance(self,link_a,link_b,q):
        self.calls+=1;objs=[]
        for n in (link_a,link_b):
            t=self.s.collision_transform(n,q)
            objs.append(fcl.CollisionObject(self.bvh[n],fcl.Transform(np.asarray(t.rotation,dtype=float),np.asarray(t.translation,dtype=float))))
        if fcl.collide(objs[0],objs[1],fcl.CollisionRequest(),fcl.CollisionResult())>0:return -1.
        return float(fcl.distance(objs[0],objs[1],fcl.DistanceRequest(),fcl.DistanceResult()))
    def certify_pair(self,link_a,link_b,required_m,budget_s):
        joints=list(self.s.model.relevant_actuated_joints_for_pair(link_a,link_b))
        centre={j:(lo+hi)/2 for j,(lo,hi) in self.box.items()}
        weight={j:self.radius[link_a].get(j,0.)+self.radius[link_b].get(j,0.) for j in joints}
        stack=[{j:self.box[j] for j in joints}];leaves=0;bound_min=math.inf;centre_min=math.inf;start=time.monotonic()
        while stack:
            cell=stack.pop();q=dict(centre);q.update({j:(lo+hi)/2 for j,(lo,hi) in cell.items()})
            d=self.distance(link_a,link_b,q);centre_min=min(centre_min,d)
            if d<required_m:
                return dict(status='FAIL',pair=[link_a,link_b],required_m=required_m,witness_distance_m=d,
                            witness_q_rad={j:q[j] for j in joints},witness_cell_rad={j:list(v) for j,v in cell.items()})
            motion=sum(weight[j]*(hi-lo)/2 for j,(lo,hi) in cell.items())
            if d-motion>=required_m:leaves+=1;bound_min=min(bound_min,d-motion);continue
            if time.monotonic()-start>budget_s:
                return dict(status='UNRESOLVED',pair=[link_a,link_b],required_m=required_m,cell_rad={j:list(v) for j,v in cell.items()},centre_distance_m=d)
            j=max(cell,key=lambda k:weight[k]*(cell[k][1]-cell[k][0]));lo,hi=cell[j];mid=(lo+hi)/2
            stack.append({**cell,j:(lo,mid)});stack.append({**cell,j:(mid,hi)})
        return dict(status='PASS',pair=[link_a,link_b],required_m=required_m,moving_joints=joints,cells=leaves,
                    minimum_lower_bound_m=bound_min,minimum_centre_distance_m=centre_min)

def main():
    a=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--band-deg',type=float,default=10.);a.add_argument('--tube-rad',type=float,default=.006)
    a.add_argument('--lower-stop-deg',type=float,default=-92.1,help='most folded LOWER start (physical stop; LF contact -92.02 deg)')
    a.add_argument('--clearance-mm',type=float,default=3.);a.add_argument('--ventral-extra-mm',type=float,default=3.)
    a.add_argument('--front-floor-deg',type=float,default=-88.5);a.add_argument('--rear-floor-deg',type=float,default=-89.)
    a.add_argument('--pair-budget-s',type=float,default=600);a.add_argument('--adjacent-budget-s',type=float,default=180)
    a.add_argument('--output',type=Path,required=True)
    args=a.parse_args()
    if args.output.exists():raise SystemExit('output exists; certificates are never overwritten')
    for name in ('band_deg','tube_rad','clearance_mm','ventral_extra_mm'):
        v=getattr(args,name)
        if not math.isfinite(v) or v<0:raise SystemExit(name+' must be finite and >= 0')
    m=PairModel();s=m.scene;head=route_head(args.front_floor_deg,args.rear_floor_deg)
    band=math.radians(args.band_deg);tube=args.tube_rad;box={};adjacent_domain={}
    for leg in LEGS:
        for part in PARTS:
            j=f'{leg}_{part}_joint';joint=s.model.joints[j];h=head[j]
            lo,hi=h-band,h+band
            if part=='lower_leg':lo=max(lo,math.radians(args.lower_stop_deg))
            box[j]=(lo-tube,hi+tube)
            # Adjacent: the commanded domain never leaves the URDF limits.
            adjacent_domain[j]=(max(lo-tube,joint.lower_limit_rad),min(hi+tube,joint.upper_limit_rad))
    r=Region(s,box);rows=[];t0=time.monotonic()
    for link_a,link_b in s.model.clearance_pairs():
        required=args.clearance_mm*1e-3+(args.ventral_extra_mm*1e-3 if 'base_link' in (link_a,link_b) else 0.)
        row=r.certify_pair(link_a,link_b,required,args.pair_budget_s);rows.append(row)
        print(link_a,link_b,row['status'],round(row.get('minimum_lower_bound_m',row.get('witness_distance_m',row.get('centre_distance_m',float('nan'))))*1e3,3),'mm',flush=True)
    adjacent=[]
    for j,(lo,hi) in adjacent_domain.items():
        res=certify(s,j,lo,hi,args.adjacent_budget_s);adjacent.append(dict(joint=j,domain_rad=[lo,hi],**res))
        print(j,res['status'],flush=True)
    meshes={n:{'sha256':sha(s.model.collision_mesh_path(n))} for n in s.model.collision_link_names}
    result=dict(schema='MATDOG_MEASURED_ENTRY_REGION_CERTIFICATE_V1',
        scope='every admissible measured start of the operational STAND entry, 4 legs, plus the two straight entry stages',
        route_head_rad=head,band_rad=band,tracking_tube_rad=tube,lower_stop_rad=math.radians(args.lower_stop_deg),
        box_rad={j:list(v) for j,v in box.items()},adjacent_domain_rad={j:list(v) for j,v in adjacent_domain.items()},
        clearance_m=args.clearance_mm*1e-3,ventral_extra_m=args.ventral_extra_mm*1e-3,
        nonadjacent_pairs=len(rows),nonadjacent_pass=sum(x['status']=='PASS' for x in rows),
        adjacent_pass=sum(x['status']=='PASS_CONTINUOUS' for x in adjacent),
        all_pass=all(x['status']=='PASS' for x in rows) and all(x['status']=='PASS_CONTINUOUS' for x in adjacent),
        distance_calls=r.calls,elapsed_s=round(time.monotonic()-t0,1),
        identity=dict(urdf_sha256=sha(URDF),route_sha256=sha(ROUTE),tool_sha256=sha(Path(__file__)),meshes=meshes,
                      python=platform.python_version(),numpy=np.__version__,fcl=getattr(fcl,'__version__','python-fcl')),
        not_modelled=['fixture','cables','battery','reprinted cover beyond the ventral allowance','servo horn hardware'],
        hardware_authorized=False,motion_eligible=False,nonadjacent=rows,adjacent=adjacent)
    args.output.write_text(json.dumps(result,indent=1)+'\n')
    print('MEASURED_ENTRY_REGION',('PASS' if result['all_pass'] else 'FAIL'),f"nonadjacent={result['nonadjacent_pass']}/{len(rows)}",
          f"adjacent={result['adjacent_pass']}/{len(adjacent)}",f"calls={r.calls}",flush=True)

if __name__=='__main__':main()
