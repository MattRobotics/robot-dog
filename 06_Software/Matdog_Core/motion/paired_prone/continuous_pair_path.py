#!/usr/bin/env python3
"""Verify a continuous two-pair profile between contact-derived samples.

Each segment has one common monotone quintic clock, individual URDF bounds,
four coupled profile values and mirrored HIP. Body Z is inferred from the
actual support-plane mesh extrema, with at most the existing 10um geometric
contact-proximity allowance. This is NOT a measured bearing-load statement.
Swept mesh separation uses interval displacement bounds, not pictures.
"""
from pathlib import Path
import json,math
import numpy as np
from paired_acquisition import Acquisition,OUT,LMIN,XR,XF
from paired_prone_model import LEGS,r_y

TOL=1e-5

def rx(h):return np.array([[1,0,0],[0,math.cos(h),-math.sin(h)],[0,math.sin(h),math.cos(h)]])

class Continuous:
    def __init__(self):
        self.a=Acquisition();self.s=self.a.s;self.mesh={n:self.s.mesh(n).hull_vertices_local for n in self.s.model.collision_link_names}
        self.radii={};self.evaluations=0;self.subdivisions=0;self.self_leaves=0;self.min_self_bound=math.inf
        for name,vertices in self.mesh.items():
            if name=='base_link':self.radii[name]={};continue
            leg=name[:2];part=name[3:];norm=float(np.linalg.norm(vertices,axis=1).max())
            lower=norm if part=='lower_leg_link' else float(np.linalg.norm(vertices+np.asarray(self.s.model.joints[leg+'_foot_joint'].origin_xyz),axis=1).max()) if part=='foot_link' else 0.
            upper=norm if part=='upper_leg_link' else .09+lower if lower else 0.
            hip=norm if part=='hip_link' else .048+upper
            self.radii[name]={leg+'_hip_joint':hip}
            if upper:self.radii[name][leg+'_upper_leg_joint']=upper
            if lower:self.radii[name][leg+'_lower_leg_joint']=lower
        self.body_r={name:float(np.linalg.norm(v,axis=1).max()) if name=='base_link' else
                     float(np.linalg.norm(self.s.model.joints[name[:2]+'_hip_joint'].origin_xyz))+
                     self.radii[name].get(name[:2]+'_hip_joint',0.) for name,v in self.mesh.items()}
    def geometry(self,v):
        p=v['p'];r=r_y(p);b=np.array([v['x'],0.,v['z']]);result={'base_link':self.mesh['base_link']@r.T+b}
        for leg in LEGS:
            h=v.get('h',0.)*(1 if leg in ('rf','rh') else -1);rh=rx(h)
            hip=np.asarray(self.s.model.joints[leg+'_hip_joint'].origin_xyz)
            upper=hip+rh@np.asarray(self.s.model.joints[leg+'_upper_leg_joint'].origin_xyz)
            u=v['ur'] if leg in ('rh','lh') else v['uf'];l=v['lr'] if leg in ('rh','lh') else v['lf']
            ru=rh@r_y(u);rl=rh@r_y(u+l)
            lower=upper+ru@np.asarray(self.s.model.joints[leg+'_lower_leg_joint'].origin_xyz)
            foot=lower+rl@np.asarray(self.s.model.joints[leg+'_foot_joint'].origin_xyz)
            for part,t,rot in (('hip',hip,rh),('upper_leg',upper,ru),('lower_leg',lower,rl),('foot',foot,rl)):
                name=leg+'_'+part+'_link';result[name]=(self.mesh[name]@rot.T+t)@r.T+b
        return result
    def corrected(self,v):
        points=self.geometry(v);minimum={n:float(p[:,2].min()) for n,p in points.items()}
        correction=max(0.,-min(minimum.values()))
        return correction,{n:z+correction for n,z in minimum.items()}
    def floor_lipschitz(self,va,vb):
        qa=self.a.q(va);qb=self.a.q(vb)
        return {n:abs(vb['z']-va['z'])+self.body_r[n]*abs(vb['p']-va['p'])+
                sum(radius*abs(qb[j]-qa[j]) for j,radius in radii.items()) for n,radii in self.radii.items()}
    def ground_proof(self,va,vb,belly):
        lips=self.floor_lipschitz(va,vb);worst=max(lips.values());stack=[(0.,1.)];leaves=0;max_correction=0;max_contact=0
        relevant=[leg+'_foot_link' for leg in LEGS]+(['base_link'] if belly else [])
        while stack:
            lo,hi=stack.pop();mid=(lo+hi)/2;v={k:va[k]+mid*(vb[k]-va[k]) for k in va}
            correction,gaps=self.corrected(v);self.evaluations+=1
            if correction>TOL or max(gaps[n] for n in relevant)>TOL:
                return False,{'s':mid,'correction_m':correction,'contact_gaps_m':{n:gaps[n] for n in relevant}}
            span=(hi-lo)/2
            bound_c=correction+worst*span
            bound_g=max(gaps[n]+(lips[n]+worst)*span for n in relevant)
            if bound_c<=TOL and bound_g<=TOL:
                max_correction=max(max_correction,bound_c);max_contact=max(max_contact,bound_g);leaves+=1
            elif hi-lo<2**-20:raise ValueError('CONTACT_INTERVAL_CERTIFICATE_UNRESOLVED')
            else:stack.extend(((lo,mid),(mid,hi)))
        return True,{'leaves':leaves,'maximum_lift_correction_upper_bound_m':max_correction,'maximum_contact_gap_upper_bound_m':max_contact}
    def self_proof(self,va,vb,minimum_separation_m=1e-12,joint_uncertainty_rad=0.):
        if not 0<=joint_uncertainty_rad<=.01:raise ValueError("TRACKING_TUBE_DOMAIN")
        return self.self_proof_q(self.a.q(va),self.a.q(vb),minimum_separation_m,joint_uncertainty_rad)
    def self_proof_q(self,qa,qb,minimum_separation_m=1e-12,joint_uncertainty_rad=0.):
        if not 0<=joint_uncertainty_rad<=.01:raise ValueError("TRACKING_TUBE_DOMAIN")
        pairs=self.s.model.clearance_pairs();stack=[(0.,1.,pairs)];leaves=0;minimum=math.inf
        while stack:
            lo,hi,pending=stack.pop();mid=(lo+hi)/2;q={j:qa[j]+mid*(qb[j]-qa[j]) for j in qa}
            transforms={n:self.s.collision_transform(n,q) for n in self.mesh}
            remaining=[]
            for na,nb in pending:
                ca=self.s.model.relevant_actuated_joints_for_link(na);cb=self.s.model.relevant_actuated_joints_for_link(nb)
                common=[]
                for ja,jb in zip(ca,cb):
                    if ja!=jb:break
                    common.append(ja)
                reference=self.s.model.joints[common[-1]].child_link if common else 'base_link'
                t=transforms[reference] if reference!='base_link' else None
                bounds=[];moves=[]
                for name in (na,nb):
                    tf=transforms[name]
                    vertices=self.mesh[name]@tf.rotation.T+tf.translation
                    if t is not None:vertices=(vertices-t.translation)@t.rotation
                    total=0.;hip_motion=0.;ul_motion=0.;hip_max=0.
                    for j,radius in self.radii[name].items():
                        if j in common:continue
                        move=radius*(abs(qb[j]-qa[j])*(hi-lo)/2+joint_uncertainty_rad);total+=move
                        if j.endswith('hip_joint'):hip_motion+=move;hip_max=max(hip_max,abs(qa[j])+joint_uncertainty_rad,abs(qb[j])+joint_uncertainty_rad)
                        else:ul_motion+=move
                    # In a common HIP/UPPER frame, all remaining axes are Y.
                    # In base, nonzero HIP can mix Z into Y; bound that term.
                    my=hip_motion+ul_motion*math.sin(min(math.pi/2,hip_max))
                    mx=ul_motion  # HIP rotation around X does not move X
                    mz=total
                    movement=np.array([mx,my,mz]);bounds.append((vertices.min(axis=0)-movement,vertices.max(axis=0)+movement));moves.append(total)
                gap=np.maximum(np.maximum(bounds[0][0]-bounds[1][1],bounds[1][0]-bounds[0][1]),0.)
                separation=float(np.linalg.norm(gap))
                if separation>minimum_separation_m:minimum=min(minimum,separation);continue
                if separation>0:
                    # AABB gaps can stay loose even at a zero-width interval.
                    # Use conservative triangle projections before subdivision;
                    # this changes proof strength only, never geometry/tolerances.
                    from matdog_startup_geometry_v5 import mesh_separation_bound
                    refined=mesh_separation_bound(self.s,(na,nb),q,max(.001,minimum_separation_m+sum(moves)+1e-5))
                    if refined.clearance_m is not None and refined.clearance_m>sum(moves)+minimum_separation_m:
                        minimum=min(minimum,refined.clearance_m-sum(moves));continue
                    remaining.append((na,nb));continue
                result=self.s.check_link_pair(na,nb,q,require_distance=True)
                if result.status=='INTERSECTING':return False,{'pair':[na,nb],'s':mid,'result':result.__dict__}
                margin=result.clearance_m
                if margin is None or margin<=sum(moves)+minimum_separation_m:
                    from matdog_startup_geometry_v5 import mesh_separation_bound
                    refined=mesh_separation_bound(self.s,(na,nb),q,max(.001,minimum_separation_m+sum(moves)+1e-5))
                    margin=max(margin or 0.,refined.clearance_m or 0.)
                if margin is not None and margin>sum(moves)+minimum_separation_m:
                    minimum=min(minimum,margin-sum(moves));continue
                remaining.append((na,nb))
            if not remaining:leaves+=1
            elif hi-lo<2**-18:return False,{'unresolved_pairs':remaining,'interval':[lo,hi]}
            else:stack.extend(((lo,mid,tuple(remaining)),(mid,hi,tuple(remaining))))
        return True,{'pairs':len(pairs),'leaves':leaves,'minimum_swept_separation_lower_bound_m':minimum}
    def exact_midpoint(self,left,right):
        va=left['parameters'];vb=right['parameters'];v={k:(va[k]+vb[k])/2 for k in va};phase=right['phase']
        if phase=='PRE_ALIGN':return self.a.prealign(v['h'])
        ca=contact_x(self.a,va);cb=contact_x(self.a,vb)
        xr=(ca[0]+cb[0])/2;xf=(ca[1]+cb[1])/2
        if phase in ('REAR_DEPLOY','FRONT_DEPLOY'):return self.a.closed(v['lr'],v['lf'],xr,xf,v)
        if phase in ('LOAD_TRANSFER','CONTACT_LOCKED_RISE'):return self.a.free(v['z'],v['lr'],v['x'],xr,xf,v)
        return self.a.flat(v['z'],v)

def contact_x(a,v):
    from core_python import contact
    return [contact(2,v['ur'],v['lr'],v['p'],v['x'],v['z'])[0],contact(0,v['uf'],v['lf'],v['p'],v['x'],v['z'])[0]]

def main():
    if (OUT/'continuous_pair_path.json').exists():raise ValueError('FROZEN_OUTPUT_EXISTS: reuse pinned evidence or choose a fresh MATDOG_PAIRED_OUTPUT; no overwrite')
    data=json.loads((OUT/'acquisition_trajectory.json').read_text());source=data['samples'];c=Continuous()
    # Source keypoints already passed all 17 mesh plane checks and all 132 pairs.
    accepted=[];nodes=[source[0]];stack=[(source[i],source[i+1],0) for i in range(len(source)-2,-1,-1)]
    while stack:
        left,right,depth=stack.pop();belly=right['phase'] in ('PRE_ALIGN','REAR_DEPLOY','FRONT_DEPLOY')
        passed,ground=c.ground_proof(left['parameters'],right['parameters'],belly)
        if not passed:
            if depth>=6:raise ValueError(('PROFILE_CONTACT_GAP_NOT_RECOVERABLE',ground,left['phase'],right['phase']))
            v=c.exact_midpoint(left,right);mid={'phase':right['phase'],'parameters':v,'time_s':(left['time_s']+right['time_s'])/2}
            stack.extend(((mid,right,depth+1),(left,mid,depth+1)));c.subdivisions+=1;continue
        clear,swept=c.self_proof(left['parameters'],right['parameters'])
        if not clear:raise ValueError(('SWEPT_MESH_FAILURE',swept,left['phase'],right['phase']))
        qa=c.a.q(left['parameters']);qb=c.a.q(right['parameters']);delta=max(abs(qb[j]-qa[j]) for j in qa)
        duration=max(right['time_s']-left['time_s'],1.875*delta/.25,math.sqrt(5.773502692*delta/2.))
        accepted.append({'phase':right['phase'],'start_parameters':left['parameters'],'end_parameters':right['parameters'],
                         'clock':'common quintic 10t^3-15t^4+6t^5','duration_s':duration,'ground_certificate':ground,'swept_certificate':swept,
                         'max_joint_velocity_rad_s':1.875*delta/duration if duration else 0.,
                         'max_joint_acceleration_rad_s2':5.773502692*delta/duration**2 if duration else 0.})
        nodes.append(right)
        if len(accepted)%10==0:print('continuous segments',len(accepted),'inserted geometry knots',c.subdivisions,'interval evaluations',c.evaluations,flush=True)
    result={'schema':'MATDOG_CONTINUOUS_PAIRED_PATH_V1','geometry_pass':True,'segments':accepted,'nodes':nodes,
            'original_contact_solved_samples':len(source),'refined_contact_knots':c.subdivisions,'interval_evaluations':c.evaluations,
            'links':17,'swept_nonadjacent_pairs_per_segment':len(c.s.model.clearance_pairs()),
            'sampled_obstruction_pairs_per_original_sample':len(c.s.model.path_obstruction_pairs()),
            'adjacent_revolute_swept_certificate':False,'ground_penetration_tolerance_m':1e-9,
            'geometric_contact_proximity_m':TOL,'body_height_rule':'nominal geometric closure height plus max(0,-minimum of all 17 STL plane extrema); lift correction <= existing contact proximity',
            'synthetic_initial_pose':True,'load_distribution_proven':False,'dynamic_stability_proven':False,
            'motion_eligible':False,'MOTION_AUTHORIZED':0,'total_duration_s':sum(s['duration_s'] for s in accepted),
            'timing_envelope_is_hardware_approved':False}
    (OUT/'continuous_pair_path.json').write_text(json.dumps(result,indent=2)+'\n')
    print('continuous paired geometry PASS',len(accepted),'segments',result['total_duration_s'],'seconds',flush=True)

if __name__=='__main__':main()
