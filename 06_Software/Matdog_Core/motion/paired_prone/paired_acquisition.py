#!/usr/bin/env python3
"""Original MATDOG two-pair, contact-constrained offline acquisition.

HIP prealignment is one mirrored clock. Deployment has two LOWER clocks;
UPPER and body pitch/height are solved from actual geometry/contact closure.
No arbitrary 12-encoder interpolation, bus, calibration writes or motion permit.
"""
from pathlib import Path
from dataclasses import asdict
import json,math,sys,os
import numpy as np
from scipy.optimize import brentq,least_squares,linprog
from scipy.spatial import ConvexHull
OUT=Path(os.environ['MATDOG_PAIRED_OUTPUT']).resolve()
OUT.mkdir(parents=True,exist_ok=True)
from paired_prone_model import PairModel,PairProfile,LEGS,JOINTS,r_y,digest,ROOT
from core_python import contact,origin,ik,LIB,ptr

XR=-.1437169534863073;XF=.1301352313457785
LMIN=-1.605702911835

class Acquisition:
    def __init__(self):
        self.m=PairModel();self.s=self.m.scene
        self.base=self.s.world_broad_phase('base_link',self.s.home_pose()).hull_vertices_world
        self.foot={i:self.s.mesh(leg+'_foot_link').hull_vertices_local for i,leg in enumerate(LEGS)}
        self.inertials=[]
        for node in self.m.xml.findall('link'):
            n=node.find('inertial');self.inertials.append((node.get('name'),float(n.find('mass').get('value')),np.array([float(x) for x in n.find('origin').get('xyz').split()])))
        self.mass=sum(m for _,m,_ in self.inertials)
    def bh(self,p):return -float((self.base@r_y(p)[2]).min())
    def floor(self,i,u,l,p,x,z,h=0):
        t,r=origin(i,u,l,h);points=self.foot[i]@r.T+t
        return float((points@r_y(p)[2]).min()+z)
    def fit(self,fn,start,lo,hi):
        result=least_squares(fn,start,bounds=(lo,hi),xtol=1e-15,ftol=1e-15,gtol=1e-15,max_nfev=160)
        residual=float(np.max(np.abs(result.fun)))
        if residual>1e-10:raise ValueError(('CONTACT_CLOSURE_INFEASIBLE',residual,result.x.tolist()))
        return result.x
    def prealign(self,h):
        # RH/RF +h and LH/LF -h are inward in the canonical FK, verified below.
        ur=math.pi/2
        fn=lambda p:min(self.floor(2,ur,LMIN,p,0,self.bh(p),h),self.floor(3,ur,LMIN,p,0,self.bh(p),-h))
        p=brentq(fn,0,.20,xtol=1e-14);z=self.bh(p)
        uf=brentq(lambda u:min(self.floor(0,u,LMIN,p,0,z,-h),self.floor(1,u,LMIN,p,0,z,h)),math.radians(65),math.radians(98),xtol=1e-14)
        return dict(ur=ur,lr=LMIN,uf=uf,lf=LMIN,p=p,x=0.,z=z,h=h)
    def q(self,v):
        h=v.get('h',0.)
        return self.m.frame(PairProfile(v['ur'],v['lr']),PairProfile(v['uf'],v['lf']),{'rh':h,'lh':-h,'rf':h,'lf':-h})
    def closed(self,lr,lf,xr,xf,start):
        def fn(v):
            ur,uf,p,x,z=v;a=contact(2,ur,lr,p,x,z);b=contact(0,uf,lf,p,x,z)
            return [a[0]-xr,self.floor(2,ur,lr,p,x,z),b[0]-xf,self.floor(0,uf,lf,p,x,z),z-self.bh(p)]
        v=self.fit(fn,[start[k] for k in ('ur','uf','p','x','z')],[-.916297857297,-.916297857297,0,-.05,0],
                   [2.138028333693,2.138028333693,.20,.05,.04])
        return dict(zip(('ur','uf','p','x','z'),v),lr=lr,lf=lf,h=0.)
    def flat(self,z,start):
        result=dict(start,p=0.,x=0.,z=z,h=0.)
        for i,u,l,x in ((2,'ur','lr',XR),(0,'uf','lf',XF)):
            # Existing G2 analytic IK selects the seed-nearest branch first.
            a=ik(i,[x,-.094 if i==2 else .094,0],[0,start[u],start[l]],0.,0.,z)
            v=self.fit(lambda v:[contact(i,*v,z=z)[0]-x,self.floor(i,*v,0.,0.,z)],a[1:3],
                       [-.916297857297,LMIN],[2.138028333693,.654498469498])
            result[u],result[l]=v
        return result
    def free(self,z,lr,x,xr,xf,start):
        v=self.fit(lambda v:[contact(2,v[0],lr,v[1],x,z)[0]-xr,self.floor(2,v[0],lr,v[1],x,z)],
                   [start['ur'],start['p']],[-.916297857297,-.002],[2.138028333693,.20])
        ur,p=v
        front=self.fit(lambda v:[contact(0,*v,p,x,z)[0]-xf,self.floor(0,*v,p,x,z)],
                       [start['uf'],start['lf']],[-.916297857297,LMIN],[2.138028333693,.654498469498])
        return dict(ur=ur,lr=lr,uf=front[0],lf=front[1],p=p,x=x,z=z,h=0.)
    def evaluate(self,v,phase,full=True):
        q=self.q(v);r=r_y(v['p']);base=np.array([v['x'],0,v['z']]);ground={};points={}
        for name in self.s.model.collision_link_names:
            t=self.s.collision_transform(name,q)
            mesh=self.s.mesh(name)
            local=mesh.triangles_local.reshape(-1,3) if full else mesh.hull_vertices_local
            # Linear plane extrema over the convex hull equal extrema over ALL
            # original vertices. Full triangle witnesses remain in keypoints.
            allpoints=(local@t.rotation.T+t.translation)@r.T+base
            i=int(np.argmin(allpoints[:,2]));minimum=float(allpoints[i,2])
            ground[name]={'minimum_z_m':minimum,'lowest_world_vertex_m':allpoints[i].tolist(),
                          'triangle':i//3 if full else None,'vertex':i%3 if full else None,
                          'extrema_method':'ALL_TRIANGLES' if full else 'EXACT_CANONICAL_VERTEX_CONVEX_HULL_EXTREMUM',
                          'ground_clear':minimum>=-1e-9,'bearing_load':'UNVERIFIED'}
            if name=='base_link' or name.endswith('foot_link'):
                points[name]=allpoints[np.abs(allpoints[:,2])<=1e-5]
        collision,pair=self.s.first_collision_at_pose(q)
        violations=[{'kind':'GROUND_PENETRATION','link':n,**value} for n,value in ground.items() if not value['ground_clear']]
        if collision:violations.append({'kind':'SELF_INTERSECTION','pair':pair})
        com=np.zeros(3)
        for name,m,c in self.inertials:
            t=self.s.collision_transform(name,q);com+=m*(r@(t.rotation@c+t.translation)+base)/self.mass
        candidates=np.concatenate([value for value in points.values() if len(value)],axis=0)
        candidates=np.unique(candidates,axis=0)
        # A static equilibrium EXISTENCE estimate, never measured load allocation.
        equilibrium=linprog(np.zeros(len(candidates)),A_eq=np.array([np.ones(len(candidates)),candidates[:,0],candidates[:,1]]),
                            b_eq=[self.mass*9.81,self.mass*9.81*com[0],self.mass*9.81*com[1]],bounds=(0,None),method='highs')
        margin=None
        if len(candidates)>=3:
            hull=ConvexHull(candidates[:,:2]);margin=float(np.min(-(hull.equations[:,:2]@com[:2]+hull.equations[:,2])))
        core=[]
        for i,leg in enumerate(LEGS):
            a=np.array([q[leg+'_'+part+'_joint'] for part in ('hip','upper_leg','lower_leg')]);out=np.zeros(4)
            code=LIB.roundtrip_g1(i,ptr(a),ptr(out))
            if code or max(abs(out[:3]-a))>1e-8:violations.append({'kind':'G1_IK_ROUNDTRIP','leg':leg,'code':code})
            c=contact(i,a[1],a[2],v['p'],v['x'],v['z'],a[0])
            core.append({'leg':leg,'G1_status':code,'G1_residual_m':out[3],'contact_reference_world_m':c[:3].tolist(),
                         'lowest_core_world_m':c[3:6].tolist(),'axis_tilt_rad':c[6],
                         'contact_model_vs_STL_plane_offset_m':float(c[2]-ground[leg+'_foot_link']['minimum_z_m'])})
        if not equilibrium.success:violations.append({'kind':'NO_QUASISTATIC_EQUILIBRIUM_UNDER_ASSUMED_GEOMETRIC_CONTACTS'})
        result={'phase':phase,'parameters':v,'q_rad':q,'ground':ground,'self_intersection':collision,'collision_pair':pair,
                'pair_fk_symmetry':self.m.mirror_residual(q),'nominal_com_world_m':com.tolist(),
                'quasistatic_contact_equilibrium_estimated':bool(equilibrium.success),'support_margin_m':margin,
                'contact_force_distribution_measured':False,'dynamic_stability_validated':False,'core':core,
                'violations':violations,'geometry_pass':not violations,'motion_eligible':False}
        if full:result['worst_nonadjacent_pair']=asdict(self.s.worst_pair_at_pose(q)[2])
        return result

def main():
    if (OUT/'acquisition_trajectory.json').exists():raise ValueError('FROZEN_OUTPUT_EXISTS: reuse pinned evidence or choose a fresh MATDOG_PAIRED_OUTPUT; no overwrite')
    a=Acquisition();family=[]
    for degrees in (0.,1.,3.):
        v=a.prealign(math.radians(degrees));result=a.evaluate(v,'C0_SYNTHETIC_INWARD_HIP_'+str(degrees))
        family.append(result);print('C0 HIP',degrees,'pitch',math.degrees(v['p']),'base Z',v['z'],'pass',result['geometry_pass'],flush=True)
    (OUT/'initial_family.json').write_text(json.dumps({'schema':'MATDOG_PAIRED_INITIAL_FAMILY_V1','initial_pose_measured':False,'operator_confirms_no_existing_prone_log':True,'candidates':family},indent=2)+'\n')
    v0=a.prealign(math.radians(1.));v1=a.prealign(0.)
    xf0=contact(0,v1['uf'],v1['lf'],v1['p'],v1['x'],v1['z'])[0]
    v2=a.closed(LMIN+.30,LMIN,XR,xf0,v1)
    v3=a.closed(LMIN+.30,LMIN+.01,XR,xf0,v2)
    v50=a.flat(.05,v3);v100=a.flat(.1,v50);v150=a.flat(.15,v100)
    def rise(z,previous):
        s=(z-v3['z'])/(.05-v3['z']);return a.free(z,v3['lr']+s*(v50['lr']-v3['lr']),v3['x']*(1-s),XR,xf0+s*(XF-xf0),previous)
    v4=rise(v3['z']+.002,v3)
    waypoints=[a.evaluate(v,p) for v,p in ((v0,'C0_BELLY_CONTACT'),(a.prealign(math.radians(.5)),'PRE_ALIGN_MID'),
                 (v1,'PRE_ALIGN_NEUTRAL'),(v2,'C1_REAR_DEPLOY'),(v3,'C2_FRONT_DEPLOY'),(v4,'C3_LOAD_TRANSFER_MODEL'),
                 (v50,'C4_RISE_50'),(v100,'C4_C_ENTRY_100'),(v150,'C5_STAND_150'))]
    result={'schema':'MATDOG_PAIRED_ACQUISITION_KEYPOINTS_V1','initial_pose_measured':False,'operator_confirms_no_prone_log':True,
            'MOTION_AUTHORIZED':0,'OTA_INGEST':0,'waypoints':waypoints,'keypoints_pass':all(w['geometry_pass'] for w in waypoints),
            'full_trajectory_generated':False,'physical_load_proven':False,'hardware_authorized':False,
            'source_pins':{str(p.relative_to(ROOT)):digest(p) for p in [a.m.scene.model.urdf_path] + [a.s.model.collision_mesh_path(n) for n in a.s.model.collision_link_names]}}
    (OUT/'acquisition_keypoints.json').write_text(json.dumps(result,indent=2)+'\n')
    for w in waypoints:print(w['phase'],w['geometry_pass'],'p',math.degrees(w['parameters']['p']),'z',w['parameters']['z'],'violations',w['violations'],flush=True)
    if not result['keypoints_pass']:return
    # Continuous scalar clocks and seed continuation; waypoint gate precedes sampling.
    smooth=lambda s:s*s*s*(10+s*(-15+6*s))
    samples=[];previous=v0;now=0.
    def emit(v,phase,t):
        nonlocal previous
        w=a.evaluate(v,phase,full=False);w['time_s']=t;samples.append(w);previous=v
        with (OUT/'acquisition_samples.jsonl').open('a') as f:f.write(json.dumps(w,separators=(',',':'))+'\n')
        if len(samples)%20==0:print('sampled',len(samples),phase,'pitch',math.degrees(v['p']),'z',v['z'],flush=True)
        if not w['geometry_pass']:raise ValueError(('TRAJECTORY_GEOMETRY_FAILED',len(samples)-1,w['violations'],v))
    for n in range(31):
        s=smooth(n/30);emit(a.prealign(math.radians(1.)*(1-s)),'PRE_ALIGN',8*n/30)
    now=8.
    xr0=contact(2,v1['ur'],v1['lr'],v1['p'],v1['x'],v1['z'])[0]
    for n in range(1,41):
        s=smooth(n/40);emit(a.closed(LMIN+.30*s,LMIN,xr0+s*(XR-xr0),xf0,previous),'REAR_DEPLOY',now+12*n/40)
    now+=12
    for n in range(1,21):
        s=smooth(n/20);emit(a.closed(LMIN+.30,LMIN+.01*s,XR,xf0,previous),'FRONT_DEPLOY',now+6*n/20)
    now+=6
    for n in range(1,61):
        s=smooth(n/60);emit(rise(v3['z']+(.05-v3['z'])*s,previous),'LOAD_TRANSFER' if n<8 else 'CONTACT_LOCKED_RISE',now+12*n/60)
    now+=12
    for n in range(1,41):
        s=smooth(n/40);emit(a.flat(.05+.05*s,previous),'FLAT_CONTACT_RISE',now+8*n/40)
    now+=8
    for n in range(1,51):
        s=smooth(n/50);emit(a.flat(.1+.05*s,previous),'C4_C_RISE',now+10*n/50)
    emit(v150,'STAND_150',56.)
    (OUT/'acquisition_trajectory.json').write_text(json.dumps({'schema':'MATDOG_PAIRED_ACQUISITION_SAMPLES_V1','samples':samples,
        'initial_pose_measured':False,'full_sampled_geometry_pass':True,'continuous_between_sample_proof':False,
        'contact_load_proven':False,'MOTION_AUTHORIZED':0},indent=2)+'\n')
    print('complete sampled acquisition',len(samples),'final Z',samples[-1]['parameters']['z'],flush=True)

if __name__=='__main__':main()
