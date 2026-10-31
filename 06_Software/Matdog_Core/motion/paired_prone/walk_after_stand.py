#!/usr/bin/env python3
"""G4-only offline STAND -> WALK -> coordinated STOP -> STAND/rearm demo.

No hardware or runtime authority. A bounded Cartesian Z adapter reconciles
the existing G2 circular reference with actual STL surface extrema; production
core/safety and collision tolerances are unchanged.
"""
import ctypes,json,math
import numpy as np
from scipy.spatial import ConvexHull
from paired_acquisition import Acquisition,OUT
from paired_prone_model import JOINTS,LEGS
from core_python import PTR,ptr

def main():
    if (OUT/'walk_after_stand.json').exists():raise ValueError('FROZEN_OUTPUT_EXISTS: reuse pinned evidence or choose a fresh MATDOG_PAIRED_OUTPUT; no overwrite')
    certificate=json.loads((OUT/'continuous_pair_path.json').read_text())
    assert certificate['geometry_pass'] and not certificate['motion_eligible']
    a=Acquisition();lib=ctypes.CDLL(str(OUT/'walk_adapter.so'))
    lib.walk_frame.argtypes=[ctypes.c_double]*6+[ctypes.c_int,ctypes.c_double,PTR,PTR]
    lib.walk_reset();samples=[];out=np.zeros(64);now=0.
    radial_bound=max(float(np.linalg.norm(a.s.mesh(leg+'_foot_link').triangles_local.reshape(-1,3)[:,[0,2]]-[0,.0149],axis=1).max())-.0149 for leg in LEGS)
    def sample(phase,cycle,rate,accel,shift,hold,origin,t,shift_v=0.,shift_a=0.):
        offset=np.zeros(4);nominal=None
        for iteration in range(5):
            code=lib.walk_frame(cycle,rate,accel,shift,shift_v,shift_a,hold,origin,ptr(offset),ptr(out))
            if code:raise ValueError(('G4_REJECTION',phase,cycle,code))
            if nominal is None:nominal=out[12:24].reshape(4,3)[:,2].copy()
            q=dict(zip(JOINTS,out[:12]));minimum=[]
            for leg in LEGS:
                points=a.s.world_broad_phase(leg+'_foot_link',q).hull_vertices_world
                minimum.append(float(points[:,2].min()+out[29]))
            error=np.asarray(minimum)-nominal
            if max(abs(error))<=1e-10:break
            offset-=error
            if max(abs(offset))>radial_bound+1e-8:raise ValueError(('STL_CONTACT_ADAPTER_BOUND',offset.tolist(),radial_bound))
        else:raise ValueError('STL_CONTACT_REFERENCE_DID_NOT_CONVERGE')
        ground={}
        for name in a.s.model.collision_link_names:
            points=a.s.world_broad_phase(name,q).hull_vertices_world+np.array([out[28],0,out[29]])
            i=int(np.argmin(points[:,2]));ground[name]={'minimum_z_m':float(points[i,2]),'lowest_world_vertex_m':points[i].tolist()}
        bad=[n for n,g in ground.items() if g['minimum_z_m']<-1e-9]
        collision,pair=a.s.first_collision_at_pose(q)
        if bad or collision:raise ValueError(('WALK_MESH_REJECTION',phase,cycle,bad,pair))
        com=np.zeros(3)
        for name,m,c in a.inertials:
            tf=a.s.collision_transform(name,q);com+=m*(tf.rotation@c+tf.translation+np.array([out[28],0,out[29]]))/a.mass
        stance=[ground[leg+'_foot_link']['lowest_world_vertex_m'][:2] for i,leg in enumerate(LEGS) if out[24+i]>.5]
        hull=ConvexHull(np.asarray(stance));margin=float(np.min(-(hull.equations[:,:2]@com[:2]+hull.equations[:,2])))
        if margin<=0:raise ValueError(('ASSUMED_STANCE_POLYGON_REJECTION',phase,cycle,margin))
        samples.append({'phase':phase,'time_s':t,'cycles':cycle,'q_rad':q,'body_position_m':[out[28],0.,out[29]],
                        'feet_reference_world_m':out[12:24].reshape(4,3).tolist(),'stance':[bool(x) for x in out[24:28]],
                        'G4_status':'OK','branches':out[30:38].reshape(4,2).astype(int).tolist(),'core_residual_m':out[62],
                        'core_velocity_rad_s':out[38:50].tolist(),'core_acceleration_rad_s2':out[50:62].tolist(),
                        'STL_reference_correction_m':offset.tolist(),'ground':ground,'self_intersection':False,
                        'nominal_com_world_m':com.tolist(),'assumed_support_polygon_margin_m':margin,
                        'load_support_proven':False,'dynamic_stability_proven':False,'geometry_pass':True})
        if len(samples)%20==0:print('G4 checked',len(samples),phase,'cycle',cycle,'assumed support margin',margin,flush=True)
    smooth=lambda u:u*u*u*(10+u*(-15+6*u))
    # G4 begins with a rearward sway. All four feet stay down during this entry.
    for n in range(17):
        u=n/16;sample('GAIT_START',0,0,0,-.04*smooth(u),1,0,8*u,
                     -.04*30*u*u*(1-u)**2/8,-.04*(60*u-180*u*u+120*u**3)/64)
    now=8.
    # Existing phase offsets/order and terminal cycle, never an XGO scheduler.
    for n in range(1,181):
        u=n/180;s=smooth(u);v=30*u*u*(1-u)*(1-u)/90.;acc=(60*u-180*u*u+120*u*u*u)/90**2
        sample('WALK' if s<.4 else 'STOPPING',s,v,acc,0,0,0,now+90*u)
    now+=90
    for n in range(1,17):
        u=n/16;sample('STOP_TO_STAND',1,0,0,-.04*(1-smooth(u)),1,0,now+8*u,
                     .04*30*u*u*(1-u)**2/8,.04*(60*u-180*u*u+120*u**3)/64)
    now+=8
    lib.walk_reset()  # mathematical history reset, never authority/permit rearm
    sample('SIMULATED_REARM_STAND',0,0,0,0,1,.005,now)
    final=certificate['nodes'][-1]['parameters'];initial=Acquisition().flat(.15,final)
    first_q=np.array(list(samples[0]['q_rad'].values()));stand_q=np.array([a.q(initial)[j] for j in JOINTS])
    delta=float(np.max(abs(first_q-stand_q)))
    result={'schema':'MATDOG_G4_AFTER_PAIRED_STAND_V1','samples':samples,'sampled_geometry_pass':True,
            'stand_entry_mesh_reference_alignment_delta_rad':delta,
            'alignment_requires_explicit_short_joint_continuity_segment':delta>1e-10,
            'STL_contact_reference_max_correction_bound_m':radial_bound,'ground_tolerance_m':1e-9,
            'first_cycle_advance_m':.005,'cycle_demo_duration_s':90.,'vx_mean_m_s':.005/90.,'vy':0.,'yaw_rate':0.,
            'STOP_is_terminal_integer_cycle':1,'rearm_is_authority_grant':False,
            'maximum_abs_core_velocity_rad_s':max(abs(x) for w in samples for x in w['core_velocity_rad_s']),
            'maximum_abs_core_acceleration_rad_s2':max(abs(x) for w in samples for x in w['core_acceleration_rad_s2']),
            'minimum_assumed_support_polygon_margin_m':min(w['assumed_support_polygon_margin_m'] for w in samples),
            'continuous_swept_walk_certificate':False,'loaded_tracking_proven':False,'dynamic_stability_proven':False,
            'hardware_authorized':False,'MOTION_AUTHORIZED':0}
    (OUT/'walk_after_stand.json').write_text(json.dumps(result,indent=2)+'\n')
    print('G4 sampled geometry PASS',len(samples),'entry alignment rad',delta,'support estimated',result['minimum_assumed_support_polygon_margin_m'],flush=True)

if __name__=='__main__':main()
