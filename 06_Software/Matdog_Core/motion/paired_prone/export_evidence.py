#!/usr/bin/env python3
"""Render/export pinned synthetic geometry. No AI images or device connection."""
import csv,hashlib,json,math,os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from paired_acquisition import Acquisition,OUT
from paired_prone_model import LEGS,JOINTS,r_y
from continuous_pair_path import Continuous
from core_python import contact

def main():
    a=Acquisition();c=Continuous();certificate=json.loads((OUT/'continuous_pair_path.json').read_text());walk=json.loads((OUT/'walk_after_stand.json').read_text());keys=json.loads((OUT/'acquisition_keypoints.json').read_text())['waypoints']
    frames=[];now=0.
    def add(v,phase,q=None,feet=None,stance=None):
        nonlocal now
        if q is None:
            lift,_=c.corrected(v);v=dict(v,z=v['z']+lift);q=a.q(v)
        if feet is None:feet=[contact(i,q[l+'_upper_leg_joint'],q[l+'_lower_leg_joint'],v['p'],v['x'],v['z'],q[l+'_hip_joint'])[:3].tolist() for i,l in enumerate(LEGS)]
        frames.append({'time_s':now,'phase':phase,'parameters':v,'q_rad':q,'feet':feet,'stance':stance,'synthetic':True})
    add(certificate['nodes'][0]['parameters'],'PRE_ALIGN')
    for s in certificate['segments']:
        now+=s['duration_s'];add(s['end_parameters'],s['phase'])
    end=now
    for w in walk['samples']:
        now=end+w['time_s'];v=dict(p=0.,x=w['body_position_m'][0],z=w['body_position_m'][2],h=0.,ur=w['q_rad']['rh_upper_leg_joint'],lr=w['q_rad']['rh_lower_leg_joint'],uf=w['q_rad']['lf_upper_leg_joint'],lf=w['q_rad']['lf_lower_leg_joint'])
        add(v,w['phase'],w['q_rad'],w['feet_reference_world_m'],w['stance'])
    # Restart is a suffix of the identical proved two-pair path, not a new route.
    now+=2.;start=next(i for i,s in enumerate(certificate['segments']) if s['start_parameters']['z']>=.05-1e-12 and s['phase']=='FLAT_CONTACT_RISE')
    add(certificate['segments'][start]['start_parameters'],'INTERMEDIATE_RESTART_50MM')
    for s in certificate['segments'][start:]:
        now+=s['duration_s'];add(s['end_parameters'],'RESTART_'+s['phase'])
    jsonl=[]
    for f in frames:
        v=f['parameters'];stamp=round(f['time_s']*1000);q=[f['q_rad'][j] for j in JOINTS]
        jsonl.append({'version':1,'now_ms':stamp,'executor_state':'OFFLINE_GEOMETRY','stand_state':f['phase'],'owner':'offline_trace','fault':False,'stale':False,
            'evidence':'SYNTHETIC_GEOMETRY_ONLY; actual=ideal simulated joints; support/load/tracking unproven; MOTION_AUTHORIZED=0',
            'telemetry':{'stamp_ms':stamp,'sampled_at_ms':stamp,'joint_names':JOINTS,'actual_rad':q,'target_rad':q,'quaternion':[math.cos(v['p']/2),0,math.sin(v['p']/2),0],
                'base_position_m':[v['x'],0,v['z']],'base_height_m':v['z'],'tracking_error_rad':None,'fault':False,
                'feet':{l.upper():{'position_m':f['feet'][i],'state':('STANCE' if f['stance'][i] else 'SWING') if f['stance'] is not None else 'UNKNOWN'} for i,l in enumerate(LEGS)}}})
    (OUT/'paired_motion_trace.jsonl').write_text(''.join(json.dumps(f,separators=(',',':'))+'\n' for f in jsonl))
    with (OUT/'paired_joint_trajectory.csv').open('w') as stream:
        wr=csv.writer(stream);wr.writerow(['time_s','phase','base_x_m','base_z_m','pitch_rad','roll_rad',*JOINTS]);wr.writerows([[f['time_s'],f['phase'],f['parameters']['x'],f['parameters']['z'],f['parameters']['p'],0,*[f['q_rad'][j] for j in JOINTS]] for f in frames])
    transforms=[]
    for w in keys:
        v=w['parameters'];r=r_y(v['p']);base=np.array([v['x'],0,v['z']]);links={}
        for name in a.s.model.collision_link_names:
            tf=a.s.collision_transform(name,w['q_rad']);matrix=np.eye(4);matrix[:3,:3]=r@tf.rotation;matrix[:3,3]=r@tf.translation+base;links[name]=matrix.tolist()
        transforms.append({'phase':w['phase'],'q_rad':w['q_rad'],'world_T_links':links,'plane_contacts':w['ground'],'core_roundtrip':w['core']})
    (OUT/'keypoint_world_transforms.json').write_text(json.dumps(transforms,indent=2)+'\n')
    display={n:a.s.mesh(n).triangles_local[::max(1,math.ceil(len(a.s.mesh(n).triangles_local)/220))] for n in a.s.model.collision_link_names}
    colors={n:('#64748b' if n=='base_link' else '#ef4444' if n.startswith(('rh','lh')) else '#2563eb') for n in display}
    def meshes(f):
        r=r_y(f['parameters']['p']);b=np.array([f['parameters']['x'],0,f['parameters']['z']]);out={}
        for n,tri in display.items():
            tf=a.s.collision_transform(n,f['q_rad']);out[n]=((tri@tf.rotation.T+tf.translation)@r.T+b)
        return out
    selected=[0,30,70,90,125,190,251]
    fig,axes=plt.subplots(2,len(selected),figsize=(17,5),sharey=True)
    for column,idx in enumerate(selected):
        f=frames[idx];m=meshes(f)
        for row,dimensions in enumerate(((0,2),(1,2))):
            ax=axes[row,column]
            for n,tri in m.items():ax.scatter(tri.reshape(-1,3)[:,dimensions[0]]*1000,tri.reshape(-1,3)[:,2]*1000,s=.12,c=colors[n])
            ax.axhline(0,c='black',lw=.6);ax.set_aspect('equal');ax.set_title(f['phase'][:18],fontsize=8);ax.set_xlabel(('X' if row==0 else 'Y')+' mm')
            if column==0:ax.set_ylabel('Z mm')
    fig.suptitle('Canonical STL projections — synthetic geometry, load unverified');fig.tight_layout();fig.savefig(OUT/'projections_xz_yz.png',dpi=160);plt.close(fig)
    t=np.array([f['time_s'] for f in frames]);fig,ax=plt.subplots(4,1,figsize=(11,10),sharex=True)
    for l in ('rh','lf'):
        for p in ('upper_leg','lower_leg'):ax[0].plot(t,np.degrees([f['q_rad'][l+'_'+p+'_joint'] for f in frames]),label=l+' '+p)
    ax[0].legend(ncol=2);ax[0].set_ylabel('pair U/L degrees')
    for l in LEGS:ax[1].plot(t,np.degrees([f['q_rad'][l+'_hip_joint'] for f in frames]),label=l)
    ax[1].legend(ncol=4);ax[1].set_ylabel('HIP degrees')
    ax[2].plot(t,[f['parameters']['z']*1000 for f in frames]);ax[2].set_ylabel('base_link Z mm')
    ax[3].plot(t,np.degrees([f['parameters']['p'] for f in frames]),label='pitch');ax[3].plot(t,np.zeros(len(t)),label='roll');ax[3].legend();ax[3].set_ylabel('degrees');ax[3].set_xlabel('study time s; rates are not hardware approved')
    fig.tight_layout();fig.savefig(OUT/'paired_angles_height_attitude.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(3,1,figsize=(11,8),sharex=True)
    for i,l in enumerate(LEGS):
        feet=np.array([f['feet'][i] for f in frames]);ax[0].plot(t,feet[:,0]*1000,label=l);ax[1].plot(t,feet[:,2]*1000,label=l)
        ax[2].plot(t,[i+(0 if f['stance'] is None else .5 if f['stance'][i] else 1) for f in frames],label=l)
    ax[0].legend(ncol=4);ax[0].set_ylabel('foot ref X mm');ax[1].set_ylabel('foot ref Z mm');ax[2].set_ylabel('contact: unknown / assumed stance / swing');ax[2].set_xlabel('study time s');fig.tight_layout();fig.savefig(OUT/'feet_contacts.png',dpi=160);plt.close(fig)
    fig=plt.figure(figsize=(8,5));axis=fig.add_subplot(projection='3d');axis.set(xlim=(-.33,.26),ylim=(-.18,.18),zlim=(-.005,.25),xlabel='X m',ylabel='Y m',zlabel='Z m');axis.set_box_aspect((.59,.36,.255));axis.view_init(20,-55)
    collections={n:Poly3DCollection(tri,facecolor=colors[n],alpha=.75,edgecolor='none') for n,tri in meshes(frames[0]).items()}
    for coll in collections.values():axis.add_collection3d(coll)
    grid=np.array([[[-.33,-.18,0],[.26,-.18,0],[.26,.18,0],[-.33,.18,0]]]);axis.add_collection3d(Poly3DCollection(grid,facecolor='#cccccc',alpha=.18))
    indexes=np.linspace(0,len(frames)-1,60).astype(int)
    def update(index):
        f=frames[indexes[index]]
        for n,tri in meshes(f).items():collections[n].set_verts(tri)
        axis.set_title(f['phase']+' | SYNTHETIC / no hardware GO',fontsize=9);return list(collections.values())
    update(0);fig.savefig(OUT/'canonical_motion_3d.png',dpi=160)
    FuncAnimation(fig,update,frames=len(indexes),interval=150,blit=False).save(OUT/'canonical_motion_3d.gif',writer=PillowWriter(fps=7),dpi=90);plt.close(fig)
    pins={str(p.relative_to(a.m.scene.model.urdf_path.parents[3])):hashlib.sha256(p.read_bytes()).hexdigest() for p in [a.m.scene.model.urdf_path]+[a.s.model.collision_mesh_path(n) for n in display]}
    artifact={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in OUT.iterdir() if p.is_file() and p.suffix in ('.png','.gif','.csv','.jsonl')}
    (OUT/'export_manifest.json').write_text(json.dumps({'schema':'MATDOG_PAIRED_EXPORT_V1','display_triangle_counts':{n:len(v) for n,v in display.items()},'collision_audit_uses_full_meshes':True,'source_pins':pins,'artifacts':artifact,'frames':len(frames),'intermediate_restart_source_segment':start,'intermediate_restart_certificate_reused':True,'load_proven':False,'hardware_authorized':False},indent=2)+'\n')
    print('EXPORT PASS',len(frames),'frames, 17 canonical STL, animation and projections')
if __name__=='__main__':main()
