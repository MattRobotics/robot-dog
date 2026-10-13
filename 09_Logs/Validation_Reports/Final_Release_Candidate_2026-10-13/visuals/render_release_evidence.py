#!/usr/bin/env python3
"""Original MATDOG visual evidence from STL, frozen knots and real test export."""
from pathlib import Path
import argparse,csv,json,math,hashlib,sys
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--runtime-csv',type=Path,required=True);p.add_argument('--viewer-trace',type=Path,required=True);a=p.parse_args()
rows=list(csv.DictReader(a.runtime_csv.open()));time=np.array([float(x['time_s']) for x in rows]);q=np.array([[float(x[f'q{i}']) for i in range(12)] for x in rows]);stand=np.array([x['phase']=='STAND' for x in rows]);seg=np.array([int(x['segment']) for x in rows]);base=json.loads((HERE.parent.parent/'Motion_Night_Run_2026-10-12/continuous_pair_path.json').read_text());recipe=json.loads((HERE.parent/'fixture_limit_compatible_route_6mrad.json').read_text())['recipe']
knots=[s['start_parameters'] for s in base['segments']]+[base['segments'][-1]['end_parameters']];original=np.array([[p['h']*(-1 if i in (0,3) else 1),p['uf'] if i<2 else p['ur'],p['lf'] if i<2 else p['lr']] for p in knots for i in range(4)]).reshape(252,12);variant=original.copy();variant[:,[2,5]]=np.maximum(variant[:,[2,5]],math.radians(recipe['front_lower_floor_deg']));variant[:,[8,11]]=np.maximum(variant[:,[8,11]],math.radians(recipe['rear_lower_floor_deg']))
fig,axes=plt.subplots(4,3,figsize=(13,9),sharex=True)
for i,ax in enumerate(axes.flat):ax.plot(np.degrees(original[:,i]),label='Original');ax.plot(np.degrees(variant[:,i]),label='Fixture variant',ls='--');ax.set_title(('LF','RF','RH','LH')[i//3]+' '+('HIP','UPPER','LOWER')[i%3]);ax.grid(alpha=.2)
axes[0,0].legend(fontsize=8);fig.suptitle('Frozen canonical route and named fixture derivative · degrees · no hardware pose assumed');fig.supxlabel('Certified knot');fig.tight_layout();fig.savefig(HERE/'route_joint_comparison.png',dpi=110);plt.close(fig)
fig,ax=plt.subplots(3,1,figsize=(12,8),sharex=True);ax[0].plot(time,q[:,2],label='LF LOWER target');ax[0].plot(time,q[:,8],label='RH LOWER target');ax[0].legend();ax[0].set_ylabel('rad');ax[1].plot(time,[float(x['target_z_m'])*1000 for x in rows],label='Virtual target reference');ax[1].plot(time,[float(x['actual_fixture_z_m'])*1000 for x in rows],label='Synthetic fixed fixture');ax[1].legend();ax[1].set_ylabel('base_link mm');ax[2].plot(time,[float(x['pitch_rad']) for x in rows],label='Virtual pitch');ax[2].plot(time,np.zeros(len(rows)),label='Roll');ax[2].set_ylabel('rad');ax[2].set_xlabel('Synthetic test clock s');ax[2].legend()
for x in ax:x.grid(alpha=.2);x.axvline(time[np.argmax(~stand)],color='#b55',ls=':')
fig.suptitle('Production planner export · complete fixture STAND and one compiled first stride · hardware=false');fig.tight_layout();fig.savefig(HERE/'runtime_targets_height_attitude.png',dpi=110);plt.close(fig)
# Public decimated replay: actual joint values are the explicitly ideal feedback
# doubles from the test, never measurements. Fixed body height stays 300 mm.
records=[]
for i in range(0,len(rows),max(1,len(rows)//2200)):
 r=rows[i];values=q[i].tolist();records.append(dict(version=1,now_ms=round(float(r['time_s'])*1000),last_sequence=int(r['sequence']),owner='offline_trace',executor_state='SYNTHETIC_TEST',stand_state=r['phase']+'_'+r['segment'],fault=False,reason='OFFLINE_SYNTHETIC_FIXED_FIXTURE_NO_LOAD_PROOF',stale=False,telemetry=dict(source='OFFLINE_SYNTHETIC',sampled_at_ms=round(float(r['time_s'])*1000),actual_rad=values,target_rad=values,quaternion=[1,0,0,0],base_position_m=[0,0,.3],base_height_m=.3,feet=None,external_support_confirmed=True)))
a.viewer_trace.write_text('\n'.join(json.dumps(x,separators=(',',':')) for x in records)+'\n')
# CAD projections/sequence use actual canonical triangles, deterministic drawing
# decimation only. Reuse the reviewed renderer; it never issues a certificate.
sys.path.insert(0,str(ROOT/'06_Software/Matdog_Core/calibration'));from matdog_geometry_scene_v5 import RobotSceneV5
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
scene=RobotSceneV5.from_urdf(ROOT/'03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf');joints=tuple(f'{leg}_{part}_joint' for leg in ('lf','rf','rh','lh') for part in ('hip','upper_leg','lower_leg'));chosen=[0,29,79,99,135,251];fig=plt.figure(figsize=(13,8));proj,paxes=plt.subplots(2,1,figsize=(11,7));counts={}
for ordinal,k in enumerate(chosen):
 ax=fig.add_subplot(2,3,ordinal+1,projection='3d');params=dict(zip(joints,variant[k]));
 for link in scene.model.collision_link_names:
  tr=scene.collision_transform(link,params);mesh=scene.mesh(link).triangles_local;stride=max(1,math.ceil(len(mesh)/450));tri=mesh[::stride]@tr.rotation.T+tr.translation+np.array([0,0,.3]);color='#7c8690' if link=='base_link' else ('#278cbe','#dc8939','#6caa55','#bd5f88')[('lf','rf','rh','lh').index(link[:2])];ax.add_collection3d(Poly3DCollection(tri,facecolor=color,edgecolor='none',alpha=.75));counts[link]=dict(triangles=len(mesh),displayed=len(tri),stride=stride)
  if ordinal in (0,5):
   for pa,dims in zip(paxes,((0,2),(1,2))):pa.add_collection(PolyCollection(tri[:,:,dims],facecolor=color,alpha=.25 if ordinal==0 else .55,edgecolor='none'))
 ax.set(xlim=(-.37,.27),ylim=(-.22,.22),zlim=(.10,.40),xlabel='X m',ylabel='Y m',zlabel='Z m');ax.set_box_aspect((.64,.44,.3));ax.view_init(elev=20,azim=-62);ax.set_title(f'knot {k} · '+('START' if k==0 else base['segments'][min(k,250)]['phase']),fontsize=9)
fig.suptitle('All 17 canonical collision STL · suspended fixture derivative · synthetic body Z=300mm');fig.tight_layout();fig.savefig(HERE/'fixture_stl_sequence.png',dpi=110);plt.close(fig)
for pa in paxes:pa.autoscale_view();pa.set_aspect('equal');pa.grid(alpha=.2)
paxes[0].set(xlabel='X m',ylabel='Z m');paxes[1].set(xlabel='Y m',ylabel='Z m');proj.suptitle('Real STL XZ / YZ projections · initial and C4 target · no contact/load assertion');proj.tight_layout();proj.savefig(HERE/'fixture_projections_xz_yz.png',dpi=110);plt.close(proj)
manifest=dict(schema='MATDOG_FINAL_VISUAL_EVIDENCE_V1',runtime_csv_sha256=hashlib.sha256(a.runtime_csv.read_bytes()).hexdigest(),frames=len(rows),public_replay_frames=len(records),canonical_route_sha256=hashlib.sha256((HERE.parent.parent/'Motion_Night_Run_2026-10-12/continuous_pair_path.json').read_bytes()).hexdigest(),fixture_recipe_sha256=json.loads((HERE.parent/'fixture_limit_compatible_route_6mrad.json').read_text())['recipe_sha256'],meshes=counts,hardware=False,body_pose='SYNTHETIC FIXED FIXTURE 300mm; not an operational measurement or preset',target_values='actual production planner test export; ideal feedback doubles',renderer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());(HERE/'render_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print('VISUAL_EVIDENCE',len(rows),'test frames;',len(counts),'STLs')
