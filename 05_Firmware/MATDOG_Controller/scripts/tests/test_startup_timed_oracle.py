#!/usr/bin/env python3
"""G3 timing/derivative oracle and existing C4 ground/support policy replay."""
import json
import math
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'06_Software/Matdog_Core/kinematics'))
from matdog_quadruped_leg_contact import leg_foot_contact_from_joint_angles, leg_joint_names
from matdog_quadruped_leg_contact_ik import solve_leg_contact_reference_ik
from matdog_offline_rest_to_stand_trajectory import _evaluate_frame as collision_policy
from matdog_offline_static_stability_support_polygon import _evaluate_frame as support_policy
from matdog_urdf_fk import canonical_urdf_path, load_urdf_joints
from matdog_contact_stand_export import C4A, C4C

LEGS=('lf','rf','rh','lh')
DURATIONS=(2.,5.,10.)
INTERVALS=100

def check(value,message):
    if not value:raise AssertionError(message)

def main():
    check(len(sys.argv)==2,'usage: test_startup_timed_oracle.py DRIVER')
    c4a=json.loads((ROOT/C4A).read_text());c4c=json.loads((ROOT/C4C).read_text())
    low=c4c['trajectory_policy']['start_body_z_m'];high=c4c['trajectory_policy']['final_body_z_m']
    urdf=canonical_urdf_path(ROOT);joints=load_urdf_joints(urdf)
    runs=[]
    for duration in DURATIONS:
        args=[sys.argv[1],str(duration),str(INTERVALS)]
        text=subprocess.check_output(args,text=True)
        check(text==subprocess.check_output(args,text=True),'non-deterministic rerun')
        runs.append(json.loads(text))
    ground_min=knee_min=support_min=math.inf
    contact_max=drift_max=0.
    for k,(duration,run) in enumerate(zip(DURATIONS,runs)):
        frames=run['frames'];check(len(frames)==INTERVALS+1,'sample count')
        peak_v=[[0.]*3 for _ in LEGS];peak_a=[[0.]*3 for _ in LEGS]
        min_margin=math.inf;previous_time=-1.;previous_s=-1.
        for index,frame in enumerate(frames):
            check(frame['status']==0 and frame['valid'],'invalid timed frame')
            check(frame['time']>previous_time and frame['s']>=previous_s,'nonmonotonic timestamps/progress')
            previous_time=frame['time'];previous_s=frame['s']
            u=index/INTERVALS;expected_s=10*u**3-15*u**4+6*u**5
            check(abs(frame['time']-duration*u)<1e-12 and abs(frame['s']-expected_s)<3e-15,'time-law mismatch')
            check(abs(frame['height']-(low+(high-low)*expected_s))<1e-12,'timing changed geometric path')
            check(frame['state']==(3 if index==INTERVALS else 2),'STAND before entire valid trajectory')
            check(frame['phase']==(0 if index==0 else 1),'incorrect support segment')
            candidate={'body_pose':{'translation_world_m':[0.,0.,frame['height']],
                         'roll_rad':0.,'pitch_rad':0.,'yaw_rad':0.,'base_link_parallel_to_ground':True},'legs':{}}
            for i,leg in enumerate(LEGS):
                data=frame['legs'][i];q=data['q'];check(data['branch']==[-1,-1],'analytic branch discontinuity')
                if k:
                    reference=runs[0]['frames'][index]['legs'][i]
                    check(q==reference['q'],'duration changed the path')
                    for j in range(3):
                        check(abs(data['v'][j]*duration-reference['v'][j]*DURATIONS[0])<1e-12,'1/T velocity scaling')
                        check(abs(data['a'][j]*duration**2-reference['a'][j]*DURATIONS[0]**2)<1e-12,'1/T^2 acceleration scaling')
                if index in (0,INTERVALS):check(data['v']==[0.,0.,0.] and data['a']==[0.,0.,0.],'nonzero endpoint rates')
                names=leg_joint_names(leg)
                for j,name in enumerate(names):
                    margin=min(q[j]-joints[name].lower_limit_rad,joints[name].upper_limit_rad-q[j])
                    check(margin>=0 and abs(margin-data['margin'][j])<1e-12,'joint limit/margin failure')
                    min_margin=min(min_margin,margin)
                    peak_v[i][j]=max(peak_v[i][j],abs(data['v'][j]));peak_a[i][j]=max(peak_a[i][j],abs(data['a'][j]))
                py=leg_foot_contact_from_joint_angles(leg,dict(zip(names,q)),repo_root=ROOT,
                    world_from_base_translation_m=(0.,0.,frame['height'])).contact
                point=py.cross_section_contact_center_world_m
                target=c4a['legs'][leg]['target_contact_reference_world_m']
                residual=math.dist(point,target)
                check(residual<1e-9 and py.support_mode=='NOMINAL_STRIP_CONTACT','invalid physical contact')
                check(math.dist(point,data['contact'])<1e-12,'world/base conversion mismatch')
                contact_max=max(contact_max,residual)
                drift_max=max(drift_max,math.dist(point,frames[0]['legs'][i]['contact']))
                candidate['legs'][leg]={'joint_positions_rad':dict(zip(names,q)),
                    'achieved_contact_reference_world_m':point,'support_mode':py.support_mode}
            # Durations have byte-identical poses (checked above). Evaluate each
            # distinct frame with the existing collision/support oracles once;
            # that same result applies to every duration's corresponding frame.
            if k==0:
                evaluated=collision_policy(repo_root=ROOT,urdf_path=urdf,frame=candidate)
                support=support_policy(frame=candidate,com_proxy_world_xy_m=(0.,0.),
                                       com_uncertainty_xy_m=0.020,min_support_margin_m=0.)
                check(evaluated['safe'],'C4 ground/knee policy failure')
                check(support['safe'],'C4 support-proxy policy failure')
                ground_min=min(ground_min,evaluated['non_foot_min_z_m'])
                knee_min=min(knee_min,evaluated['knee_clearance_min_m'])
                support_min=min(support_min,support['min_support_margin_m'])
        metrics=run['metrics']
        check(metrics['velocity']==peak_v and metrics['acceleration']==peak_a,'incorrect peak metrics')
        check(abs(metrics['margin']-min_margin)<1e-12 and metrics['branch_changes']==[0]*4,'incorrect margin/branch metrics')
        check(frames[0]['time']==0 and frames[-1]['time']==duration,'wrong start/end time')
    # Independent numerical derivatives: five-point differences of the existing
    # Python CONTACT IK at nearby times (no new Python kinematics implementation).
    velocity_error=acceleration_error=0.;solves=0
    duration=5.;du=0.001
    for index in (10,30,50,70,90):
        center=runs[1]['frames'][index];u=index/INTERVALS
        for i,leg in enumerate(LEGS):
            neighbors={}
            names=leg_joint_names(leg)
            for step in (-2,-1,1,2):
                v=u+step*du;s=10*v**3-15*v**4+6*v**5
                solved=solve_leg_contact_reference_ik(leg,
                    tuple(c4a['legs'][leg]['target_contact_reference_world_m']),repo_root=ROOT,
                    initial_guess_rad=tuple(center['legs'][i]['q']),
                    world_from_base_translation_m=(0.,0.,low+(high-low)*s),tolerance_m=1e-12)
                neighbors[step]=[solved.joint_positions_rad[n] for n in names];solves+=1
            for j in range(3):
                f0=center['legs'][i]['q'][j]
                fm2,fm1,fp1,fp2=(neighbors[s][j] for s in (-2,-1,1,2))
                velocity=(fm2-8*fm1+8*fp1-fp2)/(12*du*duration)
                acceleration=(-fp2+16*fp1-30*f0+16*fm1-fm2)/(12*du*du*duration*duration)
                ev=abs(velocity-center['legs'][i]['v'][j]);ea=abs(acceleration-center['legs'][i]['a'][j])
                check(ev<1e-7,f'analytic velocity mismatch {leg}/{index}: {ev}')
                check(ea<2e-6,f'analytic acceleration mismatch {leg}/{index}: {ea}')
                velocity_error=max(velocity_error,ev);acceleration_error=max(acceleration_error,ea)
    print(f'STARTUP_TIMED_ORACLE = PASS: {len(DURATIONS)*(INTERVALS+1)} timed frames; '
          f'{INTERVALS+1} distinct C4 ground/knee/support evaluations; {solves} Python contact IK derivative solves')
    print(f'contact residual={contact_max:.12e} m; drift={drift_max:.12e} m; '
          f'non-foot clearance={ground_min:.12e} m; knee/contact={knee_min:.12e} m; support-proxy margin={support_min:.12e} m')
    print(f'derivative comparison max velocity error={velocity_error:.12e} rad/s; acceleration error={acceleration_error:.12e} rad/s^2')
    for run in runs:print('G3_METRICS '+json.dumps(run['metrics'],sort_keys=True))

if __name__=='__main__':main()
