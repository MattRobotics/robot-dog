#!/usr/bin/env python3
"""G2 differential contact/stand validation using canonical Python tools only."""
import itertools
import json
import math
from pathlib import Path
import random
import shutil
import subprocess
import sys
import tempfile
import yaml

ROOT = Path(__file__).resolve().parents[4]
KIN = ROOT / '06_Software/Matdog_Core/kinematics'
sys.path.insert(0, str(KIN))
from matdog_urdf_fk import canonical_urdf_path, load_urdf_joints
from matdog_quadruped_leg_contact import leg_foot_contact_from_joint_angles, leg_joint_names
from matdog_quadruped_leg_contact_ik import solve_leg_contact_reference_ik
from matdog_offline_rest_to_stand_trajectory import _evaluate_frame
from matdog_contact_stand_export import render_contact, render_stand, C4A, C4C, DEST, CONFIG_RELATIVE_PATH

LEGS = ('lf','rf','rh','lh')
URDF = canonical_urdf_path(ROOT)


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def contact(leg, q, height=0.0, normal=(0.,0.,1.)):
    return leg_foot_contact_from_joint_angles(
        leg, dict(zip(leg_joint_names(leg),q)), repo_root=ROOT,
        world_from_base_translation_m=(0.,0.,height), ground_normal_world_unit=normal).contact


def export_checks():
    for name, rendered in [('FootContactData.h',render_contact()),('StandReferenceData.h',render_stand())]:
        check((ROOT/DEST/name).read_text()==rendered,f'stale {name}')
    mutations = [
        ('rigid_cylinder','center_in_foot_link_m',[0.001,0,0.0149]),
        ('rigid_cylinder','axis_in_foot_link_unit',[1,0,0]),
        ('rigid_cylinder','radius_m',float('nan')),
        ('support_surface','central_rigid_support_width_m',0.008),
    ]
    with tempfile.TemporaryDirectory(prefix='matdog-contact-export-') as directory:
        root=Path(directory)
        for relative in (URDF.relative_to(ROOT),CONFIG_RELATIVE_PATH,C4A,C4C):
            (root/relative).parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(ROOT/relative,root/relative)
        original=(ROOT/CONFIG_RELATIVE_PATH).read_text()
        for section,field,value in mutations:
            data=yaml.safe_load(original);data[section][field]=value
            (root/CONFIG_RELATIVE_PATH).write_text(yaml.safe_dump(data))
            try: render_contact(root)
            except (ValueError, RuntimeError): continue
            raise AssertionError(f'unsupported contact model accepted: {section}/{field}')
        (root/CONFIG_RELATIVE_PATH).write_text(original)
        data=json.loads((root/C4C).read_text())
        data['frames'][25]['legs']['lh']['target_contact_reference_world_m'][0]+=0.01
        (root/C4C).write_text(json.dumps(data))
        try: render_stand(root)
        except ValueError: pass
        else: raise AssertionError('unlocked C4 frame accepted')
        shutil.copyfile(ROOT/C4C,root/C4C)
        data=json.loads((root/C4A).read_text())
        data['body_pose']['roll_rad']=0.1
        (root/C4A).write_text(json.dumps(data))
        try: render_stand(root)
        except ValueError: pass
        else: raise AssertionError('rotated C4-A body accepted')
        data=json.loads((ROOT/C4A).read_text())
        data['legs']['lh']['joint_positions_rad']['lh_lower_leg_joint']=4.0
        (root/C4A).write_text(json.dumps(data))
        try: render_stand(root)
        except ValueError: pass
        else: raise AssertionError('out-of-limit C4-A seed accepted')
    return len(mutations)+3


def main():
    check(len(sys.argv)==2,'usage: test_contact_stand_oracle.py DRIVER')
    driver=sys.argv[1]
    mutations=export_checks()
    rng=random.Random(20260929)
    joints=load_urdf_joints(URDF)
    cases=[]
    for leg in LEGS:
        limits=[(joints[n].lower_limit_rad,joints[n].upper_limit_rad) for n in leg_joint_names(leg)]
        samples=[(0.,0.,0.)]
        samples.extend(itertools.product(*[(lo,(lo+hi)/2,hi) for lo,hi in limits]))
        samples.extend(tuple(rng.uniform(lo,hi) for lo,hi in limits) for _ in range(128))
        # Nominal threshold and tilt extremes, with all real distal offsets.
        samples.extend((h,0.4,-0.2) for h in (0.034906584,0.034906586,-0.034906586))
        for q in samples:
            cases.append((leg,q,(0.,0.,1.)))
        for normal in ((0.1,0.2,1.),(0.2,-0.4,0.8),(-1.,0.,1.)):
            cases.append((leg,(0.2,0.4,-0.2),normal))
    lines=[]; expected=[]
    for leg,q,normal in cases:
        fk=contact(leg,q,normal=normal)
        up=contact(leg,q).cross_section_contact_center_world_m
        lines.append(' '.join(str(v) for v in (LEGS.index(leg),*q,*up,*q,*normal,0)))
        expected.append(fk)
    payload='\n'.join(lines)+'\n'
    def run(arguments=(),text=None):
        return subprocess.run([driver,*arguments],input=text,text=True,capture_output=True,check=True).stdout
    output=run(text=payload)
    check(output==run(text=payload),'nondeterministic contact rerun')
    rows=[json.loads(line) for line in output.splitlines()]
    check(len(rows)==len(cases),'wrong contact vector count')
    max_fk=max_ik=max_tilt=0.
    fields={'center':'cylinder_center_world_m','axis':'cylinder_axis_world_unit',
            'radial':'radial_down_world_unit','reference':'cross_section_contact_center_world_m',
            'end_a':'support_strip_end_a_world_m','end_b':'support_strip_end_b_world_m','lowest':'lowest_core_point_world_m'}
    for (leg,q,normal),py,row in zip(cases,expected,rows):
        check(row['fk']['status']==0 and row['ik_status']==0,f'contact failure {leg}/{q}: {row}')
        for field,attribute in fields.items():
            error=math.dist(row['fk'][field],getattr(py,attribute))
            check(error<1e-12,f'contact FK mismatch {leg}/{field}: {error}')
            max_fk=max(max_fk,error)
        tilt=abs(row['fk']['tilt']-py.axis_tilt_from_ground_rad)
        check(tilt<1e-12,'tilt mismatch');max_tilt=max(max_tilt,tilt)
        check(row['fk']['mode']==(1 if py.support_mode=='NOMINAL_STRIP_CONTACT' else 2),'mode mismatch')
        achieved=contact(leg,row['q']).cross_section_contact_center_world_m
        target=contact(leg,q).cross_section_contact_center_world_m
        error=math.dist(achieved,target)
        check(error<=1e-9,'contact IK round-trip mismatch')
        check(max(abs(a-b) for a,b in zip(q,row['q']))<1e-8,'seed-nearest branch mismatch')
        max_ik=max(max_ik,error)

    output=run(('--stand',))
    check(output==run(('--stand',)),'nondeterministic stand rerun')
    generated=json.loads(output)
    old_a=json.loads((ROOT/C4A).read_text());old_c=json.loads((ROOT/C4C).read_text())
    check(len(generated['frames'])==len(old_c['frames'])==51,'wrong C4-C frame count')
    joint_error_a=joint_error_c=contact_residual=contact_drift=delta_difference=0.
    refined_ik_error=0.;refined_count=0
    maxima=[[0.]*3 for _ in LEGS];branch_changes=[0]*4
    min_margin=math.inf;min_ground=math.inf;min_knee=math.inf
    previous=None;first=None
    sequence=[('C4A',generated['stand'],old_a)]+[(str(i),g,r) for i,(g,r) in enumerate(zip(generated['frames'],old_c['frames']))]
    for label,frame,reference in sequence:
        check(frame['status']==0 and frame['valid'],f'invalid stand {label}')
        height=frame['height']
        check(abs(height-reference['body_pose']['translation_world_m'][2])<1e-12,'body height mismatch')
        candidate={'body_pose':{'translation_world_m':[0.,0.,height],'roll_rad':0.,'pitch_rad':0.,'yaw_rad':0.,
                               'base_link_parallel_to_ground':True},'legs':{}}
        frame_residual=frame_drift=0.
        for i,leg in enumerate(LEGS):
            values=frame['legs'][i];q=values['q'];record=reference['legs'][leg]
            py=contact(leg,q,height)
            target=record['target_contact_reference_world_m']
            point=py.cross_section_contact_center_world_m
            residual=math.dist(point,target)
            check(residual<=1e-9 and abs(point[2])<=1e-9,'unlocked physical contact')
            check(math.dist(point,values['contact'])<1e-12,'C++/Python world contact mismatch')
            check(py.support_mode=='NOMINAL_STRIP_CONTACT','stand contact mode')
            check(values['branch']==[-1,-1],'stand branch change')
            frame_residual=max(frame_residual,residual)
            archived_q=[record['joint_positions_rad'][n] for n in leg_joint_names(leg)]
            q_error=max(abs(a-b) for a,b in zip(q,archived_q))
            check(q_error<3e-4,f'archived branch mismatch {label}/{leg}: {q_error}')
            if label=='C4A':joint_error_a=max(joint_error_a,q_error)
            else:joint_error_c=max(joint_error_c,q_error)
            for j,n in enumerate(leg_joint_names(leg)):
                margin=min(q[j]-joints[n].lower_limit_rad,joints[n].upper_limit_rad-q[j])
                check(margin>=0 and abs(margin-values['margin'][j])<1e-12,'incorrect joint margin')
                if label!='C4A':min_margin=min(min_margin,margin)
            if label in ('C4A','0','25','50'):
                # Existing numerical solver refines archived joints to an exact contact target.
                solved=solve_leg_contact_reference_ik(leg,tuple(target),repo_root=ROOT,
                    initial_guess_rad=tuple(archived_q),world_from_base_translation_m=(0.,0.,height),
                    tolerance_m=1e-10)
                err=max(abs(q[j]-solved.joint_positions_rad[n]) for j,n in enumerate(leg_joint_names(leg)))
                check(err<1e-7,f'Python refined IK mismatch {label}/{leg}: {err}')
                refined_ik_error=max(refined_ik_error,err);refined_count+=1
            candidate['legs'][leg]={'joint_positions_rad':dict(zip(leg_joint_names(leg),q)),
                'achieved_contact_reference_world_m':point,'support_mode':py.support_mode}
            if label!='C4A':
                delta=[q[j]-previous['legs'][i]['q'][j] for j in range(3)] if previous else [0.]*3
                check(max(abs(a-b) for a,b in zip(delta,values['delta']))<1e-12,'incorrect per-frame delta')
                maxima[i]=[max(maxima[i][j],abs(delta[j])) for j in range(3)]
                if previous:
                    old_previous=old_c['frames'][int(label)-1]['legs'][leg]['joint_positions_rad']
                    old_delta=[record['joint_positions_rad'][n]-old_previous[n] for n in leg_joint_names(leg)]
                    delta_difference=max(delta_difference,max(abs(a-b) for a,b in zip(delta,old_delta)))
                    check(max(abs(a-b) for a,b in zip(delta,old_delta))<3e-4,'trajectory discontinuity vs C4-C')
                    branch_changes[i]+=int(values['branch']!=previous['legs'][i]['branch'])
                anchor=first['legs'][i]['contact'] if first else values['contact']
                frame_drift=max(frame_drift,math.dist(values['contact'],anchor))
        check(abs(frame['residual']-frame_residual)<1e-12,'incorrect frame residual')
        if label!='C4A':
            check(abs(frame['drift']-frame_drift)<1e-12,'incorrect frame drift')
            contact_residual=max(contact_residual,frame_residual);contact_drift=max(contact_drift,frame_drift)
            previous=frame
            if first is None:first=frame
        # Reuse existing C4 mesh-ground/knee policy on the actual new angles.
        evaluation=_evaluate_frame(repo_root=ROOT,urdf_path=URDF,frame=candidate)
        check(evaluation['safe'],f'existing C4 policy failed {label}: {evaluation["policy_status"]}')
        min_ground=min(min_ground,evaluation['non_foot_min_z_m'])
        min_knee=min(min_knee,evaluation['knee_clearance_min_m'])
    metrics=generated['metrics']
    check(metrics['samples']==51 and branch_changes==metrics['branch_changes']==[0]*4,'incorrect branch metrics')
    check(max(abs(maxima[i][j]-metrics['max_delta'][i][j]) for i in range(4) for j in range(3))<1e-12,'incorrect maxima')
    check(abs(min_margin-metrics['min_margin'])<1e-12,'incorrect minimum margin')
    check(abs(contact_residual-metrics['max_residual'])<1e-12 and abs(contact_drift-metrics['max_drift'])<1e-12,'incorrect residual/drift maxima')
    check(abs(max(max(v) for v in maxima)-metrics['max_joint_delta'])<1e-12,'incorrect global delta')
    print(f'CONTACT_STAND_ORACLE = PASS: {len(cases)} contact vectors; C4-A + 51 C4-C frames; '
          f'{refined_count} Python contact IK refinements; {mutations} rejected model/reference mutations')
    print(f'max contact FK={max_fk:.12e} m; tilt={max_tilt:.12e} rad; contact IK={max_ik:.12e} m')
    print(f'C4-A max joint difference={joint_error_a:.12e} rad; C4-C={joint_error_c:.12e} rad; '
          f'refined Python IK={refined_ik_error:.12e} rad; delta difference={delta_difference:.12e} rad')
    print('STAND_METRICS '+json.dumps(metrics,sort_keys=True))
    print(f'C4_POLICY = PASS 52/52 poses; min non-foot ground={min_ground:.12e} m; min knee/contact={min_knee:.12e} m')


if __name__=='__main__':
    main()
