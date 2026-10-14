#!/usr/bin/env python3
"""Independent URDF/contact and integral-law oracle plus explicitly synthetic plant studies.

Requires only the existing numpy/scipy/PyYAML offline tools. No device/network access.
All dynamics, delays, jitter and gains here are fixtures, not measured robot parameters.
"""
import argparse
import csv
import ctypes as C
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[4]
LEGS = ("lf", "rf", "rh", "lh")
CONTACTS = np.array([[.1301352313457785,.094,0],[.1301352313457785,-.094,0],
                     [-.1437169534863073,-.094,0],[-.1437169534863073,.094,0]])


class Bridge:
    def __init__(self, path):
        self.so=C.CDLL(str(path)); p=C.POINTER(C.c_double)
        self.so.g5b_init.argtypes=[C.c_double,C.c_double,C.c_uint,C.c_int]
        self.so.g5b_step.argtypes=[p,p]; self.so.g5b_step.restype=C.c_int
        self.so.g5b_reason.argtypes=[C.c_int]; self.so.g5b_reason.restype=C.c_char_p
        self.so.g5b_enable.argtypes=[C.c_double]
        self.so.g5b_benchmark.argtypes=[C.c_uint,p,C.POINTER(C.c_uint)]; self.so.g5b_benchmark.restype=C.c_double

    def init(self, gain=1., damping=0., delay=2, moving=False):
        assert self.so.g5b_init(gain,damping,delay,int(moving))

    def step(self,r,p,t,seq,now=None,lead=0,plane=(0,0),rate=(0,0),**fault):
        now=t if now is None else now
        data=np.array([r,p,t,seq,now,now+lead,*plane,fault.get('connected',1),1,*rate,1,
                       fault.get('qscale',1),fault.get('confidence',1),fault.get('unknown',-1),
                       fault.get('support_stamp',now),fault.get('dz',0),fault.get('far',0),0],dtype=np.float64)
        out=np.zeros(64,dtype=np.float64)
        reason=self.so.g5b_step(data.ctypes.data_as(C.POINTER(C.c_double)),out.ctypes.data_as(C.POINTER(C.c_double)))
        return self.so.g5b_reason(reason).decode(),out


class UrdfOracle:
    """XML-driven homogeneous transforms; no C++ generated geometry or solver used."""
    def __init__(self):
        self.path=ROOT/'03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
        self.xml=ET.parse(self.path).getroot()
        self.joints={j.get('name'):j for j in self.xml.findall('joint')}
        geometry=yaml.safe_load((ROOT/'06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml').read_text())
        self.center=np.array(geometry['rigid_cylinder']['center_in_foot_link_m'])
        self.axis=np.array(geometry['rigid_cylinder']['axis_in_foot_link_unit'])
        self.radius=geometry['rigid_cylinder']['radius_m']
        self.limits=np.array([[float(self.joints[f'{l}_{j}_joint'].find('limit').get(k)) for k in ('lower','upper')]
                             for l in LEGS for j in ('hip','upper_leg','lower_leg')])

    def contact(self,leg,q,body,normal):
        tf=np.eye(4); tf[:3,:3]=body[:9].reshape(3,3); tf[:3,3]=body[9:12]
        for suffix,value in zip(('hip','upper_leg','lower_leg','foot'),[*q,0.]):
            j=self.joints[f'{leg}_{suffix}_joint']; origin=j.find('origin')
            offset=np.eye(4)
            offset[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
            offset[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            angle=np.eye(4)
            if j.get('type')!='fixed':
                axis=np.fromstring(j.find('axis').get('xyz'),sep=' ')
                angle[:3,:3]=Rotation.from_rotvec(axis*value).as_matrix()
            tf=tf@offset@angle
        axis=tf[:3,:3]@self.axis; center=tf[:3,:3]@self.center+tf[:3,3]
        projected=normal-axis*np.dot(normal,axis)
        return center-self.radius*projected/np.linalg.norm(projected)


def geometry_checks(lib):
    oracle=UrdfOracle(); worst=0.; joint_error=0.; cases=0; accepted=0; refusals={}
    rng=np.random.default_rng(51014)
    for moving in (False,True):
        for _ in range(36):
            lib.init(moving=moving)
            r,p=rng.uniform(-.03,.03,2)
            plane=(0,float(rng.uniform(-.002,.002))) if moving else (0,0)
            reason,out=lib.step(r,p,.02,1,plane=plane)
            cases+=1
            if reason!='OK': refusals[reason]=refusals.get(reason,0)+1; continue
            accepted+=1
            plane_r=Rotation.from_euler('xyz',[*plane,0]).as_matrix()
            normal=plane_r[:,2]; target=CONTACTS@plane_r.T
            q=out[16:28].reshape(4,3); body=out[28:40]
            assert out[2] and out[55]==moving
            for i,leg in enumerate(LEGS):
                contact=oracle.contact(leg,q[i],body,normal)
                worst=max(worst,float(np.linalg.norm(contact-target[i])))
                # SciPy bounded least squares selects the same near-seed branch independently.
                bounds=oracle.limits[3*i:3*i+3]
                solution=least_squares(lambda qi:oracle.contact(leg,qi,body,normal)-target[i],q[i]+.001,
                    bounds=(bounds[:,0],bounds[:,1]),ftol=1e-14,xtol=1e-14,gtol=1e-14)
                assert np.linalg.norm(solution.fun)<1e-9
                joint_error=max(joint_error,float(np.max(np.abs(solution.x-q[i]))))
            assert np.all(q.ravel()>=oracle.limits[:,0]) and np.all(q.ravel()<=oracle.limits[:,1])
    assert accepted==cases and worst<1e-9 and joint_error<1e-7
    return dict(status='OFFLINE VALIDATED',cases=cases,accepted=accepted,refusals=refusals,
                worst_contact_residual_m=worst,worst_joint_difference_rad=joint_error,
                urdf_sha256=hashlib.sha256(oracle.path.read_bytes()).hexdigest())


def law_checks(lib):
    lib.init(); reference=np.zeros(2); maximum=0.; t=0.
    # Nonuniform sampling + intervening polling: oracle integrates from the last new sample.
    for k in range(1,101):
        dt=[.014,.026,.018,.022][k%4]; t+=dt
        measured=np.array([.004*math.sin(2*t),.006*math.cos(t)])
        target=np.clip(reference-dt*measured,-.03,.03)
        reference+=np.clip(target-reference,-.04*dt,.04*dt)
        reason,out=lib.step(*measured,t,k)
        assert reason=='OK'; maximum=max(maximum,float(np.max(np.abs(reference-out[:2]))))
        reason,dup=lib.step(*measured,t,k,now=t+.001,support_stamp=t)
        assert reason=='DUPLICATE_SAMPLE' and not dup[2]
    assert maximum<1e-12
    return dict(status='OFFLINE VALIDATED',new_samples=100,duplicate_polls=100,max_correction_error_rad=maximum)


def studies(lib,outdir):
    rows=[]; results=[]
    scenarios=[('level',0.,0.,0.,False),('roll_positive',.015,0.,0.,False),('roll_negative',-.015,0.,0.,False),
               ('pitch_positive',0.,.025,0.,False),('pitch_negative',0.,-.025,0.,False),('combined',.012,-.015,0.,False),
               ('periodic',.002,0.,0.,False),('delay_jitter',.01,-.015,0.,False),('saturation',0.,.3,0.,False),
               ('gradual_plane',0.,.015,0.,True),('damping_baseline',.012,.02,0.,False),('damping_experiment',.012,.02,.15,False)]
    for name,dr,dp,damping,moving in scenarios:
        lib.init(damping=damping,moving=moving,delay=3 if 'damping' in name else 2)
        t=0.; plant=np.zeros(2); history=[(0.,np.zeros(2))]; measured_previous=np.zeros(2)
        trace=[]; saturated=0; rejects={}; largest_drift=0.; largest_delta=0.; minimum_margin=math.inf
        for k in range(1,601):
            dt=[.014,.026,.018,.022][k%4] if name=='delay_jitter' else .02; t+=dt
            # Synthetic first-order command response (tau=80ms) and explicit transport delay.
            delay=.04 if 'damping' not in name else .06
            command=next((q for at,q in reversed(history) if at<=t-delay),np.zeros(2))
            plant+=(1-math.exp(-dt/.08))*(command-plant)
            disturbance=np.array([dr,dp])
            if name=='periodic': disturbance=np.array([dr*math.sin(2*math.pi*.6*t),0.])
            if moving: disturbance*=min(1.,t/3.)
            measured=plant+disturbance
            rate=(measured-measured_previous)/dt; measured_previous=measured.copy()
            latency=(.003+.002*(k%4)) if name=='delay_jitter' else 0.
            reason,o=lib.step(*measured,t,k,now=t+latency,lead=.005 if latency else 0,
                              plane=(0,dp*min(1.,t/3.)) if moving else (0,0),rate=rate)
            if reason=='OK':
                history.append((t+latency+(.005 if latency else 0),o[:2].copy())); saturated+=int(o[3]); largest_drift=max(largest_drift,o[5]);
                largest_delta=max(largest_delta,o[7]); minimum_margin=min(minimum_margin,o[6])
            else: rejects[reason]=rejects.get(reason,0)+1
            trace.append((t,*measured,*o[:2],o[10],o[11]))
            rows.append([name,k,t,*measured,*o[:2],reason,int(o[2]),int(o[3]),o[5],o[7],o[10],o[11],*o[16:28]])
        a=np.array(trace); tail=a[-100:,1:3]; initial=np.array([dr,dp]); overshoot=np.zeros(2)
        control_times=a[:,0]+a[:,5]
        correction_rates=np.abs(np.diff(np.vstack([np.zeros(2),a[:,3:5]]),axis=0))/np.diff(np.r_[0.,control_times])[:,None]
        for axis in range(2):
            if initial[axis]!=0: overshoot[axis]=max(0.,float(np.max(-np.sign(initial[axis])*a[:,1+axis])))/abs(initial[axis])
        settled=None
        if name not in ('periodic','saturation','gradual_plane'):
            bad=np.where(np.max(np.abs(a[:,1:3]),axis=1)>.001)[0]
            settled=float(a[bad[-1]+1,0]) if len(bad) and bad[-1]+1<len(a) else (0. if not len(bad) else None)
        assert not rejects, (name,rejects)
        if name not in ('periodic','saturation'): assert np.max(np.abs(tail))<1e-4,(name,tail[-1])
        if name=='saturation': assert saturated>0 and abs(a[-1,4])<=.03
        result=dict(name=name,status='SIMULATED',plant='first_order_tau_80ms_plus_delay_and_disturbance',
                    samples=600,duration_s=float(t),damping_time_s=damping,settling_time_to_1mrad_s=settled,
                    tail_max_error_rad=float(np.max(np.abs(tail))),tail_rms_error_rad=float(np.sqrt(np.mean(tail**2))),
                    overshoot_fraction=None if name=='periodic' else overshoot.tolist(),saturated_samples=saturated,rejections=rejects,
                    max_contact_residual_m=float(largest_drift),max_joint_step_rad=float(largest_delta),min_joint_margin_rad=float(minimum_margin),
                    max_correction_rate_rad_s=float(correction_rates.max()),rate_limited_samples=int(np.sum(np.max(correction_rates,axis=1)>=.04-1e-10)),
                    measurement_age_range_s=[float(a[:,5].min()),float(a[:,5].max())],measurement_to_command_range_s=[float(a[:,6].min()),float(a[:,6].max())])
        results.append(result)
        if name=='periodic':
            result['tail_rms_roll_to_disturbance_rms_ratio']=float(np.sqrt(np.mean(tail[:,0]**2))/(abs(dr)/math.sqrt(2)))
    headers=['scenario','step','time_s','measured_roll_rad','measured_pitch_rad','correction_roll_rad','correction_pitch_rad',
             'reason','target_valid','saturated','contact_residual_m','joint_step_rad','measurement_age_s','measurement_to_command_s']
    headers += [f'{leg}_{joint}_rad' for leg in LEGS for joint in ('hip','upper','lower')]
    with (outdir/'simulation_trace.csv').open('w') as f:
        writer=csv.writer(f,lineterminator='\n'); writer.writerow(headers); writer.writerows(rows)
    return results


def fault_checks(lib):
    cases=[('disconnected',dict(connected=0),'DISCONNECTED'),('invalid_quaternion',dict(qscale=2),'IMU_REJECTED'),
           ('unknown_contact',dict(unknown=2),'UNCONFIRMED_CONTACT'),('unreliable_contact',dict(confidence=.3),'UNCONFIRMED_CONTACT'),
           ('ik_unreachable',dict(dz=1),'IK_FAILURE'),('stale_support',dict(support_stamp=0),'STALE_SUPPORT')]
    results=[]
    for name,kw,expected in cases:
        lib.init(); assert lib.step(.01,0,.02,1)[0]=='OK'
        t=.04 if name!='stale_support' else .12
        # For stale support, keep IMU interarrival valid before injecting the fault.
        if name=='stale_support':
            for seq in range(2,6): assert lib.step(0,0,.02*seq,seq)[0]=='OK'
        reason,o=lib.step(0,0,t,6,**kw)
        assert reason==expected and not o[2] and np.all(o[16:28]==0),(name,reason,expected)
        again,oo=lib.step(0,0,t+.02,7)
        assert again=='FAULT_LATCHED' and not oo[2]
        results.append(dict(name=name,status='OFFLINE VALIDATED',reason=reason,target_valid=False,fault_latched=bool(o[4]),
                            correction_diagnostic_rad=o[:2].tolist(),joint_targets=o[16:28].tolist()))
    return results


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('library',type=Path); parser.add_argument('output',type=Path)
    args=parser.parse_args(); args.output.mkdir(parents=True,exist_ok=True); lib=Bridge(args.library)
    result=dict(scope='offline software and synthetic simulations only; no hardware qualification',
                geometry=geometry_checks(lib),integral_law=law_checks(lib),simulations=studies(lib,args.output),faults=fault_checks(lib))
    result['fixed_storage_bytes']=dict(pipeline=lib.so.g5b_size(),output=lib.so.g5b_output_size())
    maximum=C.c_double(); accepted=C.c_uint(); mean=lib.so.g5b_benchmark(10000,C.byref(maximum),C.byref(accepted))
    assert accepted.value==10000
    (args.output/'benchmark.json').write_text(json.dumps(dict(status='HOST MEASUREMENT',steps=10000,accepted=accepted.value,
        mean_update_us=mean,maximum_observed_update_us=maximum.value,scope='host wall clock; not ESP32 WCET or qualified servo latency'),indent=2)+'\n')
    (args.output/'offline_results.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(geometry=result['geometry'],law=result['integral_law'],simulation_scenarios=len(result['simulations']),
                         fault_scenarios=len(result['faults']),storage=result['fixed_storage_bytes'],status='PASS'),sort_keys=True))
    return 0


if __name__=='__main__': sys.exit(main())
