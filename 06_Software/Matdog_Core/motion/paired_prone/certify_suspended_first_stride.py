#!/usr/bin/env python3
"""Continuous boxes between successive production targets, plus tracking tube.
This certifies suspended self-clearance, not force-bearing contact or dynamics.
"""
import argparse,ctypes,hashlib,json,math,os,subprocess,tempfile
from pathlib import Path
import numpy as np
from continuous_pair_path import Continuous
from paired_prone_model import ROOT,JOINTS
from certify_adjacent_axis import certify
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--tube-rad',type=float,default=.002);p.add_argument('--time-scale',type=float,default=1);args=p.parse_args();assert not args.output.exists();assert 1<=args.time_scale<=100
fw=ROOT/'05_Firmware/MATDOG_Controller';c=Continuous();poses=[];dt=.05/args.time_scale
with tempfile.TemporaryDirectory(prefix='matdog-first-stride-cert-') as tmp:
 so=Path(tmp)/'adapter.so';subprocess.run(['g++','-std=c++17','-O2','-shared','-fPIC',str(fw/'scripts/tests/stl_gait_adapter.cpp'),*map(str,sorted((fw/'src/motion').glob('*.cpp'))),'-o',str(so)],check=True)
 lib=ctypes.CDLL(str(so));lib.replay_soft.argtypes=[ctypes.c_double]*6+[ctypes.c_int]+[ctypes.c_double]*2+[ctypes.POINTER(ctypes.c_double)]
 for k in range(round(106/dt)+1):
  t=k*dt;hold=t<=8 or t>=98;shift=shiftV=shiftA=rate=acc=cycle=0
  if t<=8:
   u=t/8;shift=-.04*(10*u**3-15*u**4+6*u**5);shiftV=-.04*30*u*u*(1-u)**2/8;shiftA=-.04*(60*u-180*u*u+120*u**3)/64
  elif t>=98:
   cycle=1;u=(t-98)/8;shift=-.04*(1-(10*u**3-15*u**4+6*u**5));shiftV=.04*30*u*u*(1-u)**2/8;shiftA=.04*(60*u-180*u*u+120*u**3)/64
  else:
   u=(t-8)/90;cycle=10*u**3-15*u**4+6*u**5;rate=30*u*u*(1-u)**2/90;acc=(60*u-180*u*u+120*u**3)/90**2
  output=np.zeros(40);code=lib.replay_soft(cycle,rate,acc,shift,shiftV,shiftA,int(hold),0.,.001,output.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))
  if code:raise SystemExit(f'Derivative refusal at {t}s')
  poses.append(output[:12])
report=dict(schema='MATDOG_SUSPENDED_FIRST_STRIDE_BOXES_V1',tube_rad=args.tube_rad,sample_period_s=dt,frames=len(poses),parameters=dict(period_s=90,lift_m=.005,duty=.8,sway_x_m=.04,stand_shift_s=8,advance_x_m=.005,vy=0,yaw=0,time_scale=args.time_scale,physical_sample_period_s=.05),status='IN_PROGRESS',nonadjacent=[],adjacent=[],hardware_qualified=False,source_files_sha256={str(f.relative_to(ROOT)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted((fw/'src/motion').glob('*.cpp'))})
args.output.write_text(json.dumps(report,indent=2)+'\n')
for i in range(len(poses)-1):
 ok,r=c.self_proof_q(dict(zip(JOINTS,poses[i])),dict(zip(JOINTS,poses[i+1])),.003,args.tube_rad)
 if not ok:report['status']='BLOCKED_NONADJACENT';report['failed_interval']=dict(index=i,certificate=r);break
 report['nonadjacent'].append(dict(index=i,minimum_bound_m=r['minimum_swept_separation_lower_bound_m']))
 if i%100==0:print('first_stride',i,flush=True);args.output.write_text(json.dumps(report,indent=2)+'\n')
else:
 values=np.array(poses)
 for i,j in enumerate(JOINTS):
  lo=float(values[:,i].min())-args.tube_rad;hi=float(values[:,i].max())+args.tube_rad
  r=certify(c.s,j,lo,hi,90);report['adjacent'].append(dict(joint=j,domain_rad=[lo,hi],**r));print(j,r['status'],flush=True)
  args.output.write_text(json.dumps(report,indent=2)+'\n')
 report['status']='PASS_CONTINUOUS_TARGET_BOXES' if all(r['status']=='PASS_CONTINUOUS' for r in report['adjacent']) else 'BLOCKED_ADJACENT'
report['maximum_joint_target_step_rad']=float(np.max(np.abs(np.diff(np.array(poses),axis=0))))
report['joint_target_ranges_rad']=[[float(np.min(np.array(poses)[:,i])),float(np.max(np.array(poses)[:,i]))] for i in range(12)]
report['scope']='All independent joint values between each pair of produced targets +/-tube; 3mm nonadjacent; all canonical adjacent triangles. Tube must include tracking, encoder quantization, calibration and mechanical uncertainty. Fixed fixture free volume remains a separate physical gate. No ground/load/stability claim.'
args.output.write_text(json.dumps(report,indent=2)+'\n');print(report['status'],flush=True)
