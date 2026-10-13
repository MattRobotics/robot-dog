#!/usr/bin/env python3
"""Compile the qualified STL/contact map into bounded ESP32 flash tables.
No scheduler rewrite. Full 19081 poses from the existing deterministic map.
This avoids iterative STL exponentials on the MCU's operational feedback loop.
"""
import argparse,ctypes,json,hashlib,math,subprocess,tempfile
from pathlib import Path
import numpy as np
FW=Path(__file__).resolve().parents[2];ROOT=FW.parents[1];REPORT=ROOT/'09_Logs/Validation_Reports/Final_Release_Candidate_2026-10-13';target=FW/'src/motion/FirstStrideData.h'
parser=argparse.ArgumentParser();parser.add_argument('--sway-m',type=float,choices=(.03,.04),default=.03);parser.add_argument('--output-header',type=Path,default=target);parser.add_argument('--output-receipt',type=Path,default=REPORT/'compiled_first_stride_receipt.json');args=parser.parse_args();target=args.output_header
poses=[];velocity=[];acceleration=[]
with tempfile.TemporaryDirectory(prefix='matdog-stride-compiler-') as tmp:
 so=Path(tmp)/'adapter.so';subprocess.run(['g++','-std=c++17','-O2','-shared','-fPIC',str(FW/'scripts/tests/stl_gait_adapter.cpp'),*map(str,sorted((FW/'src/motion').glob('*.cpp'))),'-o',str(so)],check=True)
 lib=ctypes.CDLL(str(so));replay=lib.replay_soft_qualified if args.sway_m==.03 else lib.replay_soft;replay.argtypes=[ctypes.c_double]*6+[ctypes.c_int]+[ctypes.c_double]*2+[ctypes.POINTER(ctypes.c_double)]
 for k in range(19081):
  t=k*.05/9;hold=t<=8 or t>=98;shift=shiftV=shiftA=rate=acc=cycle=0
  if t<=8:
   u=t/8;shift=-args.sway_m*(10*u**3-15*u**4+6*u**5);shiftV=-args.sway_m*30*u*u*(1-u)**2/8;shiftA=-args.sway_m*(60*u-180*u*u+120*u**3)/64
  elif t>=98:
   cycle=1;u=(t-98)/8;shift=-args.sway_m*(1-(10*u**3-15*u**4+6*u**5));shiftV=args.sway_m*30*u*u*(1-u)**2/8;shiftA=args.sway_m*(60*u-180*u*u+120*u**3)/64
  else:
   u=(t-8)/90;cycle=10*u**3-15*u**4+6*u**5;rate=30*u*u*(1-u)**2/90;acc=(60*u-180*u*u+120*u**3)/90**2
  out=np.zeros(40);code=replay(cycle,rate,acc,shift,shiftV,shiftA,int(hold),0,.001,out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)));assert code==0,(k,t,code)
  poses.append(out[:12]);velocity.append(out[12:24]/9);acceleration.append(out[24:36]/81)
q=np.array(poses);v=np.array(velocity);a=np.array(acceleration);qf=q.astype('<f4');vi=np.rint(v/2e-6).astype('i8');ai=np.rint(a/4e-6).astype('i8');assert np.max(abs(vi))<=32767 and np.max(abs(ai))<=32767
qerr=float(np.max(abs(qf-q)));verr=float(np.max(abs(vi*2e-6-v)));aerr=float(np.max(abs(ai*4e-6-a)));assert qerr<1e-7 and verr<=1.00000001e-6 and aerr<=2.00000001e-6
# The table is encoded in defined scalar arrays, not structure padding.
payload=qf.tobytes()+vi.astype('<i2').tobytes()+ai.astype('<i2').tobytes();data_sha=hashlib.sha256(payload).hexdigest()
def fl(x):
 s=format(float(x),'.9g');return s+('' if '.' in s or 'e' in s else '.0')+'f'
with target.open('w') as f:
 f.write('// Generated original MATDOG first-stride table. See compile_first_stride.py.\n#ifndef MATDOG_FIRST_STRIDE_DATA_H\n#define MATDOG_FIRST_STRIDE_DATA_H\n#include <stdint.h>\nnamespace matdog { namespace motion { namespace first_stride_data {\nstruct Sample {float q[12];int16_t velocity[12],acceleration[12];};\n')
 f.write(f'constexpr double kSwayM={args.sway_m:.17g};\nconstexpr unsigned kSamples=19081;\nconstexpr double kNativeStep=.05/9,kVelocityUnit=2e-6,kAccelerationUnit=4e-6,kQuantizationError={qerr:.17g};\nconstexpr char kDataSha256[]="{data_sha}";\n')
 f.write('constexpr double kMinimum[12]={'+','.join(format(float(x),'.17g') for x in qf.min(axis=0))+'};\nconstexpr double kMaximum[12]={'+','.join(format(float(x),'.17g') for x in qf.max(axis=0))+'};\nconstexpr Sample kSamplesData[]={\n')
 for i in range(len(qf)):f.write(' {{'+','.join(map(fl,qf[i]))+'},{'+','.join(map(str,vi[i]))+'},{'+','.join(map(str,ai[i]))+'}},\n')
 f.write('};\nstatic_assert(sizeof(Sample)==96,"bounded flash layout");\n} } }\n#endif\n')
report=dict(parameters=dict(sway_x_m=args.sway_m,lift_m=.005,advance_x_m=.005),producer_adapter_sha256=hashlib.sha256((FW/'scripts/tests/stl_gait_adapter.cpp').read_bytes()).hexdigest(),joint_target_ranges_rad=[[float(x),float(y)] for x,y in zip(qf.min(axis=0),qf.max(axis=0))],schema='MATDOG_COMPILED_FIRST_STRIDE_V1',frames=len(q),data_sha256=data_sha,header_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),table_bytes=96*len(q),position_error_rad=qerr,velocity_encoding_error_rad_s=verr,acceleration_encoding_error_rad_s2=aerr,maximum_velocity_rad_s=float(np.max(abs(v))),maximum_acceleration_rad_s2=float(np.max(abs(a))),maximum_joint_target_step_rad=float(np.max(abs(np.diff(qf,axis=0)))),runtime_error_tube_rad=.006-qerr,minimum_time_scale=9,scope='frozen 5mm forward, zero vy/yaw, one stride; actual runtime targets are float table rows; drive rates stay independently reviewed; no loaded/contact claim',hardware_qualified=False)
args.output_receipt.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
