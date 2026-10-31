#!/usr/bin/env python3
"""Replay the 214 frozen study poses using the production runtime adapter."""
import ctypes,json,math,subprocess,tempfile,argparse
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[4];FW=ROOT/'05_Firmware/MATDOG_Controller'
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path);args=p.parse_args()
 data=json.loads((ROOT/'09_Logs/Validation_Reports/Motion_Night_Run_2026-10-12/walk_after_stand.json').read_text());maximum=[0.,0.,0.];core_difference=[0.,0.];refused=[];qualified=0
 with tempfile.TemporaryDirectory(prefix='matdog-stl-gait-') as tmp:
  so=Path(tmp)/'adapter.so';subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-shared','-fPIC',str(FW/'scripts/tests/stl_gait_adapter.cpp'),*map(str,sorted((FW/'src/motion').glob('*.cpp'))),'-o',str(so)],check=True)
  lib=ctypes.CDLL(str(so));lib.replay_soft.argtypes=[ctypes.c_double]*6+[ctypes.c_int]+[ctypes.c_double]*2+[ctypes.POINTER(ctypes.c_double)]
  for row in data['samples']:
   phase=row['phase'];t=row['time_s'];cycle=row['cycles'];hold=phase in ('GAIT_START','STOP_TO_STAND','SIMULATED_REARM_STAND');origin=.005 if phase=='SIMULATED_REARM_STAND' else 0.
   rate=acc=shift=shiftV=shiftA=0.
   if phase=='GAIT_START':
    u=t/8;shift=-.04*(10*u**3-15*u**4+6*u**5);shiftV=-.04*30*u*u*(1-u)**2/8;shiftA=-.04*(60*u-180*u*u+120*u**3)/64
   elif phase=='STOP_TO_STAND':
    u=(t-98)/8;shift=-.04*(1-(10*u**3-15*u**4+6*u**5));shiftV=.04*30*u*u*(1-u)**2/8;shiftA=.04*(60*u-180*u*u+120*u**3)/64
   elif not hold:
    u=(t-8)/90;rate=30*u*u*(1-u)**2/90;acc=(60*u-180*u*u+120*u**3)/90**2
   outputs=[]
   for step in [0.,.001,.0005]:
    output=np.zeros(40);code=lib.replay_soft(cycle,rate,acc,shift,shiftV,shiftA,int(hold),origin,step,output.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))
    if step==0: assert code==0,(phase,t,code)
    elif code: refused.append(dict(phase=phase,time_s=t,step_s=step,reason='ADAPTED_DERIVATIVE_NOT_CONVERGED'));output=None
    
    if output is not None: assert np.all(output[36:]>=-1e-9), (t,output[36:])
    outputs.append(output)
   expected=np.array(list(row['q_rad'].values()));maximum[0]=max(maximum[0],float(np.max(abs(outputs[0][:12]-expected))))
   if outputs[1] is None or outputs[2] is None: continue
   qualified+=1
   maximum[1]=max(maximum[1],float(np.max(abs(outputs[1][12:24]-outputs[2][12:24]))))
   maximum[2]=max(maximum[2],float(np.max(abs(outputs[1][24:36]-outputs[2][24:36]))))
   core_difference[0]=max(core_difference[0],float(np.max(abs(outputs[1][12:24]-row['core_velocity_rad_s']))))
   core_difference[1]=max(core_difference[1],float(np.max(abs(outputs[1][24:36]-row['core_acceleration_rad_s2']))))
 report=dict(schema='MATDOG_SUSPENDED_C2_STL_G4_V1',samples=len(data['samples']),max_target_error_rad=maximum[0],stencil_velocity_difference_rad_s=maximum[1],stencil_acceleration_difference_rad_s2=maximum[2],
  core_derivative_omission_difference=core_difference,target_tolerance_rad=.001,velocity_convergence_tolerance_rad_s=1e-7,acceleration_convergence_tolerance_rad_s2=1e-5,
  status='FAIL' if refused else 'PASS',target_status='PASS' if maximum[0]<.001 else 'FAIL',qualified_samples=qualified,refused_stencils=refused,
  scope='frozen G4 vx=5mm/90s,vy=0,yaw=0,40mm sway; target + complete local adapted map derivative convergence only',hardware_qualified=False,all_velocity_combinations_qualified=False)
 if args.output:args.output.write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report));assert report['target_status']=='PASS';assert maximum[1]<1e-7 and maximum[2]<1e-5;assert not refused, 'Smooth suspended adapter must qualify every sample'
if __name__=='__main__':main()
