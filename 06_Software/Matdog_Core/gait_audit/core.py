"""Host-only bridge to the production gait core. Independent checks live in oracle.py."""
import ctypes as C
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
MOTION=ROOT/'05_Firmware/MATDOG_Controller/src/motion'
HERE=Path(__file__).resolve().parent
STATUS=['KINEMATICALLY_VALID','INVALID_PARAMETER','NONFINITE','IK_UNREACHABLE','JOINT_LIMIT_BLOCKED','CONTACT_INVALID','BRANCH_CHANGE','NUMERICALLY_ILL_CONDITIONED','COLLISION','SUPPORT_INVALID','STATE_ERROR','CANCELLED']
FIELDS=['type','height_m','lift_m','duty','advance_x_m','advance_y_m','yaw_rad','sway_x_m','max_condition']
def params(kind=0,height=.15,lift=.01,duty=.8,x=.01,y=0,yaw=0,sway=0,condition=10000):return np.array([kind,height,lift,duty,x,y,yaw,sway,condition],dtype=np.float64)
class Core:
 def __init__(self):
  sources=[MOTION/(n+'.cpp') for n in ('Gait','Locomotion','MotionState','StartupAcquisition','TimedStand','StandTransition','BodyPose','FootContact','LegKinematics','LegInverseKinematics','StandTrajectory')]+[HERE/'bridge.cpp']
  digest=hashlib.sha256(b''.join(p.read_bytes() for p in sources+sorted(MOTION.glob('*.h')))).hexdigest()[:20]
  path=Path(tempfile.gettempdir())/('matdog-g4-'+digest+'.so')
  if not path.exists():
   # Build privately then rename atomically so concurrent workers never load a partial library.
   part=path.with_suffix('.%d.tmp'%os.getpid())
   subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-fno-exceptions','-fno-rtti','-shared','-fPIC','-I',str(MOTION),*[str(p) for p in sources],'-o',str(part)],check=True)
   os.replace(part,path)
  self.lib=C.CDLL(str(path));self.ptr=C.POINTER(C.c_double)
  self.lib.g4_frame.argtypes=[self.ptr,*([C.c_double]*4),self.ptr,self.ptr];self.lib.g4_frame.restype=C.c_int
 def frame(self,p,s,period=1,previous=None,terminal=-1,rate=None,accel=0):
  out=np.zeros(110);p=np.asarray(p,dtype=np.float64);old=None if previous is None else np.asarray(previous['seed'],dtype=np.float64)
  code=self.lib.g4_frame(p.ctypes.data_as(self.ptr),s,1/period if rate is None else rate,accel,terminal,None if old is None else old.ctypes.data_as(self.ptr),out.ctypes.data_as(self.ptr))
  if code:return dict(status=STATUS[code],failed_leg=int(out[0]),phase=s)
  return decode(out,s)
def decode(out,s):
  body=np.eye(4);body[:3,3]=out[80:83];body[:3,:3]=out[83:92].reshape(3,3)
  return dict(status=STATUS[0],phase=s,q=out[:12].copy(),seed=out[:20].copy(),branches=out[12:20].copy(),qdot=out[20:32].copy(),qddot=out[32:44].copy(),feet=out[44:56].reshape(4,3).copy(),foot_velocity=out[56:68].reshape(4,3).copy(),foot_acceleration=out[68:80].reshape(4,3).copy(),body=body,leg_phase=out[92:104].reshape(4,3).copy(),condition=out[104:108].copy(),joint_margin=out[108],residual=out[109])

def lifecycle(core,p,times,period=1,stop_at=3.2):
 p=np.asarray(p,dtype=np.float64);times=np.asarray(times,dtype=np.float64);output=np.zeros((len(times),110));statuses=np.zeros(len(times),dtype=np.int32);states=np.zeros_like(statuses)
 fn=core.lib.g4_lifecycle;ip=C.POINTER(C.c_int);fn.argtypes=[core.ptr,C.c_double,C.c_double,core.ptr,C.c_uint,core.ptr,ip,ip];fn.restype=C.c_int
 code=fn(p.ctypes.data_as(core.ptr),period,stop_at,times.ctypes.data_as(core.ptr),len(times),output.ctypes.data_as(core.ptr),statuses.ctypes.data_as(ip),states.ctypes.data_as(ip))
 if code:raise ValueError(('lifecycle bridge failed',code,statuses.tolist()))
 return [dict(decode(out,float(t)),time_s=float(t),state=int(state)) for out,t,state in zip(output,times,states)]
