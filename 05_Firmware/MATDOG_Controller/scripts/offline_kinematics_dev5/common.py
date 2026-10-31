"""Host-only analysis utilities; production source is read-only."""
from pathlib import Path
import ctypes as ct, json,sys,subprocess,math,os
import numpy as np
HERE=Path(__file__).resolve().parent;FW=HERE.parents[1];ROOT=FW.parents[1]
sys.path.insert(0,str(FW/'scripts/offline_prone'))
from geometry import geometry,rest,rest3d,entry_q,rot,meshes
from flat_bound import boxes,limits
TICK=2*math.pi/4096
LOW=np.array([v['limits_rad'][0] for v in limits]);HIGH=np.array([v['limits_rad'][1] for v in limits])
def bridge(out):
 out=Path(out);out.mkdir(parents=True,exist_ok=True);so=out/('production-'+str(os.getpid())+'.so')
 names=['BodyPose','ContactMode','FootContact','LegKinematics','LegInverseKinematics','TiltCompensation','TiltedContactIk','ImuAttitude','Gait','GaitTiming','StandTrajectory']
 subprocess.run(['g++','-shared','-fPIC','-O2','-std=c++17',str(HERE/'bridge.cpp'),*[str(FW/'src/motion'/(n+'.cpp')) for n in names],'-o',str(so)],check=True)
 lib=ct.CDLL(str(so));ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS');lib.route.argtypes=[ct.c_uint,ptr];lib.fk.argtypes=[ct.c_uint,ptr,ptr,ptr];lib.ik.argtypes=[ct.c_uint,ptr,ptr,ptr,ptr];lib.ik6.argtypes=[ct.c_uint,ptr,ptr,ptr,ptr];lib.sample.argtypes=[ptr,ct.c_uint,ct.c_double,ptr]
 return lib
def knot(lib,k):
 q=np.zeros(15);lib.route(k,q);return q
def feet(lib,q,p):
 f=[]
 for l in range(4):
  a=np.zeros(3);assert lib.fk(l,np.ascontiguousarray(q[l*3:l*3+3]),np.ascontiguousarray(p),a);f.append(a)
 return np.array(f)
def margin(q,box):return float(np.min(np.minimum(q-np.maximum(LOW,box[0]),np.minimum(HIGH,box[1])-q))/TICK)
def dump(path,data):Path(path).write_text(json.dumps(data,indent=2,default=lambda a:a.tolist() if isinstance(a,np.ndarray) else float(a))+'\n')
