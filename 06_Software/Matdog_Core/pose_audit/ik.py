"""Builds only MATDOG pure C++ contact code into a temporary offline adapter."""
import ctypes, subprocess, tempfile
from pathlib import Path
import numpy as np
from model import ROOT,LEGS
class ContactIK:
    def __init__(self):
        self.tmp=tempfile.TemporaryDirectory(prefix='matdog-pose-ik-')
        src=ROOT/'05_Firmware/MATDOG_Controller/src/motion';out=Path(self.tmp.name)/'ik.so'
        subprocess.run(['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-fno-exceptions','-fno-rtti','-shared','-fPIC','-I'+str(src),str(Path(__file__).with_name('contact_bridge.cpp')),str(src/'FootContact.cpp'),str(src/'LegKinematics.cpp'),str(src/'LegInverseKinematics.cpp'),'-o',str(out)],check=True)
        self.lib=ctypes.CDLL(str(out));p=ctypes.POINTER(ctypes.c_double)
        self.lib.solve_contact.argtypes=[ctypes.c_uint,p,p,p];self.lib.solve_contact.restype=ctypes.c_int
    def leg(self,index,target,seed=(0,1,0)):
        a=(ctypes.c_double*3)(*target);b=(ctypes.c_double*3)(*seed);c=(ctypes.c_double*3)()
        status=self.lib.solve_contact(index,a,b,c)
        return (np.array(c) if status==0 else None),status
    def pose(self,targets,body,seed=None):
        if np.max(np.abs(body[2,:3]-[0,0,1]))>1e-12:raise ValueError('G2 +Z ground-normal boundary')
        values=[]
        for i,t in enumerate(targets):
            q,s=self.leg(i,body[:3,:3].T@(np.array(t)-body[:3,3]),seed[i*3:i*3+3] if seed is not None else (0,1,0))
            if q is None:return None,s
            values.extend(q)
        return np.array(values),0
