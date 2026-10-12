from pathlib import Path
import ctypes,os
import numpy as np
LIB=ctypes.CDLL(str(Path(os.environ['MATDOG_PAIRED_OUTPUT'])/'core_adapter.so'))
PTR=ctypes.POINTER(ctypes.c_double)
LIB.contact_fk.argtypes=[ctypes.c_int,PTR,ctypes.c_double,ctypes.c_double,ctypes.c_double,PTR]
LIB.contact_ik.argtypes=[ctypes.c_int,PTR,PTR,ctypes.c_double,ctypes.c_double,ctypes.c_double,PTR]
LIB.roundtrip_g1.argtypes=[ctypes.c_int,PTR,PTR]
LIB.origin_fk.argtypes=[ctypes.c_int,PTR,PTR]
def ptr(a):return a.ctypes.data_as(PTR)
def contact(leg,u,l,p=0,x=0,z=0,h=0):
    q=np.array([h,u,l],float);out=np.zeros(7);code=LIB.contact_fk(leg,ptr(q),p,x,z,ptr(out))
    if code:raise ValueError(('CORE_CONTACT_FK',code,q.tolist()))
    return out
def ik(leg,target,seed,p,x,z):
    target=np.asarray(target,float);seed=np.asarray(seed,float);out=np.zeros(6)
    code=LIB.contact_ik(leg,ptr(seed),ptr(target),p,x,z,ptr(out))
    if code:raise ValueError(('CORE_CONTACT_IK',code,target.tolist(),seed.tolist(),p,x,z))
    if abs(out[0])>1e-9:raise ValueError(('HIP_NEUTRAL_NOT_PRESERVED',out.tolist()))
    return out
def origin(leg,u,l,h=0):
    q=np.array([h,u,l],float);out=np.zeros(12);code=LIB.origin_fk(leg,ptr(q),ptr(out))
    if code:raise ValueError(('CORE_ORIGIN_FK',code,q.tolist()))
    return out[:3],out[3:].reshape(3,3)
