"""Independent offline oracle. NumPy is verification-only, never a core dependency."""
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
import numpy as np
from model import ROOT, URDF_REL, at_pose, load_baseline, validate_tensor

C4C_REL = "09_Logs/Validation_Reports/C4_rest_to_stand_trajectory/2026-07-08_190405_C4C_contact_locked_rest_to_stand_trajectory.json"


def np_rpy(v):
    x,y,z=v
    cx,sx,cy,sy,cz,sz=math.cos(x),math.sin(x),math.cos(y),math.sin(y),math.cos(z),math.sin(z)
    rx=np.array([[1,0,0],[0,cx,-sx],[0,sx,cx]])
    ry=np.array([[cy,0,sy],[0,1,0],[-sy,0,cy]])
    rz=np.array([[cz,-sz,0],[sz,cz,0],[0,0,1]])
    return rz@ry@rx


def oracle(pose):
    root=ET.parse(ROOT/URDF_REL).getroot()
    transforms={"base_link":np.eye(4)}
    remaining=list(root.findall("joint"))
    while remaining:
        progress=False
        for j in remaining[:]:
            parent=j.find("parent").get("link")
            if parent not in transforms: continue
            origin=j.find("origin")
            xyz=[float(v) for v in origin.get("xyz").split()]
            rpy=[float(v) for v in origin.get("rpy").split()]
            t=np.eye(4); t[:3,:3]=np_rpy(rpy); t[:3,3]=xyz
            if j.get("type")!="fixed":
                axis=np.array([float(v) for v in j.find("axis").get("xyz").split()])
                axis/=np.linalg.norm(axis)
                a,b,c=axis
                skew=np.array([[0,-c,b],[c,0,-a],[-b,a,0]])
                q=pose[j.get("name")]
                motion=np.eye(4)
                motion[:3,:3]=np.eye(3)+math.sin(q)*skew+(1-math.cos(q))*(skew@skew)
                t=t@motion
            transforms[j.find("child").get("link")]=transforms[parent]@t
            remaining.remove(j); progress=True
        if not progress: raise ValueError("Oracle tree invalid")
    bodies=[]
    for link in root.findall("link"):
        node=link.find("inertial")
        mass=float(node.find("mass").get("value"))
        origin=node.find("origin")
        c=np.array([float(v) for v in origin.get("xyz").split()])
        ir=np_rpy([float(v) for v in origin.get("rpy").split()])
        a=node.find("inertia").attrib
        inertia=np.array([[float(a[key]) for key in row] for row in
                         (("ixx","ixy","ixz"),("ixy","iyy","iyz"),("ixz","iyz","izz"))])
        t=transforms[link.get("name")]; r=t[:3,:3]@ir
        bodies.append((mass,t[:3,:3]@c+t[:3,3],r@inertia@r.T))
    m=sum(b[0] for b in bodies)
    com=sum(b[0]*b[1] for b in bodies)/m
    inertia=np.zeros((3,3))
    for mass,c,i in bodies:
        d=c-com
        inertia+=i+mass*(d@d*np.eye(3)-np.outer(d,d))
    return m,com,inertia


def run():
    baseline=load_baseline()
    frames=json.loads((ROOT/C4C_REL).read_text())["frames"]
    poses=[{n:q for leg in f["legs"].values() for n,q in leg["joint_positions_rad"].items()} for f in frames]
    root=ET.parse(ROOT/URDF_REL).getroot()
    limits={j.get("name"):(float(j.find("limit").get("lower")),float(j.find("limit").get("upper")))
            for j in root.findall("joint") if j.get("type")=="revolute"}
    poses.insert(0,{n:0. for n in limits})
    # Deterministic interior and near-boundary poses. These are mathematical fixtures.
    for sample in range(48):
        poses.append({n:lo+(hi-lo)*(0.5+0.499*math.sin((sample+1)*(k+1)*.71))
                      for k,(n,(lo,hi)) in enumerate(limits.items())})
    max_com=max_inertia=max_eigen=0.
    for pose in poses:
        m,c,i=oracle(pose)
        actual=at_pose(baseline,pose)
        if abs(m-actual.mass)>1e-12: raise AssertionError("Oracle mass mismatch")
        max_com=max(max_com,float(np.max(np.abs(c-np.array(actual.com)))))
        max_inertia=max(max_inertia,float(np.max(np.abs(i-np.array(actual.inertia)))))
        if np.linalg.eigvalsh(i)[0] <= 0: raise AssertionError("Nonphysical aggregate")
    for b in baseline.values():
        max_eigen=max(max_eigen,float(np.max(np.abs(np.linalg.eigvalsh(b.inertia)-
                               np.array(validate_tensor(b.inertia)["principal_moments_kg_m2"])))))
    if max_com>1e-12 or max_inertia>1e-12 or max_eigen>1e-12:
        raise AssertionError("Independent NumPy oracle tolerance exceeded")
    return {"status":"OFFLINE VALIDATED","poses":len(poses),"historical_c4c_poses":51,
            "synthetic_additional_poses":49,"max_com_error_m":max_com,
            "max_inertia_error_kg_m2":max_inertia,"max_principal_moment_error_kg_m2":max_eigen,
            "tolerance_absolute":1e-12,"numpy_version":np.__version__}


if __name__=="__main__":
    print(json.dumps(run(),sort_keys=True,allow_nan=False))
