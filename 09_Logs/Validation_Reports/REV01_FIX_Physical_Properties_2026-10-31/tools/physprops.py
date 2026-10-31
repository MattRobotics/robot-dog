"""Reconstruct the 17 URDF <inertial> blocks from the Solid Edge reports (CoM, mass, principal axes, radii of gyration)."""
import re, json, sys, numpy as np
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import meshprops
HERE=Path(__file__).resolve().parent
IN=HERE.parent/"input"
WT=HERE.parents[3]      # repository root
UNIT={"kg*dm^2":1e-2,"kg-m^2":1.0}
def parse(path):
    t=path.read_text(encoding="utf-8-sig"); g=lambda p: re.search(p,t,re.S)
    f=lambda s: float(s)
    d={}
    d["volume_mm3"]=f(g(r"volume=\s*([\d.\-]+)").group(1))
    d["mass_kg"]=f(g(r"massa=\s*([\d.\-]+)\s*kg").group(1))
    d["frame"]=g(r"Rispetto al sistema di coordinate (\w+)").group(1)
    com=g(r"Centro della massa:\s*X=\s*([\-\d.]+) mm\s*Y=\s*([\-\d.]+) mm\s*Z=\s*([\-\d.]+) mm")
    d["com_mm"]=[f(x) for x in com.groups()]
    u=g(r"Momento di inerzia:\s*Ixx=\s*[\-\d.]+ (kg[*\-]\w+\^2)").group(1)
    m=g(r"Ixx=\s*([\-\d.]+).*?Iyy=\s*([\-\d.]+).*?Izz=\s*([\-\d.]+).*?Ixy=\s*([\-\d.]+).*?Ixz=\s*([\-\d.]+).*?Iyz=\s*([\-\d.]+)")
    xx,yy,zz,xy,xz,yz=[f(x)*UNIT[u] for x in m.groups()]
    d["global_report"]={"unit":u,"Ixx":xx,"Iyy":yy,"Izz":zz,"Ixy":xy,"Ixz":xz,"Iyz":yz}
    ax=re.findall(r"^\s*([123])=\s*([\-\d.]+)\s+([\-\d.]+)\s+([\-\d.]+)\s*$",t,re.M)
    d["axes"]=[[f(a),f(b),f(c)] for _,a,b,c in ax]
    pm=g(r"I1=\s*([\-\d.]+).*?I2=\s*([\-\d.]+).*?I3=\s*([\-\d.]+)")
    d["principal_report"]=[f(x)*UNIT[u] for x in pm.groups()]
    k=g(r"K1=\s*([\-\d.]+) mm\s*K2=\s*([\-\d.]+) mm\s*K3=\s*([\-\d.]+) mm")
    d["K_mm"]=[f(x) for x in k.groups()]
    return d
def nearest_rotation(R):
    U,S,Vt=np.linalg.svd(np.array(R)); Q=U@Vt
    return Q, np.linalg.det(np.array(R)), np.abs(np.array(R)@np.array(R).T-np.eye(3)).max()
def tensor_at_com(d):
    Q,det,ortho=nearest_rotation(d["axes"])               # rows = principal axes in link frame
    m=d["mass_kg"]; lam=np.array([m*(k*1e-3)**2 for k in d["K_mm"]])
    I=sum(lam[i]*np.outer(Q[i],Q[i]) for i in range(3))
    return I,lam,Q,det,ortho
def to_origin(I,m,c):
    return I+m*(np.dot(c,c)*np.eye(3)-np.outer(c,c))
if __name__=="__main__":
    out={}
    for name in ["base_link","lf_hip_link","lf_upper_leg_link","lf_lower_leg_link","lf_foot_link"]:
        d=parse(IN/f"{name}__physprop.txt"); I,lam,Q,det,ortho=tensor_at_com(d); c=np.array(d["com_mm"])*1e-3; m=d["mass_kg"]
        Io=to_origin(I,m,c); g=d["global_report"]
        print(f"== {name} m={m} frame={d['frame']} det(printed axes)={det:+.4f} ortho_err={ortho:.4f}")
        print("   principal report vs m*K^2 (kg m2):",np.round(d["principal_report"],7),np.round(lam,8))
        rep=np.array([[g["Ixx"],g["Ixy"],g["Ixz"]],[g["Ixy"],g["Iyy"],g["Iyz"]],[g["Ixz"],g["Iyz"],g["Izz"]]])
        print("   origin tensor rebuilt:",np.round(Io,6).tolist())
        print("   origin report (as +products):",np.round(rep,6).tolist())
        # report resolution is 0.0005 kg*dm2 = 5e-6 kg m2
        print("   diag err (kg m2):",np.round(np.diag(Io)-np.diag(rep),7), " off-diag rebuilt xy,xz,yz:",np.round([Io[0,1],Io[0,2],Io[1,2]],7)," report:",[g['Ixy'],g['Ixz'],g['Iyz']])
        out[name]={"mass_kg":m,"com_m":c.tolist(),"I_com":I.tolist(),"lambda":lam.tolist(),"axes_ortho":Q.tolist()}
    json.dump(out,open(HERE.parent/"data"/"lf_blocks.json","w"),indent=1)
