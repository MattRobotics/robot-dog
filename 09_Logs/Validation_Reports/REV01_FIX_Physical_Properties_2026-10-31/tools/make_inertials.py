"""Build the final 17 <inertial> blocks (SI, link frames) and the checks. Deterministic; reads only the Solid Edge reports."""
import json, re, sys, numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent)); import physprops as P, assemble as A
OUT=Path(__file__).resolve().parents[1]/"data"
lf={}; rep={}
for n in ["base_link","lf_hip_link","lf_upper_leg_link","lf_lower_leg_link","lf_foot_link"]:
    d=P.parse(P.IN/f"{n}__physprop.txt"); I,lam,Q,det,ortho=P.tensor_at_com(d)
    lf[n]=(d["mass_kg"],np.array(d["com_mm"])*1e-3,I); rep[n]=d
MY=np.diag([1,-1,1.])
RULE={"hip":{"lf":"CAD direct","rf":"Y reflection of LF","rh":"Y reflection of LF (rear hip mesh differs from LF by 0.21 mm CoM / 0.8 % inertia: mesh estimate, not CAD)","lh":"same left-side frame as LF (rear hip mesh differs from LF by 0.21 mm CoM / 0.8 % inertia: mesh estimate, not CAD)"},
      "upper_leg":{"lf":"CAD direct","rf":"Y reflection of LF","rh":"Y reflection of LF","lh":"same left-side frame as LF"},
      "lower_leg":{"lf":"CAD direct","rf":"Y reflection of LF","rh":"Y reflection of LF","lh":"same left-side frame as LF"},
      "foot":{"lf":"CAD direct","rf":"identical foot geometry (LF copied)","rh":"identical foot geometry (LF copied)","lh":"identical foot geometry (LF copied)"}}
blocks={}; order=["base_link"]+[f"{l}_{k}_link" for l in("lf","rf","rh","lh") for k in("hip","upper_leg","lower_leg","foot")]
for n in order:
    if n=="base_link": m,c,I=lf[n]; M=np.eye(3); rule="CAD direct"; src="base_link__physprop.txt"
    else:
        l,k=n.split("_",1)[0], n.split("_",1)[1][:-5]; m,c0,I0=lf[f"lf_{k}_link"]
        M={"lf":np.eye(3),"lh":np.eye(3),"rf":MY,"rh":MY}[l]
        if k=="foot": M=np.eye(3)
        c=M@c0; I=M@I0@M.T; rule=RULE[k][l]; src=f"lf_{k}_link__physprop.txt"
    I=(I+I.T)/2
    w=np.linalg.eigvalsh(I)
    assert m>0 and w.min()>0 and w[0]+w[1]>=w[2]*(1-1e-9) and np.allclose(I,I.T,atol=0)
    blocks[n]={"mass_kg":m,"com_m":[float(x) for x in c],"ixx":I[0,0],"ixy":I[0,1],"ixz":I[0,2],"iyy":I[1,1],"iyz":I[1,2],"izz":I[2,2],
               "rule":rule,"source":src,"principal_moments":[float(x) for x in w],"min_eig":float(w.min())}
json.dump(blocks,open(OUT/"inertial_blocks.json","w"),indent=1,default=float)
print("%-22s %8s  %-26s %12s %12s %12s %13s %13s %13s"%("link","mass","CoM [mm]","ixx","iyy","izz","ixy","ixz","iyz"))
for n,b in blocks.items():
    print("%-22s %8.3f  %-26s %12.5e %12.5e %12.5e %13.5e %13.5e %13.5e"%(n,b["mass_kg"],np.round(np.array(b["com_m"])*1e3,2).tolist().__str__(),b["ixx"],b["iyy"],b["izz"],b["ixy"],b["ixz"],b["iyz"]))
print("sum mass:",round(sum(b['mass_kg'] for b in blocks.values()),4))
