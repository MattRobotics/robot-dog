import numpy as np, sys, json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent)); import meshprops
D=Path(__file__).resolve().parents[4]/"03_CAD/URDF/matt_robodog_rev00/meshes/collision"
T={"identity":np.eye(3),"Y-reflection":np.diag([1,-1,1.]),"X-reflection":np.diag([-1,1,1.]),"Z-rot180":np.diag([-1,-1,1.])}
res={}
P={}
for leg in("lf","rf","rh","lh"):
    for k in("hip","upper_leg","lower_leg","foot"):
        v,c,I=meshprops.props(meshprops.load_stl(D/f"{leg}_{k}_link.stl")); P[(leg,k)]=(v,c,I)
print("%-6s %-9s %-13s %12s %12s"%("link","leg","best map","dCoM [mm]","dI rel"))
for k in("hip","upper_leg","lower_leg","foot"):
    v0,c0,I0=P[("lf",k)]
    for leg in("rf","rh","lh"):
        v,c,I=P[(leg,k)]; best=None
        for n,M in T.items():
            dc=np.linalg.norm(M@c0-c); dI=np.linalg.norm(M@I0@M.T-I)/np.linalg.norm(I)
            if best is None or dc+dI<best[1]+best[2]: best=(n,dc,dI)
        ident=(np.linalg.norm(c0-c),np.linalg.norm(I0-I)/np.linalg.norm(I))
        print("%-6s %-9s %-13s %12.4f %12.2e   (identity: %.4f mm, %.2e)  dVol=%.2e"%(k,leg,*best,*ident,(v-v0)/v0))
        res[f"{leg}_{k}"]={"map":best[0],"dCoM_mm":best[1],"dI_rel":best[2],"dVol_rel":(v-v0)/v0}
json.dump(res,open(Path(__file__).resolve().parents[1]/"data"/"symmetry_mesh_check.json","w"),indent=1)
