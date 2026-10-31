"""Reconciliation of the Solid Edge reports: volumes, masses, the 9 g residual, moment cross-checks."""
import json, sys, numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent)); import physprops as P, assemble as A
OUT=Path(__file__).resolve().parents[1]/"data"
names=["base_link","lf_hip_link","lf_upper_leg_link","lf_lower_leg_link","lf_foot_link"]
d={n:P.parse(P.IN/f"{n}__physprop.txt") for n in names}; asm=P.parse(P.IN/"matdog_urdf_assembly__physprop.txt")
res={}
vol_sum=d["base_link"]["volume_mm3"]+4*sum(d[n]["volume_mm3"] for n in names[1:])
res["volume"]={"sum_links_mm3":vol_sum,"assembly_mm3":asm["volume_mm3"],"difference_mm3":asm["volume_mm3"]-vol_sum}
m={n:d[n]["mass_kg"] for n in names}; msum=m["base_link"]+4*sum(m[n] for n in names[1:])
res["mass"]={"sum_links_kg":msum,"assembly_kg":asm["mass_kg"],"residual_g":(asm["mass_kg"]-msum)*1e3,
   "max_rounding_bound_g_if_legs_identical":(0.5+4*4*0.5),"max_truncation_bound_g_if_legs_identical":(1.0+4*4*1.0),
   "note":"base 1 value + 4 distinct leg values replicated x4 legs: rounding error <= 0.5 g + 4*4*0.5 g = 8.5 g; truncation error < 1 g + 16*1 g"}
res["density_g_cm3"]={n:d[n]["mass_kg"]*1e3/(d[n]["volume_mm3"]*1e-3) for n in names}
# printed global moments vs rebuilt (kg m2): resolution 5e-6
rows={}
for n in names:
    I,lam,Q,det,ortho=P.tensor_at_com(d[n]); c=np.array(d[n]["com_mm"])*1e-3; Io=P.to_origin(I,d[n]["mass_kg"],c); g=d[n]["global_report"]
    rep=np.array([[g["Ixx"],-g["Ixy"],-g["Ixz"]],[-g["Ixy"],g["Iyy"],-g["Iyz"]],[-g["Ixz"],-g["Iyz"],g["Izz"]]])   # products -> tensor entries
    rows[n]={"max_abs_error_kg_m2":float(np.abs(Io-rep).max()),"report_resolution_kg_m2":5e-6,"axes_det":det,"axes_orthogonality_error":ortho,
             "principal_I_printed_vs_mK2":[[float(a),float(b)] for a,b in zip(d[n]["principal_report"],lam)]}
res["moment_crosscheck"]=rows
# sensitivity of inertia to the 1 g mass resolution
res["mass_resolution_relative"]={n:0.0005/d[n]["mass_kg"] for n in names}
# aggregate
pos=A.link_positions(P.WT/"03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf"); M,com,I=A.aggregate(A.build_blocks(),pos)
ar=P.tensor_at_com(asm)[0]
res["aggregate"]={"mass_kg":M,"com_mm":(com*1e3).tolist(),"assembly_com_mm":asm["com_mm"],"com_deviation_mm":float(np.linalg.norm(com*1e3-np.array(asm["com_mm"]))),
   "K_mm_model":(np.sqrt(np.sort(np.linalg.eigvalsh(I))[::-1]/M)*1e3).tolist(),"K_mm_assembly":asm["K_mm"],
   "principal_moment_rel_dev":((np.sort(np.linalg.eigvalsh(I))[::-1]-np.sort(np.linalg.eigvalsh(ar))[::-1])/np.sort(np.linalg.eigvalsh(ar))[::-1]).tolist()}
# what-if: where would a 9 g mass have to sit to explain the CoM offset (diagnostic only)
dcom=(np.array(asm["com_mm"])-com*1e3); res["aggregate"]["implied_9g_position_mm_if_single_mass"]=(np.array(asm["com_mm"])+dcom*asm["mass_kg"]*1e3/9.0).tolist()
json.dump(res,open(OUT/"closure_checks.json","w"),indent=1,default=float)
print(json.dumps({k:res[k] for k in("volume","mass","aggregate")},indent=1,default=float)[:2400])
