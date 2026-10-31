"""Markdown tables for REPORT.md: new vs previous (REV00) inertial blocks and nominal-model aggregates."""
import sys, json, numpy as np, xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent)); import assemble as A, physprops as P
old_urdf=Path(sys.argv[1]); new=json.load(open(Path(__file__).resolve().parents[1]/"data"/"inertial_blocks.json"))
def blocks_from_urdf(p):
    r=ET.parse(p).getroot(); out={}
    for l in r.findall("link"):
        i=l.find("inertial"); a=i.find("inertia").attrib
        c=np.array([float(x) for x in i.find("origin").get("xyz").split()])
        I=np.array([[a["ixx"],a["ixy"],a["ixz"]],[a["ixy"],a["iyy"],a["iyz"]],[a["ixz"],a["iyz"],a["izz"]]],dtype=float)
        out[l.get("name")]=(float(i.find("mass").get("value")),c,I)
    return out
old=blocks_from_urdf(old_urdf); pos=A.link_positions(P.WT/"03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf")
print("### Per-link table (SI; CoM in the link frame; tensor about the CoM in link axes)\n")
print("| link | m [kg] | CoM x,y,z [mm] | Ixx | Iyy | Izz | Ixy | Ixz | Iyz | rule |"); print("|---|---|---|---|---|---|---|---|---|---|")
for n,b in new.items():
    print("| %s | %.3f | %.2f, %.2f, %.2f | %.5e | %.5e | %.5e | %.4e | %.4e | %.4e | %s |"%(n,b["mass_kg"],*(np.array(b["com_m"])*1e3),b["ixx"],b["iyy"],b["izz"],b["ixy"],b["ixz"],b["iyz"],b["rule"].split(" (")[0]))
print("\n### Change vs previous REV00 blocks\n"); print("| link | m old → new [kg] | ΔCoM [mm] | Ixx old → new | Iyy old → new | Izz old → new |"); print("|---|---|---|---|---|---|")
for n,b in new.items():
    m0,c0,I0=old[n]; dc=(np.array(b["com_m"])-c0)*1e3
    print("| %s | %.4f → %.3f | %.2f, %.2f, %.2f | %.4e → %.4e | %.4e → %.4e | %.4e → %.4e |"%(n,m0,b["mass_kg"],*dc,I0[0,0],b["ixx"],I0[1,1],b["iyy"],I0[2,2],b["izz"]))
nb=A.build_blocks()
for label,bl in (("previous REV00 blocks",old),("new nominal blocks",nb)):
    M,com,I=A.aggregate(bl,pos); w=np.sort(np.linalg.eigvalsh(I))[::-1]
    print("\n%s: mass %.4f kg, CoM [mm] %s, I about CoM (base axes) diag %s, off-diag xy,xz,yz %s, principal %s kg m2"%(label,M,np.round(com*1e3,3).tolist(),np.round(np.diag(I),5).tolist(),np.round([I[0,1],I[0,2],I[1,2]],6).tolist(),np.round(w,5).tolist()))
