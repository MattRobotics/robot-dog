import numpy as np, json, sys, xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent)); import physprops as P
WT=P.WT
def link_positions(urdf):
    r=ET.parse(urdf).getroot(); parent={}; off={}
    for j in r.findall("joint"):
        xyz=np.array([float(x) for x in j.find("origin").get("xyz").split()])
        assert j.find("origin").get("rpy","0 0 0").split()==["0","0","0"]
        parent[j.find("child").get("link")]=j.find("parent").get("link"); off[j.find("child").get("link")]=xyz
    pos={"base_link":np.zeros(3)}
    def get(l):
        if l not in pos: pos[l]=get(parent[l])+off[l]
        return pos[l]
    for l in list(parent): get(l)
    return pos
MY=np.diag([1,-1,1.])
def build_blocks():
    lf={}
    for n in ["base_link","lf_hip_link","lf_upper_leg_link","lf_lower_leg_link","lf_foot_link"]:
        d=P.parse(P.IN/f"{n}__physprop.txt"); I,lam,Q,det,ortho=P.tensor_at_com(d)
        lf[n]=(d["mass_kg"],np.array(d["com_mm"])*1e-3,I)
    blocks={"base_link":lf["base_link"]}
    for leg,M in (("lf",np.eye(3)),("rf",MY),("rh",MY),("lh",np.eye(3))):
        for k in("hip","upper_leg","lower_leg","foot"):
            m,c,I=lf[f"lf_{k}_link"]; blocks[f"{leg}_{k}_link"]=(m,M@c,M@I@M.T)
    return blocks
def aggregate(blocks,pos):
    M=sum(b[0] for b in blocks.values()); com=sum(b[0]*(pos[n]+b[1]) for n,b in blocks.items())/M
    I=np.zeros((3,3))
    for n,(m,c,Ic) in blocks.items():
        r=pos[n]+c-com; I+=Ic+m*(np.dot(r,r)*np.eye(3)-np.outer(r,r))
    return M,com,I
if __name__=="__main__":
    pos=link_positions(WT/"03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf")
    blocks=build_blocks(); M,com,I=aggregate(blocks,pos)
    print("17-link model: mass %.4f kg  CoM [mm] = %s"%(M,np.round(com*1e3,3)))
    w,V=np.linalg.eigh(I); print("principal moments [kg m2]:",np.round(w[::-1],6)," K [mm]:",np.round(np.sqrt(w[::-1]/M)*1e3,2))
    a=P.parse(P.IN/"matdog_urdf_assembly__physprop.txt"); print("assembly report: mass",a["mass_kg"],"CoM",a["com_mm"],"frame",a["frame"])
    print("  principal K report:",a["K_mm"]," I:",np.round(np.array(a['principal_report']),6))
    # report in Base frame; compare with 17-link model rescaled to assembly mass too
    Ia,lam,Q,det,ortho=P.tensor_at_com(a); print("assembly tensor about CoM (report-rebuilt):\n",np.round(Ia,6)); print("17-link tensor about CoM:\n",np.round(I,6))
    print("CoM deviation [mm]:",np.round((com-np.array(a['com_mm'])*1e-3)*1e3,3)," |d|=%.3f"%(np.linalg.norm(com-np.array(a['com_mm'])*1e-3)*1e3))
    print("tensor deviation (diag) rel:",np.round((np.diag(I)-np.diag(Ia))/np.diag(Ia),4))
    out={"model_mass":M,"model_com_m":com.tolist(),"model_I_com":I.tolist(),"report_assembly_I_com":Ia.tolist()}
    json.dump(out,open(Path(__file__).resolve().parents[1]/"data"/"assembly_check.json","w"),indent=1)
