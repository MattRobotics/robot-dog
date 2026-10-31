"""Update the canonical workbook with the 2026-10-30 Solid Edge physical properties. Keeps formulas/styles; the previous version is kept by Git."""
import sys, json, hashlib, re, copy
from pathlib import Path
from lxml import etree
sys.path.insert(0,str(Path(__file__).parent)); import xl
X=xl.X
SRC,DST,INPUT_XLSX,BLOCKS,ASM=[Path(a) for a in sys.argv[1:6]]
blocks=json.load(open(BLOCKS)); asm=json.load(open(ASM))
b=xl.Book(SRC)
EV="Weight_Evidence_2026-10-30"; evq="'%s'"%EV
# ---- 1. history is kept by Git (commit 0f8f7ff holds the pre-update workbook); no duplicate sheets or files
# ---- 2. weight evidence sheet: the supplied workbook, formulas intact
inp=xl.Book(INPUT_XLSX); isheet=inp["Foglio1"]; tree=copy.deepcopy(isheet.tree)
for c in tree.iter(X("c")):
    if "s" in c.attrib: del c.attrib["s"]
    if c.get("t")=="s":
        v=c.find(X("v")); v.text=inp.sst[int(v.text)]; c.set("t","str")
for tag in ("tableParts","legacyDrawing","drawing"):
    for e in tree.findall(X(tag)): tree.remove(e)
ev=b.add_sheet(EV,tree)
ev.set_text("P1","Source: 'Evidenze peso e calcoli.xlsx' (sha256 a3b1fc77e9d9b4d49b22714d9adae554e33155379f7c88815c30cfbdffda76a3), copied unchanged on 2026-10-31: values and formulas as supplied.")
# ---- 3. Inertial_Properties
ip=b["Inertial_Properties"]
ip.set_text("A1","MATDOG — Proprietà inerziali REV01-FIX (Solid Edge 2026-10-30, configurazione nominale con Jetson Orin)")
ip.set_text("A2","Fonte canonica per il modello dinamico: masse dei report Solid Edge (risoluzione 1 g), baricentri per-link e tensori d’inerzia al centro di massa, espressi nei frame URDF dei link. Un’unica configurazione nominale, comprensiva della Jetson Orin (175 g, nel base_link). Lo storico REV00 è in Git (commit 0f8f7ff).")
order=list(blocks)
for i,name in enumerate(order):
    r=5+i; bl=blocks[name]; assert b.raw("Inertial_Properties",f"A{r}")==name,(r,name)
    ip.set_num(f"D{r}",bl["mass_kg"])
    for col,v in zip("EFG",bl["com_m"]): ip.set_num(f"{col}{r}",round(v,5))
    for col,k in zip("HIJKLM",("ixx","ixy","ixz","iyy","iyz","izz")): ip.set_num(f"{col}{r}",bl[k])
    direct=bl["rule"]=="CAD direct"
    ip.set_text(f"N{r}","DIRECT CAD" if direct else "DERIVED BY SYMMETRY")
    ip.set_text(f"O{r}","CAD direct: principal axes + radii of gyration" if direct else "CAD symmetry-derived from LF: "+bl["rule"])
    ip.set_text(f"P{r}",bl["source"]+" (Solid Edge 2026-10-30)")
    ip.set_text(f"Q{r}","URDF <inertial> updated 2026-10-31")
lab=ip.style_of("A5"); num=ip.style_of("D5")
ip.set_text("A23","Nominal model total (17 links) [kg]",lab); ip.set_formula("D23","SUM(D5:D21)",num)
ip.set_text("A24","Solid Edge assembly report [kg]",lab);   ip.set_formula("D24","CAD_Source_Data!D10",num)
ip.set_text("A25","Difference model − assembly [kg]",lab);  ip.set_formula("D25","D23-D24",num)
ip.set_text("A26","Origin of the difference: per-link masses are reported at 1 g resolution; the 17 link volumes close on the assembly volume within 1.1 mm³, so no solid is missing. Not allocated to any link (see Mass_Reconciliation).",lab)
# ---- 4. CAD_Source_Data
cs=b["CAD_Source_Data"]
cs.set_text("A1","MATDOG — Dati CAD sorgente per proprietà fisiche (Solid Edge 2026-10-30)")
cs.set_text("A2","Trascrizione strutturata dei report Solid Edge ricevuti il 2026-10-30 (configurazione nominale con Jetson Orin). I report per-link riportano la massa a 0.001 kg; i momenti 'global' sono riferiti all’origine CAD, non al CoM, e sono in kg·dm² come stampati. Gli originali sono in 09_Logs/Validation_Reports/REV01_FIX_Physical_Properties_2026-10-31/input/.")
cs.set_text("Q4","Use in REV01-FIX")
for col,u in zip("HIJKLM",("Ixx global [kg·dm²]","Iyy global [kg·dm²]","Izz global [kg·dm²]","Ixy product [kg·dm²]","Ixz product [kg·dm²]","Iyz product [kg·dm²]")): cs.set_text(f"{col}4",u)
rep={}
sys.path.insert(0,str(Path(__file__).parent)); import physprops as P
IN=INPUT_XLSX.parent
rows=[("base_link","base_link__physprop.txt","base_link","Direct","Includes the Jetson Orin (175 g). Mass used = Solid Edge report."),
      ("lf_hip_link","lf_hip_link__physprop.txt","lf_hip_link","Direct LF / symmetry seed","RF = Y reflection of LF; LH = LF frame; RH = Y reflection (rear hip meshes differ slightly from LF: see Physical_Property_Method)."),
      ("lf_upper_leg_link","lf_upper_leg_link__physprop.txt","lf_upper_leg_link","Direct LF / symmetry seed","RF and RH are derived by local Y reflection; LH uses the same left-side local geometry (mesh-verified)."),
      ("lf_lower_leg_link","lf_lower_leg_link__physprop.txt","lf_lower_leg_link","Direct LF / symmetry seed","RF and RH are derived by local Y reflection; LH uses the same left-side local geometry (mesh-verified)."),
      ("lf_foot_link","lf_foot_link__physprop.txt","lf_foot_link","Direct LF / identical foot seed","All four foot links use the identical CAD foot geometry. Global moments print as 0.000 because of rounding (< 5e-6 kg·m²), not because the inertia is zero.")]
for i,(nm,fn,scope,use,note) in enumerate(rows):
    r=5+i; d=P.parse(IN/fn); g=d["global_report"]; k=1e2     # report unit kg*dm^2 stored in kg*m^2 by parse -> back to kg*dm^2
    cs.set_text(f"A{r}",fn); cs.set_text(f"B{r}",scope); cs.set_num(f"C{r}",d["mass_kg"]); cs.set_formula(f"D{r}",f"C{r}")
    for col,v in zip("EFG",d["com_mm"]): cs.set_num(f"{col}{r}",v)
    for col,kk in zip("HIJKLM",("Ixx","Iyy","Izz","Ixy","Ixz","Iyz")): cs.set_num(f"{col}{r}",round(g[kk]*k,4))
    for col,v in zip("NOP",d["K_mm"]): cs.set_num(f"{col}{r}",v)
    cs.set_text(f"Q{r}",use); cs.set_text(f"R{r}",note)
d=P.parse(IN/"matdog_urdf_assembly__physprop.txt"); g=d["global_report"]
cs.set_text("A10","matdog_urdf_assembly__physprop.txt"); cs.set_text("B10","17-link assembly (frame Base; moments about the CoM, kg·m² printed, converted to kg·dm² here)")
cs.set_num("C10",d["mass_kg"]); cs.set_formula("D10","C10")
for col,v in zip("EFG",d["com_mm"]): cs.set_num(f"{col}{10}",v)
for col,kk in zip("HIJKLM",("Ixx","Iyy","Izz","Ixy","Ixz","Iyz")): cs.set_num(f"{col}10",round(g[kk]*1e2,4))
for col,v in zip("NOP",d["K_mm"]): cs.set_num(f"{col}10",v)
cs.set_text("Q10","Aggregate validation")
cs.set_text("R10","Includes the Jetson Orin. Rebuilt 17-link model: 2.928 kg, CoM deviation 0.42 mm, K within 0.2 %. Volume: assembly 1970535.851 mm³ vs sum of link volumes 1970534.788 mm³.")
# ---- 5. Mass_Summary
ms=b["Mass_Summary"]
ms.set_text("A1","MATDOG — Riepilogo masse REV01-FIX (CAD 2026-10-30)")
ms.set_text("A2","Un’unica configurazione nominale: 17 link con Jetson Orin (2.928 kg per-link; 2.937 kg assieme Solid Edge). La pesata reale del 2026-10-30 (2.807 kg, senza Jetson) è evidenza storica dell’hardware attualmente assemblato. Dettaglio in Mass_Reconciliation.")
ms.set_formula("B5","Inertial_Properties!D23*1000"); ms.set_text("A5","URDF modeled links mass [g]")
ms.set_formula("B6","Inertial_Properties!D23"); 
ms.set_formula("B7","SUM(Inertial_Properties!D6:D21)*1000"); ms.set_formula("B8","Inertial_Properties!D5*1000")
ms.set_formula("B9","CAD_Source_Data!D10*1000")
ms.set_text("A10","Weight-breakdown calculated, WITH Jetson [g]"); ms.set_formula("B10",f"{evq}!J11+{evq}!F16")
ms.set_text("A11","Physical robot measured 2026-10-30, WITHOUT Jetson [g]"); ms.set_formula("B11",f"{evq}!J2")
ms.set_text("A13","Standalone neck yaw link in REV01-FIX"); ms.set_text("B13","No")
ms.set_text("B14","CAD CoM + inertia tensor updated 2026-10-30 (REV01-FIX)")
lab=ms.style_of("A9"); val=ms.style_of("B9")
for r,(a,f) in enumerate((("Jetson Orin, inside the base_link CAD [g]",f"{evq}!F16"),("CAD assembly WITHOUT Jetson [g]","B9-B15"),
        ("Measured − (CAD assembly without Jetson) [g]","B11-B16"),("Weight workbook: measured − analytic without Jetson [g]",f"{evq}!J13"),
        ("Model links − CAD assembly [g]","B5-B9")),start=15):
    ms.set_text(f"A{r}",a,lab); ms.set_formula(f"B{r}",f,val)
ms.set_text("A20","Interpretation",lab); ms.set_text("B20","Residual 38.8 g (workbook) / 45 g (vs CAD): unmodeled internal wiring is a HYPOTHESIS, not a measurement; no mass is added to the model.",lab)
# ---- 6. Mass_Reconciliation
mr=b["Mass_Reconciliation"]
mr.set_text("A1","MATDOG — Riconciliazione masse REV01-FIX")
mr.set_text("A2","Il modello dinamico usa le masse per-link dei report Solid Edge 2026-10-30 (1 g di risoluzione); non è stata applicata alcuna scala. Il calcolo pesi e la pesata reale (senza Jetson) sono conservati come controlli di configurazione, non come scala automatica del tensore d’inerzia.")
mr.set_formula("B5","Inertial_Properties!D23"); mr.set_formula("B6","CAD_Source_Data!D10")
mr.set_text("A8","Weight-breakdown calculated, WITH Jetson [kg]"); mr.set_formula("B8",f"({evq}!J11+{evq}!F16)/1000")
mr.set_text("A9","Physical robot measured 2026-10-30, WITHOUT Jetson [kg]"); mr.set_formula("B9",f"{evq}!J2/1000")
mr.set_text("A10","CAD assembly without Jetson − measured robot [kg]"); mr.set_formula("B10","(B6-B12)-B9")
mr.set_text("A11","(CAD assembly without Jetson − measured robot) / CAD without Jetson"); mr.set_formula("B11","B10/(B6-B12)")
mr.set_text("A12","Jetson Orin, inside the base_link CAD [kg]"); mr.set_formula("B12",f"{evq}!F16/1000")
mr.set_text("A13","5th motor housing / neck yaw"); mr.set_text("B13","Not separately modeled. The weight workbook counts 5 motor housings in the base group; whether the Solid Edge base_link mass includes the fifth is NOT stated by the evidence (UNKNOWN).")
s14=mr.style_of("A12"); s14b=mr.style_of("B12")
mr.set_text("A14","Weight workbook: measured − analytic without Jetson [kg]",s14); mr.set_formula("B14",f"{evq}!J13/1000",s14b)
refs={17:f"({evq}!F33+{evq}!M6+{evq}!M7)/1000",18:f"{evq}!F41/1000",19:f"{evq}!F49/1000",20:f"{evq}!F57/1000",21:f"{evq}!F64/1000"}
cadrow={17:"D5",18:"D6",19:"D7",20:"D8",21:"D9"}
for r in range(17,22):
    mr.set_formula(f"C{r}",f"Inertial_Properties!{cadrow[r]}"); mr.set_formula(f"E{r}",refs[r])
    mr.set_text(f"H{r}","CAD direct (Solid Edge 2026-10-30)" if r==17 else "CAD direct / link CAD report")
mr.set_text("H22","Residual 9 g vs assembly")
mr.set_text("A24","Interpretation: the 17 per-link masses sum to 2.928 kg; the Solid Edge assembly reports 2.937 kg. The 9 g residual is NOT missing geometry (the 17 link volumes close on the assembly volume within 1.1 mm³). It lies at the 1 g resolution of the mass reports: rounding of the 5 distinct reported masses explains at most ±8.5 g if the four legs are identical, so 9 g exceeds that bound by 0.5 g — either truncation instead of rounding, or legs not exactly identical in mass; it cannot be resolved without per-link reports of RF/RH/LH or unrounded values. No mass is added or scaled. The measured robot (2.807 kg, no Jetson) is 45 g above the CAD assembly without Jetson (2.762 kg); the weight workbook attributes 38.8 g to unmodeled wiring, which is a hypothesis.")
# ---- 7. Physical_Property_Method
pm=b["Physical_Property_Method"]
pm.set_text("A1","MATDOG — Metodo di conversione CAD → URDF <inertial> (REV01-FIX)")
T={5:"1. Data scope — Base link and LF subassemblies were exported from Solid Edge on 2026-10-30 in the single nominal configuration that includes the Jetson Orin. Each report prints the CoM, principal-axis directions and radii of gyration. Moments 'rispetto agli assi globali' are about the CAD origin, NOT the CoM: they are used only as a cross-check. The full assembly report is an aggregate validation.",
   6:"2. Mass source — URDF masses are the Solid Edge per-link values (1 g resolution): base 1.828 kg, hip 0.050 kg, upper 0.152 kg, lower 0.059 kg and foot 0.014 kg; 17 links = 2.928 kg. The assembly reports 2.937 kg; the 9 g residual is documented in Mass_Reconciliation and not forced.",
   7:"3. CoM source — The CoM of each report (mm) is converted to metres and used as <inertial><origin xyz=...> in the link frame (the report frame equals the URDF link frame, verified against the REV00 hip CoM).",
   8:"4. Inertia construction — The tensor at the CoM is rebuilt from principal axes and radii of gyration: I_C = Rᵀ diag(m·K₁², m·K₂², m·K₃²) R (units mm, kg·dm² converted to m, kg·m²). Check: the rebuilt tensors moved to the CAD origin reproduce the printed moments within the report rounding (diagonal error ≤ 4.4e-6 kg·m², i.e. below the 5e-6 rounding of 0.0005 kg·dm²). A printed '0.000' (foot) is a rounded small value, not zero inertia.",
   9:"5. Axis cleanup — The printed principal-axis matrices (3 decimals) are orthonormalised to the nearest rotation by SVD before the reconstruction (orthogonality error ≤ 9e-4 before cleanup).",
   10:"6. Right-side derivation — RF = local Y reflection of LF: M = diag(1,−1,1); CoM' = M·CoM, I' = M·I·Mᵀ. Verified on the collision meshes: RF vs reflected LF differs by ≤ 0.0002 mm in CoM and ≤ 8e-6 relative in inertia for hip, upper and lower leg.",
   11:"7. Rear / foot derivation — LH uses the LF frame (upper, lower and foot meshes identical to numerical precision); RH = Y reflection (upper, lower exact). The rear HIP collision meshes differ slightly from the front ones (volume −0.35 %, CoM 0.21 mm, inertia 0.8 % on the mesh estimate): rear hips inherit the LF CAD properties, an uncertainty below the 1 g mass resolution (2 %). Feet are identical.",
   12:"8. Aggregate validation — Rebuilt 17-link model: 2.928 kg, CoM = [0.688, −0.275, 17.286] mm, K = [115.39, 106.76, 73.74] mm. Solid Edge assembly (frame Base): 2.937 kg, CoM = [0.78, −0.59, 17.02] mm, K = [115.56, 106.96, 73.98] mm. CoM deviation 0.42 mm; principal moments within 1 %.",
   13:"9. Product-of-inertia convention — The printed products are ∫xy dm etc.; the URDF tensor off-diagonals are their negatives. Verified on the base_link (printed Ixz −0.021 kg·dm² ⇔ tensor +2.07e-4 kg·m²).",
   14:"10. Neck yaw actuator — No new evidence in the 2026-10-30 package; nothing is added. Whether the base_link CAD includes a fifth motor housing is not stated.",
   15:"11. Simulation scope — A single nominal configuration, including the Jetson Orin. The robot weighed on 2026-10-30 (2.807 kg, without Jetson) is historical evidence of the hardware assembled at that date, not a second configuration. No dynamic hardware validation has been performed on these properties."}
for r,t in T.items(): pm.set_text(f"B{r}",t)
# ---- 8. URDF_Audit
ua=b["URDF_Audit"]
ua.set_text("A1","MATDOG — Audit URDF REV01-FIX con proprietà dinamiche")
ua.set_text("C10","34 riferimenti mesh (visual + collision), tutti a meshes/collision/*.stl: una sola STL per link per entrambi; 17 asset unici; ogni mesh dichiara scale=\"0.001 0.001 0.001\".")
ua.set_text("D10","Unificati in REV01-FIX (STL solo estetiche rimosse).")
ua.set_text("C13","2.928 kg from the Solid Edge per-link reports of 2026-10-30 (assembly 2.937 kg; 9 g = 1 g report resolution); every modeled link has a CAD CoM in its local URDF frame.")
ua.set_text("C14","Tensors reconstructed at CoM from principal axes + radii of gyration; positive definite, symmetric, triangle inequality verified; symmetry-derived legs aggregate-check against the full CAD assembly (CoM 0.42 mm).")
ua.set_text("C15","No standalone neck yaw link or mounted-frame inertial report is included.")
for col,v in zip("ABCDE",("Hip origin Z","PASS","LF/RF 0.0487 m, RH/LH 0.0287 m (+2.2 mm vs REV00); X/Y unchanged.","Aggiornato in REV01-FIX.","Canonico")):
    ua.set_text(f"{col}16",v,ua.style_of(f"{col}15"))
# ---- 8b. single source of truth for masses; unified mesh paths; notes
mt=b["Mesh_Transforms"]; le=b["Link_Engineering"]
for i,name in enumerate(order):
    ir=5+i; r_mt=6+i; r_le=5+i
    assert b.raw("Mesh_Transforms",f"A{r_mt}")==name and b.raw("Link_Engineering",f"A{r_le}")==name,name
    path=f"meshes/collision/{name}.stl"
    mt.set_text(f"B{r_mt}",path); mt.set_text(f"C{r_mt}",path)
    mt.set_formula(f"Q{r_mt}",f"Inertial_Properties!D{ir}"); mt.set_formula(f"P{r_mt}",f"Q{r_mt}*1000")
    mt.set_text(f"S{r_mt}",blocks[name]["source"]+" (Solid Edge 2026-10-30)")
    le.set_text(f"C{r_le}",path); le.set_formula(f"E{r_le}",f"Inertial_Properties!D{ir}"); le.set_formula(f"D{r_le}",f"E{r_le}*1000")
    le.set_text(f"H{r_le}",blocks[name]["source"]+" (Solid Edge 2026-10-30)")
mt.set_text("A2","Origini mesh visual/collision: xyz = 0,0,0 m · rpy = 0°,0°,0°. Visual e collision usano la stessa STL per link (meshes/collision/<link>.stl, REV01-FIX). Gli STL CAD restano in millimetri; ogni <mesh> URDF applica scale=\"0.001 0.001 0.001\". Le masse sono lette da Inertial_Properties.")
le.set_text("A2","Masse, CoM e inerzie dinamiche sono definite dai report Solid Edge del 2026-10-30. Il foglio Inertial_Properties è la sorgente canonica dei valori URDF (le masse qui sono formule verso di esso); Engineering_Weight_Source resta un confronto storico.")
rn=b["Reference_Notes"]
rn.set_text("A1","MATDOG — Riferimento cinematica REV01-FIX (modello matt_robodog_rev00)")
rn.set_text("C12","Visual e collision usano la stessa STL per link (meshes/collision/<link>.stl): valida per verifica geometrica; non ancora ottimizzata per fisica.")
rn.set_text("A22","Proprietà dinamiche REV01-FIX (Solid Edge 2026-10-30)")
rn.set_text("C27","Nessun link separato: non è stato aggiunto senza un report CAD della sua posa montata.")
# ---- 9. Joint_Parameters hip Z (REV01-FIX) and notes
jp=b["Joint_Parameters"]
for r,v in ((8,48.7),(12,48.7),(16,28.7),(20,28.7)): jp.set_num(f"M{r}",v)
jp.set_text("D4","REV01-FIX — hip origin Z +2.2 mm; physical properties 2026-10-30")
rn=b["Reference_Notes"]
rn.set_text("C23","Masse per-link dai report Solid Edge 2026-10-30: totale 17-link = 2.928 kg (assieme 2.937 kg, residuo 9 g = risoluzione 1 g). Vedi Mass_Reconciliation.")
rn.set_text("C25","Tensori al CoM ricostruiti da assi principali + raggi di girazione CAD e inseriti come blocchi <inertial> (configurazione nominale con Jetson Orin).")
rn.set_text("C26","Somma 17 link vs assieme CAD: scarto massimo CoM 0.42 mm; momenti principali entro 1 %.")
# ---- 10. evidence log
lg=b.add_sheet("Physical_Evidence_Log",etree.fromstring(b'<x:worksheet xmlns:x="%s"><x:cols><x:col min="1" max="1" width="52" customWidth="1"/><x:col min="2" max="2" width="70" customWidth="1"/><x:col min="3" max="3" width="14" customWidth="1"/><x:col min="4" max="4" width="60" customWidth="1"/></x:cols><x:sheetData/></x:worksheet>'%xl.NS_X.encode()))
hd=b["Mass_Summary"].style_of("A4"); tx=b["Mass_Summary"].style_of("A5")
for col,t in zip("ABCD",("Evidence file","SHA-256","Role","Location")): lg.set_text(f"{col}1",t,hd)
man=json.load(open(IN/"SHA256SUMS.json"))
loc="09_Logs/Validation_Reports/REV01_FIX_Physical_Properties_2026-10-31/input/"
r=2
for e in man:
    lg.set_text(f"A{r}",e["name"],tx); lg.set_text(f"B{r}",e["sha256"],tx); lg.set_text(f"C{r}","CURRENT",tx); lg.set_text(f"D{r}",loc+e["name"],tx); r+=1
hist=hashlib.sha256(open(SRC,"rb").read()).hexdigest()
lg.set_text(f"A{r}","Coordinate_Cinematiche_matt_robodog_rev00.xlsx (before the 2026-10-31 update)",tx); lg.set_text(f"B{r}",hist,tx); lg.set_text(f"C{r}","HISTORICAL (Git)",tx)
lg.set_text(f"D{r}","git show 0f8f7ff:03_CAD/URDF/matt_robodog_rev00/reference/Coordinate_Cinematiche_matt_robodog_rev00.xlsx",tx); r+=1
lg.set_text(f"A{r}","REV00 physical-property reports (physprop__*.txt, weight_breakdown_summary.txt)",tx); lg.set_text(f"B{r}","not in the repository; their values were transcribed in the superseded sheets of the previous workbook version",tx); lg.set_text(f"C{r}","HISTORICAL (Git)",tx)
# ---- 11. recalc
etree.SubElement(b.wb,X("calcPr"),fullCalcOnLoad="1")
formulas,changed=b.recalc()
b.save(DST)
print("formulas:",len(formulas),"cached values refreshed:",len(changed))
