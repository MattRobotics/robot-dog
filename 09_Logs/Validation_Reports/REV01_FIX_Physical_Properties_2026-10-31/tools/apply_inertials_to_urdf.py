"""Rewrite ONLY the <inertial> blocks (and their provenance comments) of the operational URDF from inertial_blocks.json."""
import json, re, sys
from pathlib import Path
URDF=Path(sys.argv[1]); BLOCKS=json.load(open(sys.argv[2]))
fmt=lambda v: "0" if v==0 else format(v,".12g")
text=URDF.read_text(encoding="utf-8")
HEAD=re.compile(r"(  <!-- MATDOG DYNAMIC PROPERTIES REV00 \|)[^\n]*-->")
new_head=("  <!-- MATDOG DYNAMIC PROPERTIES REV01-FIX | 17-link CAD nominal mass: 2.928 kg (Solid Edge assembly report: 2.937 kg; the 9 g difference is the 1 g resolution of the per-link mass reports, "
          "the 17 link volumes close on the assembly volume within 1.1 mm3) | Single nominal configuration INCLUDING the Jetson Orin (175 g, inside base_link). The robot weighed on 2026-10-30 (2.807 kg) did not carry the Jetson: it is historical evidence, not a second configuration. "
          "Standard URDF <inertial> blocks come from Solid Edge physical-property reports of 2026-10-30; CoM origins are in each local link frame; inertia tensors are about those CoMs, in link axes, reconstructed from principal axes + radii of gyration. "
          "No hardware dynamic validation. Provenance: 09_Logs/Validation_Reports/REV01_FIX_Physical_Properties_2026-10-31/ and Coordinate_Cinematiche_matt_robodog_rev00.xlsx / Inertial_Properties. -->")
assert len(HEAD.findall(text))==1
text=HEAD.sub(lambda m:new_head,text)
for name,b in BLOCKS.items():
    pat=re.compile(r"(    <!-- MATDOG_DYNAMIC_PROPERTIES \|[^\n]*-->\n)?    <inertial>\n      <origin xyz=\"[^\"]*\" rpy=\"0 0 0\" />\n      <mass value=\"[^\"]*\" />\n      <inertia [^\n]*/>\n    </inertial>")
    # locate the link block
    ls=text.index(f'<link name="{name}">'); le=text.index("</link>",ls)
    seg=text[ls:le]
    assert len(pat.findall(seg))==1, name
    kind="CAD direct: principal axes + radii of gyration" if b["rule"]=="CAD direct" else "CAD symmetry-derived from LF: "+b["rule"]
    comment=(f"    <!-- MATDOG_DYNAMIC_PROPERTIES | mass_kg={b['mass_kg']:.6f} | coM_frame=local_link | tensor_frame=local_link_at_com | method={kind} | "
             f"source={b['source']} (Solid Edge 2026-10-30, nominal configuration incl. Jetson Orin) -->\n")
    blk=(f"    <inertial>\n      <origin xyz=\"{' '.join(fmt(round(c,5)) for c in b['com_m'])}\" rpy=\"0 0 0\" />\n"
         f"      <mass value=\"{b['mass_kg']:.6f}\" />\n"
         f"      <inertia ixx=\"{fmt(b['ixx'])}\" ixy=\"{fmt(b['ixy'])}\" ixz=\"{fmt(b['ixz'])}\" iyy=\"{fmt(b['iyy'])}\" iyz=\"{fmt(b['iyz'])}\" izz=\"{fmt(b['izz'])}\" />\n    </inertial>")
    seg=pat.sub(lambda m:comment+blk,seg)
    text=text[:ls]+seg+text[le:]
URDF.write_text(text,encoding="utf-8")
print("rewrote inertials of",len(BLOCKS),"links")
