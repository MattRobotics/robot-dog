"""Offline post-integration regression of the canonical model (read-only).
usage: verify_integration.py <repo_root>"""
import hashlib, json, struct, subprocess, sys, xml.etree.ElementTree as ET
from pathlib import Path
R = Path(sys.argv[1]).resolve(); M = R / "03_CAD/URDF/matt_robodog_rev00"; ok = True
def check(name, cond, detail=""):
    global ok; ok &= bool(cond); print(("PASS " if cond else "FAIL ") + name + (f"  [{detail}]" if detail else ""))
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
root = ET.parse(M / "matt_robodog_rev00.urdf").getroot()
links = root.findall("link"); check("17 links", len(links) == 17)
same = 0; files = set(); scale_ok = True; origin_ok = True
for l in links:
    v = l.find("visual/geometry/mesh"); c = l.find("collision/geometry/mesh")
    if v.get("filename") == c.get("filename") == f"meshes/collision/{l.get('name')}.stl": same += 1
    files.add(c.get("filename"))
    scale_ok &= v.get("scale") == c.get("scale") == "0.001 0.001 0.001"
    ov, oc = l.find("visual/origin"), l.find("collision/origin")
    origin_ok &= (ov.get("xyz"), ov.get("rpy")) == (oc.get("xyz"), oc.get("rpy")) == ("0 0 0", "0 0 0")
check("17/17 visual == collision (same file, meshes/collision/<link>.stl)", same == 17)
check("scale 0.001 on all visual+collision", scale_ok); check("visual/collision origin identical (0 0 0 / 0 0 0)", origin_ok)
stl = sorted((M / "meshes").rglob("*.stl"))
check("exactly 17 STL in model dir", len(stl) == 17 and {str(p.relative_to(M)) for p in stl} == files, f"{len(stl)} files")
check("no STL outside meshes/collision", all(p.parent.name == "collision" for p in stl))
tri = 0; lfs = False; bad = []
for p in stl:
    d = p.read_bytes(); lfs |= d.startswith(b"version https://git-lfs")
    n = struct.unpack("<I", d[80:84])[0]
    if len(d) != 84 + 50 * n or n == 0: bad.append(p.name)
    tri += n
check("no LFS pointer used as mesh; all binary STL well-formed", not lfs and not bad, str(bad))
check("total triangles 134484", tri == 134484, str(tri))
check("base_link SHA == REV01-FIX", sha(M / "meshes/collision/base_link.stl") == "c2eadf6d6db1b03379cf256b542dadbcd8d63076179bc84ebd7169634e9f0097")
check("base_link 36832 triangles", struct.unpack("<I", (M / "meshes/collision/base_link.stl").read_bytes()[80:84])[0] == 36832)
manifest = {l.split(maxsplit=1)[1]: l.split()[0] for l in (M / "SHA256SUMS.txt").read_text().splitlines()}
check("SHA256SUMS: 19 entries, all match", len(manifest) == 19 and all(sha(M / k) == v for k, v in manifest.items()))
orig = {}  # original 16 leg collision hashes from the REV00 manifest in git history
old = subprocess.run(["git", "-C", str(R), "show", "origin/main:03_CAD/URDF/matt_robodog_rev00/SHA256SUMS.txt"], capture_output=True, text=True).stdout
for line in old.splitlines():
    h, p = line.split(maxsplit=1)
    if p.startswith("meshes/collision/") and not p.endswith("base_link.stl"): orig[p] = h
check("other 16 collision STL identical to origin/main", len(orig) == 16 and all(sha(M / p) == h for p, h in orig.items()))
hips = {j.get("name"): j.find("origin").get("xyz") for j in root.findall("joint") if j.get("name").endswith("_hip_joint")}
check("HIP origins", hips == {"lf_hip_joint": "0.1125 0.0475 0.0487", "rf_hip_joint": "0.1125 -0.0475 0.0487", "rh_hip_joint": "-0.1125 -0.0475 0.0287", "lh_hip_joint": "-0.1125 0.0475 0.0287"}, str(hips))
check("16 joints, 12 revolute", len(root.findall("joint")) == 16 and sum(j.get("type") == "revolute" for j in root.findall("joint")) == 12)
h = (R / "05_Firmware/MATDOG_Controller/src/motion/LegGeometryData.h").read_text()
check("LegGeometryData.h embeds new URDF SHA", f'"{sha(M / "matt_robodog_rev00.urdf")}"' in h)
check("LegGeometryData.h hip Z 0.0487/0.0287", h.count(", 0.0487}") == 2 and h.count(", 0.0287}") == 2)
check("LegGeometryData.h has no old hip Z", "0.0465}" not in h and "0.026499999999999999" not in h)
print("URDF sha256", sha(M / "matt_robodog_rev00.urdf")); print("LegGeometryData.h sha256", sha(R / "05_Firmware/MATDOG_Controller/src/motion/LegGeometryData.h"))
print("OVERALL", "PASS" if ok else "FAIL"); sys.exit(0 if ok else 1)
