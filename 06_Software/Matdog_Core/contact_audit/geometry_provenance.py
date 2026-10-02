"""Verify that the geometry G4 used is the canonical one, and that generated G1/G2/G3.5/G4 data agree with it.

Read-only. The calibration worktree is only READ (hashes of canonical files); nothing there is modified.
"""
import json
import re
import subprocess
import sys
import numpy as np
from common import ROOT, URDF_DIR, G4, OUT, sha, save, foot_vertices, g2_cylinder

CALIBRATION_WORKTREE = ROOT.parent.parent / 'github/robot-dog'


def sums_check(base):
    bad, n = [], 0
    for line in (base / 'SHA256SUMS.txt').read_text().splitlines():
        if not line.strip():
            continue
        digest, rel = line.split(None, 1)
        rel = rel.strip().lstrip('*')
        n += 1
        if sha(base / rel) != digest:
            bad.append(rel)
    return n, bad


def main():
    n, bad = sums_check(URDF_DIR)
    urdf = sha(URDF_DIR / 'matt_robodog_rev00.urdf')
    result = dict(urdf_sha256=urdf, sha256sums_entries=n, sha256sums_mismatches=bad)
    # recorded hashes in every generation that consumed the geometry
    yaml_text = (ROOT / '06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml').read_text()
    g2_urdf = re.search(r'canonical_urdf:.*?sha256:\s*([0-9a-f]{64})', yaml_text, re.S).group(1)
    g2_mesh = re.search(r'canonical_mesh_sha256:\s*([0-9a-f]{64})', yaml_text).group(1)
    result['g2_yaml'] = dict(urdf_sha256=g2_urdf, matches=g2_urdf == urdf, mesh_sha256=g2_mesh,
                             mesh_is_visual_foot_stl=g2_mesh == sha(URDF_DIR / 'meshes/lf_foot_link.stl'),
                             mesh_is_collision_foot_stl=g2_mesh == sha(URDF_DIR / 'meshes/collision/lf_foot_link.stl'))
    pose = json.loads((ROOT / '09_Logs/Validation_Reports/G35_Pose_Audit/pose_library.json').read_text())['provenance']['canonical_sources']
    result['g35_pose_library'] = {k: dict(recorded=v, current=sha(ROOT / k) if (ROOT / k).exists() else None) for k, v in pose.items() if 'matt_robodog_rev00' in k or 'FOOT_CONTACT' in k}
    d4 = json.loads((G4 / 'definitions.json').read_text())['canonical_sources']
    result['g4_definitions'] = {k: dict(recorded=v, current=sha(ROOT / k) if (ROOT / k).exists() else None) for k, v in d4.items() if 'matt_robodog_rev00' in k or 'FOOT_CONTACT' in k}
    for section in ('g35_pose_library', 'g4_definitions'):
        result[section + '_all_match'] = all(v['recorded'] == v['current'] for v in result[section].values())
    # generated G1/G2 artifacts agree with the URDF (their own --check modes)
    checks = {}
    for name in ('matdog_motion_geometry_export.py', 'matdog_contact_stand_export.py'):
        p = subprocess.run([sys.executable, str(ROOT / '06_Software/Matdog_Core/kinematics' / name), '--check'], capture_output=True, text=True)
        checks[name] = dict(returncode=p.returncode, tail=(p.stdout + p.stderr).strip().splitlines()[-1:] )
    result['generated_geometry_checks'] = checks
    # canonical files in the separate calibration worktree (read only)
    cal = {}
    if CALIBRATION_WORKTREE.exists():
        cal_urdf = CALIBRATION_WORKTREE / '03_CAD/URDF/matt_robodog_rev00'
        diffs = [str(p.relative_to(URDF_DIR)) for p in sorted(URDF_DIR.rglob('*')) if p.is_file() and p.suffix in ('.urdf', '.stl')
                 and (not (cal_urdf / p.relative_to(URDF_DIR)).exists() or sha(p) != sha(cal_urdf / p.relative_to(URDF_DIR)))]
        yaml_cal = CALIBRATION_WORKTREE / '06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml'
        cal = dict(worktree_present=True, urdf_and_stl_files_differing=diffs,
                   foot_contact_yaml_identical=yaml_cal.exists() and sha(yaml_cal) == sha(ROOT / '06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml'),
                   head=subprocess.run(['git', '-C', str(CALIBRATION_WORKTREE), 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip())
    else:
        cal = dict(worktree_present=False)
    result['calibration_worktree_read_only_comparison'] = cal
    # four feet: byte-different collision STLs, geometrically one foot
    feet = {}
    ref = None
    for leg in ('lf', 'rf', 'rh', 'lh'):
        for kind in ('collision', 'visual'):
            v, meta = foot_vertices(kind, leg)
            key = lambda a: a[np.lexsort(a.T[::-1])]
            if kind == 'collision':
                ref = v if leg == 'lf' else ref
                spread = float(np.abs(key(v) - key(ref)).max()) if len(v) == len(ref) else None
            else:
                spread = None
            feet[f'{leg}_{kind}'] = dict(**meta, unique_vertices=int(len(v)), max_vertex_spread_vs_lf_collision_m=spread)
    result['foot_meshes'] = feet
    result['g2_cad_nominal'] = {k: (v.tolist() if hasattr(v, 'tolist') else v) for k, v in g2_cylinder().items()}
    result['assessment'] = dict(
        geometry_consistent=(not bad and result['g2_yaml']['matches'] and result['g35_pose_library_all_match'] and result['g4_definitions_all_match']
                             and all(c['returncode'] == 0 for c in checks.values()) and not cal.get('urdf_and_stl_files_differing')),
        notes=['Geometry V5 (calibration compiler) consumes the same URDF and meshes; it does not define a different foot or contact geometry.',
               'The G2 YAML mesh audit (hash e43737..., lowest z 16.7 um) refers to the VISUAL foot STL; G3.5/G4 collision checks use the COLLISION foot STL (different file, different tessellation and registration).',
               'The four collision foot STLs differ in bytes (triangle order) but hold the same vertex set in foot_link.'])
    save('geometry_provenance.json', result)
    print(json.dumps(result['assessment'], indent=1))
    return 0 if result['assessment']['geometry_consistent'] else 1


if __name__ == '__main__':
    sys.exit(main())
