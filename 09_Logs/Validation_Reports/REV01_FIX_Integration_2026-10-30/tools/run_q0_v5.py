"""Offline: run the real Geometry Compiler V5 kernel on a URDF (q=0 gate first).

Usage: python3 -I run_q0_v5.py <repo_root_copy> <urdf> <out_json> [--full]
No hardware, no repo writes outside <out_json>.
"""
import json
import sys
from pathlib import Path

repo_root = Path(sys.argv[1]).resolve()
urdf = Path(sys.argv[2]).resolve()
out = Path(sys.argv[3])
full = "--full" in sys.argv
sys.path.insert(0, str(repo_root / "06_Software/Matdog_Core/calibration"))

import matdog_geometry_compiler_v5 as c5  # noqa: E402
from matdog_geometry_scene_v5 import RobotSceneV5  # noqa: E402

res = {"urdf": str(urdf), "kernel_dir": str(repo_root / "06_Software/Matdog_Core/calibration")}

scene = RobotSceneV5.from_urdf(urdf)
q0 = scene.q0_active_pair_results()
res["q0_active_pairs"] = {
    j: {
        "pair": list(scene.model.active_revolute_pair(j)),
        "status": r.status,
        "clearance_m": r.clearance_m,
        "witness_tri_a": r.witness_triangle_a,
        "witness_tri_b": r.witness_triangle_b,
    }
    for j, r in q0.items()
}
res["q0_count"] = len(q0)
res["q0_intersecting"] = sorted(j for j, r in q0.items() if r.status == "INTERSECTING")

if full:
    try:
        run = c5.run_geometry_compiler_v5(repo_root=repo_root, urdf_path=urdf, workers=1)
        res["compiler"] = "COMPLETED"
        res["runtime_seconds"] = run.runtime_seconds
        res["semantic_content_sha256"] = run.profile["semantic_content_sha256"]
        res["q0_status_by_joint"] = run.q0_status_by_joint
    except c5.GeometryCompilerV5Error as exc:
        res["compiler"] = "STOP"
        res["compiler_error"] = str(exc)

out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(json.dumps(res, indent=2, sort_keys=True))
print(json.dumps({k: v for k, v in res.items() if k != "q0_active_pairs"}, indent=2))
for j, v in res["q0_active_pairs"].items():
    print(f"{j:14s} {v['pair'][0]:>14s}/{v['pair'][1]:<14s} {v['status']:18s} w=({v['witness_tri_a']},{v['witness_tri_b']})")
