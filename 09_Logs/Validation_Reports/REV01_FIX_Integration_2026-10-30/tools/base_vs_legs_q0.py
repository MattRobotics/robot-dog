"""Offline: V5-kernel boolean q=0 check of base_link vs every non-hip leg link (REV00 and REV01)."""
import sys, json
from pathlib import Path
repo = Path(sys.argv[1]).resolve(); out = Path(sys.argv[2])
sys.path.insert(0, str(repo / "06_Software/Matdog_Core/calibration"))
from matdog_geometry_scene_v5 import RobotSceneV5
U = repo/"03_CAD/URDF"; res={}
for tag,p in (("REV00","matt_robodog_rev00/matt_robodog_rev00.urdf"),("REV01","matt_robodog_rev01_candidate/matt_robodog_rev01_candidate.urdf"),("REV01FIX","matt_robodog_rev01fix_candidate/matt_robodog_rev01fix_candidate.urdf")):
    sc = RobotSceneV5.from_urdf(U/p); pose = sc.home_pose(); res[tag]={}
    for leg in ("lf","rf","rh","lh"):
        for part in ("upper_leg","lower_leg","foot"):
            l=f"{leg}_{part}_link"; r=sc.check_link_pair("base_link",l,pose,require_distance=False); res[tag][l]=r.status
    bad={k:v for k,v in res[tag].items() if v=="INTERSECTING"}
    print(tag, "pairs", len(res[tag]), "INTERSECTING:", bad or "none"); print("  statuses:", sorted(set(res[tag].values())))
out.write_text(json.dumps(res,indent=1))
