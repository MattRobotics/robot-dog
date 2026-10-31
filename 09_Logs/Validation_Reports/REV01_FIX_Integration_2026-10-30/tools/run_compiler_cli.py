"""Offline: run the unmodified V5 compiler CLI (main()) with the kernel dir on sys.path (python -I drops it)."""
import sys
from pathlib import Path
repo = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(repo / "06_Software/Matdog_Core/calibration"))
import matdog_geometry_compiler_v5 as c5
sys.argv = ["matdog_geometry_compiler_v5.py", "--repo-root", str(repo)] + sys.argv[2:]
raise SystemExit(c5.main())
