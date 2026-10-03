"""Run the unchanged G4 independent oracle against the CURRENT source, writing into the G5-A evidence directory only.

The G5-A `core.py` has the same module name as gait_audit/core.py, so gait_audit is placed first on the path and G5-A's core is never imported here.
"""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / '06_Software/Matdog_Core/gait_audit'))
import oracle  # gait_audit/oracle.py

oracle.OUT = ROOT / '09_Logs/Validation_Reports/G5A_Stabilization_Feasibility'
sys.argv = ['oracle.py', '--output', 'oracle_results_g5a.json']
oracle.main()
