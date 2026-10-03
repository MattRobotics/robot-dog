"""Run the unchanged G4 independent oracle against the CURRENT source, writing into the G5-A evidence directory only."""
import sys
from core import OUT, ROOT
sys.path.insert(0, str(ROOT / '06_Software/Matdog_Core/gait_audit'))
import oracle  # gait_audit/oracle.py

oracle.OUT = OUT
sys.argv = ['oracle.py', '--output', 'oracle_results_g5a.json']
oracle.main()
