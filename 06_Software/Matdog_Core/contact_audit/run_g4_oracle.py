"""Run the unchanged G4 independent oracle against the CURRENT source without touching G4 evidence.

gait_audit/oracle.py writes into the G4 evidence directory. This wrapper redirects its output directory to the G4.1
evidence directory, so accepted G4 artifacts (including oracle_results.json) stay byte-identical.
"""
import sys
import common
from common import OUT
import oracle  # gait_audit/oracle.py (common put gait_audit on sys.path)

oracle.OUT = OUT
sys.argv = ['oracle.py', '--output', 'oracle_results_g41.json']
oracle.main()
