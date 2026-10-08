#!/usr/bin/env python3
"""Only the checks affected by the explicit startup path. No hardware I/O."""
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import static_audit as audit


class StartupAuditTests(unittest.TestCase):
    def setUp(self):
        audit.failures.clear()
        self.files=[(p,audit.strip_comments(p.read_text())) for p in audit.iter_source_files()]
    def test_affected_contracts(self):
        for check in (audit.check_startup_servo_selftest_wiring,audit.check_startup_recovery_wiring,audit.check_actuator_runtime_boundaries,
                audit.check_calibration_execution_engine_boundaries,audit.check_actuator_infrastructure_wired_fail_closed,
                audit.check_first_motion_command_wiring,audit.check_full_leg_calibration_wiring):check(self.files)
        audit.check_calibration_persistence_integration(self.files,audit.SKETCH_DIR)
        self.assertEqual(audit.failures,[])
    def test_startup_servo_selftest_contract(self):
        audit.failures.clear()
        audit.check_startup_servo_selftest_wiring(self.files)
        self.assertEqual(audit.failures, [])

        mutations = [
            ("startup_servo_census_pending_ = servo_census_.start();",
             "startup_servo_census_pending_ = false;"),
            ("census.verdict == servo::CensusVerdict::PASS",
             "false"),
            ("servo_preflight_.begin(&servo_bus_);",
             "servo_preflight_.begin(&servo_bus_); servo_preflight_.start();"),
        ]

        for old, new in mutations:
            audit.failures.clear()
            changed = [
                (p, code.replace(old, new, 1)
                 if p.name == "Controller.cpp" else code)
                for p, code in self.files
            ]
            audit.check_startup_servo_selftest_wiring(changed)
            self.assertTrue(audit.failures, old)

    def test_certificate_and_authority_mutations_rejected(self):
        for name,old,new in [('StartupRecoveryReference.cpp','lo=388;hi=408','lo=388;hi=409'),
                ('StartupRecoveryReference.h','{22,2106,-1','{22,2107,-1'),
                ('CommandRouter.cpp','!q->ready(millis())','false'),
                ('FullLegCalibrationExecutor.cpp','!context.startup_motion_permit','false')]:
            audit.failures.clear()
            changed=[(p,code.replace(old,new) if p.name==name else code) for p,code in self.files]
            audit.check_startup_recovery_wiring(changed)
            self.assertTrue(audit.failures,(name,old))

if __name__=='__main__':unittest.main(argv=[sys.argv[0]])
