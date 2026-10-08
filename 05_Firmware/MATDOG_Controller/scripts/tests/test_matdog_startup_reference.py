#!/usr/bin/env python3
"""Synthetic evidence only. No serial or device imports."""
import json
from pathlib import Path
import re
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import matdog_startup_reference as reference


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.log=self.root/'hw_session_all_20261003_134852.log'
        self.q0=self.root/'q0_promoted.json'
        self.export=self.root/'evidence_export_20261003_134852_after_failure.txt'
        events=['SYS PORT_OPEN test','RX SYSTEM_BOOT_COMPLETE ready','RX reset_reason : POWERON',
            'RX SOURCE_SIGNATURE build_id=be0c12979e5b synthetic profile=ROBOT_POWERED board=YD-ESP32-S3 N16R8',
            'RX CALIBRATION_PERSISTENCE_BOOT nvs=READY verdict=NO_RECORD available=0 motion_authorized=0',
            'TX @CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE',
            'RX CALIBRATION_Q0 state=COMPLETE failure=NONE session=1 sample_passes=9/9 next_joint=0 candidates=12/12']
        rows=[]
        for bus,tick in reference.Q0.items():
            leg=['LF','RF','RH','LH'][bus//10-1];joint={1:'LOWER',2:'UPPER',3:'HIP'}[bus%10]
            events.append(f'RX   Q0 bus={bus} leg={leg} joint={joint} unit={reference.UNITS[bus]} tick={tick} spread=0 samples=9 state=CANDIDATE estimator=MANUAL_ZERO_POSE')
            if bus<30:rows.append(f'CALIBRATION_EVIDENCE_Q0 leg={leg} joint={joint} unit={reference.UNITS[bus]} bus={bus} present=1 q0_tick={tick} state=PROMOTED origin=LIVE_SESSION geometry={reference.GEOMETRY}')
        events+=['RX CALIBRATION_Q0_PROMOTE=OK admitted=12/12 source=CURRENT_BOOT_CAPTURE capture_session=1',
            'RX CALIBRATION_SESSION=ACTIVE leg=LF session=1 live=1',
            'RX CALIBRATION_FULL_LEG_RESULT leg=LF verdict=HARDWARE_CONTACT_CALIBRATED failure=NONE',
            'RX CALIBRATION_SESSION=ACTIVE leg=RF session=2 live=1',
            'RX decision=CONFIRMED samples=95,117,34 count=3 published=117 limit=70',
            'RX failed_phase=LOWER_MAX probe_phase=SAFE_OFF_REQUIRED probe_failure=OVER_TEMPERATURE',
            'RX CALIBRATION_FULL_LEG_RESULT leg=RF verdict=FAILED failure=EXECUTOR_FAILED','SYS PORT_CLOSED']
        self.log.write_text(''.join(f'2026-10-03T13:48:55.{i:03} {e}\n' for i,e in enumerate(events)))
        self.q0.write_text(json.dumps({str(b):p for b,p in reference.Q0.items()}))
        self.export.write_text('\n'.join(rows+[
            f'CALIBRATION_EVIDENCE_LEG leg=LF synthetic session=1 geometry={reference.GEOMETRY}',
            f'CALIBRATION_EVIDENCE_LEG leg=RF synthetic session=2 geometry={reference.GEOMETRY}',
            'total_contacts_accepted=6 all_contact_calibrated=0']))
    def tearDown(self):self.tmp.cleanup()
    def test_complete_same_boot_and_firmware_table(self):
        report=reference.verify(self.root)
        self.assertEqual(report['reference_status'],'VERIFIED_12_OF_12')
        self.assertFalse(report['persisted_in_controller']);self.assertFalse(report['automatic_motion_authority'])
        header=(Path(__file__).resolve().parents[2]/'src/calibration/StartupRecoveryReference.h').read_text()
        rows=re.findall(r'\{(\d+),(\d+),(-?\d+),"(\w+)"\}',header)
        self.assertEqual({int(b):(int(q),int(d),u) for b,q,d,u in rows},
                         {b:(q,reference.DIRECTION[b],reference.UNITS[b]) for b,q in reference.Q0.items()})
    def test_incompatible_q0_and_duplicate_json_refused(self):
        original=self.q0.read_text()
        for changed in [original.replace('"22": 2106','"22": 2107'),original[:-1]+',"22":2106}',
                        original.replace('"22": 2106','"22": true')]:
            with self.subTest(changed=changed):
                self.q0.write_text(changed)
                with self.assertRaises(ValueError):reference.verify(self.root)
        self.q0.write_text(original)
    def test_boot_identity_capture_and_phase_mutations_refused(self):
        original=self.log.read_text()
        for changed in [original+original.splitlines()[1]+'\n',original.replace('unit=NEW03','unit=WRONG'),
                        original.replace('tick=2106','tick=2107'),original.replace('session=2 live','session=3 live'),
                        original.replace('candidates=12/12','candidates=11/12'),
                        original.replace('failed_phase=LOWER_MAX','failed_phase=UPPER_MAX')]:
            self.log.write_text(changed)
            with self.assertRaises(ValueError):reference.verify(self.root)
        self.log.write_text(original)
    def test_export_identity_geometry_and_session_mutations_refused(self):
        original=self.export.read_text()
        for changed in [original.replace('joint=UPPER','joint=HIP',1),original.replace(reference.GEOMETRY,'wrong',1),
                        original.replace('session=2','session=3'),original.replace('q0_tick=2106','q0_tick=2107')]:
            self.export.write_text(changed)
            with self.assertRaises(ValueError):reference.verify(self.root)
    def test_pose_classifier_absolute_support_bands(self):
        q0=dict(reference.Q0);self.assertEqual(reference.classify_positions(q0),'NOMINAL')
        pose=dict(q0);pose.update({21:2348,22:1080,32:1665})
        self.assertEqual(reference.classify_positions(pose),'RF_LOWER_MAX_RETURN')
        for bus,p in [(23,q0[23]+11),(32,q0[32]-409),(22,1093),(21,1997+368),(21,4096)]:
            bad=dict(pose);bad[bus]=p
            with self.assertRaises(ValueError):reference.classify_positions(bad)
        del pose[11]
        with self.assertRaises(ValueError):reference.classify_positions(pose)

if __name__=='__main__':unittest.main()
