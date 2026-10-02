"""Offline gates for the G4 gait study artifacts: preservation, pass predicate, representatives, derivatives."""
import hashlib
import json
import re
import unittest
from pathlib import Path
import numpy as np
from survey import OUT  # before contact_discrepancy: it prepends pose_audit/, which has its own survey.py
import audit_gate
import contact_discrepancy
import importlib.util
from core import ROOT
# pose_audit/ is on sys.path and has its own render.py: load the gait renderer by path.
_spec = importlib.util.spec_from_file_location('gait_render', Path(__file__).with_name('render.py'))
render = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(render)

CHECKPOINT = json.loads((OUT / 'checkpoint_manifest.json').read_text())
# Raw checkpoint evidence that must stay byte-identical. oracle_results.json and definitions.json are
# preserved too; the final oracle run is written to oracle_results_final.json.
RAW = ['full_cases.json', 'refinement_full.json', 'screen.json', 'refinement_screen.json', 'envelope_grid.csv', 'lifecycle_audit.json',
       'oracle_results.json', 'definitions.json', 'xgo_architecture.json']


def load(name):
    return json.loads((OUT / name).read_text())


class PreservationTests(unittest.TestCase):
    def test_raw_checkpoint_evidence_is_byte_identical(self):
        for name in RAW:
            rec = CHECKPOINT['files']['09_Logs/Validation_Reports/G4_Gait_Envelope/' + name]
            data = (OUT / name).read_bytes()
            self.assertEqual(hashlib.sha256(data).hexdigest(), rec['sha256'], name)
            self.assertEqual(len(data), rec['bytes'], name)

    def test_checkpoint_logs_and_handoff_preserved(self):
        for rel, rec in CHECKPOINT['files'].items():
            if '/checkpoint_logs/' in rel or rel.endswith(('G4_ORIGINAL_REQUEST.txt', 'G4_CHECKPOINT_REQUEST.txt', 'G4_CLAUDE_HANDOFF_2026-09-29.md')):
                self.assertEqual(hashlib.sha256((ROOT / rel).read_bytes()).hexdigest(), rec['sha256'], rel)


class PredicateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.audit = audit_gate.build()

    def test_saved_audit_is_reproducible(self):
        self.assertEqual(self.audit, load('full_case_audit.json'))

    def test_counts_and_semantics_of_complete(self):
        s = self.audit['summary']
        self.assertEqual((s['total_cases'], s['full_mesh_validated']), (26, 19))
        self.assertEqual(s['by_type'], {'WALK': {'total': 12, 'validated': 9}, 'TROT': {'total': 14, 'validated': 10}})
        # per-case `complete` is `not errors`: here it coincides with the independent predicate for every case
        self.assertEqual(s['raw_flag_disagreements'], [])
        self.assertEqual(s['raw_metric_disagreements'], [])
        for raw in ('full_cases.json', 'refinement_full.json'):
            self.assertTrue(load(raw)['complete'])  # top-level: all planned cases were evaluated, nothing more

    def test_checkpoint_aggregates_confirmed(self):
        a = self.audit['summary']['aggregate_all_validated']
        h = json.loads((OUT / 'handoff_state.json').read_text())['passing_full_case_aggregate']
        self.assertAlmostEqual(a['min_joint_margin_rad'], h['min_joint_margin_rad'], 15)
        self.assertAlmostEqual(a['min_self_separation_m'], h['min_self_separation_m'], 15)
        self.assertAlmostEqual(a['min_nonfoot_ground_m'], h['min_nonfoot_ground_m'], 15)
        self.assertAlmostEqual(a['max_condition'], h['max_condition'], 12)
        self.assertAlmostEqual(self.audit['summary']['min_walk_support_margin_m'], h['min_walk_support_margin_m'], 15)
        self.assertLess(a['max_stance_drift_m'], 1e-15)
        self.assertEqual(a['total_branch_changes'], 0)

    def test_failed_candidates_stay_failed(self):
        failed = {c['id'] for c in self.audit['cases'] if not c['FULL_MESH_VALIDATED']}
        self.assertEqual(failed, {34, 226, 925, 1311, 1435, 1441, 1948})

    def test_predicate_rejects_tampering(self):
        case = json.loads(json.dumps(next(c for _, c in audit_gate.load() if c['id'] == 287)))
        case['frames'][10]['errors'] = ['GROUND_PENETRATION:rh_foot_link']
        self.assertFalse(audit_gate.audit_case('x', case)['FULL_MESH_VALIDATED'])
        case = json.loads(json.dumps(next(c for _, c in audit_gate.load() if c['id'] == 287)))
        case['frames'][10]['support_margin_m'] = -1e-3
        self.assertFalse(audit_gate.audit_case('x', case)['FULL_MESH_VALIDATED'])


class ContactDiscrepancyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = load('contact_discrepancy.json')

    def test_mesh_offset_and_prediction(self):
        fm = self.data['foot_mesh']
        self.assertAlmostEqual(fm['mesh_circle_centre_offset_um']['magnitude'], 5.775, 2)
        self.assertLess(self.data['measured_vs_prediction']['max_abs_error_um'], 1e-9)
        self.assertGreater(fm['tessellation_residual_um']['min'], -0.01)  # tessellation never adds penetration beyond the offset
        self.assertGreater(fm['fraction_of_pitch_angles_below_minus_1um'], 0.4)

    def test_lifecycle_rejection_is_recorded_and_unchanged(self):
        lifecycle = load('lifecycle_audit.json')['cases']
        self.assertEqual([c['complete_sampled_geometry_valid'] for c in lifecycle], [False, False])
        self.assertEqual([c['failure_counts'] for c in self.data['lifecycle']], [c['failure_counts'] for c in lifecycle])
        self.assertAlmostEqual(self.data['lifecycle'][0]['worst_mesh_ground_um'], -1.2163, 3)
        self.assertAlmostEqual(self.data['lifecycle'][1]['worst_mesh_ground_um'], -1.1830, 3)

    def test_mesh_is_not_changed_by_tools(self):
        self.assertEqual(self.data['unchanged'][1], '1 um ground tolerance')


class RepresentativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reps = load('representatives.json')['representatives']

    def test_gates_and_ids(self):
        ids = {r['id'] for r in self.reps}
        self.assertEqual(ids, {357, 287, 61, 309})
        self.assertNotIn(1311, ids)
        for r in self.reps:
            self.assertTrue(r['gates']['R1_full_mesh_audit'])
            self.assertGreaterEqual(r['gates']['R2_pitch_sweep_min_delta_um'], -1.0)
            self.assertEqual(r['status'] == 'FULL_MESH_VALIDATED', r['gates']['R3_ground_screen_1600'])
            if r['type'] == 'TROT':
                self.assertEqual(r['dynamic_stability'], 'NOT_YET_PROVEN')

    def test_render_refuses_failed_candidate(self):
        failed = next(c for c in load('full_cases.json')['cases'] if c['id'] == 1311)
        self.assertFalse(failed['complete'])
        with self.assertRaises(AssertionError):
            render.representative(dict(id=1311, frames_source='full_cases.json', status='FULL_MESH_VALIDATED'))

    def test_dense_ground_probe_documents_fragility(self):
        probe = load('dense_ground_probe.json')['pass_counts_by_density']
        counts = [probe[str(n)]['passing'] for n in (80, 200, 400, 800, 1600)]
        self.assertEqual(counts[0], 19)
        self.assertEqual(counts, sorted(counts, reverse=True))
        self.assertLess(counts[-1], counts[0])


class DerivativeTests(unittest.TestCase):
    def test_scaling_and_time_independence(self):
        data = load('derivative_tables.json')
        self.assertEqual(set(data['cases']), {'walk', 'walk_100mm', 'trot', 'trot_120mm'})
        for case in data['cases'].values():
            rows = {r['period_s']: r for r in case['periods']}
            self.assertEqual(sorted(rows), [0.5, 1.0, 2.0, 4.0])
            for T, r in rows.items():
                self.assertEqual(r['max_q_difference_vs_T0p5_rad'], 0.0)
                np.testing.assert_allclose(np.array(r['peak_qdot_rad_s']) * T, np.array(rows[1.0]['peak_qdot_rad_s']), rtol=1e-12, atol=1e-12)
                np.testing.assert_allclose(np.array(r['peak_qddot_rad_s2']) * T * T, np.array(rows[1.0]['peak_qddot_rad_s2']), rtol=1e-12, atol=1e-12)
                self.assertLess(r['finite_difference_relative_error']['qdot'], 1e-4)
                self.assertTrue(np.isfinite(r['peak_qddot_rad_s2']).all())


class SafetyBoundaryTests(unittest.TestCase):
    FORBIDDEN = re.compile(r'\b(ServoBus|GoalPosition|Torque_?ON|EEPROM|motorDirection|SerialPort|Serial\.|raw_ticks|flash_app)\b|import serial|open\(.*/dev/tty')

    def test_gait_study_has_no_hardware_dependency(self):
        files = [p for p in Path(__file__).parent.glob('*.py') if p.name != Path(__file__).name]
        files += list((ROOT / '05_Firmware/MATDOG_Controller/src/motion').glob('Gait.*')) + list((ROOT / '05_Firmware/MATDOG_Controller/src/motion').glob('Locomotion.*'))
        for p in files:
            self.assertIsNone(self.FORBIDDEN.search(p.read_text()), p.name)


if __name__ == '__main__':
    unittest.main(verbosity=2)
