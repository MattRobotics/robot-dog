"""Offline gates for the G4.1 contact reconciliation: geometry, root cause, contract, lifecycles, preservation, replay."""
import hashlib
import json
import re
import unittest
from pathlib import Path
import numpy as np
import common
from common import OUT, G4, ROOT, load, foot_vertices, delta_um, g2_cylinder, UM
import contact_model as cm

G4_CHECKPOINT_FINAL_HEAD = '3ce85f92e2b5381b65e327dd371b2f164308f326'


class GeometryTests(unittest.TestCase):
    def test_provenance_is_consistent(self):
        d = load('geometry_provenance.json')
        self.assertTrue(d['assessment']['geometry_consistent'])
        self.assertEqual(d['sha256sums_mismatches'], [])
        self.assertTrue(d['g2_yaml']['matches'])
        self.assertTrue(d['g2_yaml']['mesh_is_visual_foot_stl'] and not d['g2_yaml']['mesh_is_collision_foot_stl'])
        self.assertEqual(d['calibration_worktree_read_only_comparison']['urdf_and_stl_files_differing'], [])

    def test_four_feet_share_one_collision_geometry(self):
        ref = np.unique(np.round(foot_vertices('collision', 'lf')[0], 12), axis=0)
        key = lambda a: a[np.lexsort(a.T[::-1])]
        for leg in common.LEGS:
            v = foot_vertices('collision', leg)[0]
            self.assertLess(np.abs(key(v) - key(ref)).max(), 1e-12)


class RootCauseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rc = load('root_cause.json')

    def test_g4_offset_reproduced(self):
        col, vis = self.rc['meshes']
        mm = col['nominal_radius_translated_circle_minimax']
        self.assertAlmostEqual(mm['offset_um'], 5.775, 3)
        # the collision and the visual STL carry the SAME translation: it is not an export artefact of one file
        mv = vis['nominal_radius_translated_circle_minimax']
        self.assertLess(np.hypot(mm['dx_um'] - mv['dx_um'], mm['dz_um'] - mv['dz_um']), 0.01)
        # vertices lie inside (and on) the translated nominal-radius circle: a rigid translation of the nominal cylinder
        self.assertLess(mm['max_radial_excess_um'], 0.001)
        self.assertGreater(mm['vertices_within_0p01um_of_circle'], 300)

    def test_numeric_error_is_negligible(self):
        for m in self.rc['meshes']:
            self.assertLess(m['numeric_error']['float64_vs_longdouble_um'], 1e-6)
            self.assertLess(m['numeric_error']['float32_stl_quantisation_max_um'], 0.01)

    def test_exact_support_bound(self):
        cyl = g2_cylinder()
        v = foot_vertices('collision', 'lf')[0]
        wxz = np.hypot(v[:, 0] - cyl['center'][0], v[:, 2] - cyl['center'][2])
        bound = (cyl['radius'] - wxz.max()) * UM
        d = delta_um(v, cyl['center'], cyl['radius'], np.arange(-180, 180, 0.01))
        self.assertGreaterEqual(d.min(), bound - 1e-9)
        self.assertAlmostEqual(bound, -5.775, 2)

    def test_registered_reference_contains_every_vertex(self):
        reg = cm.registered()
        v = foot_vertices('collision', 'lf')[0]
        self.assertLessEqual(np.hypot(v[:, 0] - reg.center[0], v[:, 2] - reg.center[2]).max(), reg.radius + 1e-15)
        d = delta_um(v, reg.center, reg.radius, np.arange(-180, 180, 0.01))
        self.assertGreaterEqual(d.min(), -1e-6)  # exact reference: the mesh never goes below it (nm)
        self.assertLess(abs(reg.radius - 0.0149), 1e-9)


class ContactModelTests(unittest.TestCase):
    def test_matches_g2_python(self):
        from matdog_foot_contact import load_foot_contact_model, contact_from_foot_pose
        g2 = load_foot_contact_model(ROOT)
        ref = cm.nominal()
        rng = np.random.default_rng(7)
        for _ in range(50):
            a = rng.uniform(-1.2, 1.2)
            c, s = np.cos(a), np.sin(a)
            R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
            t = rng.uniform(-.1, .1, 3)
            tf = np.eye(4); tf[:3, :3] = R; tf[:3, 3] = t
            point, a_end, b_end = cm.contact(tf, ref)
            g = contact_from_foot_pose(g2, tuple(map(tuple, R)), tuple(t))
            np.testing.assert_allclose(point, g.cross_section_contact_center_world_m, atol=1e-14)
            ends = sorted([tuple(np.round(a_end, 12)), tuple(np.round(b_end, 12))])
            gends = sorted([tuple(np.round(g.support_strip_end_a_world_m, 12)), tuple(np.round(g.support_strip_end_b_world_m, 12))])
            np.testing.assert_allclose(ends, gends, atol=1e-12)


class AlternativesTests(unittest.TestCase):
    def test_registered_reference_removes_penetration_exactly(self):
        alt = load('alternatives.json')['alternatives']
        for name in ('WALK_357', 'TROT_61'):
            self.assertGreater(alt[name]['A0_G2_nominal_strict_mesh_test']['penetration_samples_below_minus_1um'], 0)
            a1 = alt[name]['A1_registered_reference']
            self.assertEqual(a1['penetration_samples_below_minus_1um'], 0)
            self.assertGreaterEqual(a1['mesh_minus_reference_um_all'][0], -1e-3)
            self.assertLess(a1['mesh_minus_reference_um_stance'][1], 10.0)  # inside the declared 10 um contact patch band
            self.assertGreater(alt[name]['A4_circumscribing_cylinder']['mesh_minus_reference_um_stance'][1], 10.0)  # hovers beyond the patch band
        impact = alt['A1_registered_reference_impact']
        self.assertLess(impact['max_abs_delta_q_rad'], 1e-3)
        self.assertLess(impact['relative_qdot_change'], 1e-3)


class LifecycleTests(unittest.TestCase):
    CASES = ('WALK_357', 'WALK_287', 'TROT_61', 'TROT_309')

    def test_results_for_every_case_and_reference(self):
        for case in self.CASES:
            for variant in ('G2_NOMINAL', 'G2_1_REGISTERED_CANDIDATE'):
                d = load(f'lifecycle_{case}_{variant}.json')
                self.assertEqual(d['samples'], 3501)
                st = d['all_statuses']
                self.assertNotIn('FAILED', st.values(), (case, variant, st))
                self.assertNotIn('UNRESOLVED', st.values(), (case, variant))
                v = d['verification']
                self.assertEqual(v['legacy_v1_strict']['verdict'], 'FAIL')  # the legacy rule still rejects: recorded, not hidden
                self.assertEqual(d['terminal']['final_state'], 'STAND')
                self.assertEqual(d['terminal']['final_max_abs_qdot'], 0.0)
                self.assertLess(d['terminal']['max_contact_difference_in_body_frame_m'], 1e-9)
                self.assertLess(d['terminal']['max_joint_difference_rad'], 1e-9)
                self.assertGreater(v['nonfoot_ground_clearance']['certified_lower_bound_m'], 1e-3)
                self.assertGreater(v['self_separation']['certified_lower_bound_m'], 0.005)
                self.assertGreater(v['joint_limit_margin']['certified_lower_bound_rad'], 0.2)
                self.assertEqual(v['ik_branch']['changes_at_samples'], 0)
                if case.startswith('WALK'):
                    self.assertGreater(v['support_margin_walk']['certified_lower_bound_m'], 0.0)
                if variant == 'G2_1_REGISTERED_CANDIDATE':
                    self.assertEqual(v['legacy_v1_strict']['penetration_samples'], 0)  # registration is the whole penetration story
                    self.assertGreater(v['legacy_v1_strict']['undeclared_samples'], 0)  # the band events are a separate, semantic issue
                else:
                    self.assertGreater(v['legacy_v1_strict']['penetration_samples'], 0)
                    self.assertAlmostEqual(v['legacy_v1_strict']['worst_foot_mesh_z_um'], -1.2617, 2)

    def test_convergence_confirms_certified_bounds(self):
        for case in self.CASES:
            coarse = load(f'lifecycle_{case}_G2_NOMINAL.json')['verification']
            fine = load(f'lifecycle_{case}_G2_NOMINAL_fine.json')['verification']
            pairs = [('joint_limit_margin', 'certified_lower_bound_rad', 'sampled_min_rad'), ('nonfoot_ground_clearance', 'certified_lower_bound_m', 'sampled_min_m'),
                     ('self_separation', 'certified_lower_bound_m', 'sampled_min_m')]
            if case.startswith('WALK'):
                pairs.append(('support_margin_walk', 'certified_lower_bound_m', 'sampled_min_m'))
            for q, lo, mn in pairs:
                self.assertLessEqual(coarse[q][lo], fine[q][mn] + 1e-12, (case, q))  # a certified lower bound never exceeds a finer sample
                self.assertGreaterEqual(fine[q][mn], coarse[q][mn] - 1e-9, (case, q))  # and finer sampling does not find a worse minimum

    def test_deterministic_replay(self):
        import lifecycle_v2 as lc
        a = lc.run('WALK_357', 'G2_NOMINAL', 0.05)
        b = lc.run('WALK_357', 'G2_NOMINAL', 0.05)
        for k in ('t', 'nonfoot', 'sep', 'jm', 'support', 'min_delta_tread_um'):
            x = np.array([np.nan if v is None else v for v in a[1][k]]); y = np.array([np.nan if v is None else v for v in b[1][k]])
            np.testing.assert_array_equal(x, y)
        np.testing.assert_array_equal(a[3], b[3])


class PreservationTests(unittest.TestCase):
    def test_g4_accepted_evidence_unchanged(self):
        import subprocess
        names = ['REPORT.md', 'validation_results.json', 'artifact_manifest.json', 'full_cases.json', 'lifecycle_audit.json', 'representatives.json']
        for n in names:
            out = subprocess.run(['git', 'show', f'{G4_CHECKPOINT_FINAL_HEAD}:09_Logs/Validation_Reports/G4_Gait_Envelope/{n}'], cwd=ROOT, capture_output=True)
            self.assertEqual(out.returncode, 0)
            self.assertEqual(hashlib.sha256(out.stdout).hexdigest(), hashlib.sha256((G4 / n).read_bytes()).hexdigest(), n)

    def test_accepted_g2_contact_files_unchanged(self):
        import subprocess
        for rel in ('06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml', '05_Firmware/MATDOG_Controller/src/motion/FootContactData.h',
                    '05_Firmware/MATDOG_Controller/src/motion/Gait.cpp', '05_Firmware/MATDOG_Controller/src/motion/Gait.h', '03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'):
            out = subprocess.run(['git', 'diff', '--quiet', G4_CHECKPOINT_FINAL_HEAD, '--', rel], cwd=ROOT)
            self.assertEqual(out.returncode, 0, rel)


class SafetyBoundaryTests(unittest.TestCase):
    FORBIDDEN = re.compile(r'\b(ServoBus|GoalPosition|Torque_?ON|EEPROM|motorDirection|SerialPort|Serial\.|raw_ticks|flash_app)\b|import serial|open\(.*/dev/tty')

    def test_no_hardware_tokens(self):
        for p in list(Path(__file__).parent.glob('*.py')) + list((ROOT / '05_Firmware/MATDOG_Controller/src/motion').glob('ContactMode.*')):
            if p.name in (Path(__file__).name, 'validate.py'):
                continue
            self.assertIsNone(self.FORBIDDEN.search(p.read_text()), p.name)


if __name__ == '__main__':
    unittest.main(verbosity=2)
