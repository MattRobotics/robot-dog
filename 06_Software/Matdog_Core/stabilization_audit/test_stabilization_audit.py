"""Offline gates for the G5-A evidence: independent oracles, simulations, dynamics, provenance, classification, preservation."""
import json
import re
import subprocess
import unittest
from pathlib import Path
import numpy as np
from core import Lib, OUT, ROOT
from dynamics import Robot, G

G41_HEAD = '3974e9e213555285612cd29e373751d1dd862225'   # accepted G4.1 checkpoint + decision record the G5-A branch starts from


def load(name):
    return json.loads((OUT / name).read_text())


class OracleTests(unittest.TestCase):
    def test_frame_conventions_and_quaternion_tilt(self):
        f = load('frames_oracle.json')
        self.assertTrue(f['all_conventions_hold'])
        self.assertLess(f['quaternion_tilt_max_error_rad'], 1e-12)
        for k in ('positive_roll_left_hip_rises', 'positive_pitch_front_goes_down', 'lf_hip_is_left_and_front', 'rh_hip_is_right_and_rear'):
            self.assertTrue(f['conventions'][k], k)

    def test_tilted_ik_matches_independent_urdf_solver(self):
        k = load('ik_oracle.json')
        self.assertLess(k['worst_independent_contact_residual_m'], 1e-9)
        self.assertLess(k['worst_joint_difference_rad'], 1e-7)
        ok = [c for c in k['checks'] if c['status'] == 'OK']
        self.assertGreaterEqual(len(ok), 14)
        for c in ok:
            self.assertLess(c['cpp_drift_m'], 1e-9)
            self.assertAlmostEqual(c['cpp_margin_rad'], c['independent_margin_rad'], 8)
            self.assertLess(c['max_axis_tilt_deg'], 2.0001)       # nominal-strip policy of G2 preserved
        env = k['single_axis_envelope']
        self.assertAlmostEqual(env['roll_+']['max_tested_ok_deg'], 8.0)
        self.assertEqual(env['roll_+']['cause'], 'CONTACT_MODE')
        self.assertTrue(env['roll_+']['solves_if_edge_biased_contact_allowed'])
        for key in ('pitch_+', 'pitch_-'):
            self.assertFalse(env[key]['any_in_limit_solution_exists_independent'])   # genuine infeasibility, not a solver miss

    def test_live_bridge_agrees_with_saved_oracle(self):
        lib = Lib(); c = lib.compensate(.03, -.02)
        self.assertEqual(c['status'], 'OK')
        self.assertLess(c['drift'], 1e-9)


class SimulationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.r = {x['name'][:3].strip(): x for x in load('perturbation_results.json')['runs']}
        cls.all = load('perturbation_results.json')

    def test_step_disturbances_are_rejected_with_bounded_command(self):
        for key, sign in (('S1', -1), ('S2', 1)):
            run = self.r[key]
            self.assertAlmostEqual(abs(run['final_roll_cmd_deg']) + abs(run['final_pitch_cmd_deg']), 2.0, delta=0.1)
            self.assertLessEqual(run['max_command_rate_deg_s'], run['rate_limit_deg_s'] + 1e-9)
            self.assertLessEqual(run['peak_abs_roll_cmd_deg'], run['correction_limit_deg'] + 1e-9)
            self.assertEqual(run['ik_failures'], 0)
            self.assertLess(run['max_contact_drift_m'], 1e-9)
            self.assertGreater(run['min_joint_margin_rad'], 0)

    def test_failsafe_behaviour(self):
        s6 = self.r['S6']; self.assertGreater(s6['state_counts']['FAULT'], 0); self.assertEqual(s6['final_roll_cmd_deg'], 0.0)   # sensor loss -> latched fault, correction ramped out
        s7 = self.r['S7']; self.assertEqual(s7['samples_rejected'], 20); self.assertEqual(s7['state_counts']['FAULT'], 0)         # isolated corrupt samples never produce a fault or a correction
        s8 = self.r['S8']; self.assertAlmostEqual(abs(s8['final_roll_cmd_deg']), s8['correction_limit_deg'], 6)                     # saturation at the bound
        s9 = self.r['S9']; self.assertAlmostEqual(s9['final_roll_cmd_deg'], self.r['S1']['final_roll_cmd_deg'], 6)                  # heading drift is irrelevant
        self.assertTrue(self.all['checks']['gain_105_percent_of_bound_refused'])
        self.assertTrue(self.all['checks']['enable_refused_while_latched'])

    def test_unknown_mounting_offset_is_visible(self):
        s10, s11 = self.r['S10'], self.r['S11']
        self.assertAlmostEqual(s10['final_roll_cmd_deg'], -0.5, delta=0.05)    # uncalibrated: the controller would tilt the real body by the offset
        self.assertAlmostEqual(s11['final_roll_cmd_deg'], 0.0, delta=0.05)

    def test_deterministic_replay(self):
        import perturbation_sim as ps
        lib = Lib()
        a = ps.run(lib, 'x', lambda t: (0.02 if t > .5 else 0, 0)); b = ps.run(lib, 'x', lambda t: (0.02 if t > .5 else 0, 0))
        self.assertEqual(json.dumps(a, sort_keys=True), json.dumps(b, sort_keys=True))


class DynamicsTests(unittest.TestCase):
    def test_rnea_against_energy_methods(self):
        r = Robot(); rng = np.random.default_rng(3); worst_pow = 0.0
        for leg in ('lf', 'rh'):
            for _ in range(10):
                q = rng.uniform([-.3, -.5, -1.2], [.3, 1.8, .5]); qd = rng.uniform(-2, 2, 3); qdd = rng.uniform(-30, 30, 3)
                g = r.rnea(leg, q, np.zeros(3), np.zeros(3), [0, 0, G])
                num = np.array([(r.potential(leg, q + 1e-6 * np.eye(3)[j]) - r.potential(leg, q - 1e-6 * np.eye(3)[j])) / 2e-6 for j in range(3)])
                self.assertLess(np.abs(g - num).max(), 1e-8)
                M = np.column_stack([r.rnea(leg, q, np.zeros(3), np.eye(3)[j], [0, 0, 0]) for j in range(3)])
                self.assertLess(np.abs(M - M.T).max(), 1e-12)
                self.assertLess(abs(r.kinetic(leg, q, qd) - .5 * qd @ M @ qd), 1e-9)
                # energy balance with velocity-dependent terms: tau.qd = d/dt (T + V) along q(t) = q + qd t + qdd t^2 / 2
                tau = r.rnea(leg, q, qd, qdd, [0, 0, G])
                h = 1e-5
                E = lambda t: r.kinetic(leg, q + qd * t + .5 * qdd * t * t, qd + qdd * t) + r.potential(leg, q + qd * t + .5 * qdd * t * t)
                worst_pow = max(worst_pow, abs(tau @ qd - (E(h) - E(-h)) / (2 * h)))
        self.assertLess(worst_pow, 5e-5)

    def test_mass_properties(self):
        self.assertAlmostEqual(load('stability_assessment.json')['mass_properties']['total_mass_kg'], 2.48, 9)


class AnalysisTests(unittest.TestCase):
    def test_provenance(self):
        p = load('provenance.json')
        self.assertTrue(p['verdict']['geometry_consistent']); self.assertEqual(len(p['full_calibration']['contacts']), 24)
        self.assertFalse(p['calibration_persistence']['merged_into_main'])
        self.assertFalse(p['hardware_authorization']['stand_or_gait_authorized'])

    def test_stability_levels_are_distinct(self):
        s = load('stability_assessment.json')
        w = s['walk_cycles']['WALK_357']
        self.assertGreater(w[0]['min_static_margin_mm'], 0)            # quasi-static support positive ...
        self.assertLess(w[0]['min_zmp_margin_mm'], 0)                  # ... while the same cycle at T = 1 s is NOT supported dynamically (ZMP approximation)
        self.assertGreater(w[2]['min_zmp_margin_mm'], 0)               # slow enough, it is
        self.assertLess(w[0]['min_static_margin_mm'], 3)               # the quasi-static margin is a few millimetres

    def test_feasibility_classification(self):
        f = load('actuator_feasibility.json')
        verdicts = {v for r in f['rows'] for l in r['limits'] for v in l['verdicts'].values()}
        self.assertNotIn('VERIFIED', verdicts)                         # nothing is measured under load
        for r in f['rows']:
            for l in r['limits']:
                if l['provenance'] == 'UNMEASURED':
                    self.assertEqual(set(l['verdicts'].values()), {'REQUIRES_MEASUREMENT'})
        w = next(b for b in f['urdf_velocity_budget_check'] if b['case'] == 'WALK_357' and b['period_s'] == 1.0)
        self.assertFalse(w['within_budget'])                           # 3.16 rad/s > the URDF 3.037 rad/s budget
        self.assertGreater(f['minimum_periods']['WALK_357']['period_s_where_accel_equals_bench_profile'], 3.0)

    def test_joint_range(self):
        j = load('joint_range.json')
        self.assertTrue(j['cases']['STAND_RISE_0.100_to_0.150']['all_inside_measured_contacts'])
        self.assertFalse(j['cases']['STAND_TILT_COMPENSATED_+-3deg']['all_inside_measured_contacts'])
        a = j['tilt_authority_deg_by_body_height']['0.15']
        self.assertLess(a['pitch-']['measured'], a['pitch-']['urdf'])   # the real mechanical stops are tighter than the URDF


class PreservationAndSafetyTests(unittest.TestCase):
    def test_accepted_work_untouched(self):
        for path in ('06_Software/Matdog_Core/kinematics', '03_CAD', '09_Logs/Validation_Reports/G4_Gait_Envelope', '09_Logs/Validation_Reports/G41_Contact_Reconciliation',
                     '09_Logs/Validation_Reports/G35_Pose_Audit', '06_Software/Matdog_Core/gait_audit', '06_Software/Matdog_Core/contact_audit', '06_Software/Matdog_Core/pose_audit'):
            self.assertEqual(subprocess.run(['git', 'diff', '--quiet', G41_HEAD, '--', path], cwd=ROOT).returncode, 0, path)
        for f in ('Gait', 'Locomotion', 'ContactMode', 'FootContact', 'BodyPose', 'StandTrajectory', 'LegKinematics', 'StartupAcquisition', 'TimedStand', 'StandTransition', 'MotionState'):
            for ext in ('.h', '.cpp'):
                rel = f'05_Firmware/MATDOG_Controller/src/motion/{f}{ext}'
                if (ROOT / rel).exists():
                    self.assertEqual(subprocess.run(['git', 'diff', '--quiet', G41_HEAD, '--', rel], cwd=ROOT).returncode, 0, rel)

    def test_no_hardware_tokens(self):
        bad = re.compile(r'\b(ServoBus|GoalPosition|Torque_?ON|EEPROM|motorDirection|SerialPort|Serial\.|raw_ticks|flash_app|Adafruit|SPI\.h|Arduino\.h)\b|import serial|open\(.*/dev/tty')
        files = list(Path(__file__).parent.glob('*.py')) + list(Path(__file__).parent.glob('*.cpp'))
        files += [ROOT / f'05_Firmware/MATDOG_Controller/src/motion/{n}' for n in ('ImuAttitude.h', 'ImuAttitude.cpp', 'BodyStabilizer.h', 'BodyStabilizer.cpp', 'TiltedContactIk.h', 'TiltedContactIk.cpp', 'TiltCompensation.h', 'TiltCompensation.cpp', 'ActuatorEnvelope.h', 'ActuatorEnvelope.cpp')]
        for p in files:
            if p.name in (Path(__file__).name, 'validate.py', 'build_report.py', 'provenance.py'):
                continue
            self.assertIsNone(bad.search(p.read_text()), p.name)


if __name__ == '__main__':
    unittest.main(verbosity=2)
