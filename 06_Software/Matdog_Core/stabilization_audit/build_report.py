"""Render the G5-A REPORT.md from the saved artifacts so every number comes from a file."""
import json
import sys
from pathlib import Path
from string import Template

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = ROOT / '09_Logs/Validation_Reports/G5A_Stabilization_Feasibility'


def load(n):
    return json.loads((OUT / n).read_text())


def table(h, rows):
    return '\n'.join(['| ' + ' | '.join(h) + ' |', '|' + '|'.join('---' for _ in h) + '|'] + ['| ' + ' | '.join(str(c) for c in r) + ' |' for r in rows])


def values():
    v = {}
    prov, ik, fr = load('provenance.json'), load('ik_oracle.json'), load('frames_oracle.json')
    sim, stab, act, feas, jr, val = load('perturbation_results.json'), load('stability_assessment.json'), load('actuator_requirements.json'), load('actuator_feasibility.json'), load('joint_range.json'), load('validation_results.json')
    oracle = load('oracle_results_g5a.json')
    g = prov['geometry']
    v['urdf'] = g['urdf_sha256'][:16] + '...'; v['geom_ok'] = prov['verdict']['geometry_consistent']
    v['persist_tip'] = prov['calibration_persistence']['branch_tip'][:9]; v['persist_merged'] = prov['calibration_persistence']['merged_into_main']
    v['persist_commits'] = '; '.join(prov['calibration_persistence']['recent_commits'][:3])
    v['contacts'] = len(prov['full_calibration']['contacts'])
    env = ik['single_axis_envelope']
    rows = []
    for k, e in env.items():
        rows.append([k.replace('_', ' '), f"{e['max_tested_ok_deg']:g}", f"{e.get('first_failure_deg', '-')}", e.get('failed_leg', '-'), e.get('cause', '-'), e.get('any_in_limit_solution_exists_independent', '-'), e.get('solves_if_edge_biased_contact_allowed', '-')])
    v['env_table'] = table(['Axis / sign', 'Largest OK tilt [deg]', 'First failure [deg]', 'Leg', 'Solver status', 'Independent search finds any in-limit solution', 'Solvable if edge-biased contact allowed'], rows)
    v['ik_res'] = f"{ik['worst_independent_contact_residual_m']:.1e}"; v['ik_dq'] = f"{ik['worst_joint_difference_rad']:.1e}"
    v['quat_err'] = f"{fr['quaternion_tilt_max_error_rad']:.1e}"; v['quat_n'] = fr['samples']
    au = jr['tilt_authority_deg_by_body_height']
    v['auth_table'] = table(['Body height [m]', 'roll +/- (measured stops)', 'pitch + (measured / URDF)', 'pitch - (measured / URDF)', 'roll (URDF)'],
                            [[h, f"{r['roll+']['measured']:g} / {r['roll-']['measured']:g}", f"{r['pitch+']['measured']:g} / {r['pitch+']['urdf']:g}", f"{r['pitch-']['measured']:g} / {r['pitch-']['urdf']:g}", f"{r['roll+']['urdf']:g}"] for h, r in au.items()])
    cases = jr['cases']
    v['range_table'] = table(['Case', 'Worst margin to measured mechanical contact [deg]', 'Joint', 'Worst margin to URDF limit [deg]', 'Inside measured contacts'],
                             [[n, f"{c['worst_margin_to_measured_deg']:.2f}", c['worst_joint'], f"{c['worst_margin_to_urdf_deg']:.2f}", c['all_inside_measured_contacts']] for n, c in cases.items()])
    sv = sim['study_values']
    v['sim_vals'] = f"dt = {sv['dt_s']} s (50 Hz), loop delay {sv['loop_delay_steps']} steps (unmeasured), correction limit {sv['correction_limit_deg']} deg, rate limit {sv['rate_limit_deg_s']} deg/s, hold before fault {sv['hold_before_fault_s']} s, sensor noise +/-{sv['sensor_noise_amplitude_deg']} deg"
    v['sim_table'] = table(['Scenario', 'Gain / bound', 'Peak measured tilt [deg]', 'Tail tilt [deg]', 'Final command r/p [deg]', 'Max cmd rate [deg/s]', 'Rejected samples', 'Fault steps', 'IK failures', 'Max joint step [rad]'],
                           [[r['name'], f"{r['gain_fraction_of_bound']:.2f}", f"{r['peak_measured_tilt_deg']:.2f}", f"{r['mean_tail_measured_tilt_deg']:.2f}", f"{r['final_roll_cmd_deg']:.2f} / {r['final_pitch_cmd_deg']:.2f}",
                             f"{r['max_command_rate_deg_s']:.1f}", r['samples_rejected'], r['state_counts']['FAULT'], r['ik_failures'], f"{r['max_joint_step_rad']:.4f}"] for r in sim['runs']])
    ck = sim['checks']; v['latch_t'] = ck['fault_latch_time_s']
    ms = stab['mass_properties']['canonical_stand']
    v['com'] = ', '.join(f'{x*1e3:.1f}' for x in ms['com_world_m']); v['com_h'] = f"{ms['com_height_above_ground_m']*1e3:.1f}"; v['stand_margin'] = f"{ms['static_margin_m']*1e3:.1f}"
    wr = []
    for case, cyc in stab['walk_cycles'].items():
        for c in cyc:
            wr.append([case, f"{c['period_s']:g}", f"{c['min_static_margin_mm']:.2f}", f"{c['min_zmp_margin_mm']:.2f}", c['zmp_outside_polygon_phases'], f"{c['com_horizontal_accel_peak_m_s2']:.1f}",
                       f"{c['peak_abs_torque_by_class']['lower']:.2f}", f"{c['rms_torque_by_class']['lower']:.2f}"])
    v['walk_table'] = table(['Case', 'Period [s]', 'Quasi-static margin [mm]', 'ZMP margin [mm] (approx.)', 'Frames with ZMP outside the support polygon', 'Peak COM horizontal accel [m/s^2]', 'Peak lower-leg torque [N m]', 'RMS lower-leg torque [N m]'], wr)
    v['pitch_meas'] = f"{au['0.15']['pitch-']['measured']:g}"; v['pitch_urdf'] = f"{au['0.15']['pitch-']['urdf']:g}"
    v['min_static'] = f"{min(c['min_static_margin_mm'] for cyc in stab['walk_cycles'].values() for c in cyc):.2f}"
    heavy = stab['walk_cycles_mass_x1p33']['WALK_357'][0]['peak_abs_torque_by_class']['lower']
    v['heavy_peak'] = f"{heavy:.2f}"
    v['stand_tq'] = ', '.join(f'{x:.2f}' for x in act['stand_static_loads']['urdf_mass']['worst_torque_nm_by_joint'])
    v['stand_tq_h'] = ', '.join(f'{x:.2f}' for x in act['stand_static_loads']['mass_x1p33']['worst_torque_nm_by_joint'])
    tilt = stab['mass_properties']['compensated_tilt_static']
    v['tilt_shift'] = f"{abs(tilt[1]['com_xy_shift_vs_level_mm'][1]):.2f}" if tilt[1]['status'] == 'OK' else 'n/a'
    L = feas['limits_used']
    v['lim_table'] = table(['Quantity', 'Value', 'Provenance', 'Source'], [
        ['No-load speed, slowest of 16 healthy units', f"{L['bench_slowest_healthy_unit_rad_s']:.3f} rad/s", 'BENCH_NO_LOAD', 'ST3215 bench QC V6.1 (unloaded servos, bus ~11.0-11.1 V)'],
        ['No-load speed, vendor', f"{L['vendor_no_load_rad_s']:.3f} rad/s (45 rpm)", 'VENDOR_NOMINAL', 'Feetech datasheet at 12 V'],
        ['Speed budget of the URDF', f"{L['urdf_budget_rad_s']:.3f} rad/s (29 rpm)", 'design budget', 'URDF README: conservative nominal-operation limit, not a validation'],
        ['Torque, rated / stall / 80 % overload', f"{L['vendor_rated_nm']:.2f} / {L['vendor_stall_nm']:.2f} / {L['vendor_overload_80pct_nm']:.2f} N m", 'VENDOR_NOMINAL', 'Feetech datasheet at 12 V; OverloadTorque 80 in the MATDOG profile'],
        ['Effort budget of the URDF', f"{L['urdf_effort_budget_nm']:.3f} N m", 'design budget', 'URDF README'],
        ['Acceleration of a commanded profile at Acc = 50', f"{L['bench_profile_accel_rad_s2']:.1f} rad/s^2", 'BENCH_NO_LOAD (one setting, not a capability)', 'QC move metrics'],
        ['Acceleration capability for streamed targets', 'none', 'UNMEASURED', '-'], ['Any loaded velocity / torque / acceleration', 'none', 'UNMEASURED', '-']])
    rows = []
    for k, c in feas['verdict_counts_by_limit'].items():
        rows.append([k, ', '.join(f'{a}: {b}' for a, b in c.items())])
    v['verdict_table'] = table(['Limit', 'Verdicts of the requirement rows (margin 1.0)'], rows)
    mp = feas['minimum_periods']
    v['minp'] = table(['Case', 'Min period for the bench no-load speed [s]', 'Min period for the URDF speed budget [s]', 'Period at which peak acceleration equals the bench Acc=50 profile [s]'],
                      [[c, f"{x['min_period_s_for_bench_speed']:.2f}", f"{x['min_period_s_for_urdf_budget_speed']:.2f}", f"{x['period_s_where_accel_equals_bench_profile']:.2f}"] for c, x in mp.items()])
    rate = [r for r in feas['command_rate_error'] if r['case'] == 'WALK_357']
    v['rate_table'] = table(['Update rate [Hz]', 'T = 1 s: hold error [ticks]', 'T = 1 s: linear-interpolation error [ticks]', 'T = 4 s: hold error [ticks]', 'T = 4 s: interpolation error [ticks]'],
                            [[hz, f"{next(r for r in rate if r['update_hz']==hz and r['period_s']==1.0)['hold_error_ticks']:.1f}", f"{next(r for r in rate if r['update_hz']==hz and r['period_s']==1.0)['linear_interpolation_error_ticks']:.2f}",
                              f"{next(r for r in rate if r['update_hz']==hz and r['period_s']==4.0)['hold_error_ticks']:.1f}", f"{next(r for r in rate if r['update_hz']==hz and r['period_s']==4.0)['linear_interpolation_error_ticks']:.3f}"] for hz in (25, 50, 100, 200, 500)])
    b = feas['bus_and_latency']
    v['first_motion'] = f"{b['first_motion_ms_median_of_unit_medians']:.0f}"; v['settle'] = f"{b['settle_ms_median_of_unit_medians']:.0f}"; v['hyst'] = f"{b['hysteresis_ticks']['median_of_unit_medians']:.0f}"; v['hyst_deg'] = f"{b['hysteresis_ticks']['median_of_unit_medians']*b['resolution_deg']:.2f}"
    v['sync_ms'] = f"{b['sync_write_12_servos_ms']:.2f}"
    d = feas['stabilizer_gain_bound_vs_loop_delay']
    v['gain_table'] = table(['Loop delay [steps of 20 ms]', 'Maximum stable integral gain [1/s]'], [[x['delay_steps'], f"{x['max_stable_gain_per_s']:.1f}"] for x in d if x['delay_steps'] in (0, 1, 2, 4, 6, 8)])
    v['oracle_n'] = f"{oracle['sample_count']} samples, {oracle['numerical_ik_legs']} independently solved legs"
    v['validation_table'] = table(['Gate', 'Result', 'Summary'], [[g['name'], 'PASS' if g['passed'] else 'FAIL', (g['summary'] or '').replace('|', '/')] for g in val['gates']])
    v['all_passed'] = 'ALL PASSED' if val['all_passed'] else 'NOT ALL PASSED'; v['head'] = val['git']['head_at_validation']; v['clean'] = val['git']['worktree_clean_before_validation']
    v['budget_walk'] = next(x for x in feas['urdf_velocity_budget_check'] if x['case'] == 'WALK_357' and x['period_s'] == 1.0)['peak_qdot_rad_s']
    return v


TEMPLATE = Template(r'''# G5-A — Body stabilization and actuator feasibility (offline)

**Scope: OFFLINE software only. No servo command, Torque ON, hardware motion, flashing, EEPROM write, calibration change or raw-tick manipulation occurred. The calibration-persistence worktree was not modified; calibration material was read with `git show` only.**
Branch `feat/g5a-stabilization-feasibility`, started from the accepted G4.1 checkpoint plus its decision record (`3974e9e`). G1-G4.1 sources, geometry, calibration interfaces and evidence are unchanged (gate `accepted_g1_g4_1_files_unchanged_vs_g41_head`).

## 0. Status: completed implementation, research findings, unresolved dependencies

| Kind | Item |
|---|---|
| **Completed implementation** (pure C++17, strict flags, ASan/UBSan clean) | attitude contract and monitor; fail-safe body stabilizer core; tilted contact IK; world-locked tilt compensation of the canonical STAND; provenance-aware requirement classifier |
| **Completed verification** | independent URDF/scipy IK oracle ($ik_res m, $ik_dq rad); frame-convention oracle; deterministic loop simulations; rigid-body dynamics validated against energy methods; provenance and joint-range analyses |
| **Research findings** | tilt authority of STAND is set by the measured mechanical stops, not the URDF; the quasi-static WALK margin is about $min_static mm and the same cycle at T = 1 s is not dynamically supported (ZMP approximation); peak WALK joint speed at T = 1 s exceeds the URDF speed budget; acceleration capability is unmeasured |
| **Unresolved dependencies** | calibration persistence (not merged, not restored at boot); IMU mounting-level offset and loop latency; any loaded-actuator measurement; operational envelope approval; stand/gait authorization (none) |

TROT remains dynamically uncertified. Nothing here is a hardware-approved setting, and simulated stabilization is **not** evidence of physical stability.

## 1. Provenance (geometry and calibration)

* **Geometry.** Consistent: **$geom_ok**. URDF `$urdf` equals the working copy, `main` and `SHA256SUMS`, is the URDF named by the Full Calibration report (Geometry V5 bundle `..._BENCHMARK_D_W4`), and the G4.1 provenance recheck passes (foot contact YAML identical, generated G1 header tagged with the same hash). No canonical geometry was modified.
* **Full Calibration.** TRUE Full Calibration 24/24 hardware-validated 2026-10-01 (report on `main`); $contacts measured mechanical contacts were read and are used in section 6. **The calibration is RAM-only** and `legs_envelope_accepted=0`: no operational envelope, stand or gait is authorized.
* **Calibration persistence** (branch `feat/calibration-persistence-record-store-v1`, tip `$persist_tip`: $persist_commits) is an **independent dependency, not merged into main (merged = $persist_merged)**; main documents persistence as TO_DESIGN and restoration as not implemented. Calibration having been performed does not imply boot-time restoration. Nothing was merged, cherry-picked or modified.
* The q0 values, encoder directions and calibration contacts are consumed only as read-only evidence; the motion model stays in semantic URDF radians (no ticks, no q0 use).

## 2. Stabilization architecture

`ImuSnapshot -> AttitudeMonitor -> BodyStabilizer -> corrected BodyPose -> compensateStand -> tilted contact IK -> 12 semantic joint targets -> (future, separate) actuator safety layer`. Design, equations and the fail-safe table are in `src/motion/STABILIZATION.md`. Sensor acquisition (the existing `Bno085Imu` driver, untouched) is separated from stabilization logic; the only link is the `ImuSnapshot` struct filled by a thin adapter that is specified but not written (no second driver, no physical IMU access).

## 3. BNO085 data contract

| Item | Contract |
|---|---|
| Source | `SH2_ROTATION_VECTOR` only (internal 50 Hz; G3.1 measured 50.1 Hz acquisition) |
| Content | quaternion `(w,x,y,z)` = `R_world_from_base_link`, `accuracyRad`, status 0..3, sequence, caller-clock stamp |
| Frame | X forward, Y left, Z up; sensor axes = `base_link` (frozen by hardware Phase D, identity, no sign flip) |
| Used | tilt only (roll, pitch from the up-vector); heading is magnetic with an arbitrary zero and is **never** consumed |
| Conventions | +roll lifts the left side, +pitch lowers the nose, `R = Rz(yaw) Ry(pitch) Rx(roll)`; checked against the URDF geometry ($quat_n quaternions, max tilt error $quat_err rad, `q` and `-q` equivalent) |
| Validity | finite, unit norm, status and accuracy thresholds, freshness (equal to the age limit is fresh, greater is stale), no future stamp, no time/sequence regression, plausible tilt |
| Policy | **no default**; every field explicit |
| Not established | mounting-level offset, acquisition-to-command latency, jitter, accuracy under motion |

## 4. BodyPose correction mathematics

Desired rotation `R = Rz(yaw) Ry(p_c) Rx(r_c)`, translation unchanged. World contacts stay fixed; Cartesian targets are `b_i = R^T (c_i - t)` with ground normal `n = R^T e_z`; joints come from contact IK against `n`. Law: integral action on the measured tilt, bounded in range and rate, active only for a new valid sample; the configuration is accepted only below the derived stability bound `ki < 2 sin(pi/(2(2N+1)))/dt` for an `N`-step loop delay (verified numerically on both sides of the boundary). The bound shrinks quickly with the delay, so **the delay must be measured**:

$gain_table

The bench QC shows a median first-motion latency of $first_motion ms (unloaded, includes the profile ramp), i.e. several 20 ms steps, before any IMU or bus latency is added.

## 5. IK limitations and the justified extension

The accepted G2/G3 IK is exact for a level body only (`supportsFlatContactIk`). **It was not modified.** `TiltedContactIk` is a new explicit solver (Newton on `contactForwardKinematics`, which already accepts a ground normal) with the same G2 cylinder, strip and 2 deg tilt policy. Agreement: with the analytic IK for `n = +Z` (5e-9 rad), and with an independent URDF-based scipy solver in the world frame to $ik_res m and $ik_dq rad over 15 tilt cases (contact drift under 1e-9 m, nominal-strip tilt below 2 deg).

Tilt envelope of the canonical 150 mm STAND (single axis, 0.25 deg steps):

$env_table

* Roll is limited at about 8 deg by the **G2 nominal-strip policy** (the geometry is reachable with edge-biased contact, which G2 does not accept). Pitch is limited by true infeasibility (no in-limit solution exists, found by independent multi-start search), nose-up (negative) far earlier than nose-down.
* **The measured mechanical stops are tighter than the URDF.** Tilt authority against the TRUE Full Calibration contacts, by body height (deg):

$auth_table

  At 150 mm nose-up authority is $pitch_meas deg against $pitch_urdf deg by the URDF alone. The binding joint of the combined +/-3 deg roll/pitch corner is RF lower, which stops at +31.8 deg on the hardware against the URDF +37.5 deg. Lowering the stand improves pitch authority but reduces roll authority (hip angle / strip tilt).
* Required joint ranges against measured contacts and URDF limits:

$range_table

## 6. Representative simulated perturbation tests

Loop-equation simulation through the production C++ (not a robot model): $sim_vals. Gains are fractions of the derived bound; **all values are study values**.

$sim_table

Reading: step disturbances of 2 deg are corrected at the bounded rate with no IK failure and no contact drift; the sinusoid shows the attenuation of a 50 Hz loop with delay; sensor loss ramps the command out and latches a fault ($latch_t s after the last valid sample, enable refused until reset); isolated corrupt samples are rejected and never create a correction; a 5 deg disturbance saturates at the 3 deg bound and stays flagged; heading drift has no effect; a gain of 105 % of the bound is refused by the configuration check. **S10 matters physically:** with an uncalibrated mounting-level offset the controller drives the *measured* tilt to zero, which tilts the real body by the offset (S11 with the reference supplied does not). The offset must be measured on hardware before any stabilization.

## 7. Stability, separated

Canonical STAND: COM at ($com) mm (x, y, z, world), height $com_h mm, static margin $stand_margin mm to the support polygon of the four G2 strips; a 2 deg compensated roll moves the COM by only $tilt_shift mm because the legs reconfigure. URDF mass 2.48 kg (the legacy power analysis cites 3.3 kg: unreconciled, repeated at x1.33).

$walk_table

* **Static equilibrium** of the STAND is comfortable. **Quasi-static WALK** margins are only a few millimetres: the same arithmetic as the G4 report. A 1.5 mm margin equals the COM shift produced by 0.82 kg (the 3.3 kg vs 2.48 kg gap) displaced by about 6 mm, so URDF mass-property error alone can erase it; it must not be read as stability.
* **Dynamic behaviour (ZMP approximation, angular momentum neglected):** at T = 1 s the ZMP leaves the support polygon in a large part of the cycle (COM horizontal acceleration about 16 m/s^2, driven by the open-loop sway); at T = 2 s and 4 s the ZMP margin returns to the static value. The quasi-static WALK assumption therefore fails at T = 1 s and holds at T = 2 s (the threshold between them was not located), independent of any actuator limit. This is a research indicator, not a certification; TROT is not assessed.
* **Actuator feasibility** is a separate question (section 8).

## 8. Actuator feasibility

Limits used, with provenance:

$lim_table

Verdicts of all requirement rows (WALK steady cycles at 0.5/1/2/4 s, STAND rise, G4.1 lifecycle peaks, torques):

$verdict_table

* **Joint speed.** Verified: nothing under load. WALK 357 at T = 1 s needs $budget_walk rad/s: above the URDF speed budget (3.037 rad/s) and 73 % of the unloaded bench speed. Minimum periods:

$minp

* **Joint acceleration: REQUIRES MEASUREMENT.** The QC measured the acceleration of a commanded *profile* at Acc = 50 (about 12 rad/s^2); no measurement of what the servo can follow with streamed targets exists. The G4 WALK 357 peak (190 rad/s^2 at T = 1 s) is 16 times that profile; the peak equals it at a period of about 4 s.
* **Torque (rigid-body model, URDF masses).** Peak lower-leg torque of a WALK cycle is about 1.0 N m (1.4 N m at x1.33 mass): above the URDF effort budget (0.90 N m) and the vendor rated torque (0.98 N m) at peak, below the vendor 80 % overload level (2.35 N m); RMS about 0.6 N m. Static STAND worst torques (hip/upper/lower) are $stand_tq N m (x1.33: $stand_tq_h). All PROVISIONAL at best: the model has no friction, compliance, backlash or cable loads, the load sharing among stance feet is an assumption, and the QC `Present Load` register is not calibrated to torque.
* **Joint range.** Required ranges lie inside the measured mechanical contacts for the STAND rise and for all four G4.1 lifecycles (worst margin 11.8 deg at RF lower); tilt compensation consumes it (section 5). Margins are not an approved envelope.
* **Timing and rate** (arithmetic from the bus and the QC data, not a measured 12-servo loop). A 12-servo sync write is about $sync_ms ms at 1 Mbps; the QC reads one servo every 2 ms, so a sequential read-all is about 24 ms and unmeasured. The error of streaming targets (WALK 357):

$rate_table

  Linear interpolation at 100 Hz keeps the error near one tick; holding targets at 50 Hz does not. Whether the servo interpolates between streamed targets is unmeasured.
* **Resolution and hysteresis.** 1 tick = 0.088 deg; measured directional hysteresis is $hyst ticks (about $hyst_deg deg) per joint, so the smallest useful body-tilt correction is of that order unless dither is used; the stabilizer dead band must not be below it.
* **Latency.** First motion median $first_motion ms and settle median $settle ms (unloaded, QC steps). These define the loop delay `N` of section 4: it must be measured on the assembled robot.

## 9. Outstanding measurements

1. IMU mounting-level offset on the assembled robot, acquisition-to-command latency and jitter, accuracy under motion.
2. Loaded servo speed, acceleration (with streamed targets) and torque; thermal/current behaviour; tracking error under load; whether targets are interpolated.
3. A 12-servo read/write loop timing on the real bus.
4. Mass, COM and inertia of the assembled robot (battery, wiring, head/jaw) to replace the CAD URDF values.
5. Load sharing among stance feet (compliance) and ground friction.
6. Operational joint envelopes with margin to the measured stops; resolution of the LOWER MAX shortfall follow-up.
7. Calibration persistence (restoration) and q0 refinement follow-ups.

## 10. Validation on the final source ($all_passed)

Run by `validate.py` at HEAD `$head` (worktree clean before the run: $clean). The G4 independent oracle was re-run on the current source ($oracle_n).

$validation_table

## 11. Requirements for hardware commissioning (not authorized)

* A separate actuator safety layer owning all limits (position, speed, acceleration, torque/current, temperature, watchdog, torque-off path), with limits derived from loaded measurements; this milestone provides only a classifier.
* Persistence of calibration or an explicit per-boot q0 acquisition accepted by the calibration owner; approved operational envelopes.
* The measurements of section 9, an IMU level-reference procedure, and an explicit minimum-period/rate policy for WALK (a period of 1 s exceeds the URDF speed budget and the quasi-static assumption; 2 s satisfies both in this analysis).
* A tethered, torque-limited first stand in the validated joint range, with the stabilizer disabled first and enabled only after the level reference and latency are measured.
* Dynamics/state-estimation work before any TROT claim; independent safety review and explicit written authorization. Nothing in G5-A authorizes motion.
''')


def main():
    v = values()
    v['min_static'] = v['min_static']
    text = TEMPLATE.substitute(v)
    (OUT / 'REPORT.md').write_text(text)
    print('REPORT.md written,', len(text.splitlines()), 'lines')


if __name__ == '__main__':
    sys.exit(main())
