# G5-A — Body stabilization and actuator feasibility (offline)

**Scope: OFFLINE software only. No servo command, Torque ON, hardware motion, flashing, EEPROM write, calibration change or raw-tick manipulation occurred. The calibration-persistence worktree was not modified; calibration material was read with `git show` only.**
Branch `feat/g5a-stabilization-feasibility`, started from the accepted G4.1 checkpoint plus its decision record (`3974e9e`). G1-G4.1 sources, geometry, calibration interfaces and evidence are unchanged (gate `accepted_g1_g4_1_files_unchanged_vs_g41_head`).

## 0. Status: completed implementation, research findings, unresolved dependencies

| Kind | Item |
|---|---|
| **Completed implementation** (pure C++17, strict flags, ASan/UBSan clean) | attitude contract and monitor; fail-safe body stabilizer core; tilted contact IK; world-locked tilt compensation of the canonical STAND; provenance-aware requirement classifier |
| **Completed verification** | independent URDF/scipy IK oracle (5.3e-14 m, 2.0e-11 rad); frame-convention oracle; deterministic loop simulations; rigid-body dynamics validated against energy methods; provenance and joint-range analyses |
| **Research findings** | tilt authority of STAND is set by the measured mechanical stops, not the URDF; the quasi-static WALK margin is about 1.55 mm and the same cycle at T = 1 s is not dynamically supported (ZMP approximation); peak WALK joint speed at T = 1 s exceeds the URDF speed budget; acceleration capability is unmeasured |
| **Unresolved dependencies** | calibration persistence (not merged, not restored at boot); IMU mounting-level offset and loop latency; any loaded-actuator measurement; operational envelope approval; stand/gait authorization (none) |

TROT remains dynamically uncertified. Nothing here is a hardware-approved setting, and simulated stabilization is **not** evidence of physical stability.

## 1. Provenance (geometry and calibration)

* **Geometry.** Consistent: **True**. URDF `3890a3f0732dbed8...` equals the working copy, `main` and `SHA256SUMS`, is the URDF named by the Full Calibration report (Geometry V5 bundle `..._BENCHMARK_D_W4`), and the G4.1 provenance recheck passes (foot contact YAML identical, generated G1 header tagged with the same hash). No canonical geometry was modified.
* **Full Calibration.** TRUE Full Calibration 24/24 hardware-validated 2026-10-01 (report on `main`); 24 measured mechanical contacts were read and are used in section 6. **The calibration is RAM-only** and `legs_envelope_accepted=0`: no operational envelope, stand or gait is authorized.
* **Calibration persistence** (branch `feat/calibration-persistence-record-store-v1`, tip `b19a98e0d`: b19a98e M0.4: qualify pinned ROBOT_POWERED migration and writer; 2757604 docs(m0): align assembled hardware and block unproven recovery; 174aa04 fix(m0): disable migration write retries and clarify USB isolation) is an **independent dependency, not merged into main (merged = False)**; main documents persistence as TO_DESIGN and restoration as not implemented. Calibration having been performed does not imply boot-time restoration. Nothing was merged, cherry-picked or modified.
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
| Conventions | +roll lifts the left side, +pitch lowers the nose, `R = Rz(yaw) Ry(pitch) Rx(roll)`; checked against the URDF geometry (4000 quaternions, max tilt error 2.8e-16 rad, `q` and `-q` equivalent) |
| Validity | finite, unit norm, status and accuracy thresholds, freshness (equal to the age limit is fresh, greater is stale), no future stamp, no time/sequence regression, plausible tilt |
| Policy | **no default**; every field explicit |
| Not established | mounting-level offset, acquisition-to-command latency, jitter, accuracy under motion |

## 4. BodyPose correction mathematics

Desired rotation `R = Rz(yaw) Ry(p_c) Rx(r_c)`, translation unchanged. World contacts stay fixed; Cartesian targets are `b_i = R^T (c_i - t)` with ground normal `n = R^T e_z`; joints come from contact IK against `n`. Law: integral action on the measured tilt, bounded in range and rate, active only for a new valid sample; the configuration is accepted only below the derived stability bound `ki < 2 sin(pi/(2(2N+1)))/dt` for an `N`-step loop delay (verified numerically on both sides of the boundary). The bound shrinks quickly with the delay, so **the delay must be measured**:

| Loop delay [steps of 20 ms] | Maximum stable integral gain [1/s] |
|---|---|
| 0 | 100.0 |
| 1 | 50.0 |
| 2 | 30.9 |
| 4 | 17.4 |
| 6 | 12.1 |
| 8 | 9.2 |

The bench QC shows a median first-motion latency of 72 ms (unloaded, includes the profile ramp), i.e. several 20 ms steps, before any IMU or bus latency is added.

## 5. IK limitations and the justified extension

The accepted G2/G3 IK is exact for a level body only (`supportsFlatContactIk`). **It was not modified.** `TiltedContactIk` is a new explicit solver (Newton on `contactForwardKinematics`, which already accepts a ground normal) with the same G2 cylinder, strip and 2 deg tilt policy. Agreement: with the analytic IK for `n = +Z` (5e-9 rad), and with an independent URDF-based scipy solver in the world frame to 5.3e-14 m and 2.0e-11 rad over 15 tilt cases (contact drift under 1e-9 m, nominal-strip tilt below 2 deg).

Tilt envelope of the canonical 150 mm STAND (single axis, 0.25 deg steps):

| Axis / sign | Largest OK tilt [deg] | First failure [deg] | Leg | Solver status | Independent search finds any in-limit solution | Solvable if edge-biased contact allowed |
|---|---|---|---|---|---|---|
| roll + | 8 | 8.25 | lf | CONTACT_MODE | True | True |
| roll - | 8 | 8.25 | rf | CONTACT_MODE | True | True |
| pitch + | 14.5 | 14.75 | rh | NO_CONVERGENCE | False | False |
| pitch - | 5.5 | 5.75 | lf | NO_CONVERGENCE | False | False |

* Roll is limited at about 8 deg by the **G2 nominal-strip policy** (the geometry is reachable with edge-biased contact, which G2 does not accept). Pitch is limited by true infeasibility (no in-limit solution exists, found by independent multi-start search), nose-up (negative) far earlier than nose-down.
* **The measured mechanical stops are tighter than the URDF.** Tilt authority against the TRUE Full Calibration contacts, by body height (deg):

| Body height [m] | roll +/- (measured stops) | pitch + (measured / URDF) | pitch - (measured / URDF) | roll (URDF) |
|---|---|---|---|---|
| 0.15 | 8 / 8 | 13 / 14.5 | 3.75 / 5.5 | 8 |
| 0.14 | 7.75 / 7.75 | 18 / 19.5 | 9 / 10.5 | 7.75 |
| 0.13 | 7.25 / 7.25 | 20 / 20 | 14 / 15.75 | 7.25 |
| 0.12 | 7 / 7 | 20 / 20 | 19.25 / 20 | 7 |
| 0.10 | 6 / 6 | 20 / 20 | 16.5 / 16.5 | 6 |

  At 150 mm nose-up authority is 3.75 deg against 5.5 deg by the URDF alone. The binding joint of the combined +/-3 deg roll/pitch corner is RF lower, which stops at +31.8 deg on the hardware against the URDF +37.5 deg. Lowering the stand improves pitch authority but reduces roll authority (hip angle / strip tilt).
* Required joint ranges against measured contacts and URDF limits:

| Case | Worst margin to measured mechanical contact [deg] | Joint | Worst margin to URDF limit [deg] | Inside measured contacts |
|---|---|---|---|---|
| STAND_RISE_0.100_to_0.150 | 11.79 | rf_lower | 17.47 | True |
| STAND_TILT_COMPENSATED_+-3deg | -1.76 | rf_lower | 3.92 | False |
| STAND_TILT_COMPENSATED_+-5deg | -3.54 | rf_lower | 2.14 | False |
| LIFECYCLE_WALK_357 | 11.79 | rf_lower | 17.47 | True |
| LIFECYCLE_WALK_287 | 11.79 | rf_lower | 17.47 | True |
| LIFECYCLE_TROT_61 | 11.79 | rf_lower | 17.47 | True |
| LIFECYCLE_TROT_309 | 11.79 | rf_lower | 17.47 | True |

## 6. Representative simulated perturbation tests

Loop-equation simulation through the production C++ (not a robot model): dt = 0.02 s (50 Hz), loop delay 2 steps (unmeasured), correction limit 3 deg, rate limit 4 deg/s, hold before fault 0.5 s, sensor noise +/-0.1 deg. Gains are fractions of the derived bound; **all values are study values**.

| Scenario | Gain / bound | Peak measured tilt [deg] | Tail tilt [deg] | Final command r/p [deg] | Max cmd rate [deg/s] | Rejected samples | Fault steps | IK failures | Max joint step [rad] |
|---|---|---|---|---|---|---|---|---|---|
| S1 roll step +2 deg | 0.50 | 2.02 | 0.09 | -1.99 / -0.03 | 4.0 | 0 | 0 | 0 | 0.0037 |
| S2 pitch step -2 deg (nose up) | 0.50 | 2.11 | 0.09 | 0.01 / 1.97 | 4.0 | 0 | 0 | 0 | 0.0045 |
| S3 combined roll +2 pitch +2 deg | 0.50 | 2.86 | 0.09 | -1.99 / -2.03 | 4.0 | 0 | 0 | 0 | 0.0065 |
| S4 slow roll ramp to 3 deg | 0.50 | 0.19 | 0.08 | -2.98 / -0.03 | 2.8 | 0 | 0 | 0 | 0.0031 |
| S5 sinusoidal roll 1.5 deg at 0.2 Hz | 0.50 | 0.28 | 0.11 | 0.02 / -0.03 | 4.0 | 0 | 0 | 0 | 0.0038 |
| S6 sensor dropout 3.0-4.5 s during a +2 deg disturbance | 0.50 | 2.10 | 2.00 | 0.00 / 0.00 | 4.0 | 71 | 322 | 0 | 0.0037 |
| S7 corrupted samples (NaN / low status / bad norm / accuracy) | 0.50 | 2.02 | 0.09 | -1.99 / -0.03 | 4.0 | 20 | 0 | 0 | 0.0037 |
| S8 disturbance beyond the correction limit (+5 deg) | 0.50 | 5.02 | 2.00 | -3.00 / -0.03 | 4.0 | 0 | 0 | 0 | 0.0037 |
| S9 heading drift 30 deg/s (yaw must not matter) | 0.50 | 2.02 | 0.09 | -1.99 / -0.03 | 4.0 | 0 | 0 | 0 | 0.0037 |
| S10 unknown IMU mounting offset 0.5 deg, no disturbance | 0.50 | 0.60 | 0.09 | -0.49 / -0.03 | 4.0 | 0 | 0 | 0 | 0.0034 |
| S11 same offset with a calibrated level reference | 0.50 | 0.65 | 0.50 | 0.01 / -0.03 | 2.6 | 0 | 0 | 0 | 0.0031 |
| S12 gain at 25 percent of the bound | 0.25 | 1.97 | 0.08 | -1.99 / -0.01 | 4.0 | 0 | 0 | 0 | 0.0026 |
| S13 gain at 90 percent of the bound | 0.90 | 2.14 | 0.12 | -1.89 / -0.12 | 4.0 | 0 | 0 | 0 | 0.0058 |

Reading: step disturbances of 2 deg are corrected at the bounded rate with no IK failure and no contact drift; the sinusoid shows the attenuation of a 50 Hz loop with delay; sensor loss ramps the command out and latches a fault (0.64 s after the last valid sample, enable refused until reset); isolated corrupt samples are rejected and never create a correction; a 5 deg disturbance saturates at the 3 deg bound and stays flagged; heading drift has no effect; a gain of 105 % of the bound is refused by the configuration check. **S10 matters physically:** with an uncalibrated mounting-level offset the controller drives the *measured* tilt to zero, which tilts the real body by the offset (S11 with the reference supplied does not). The offset must be measured on hardware before any stabilization.

## 7. Stability, separated

Canonical STAND: COM at (-8.5, -0.3, 157.8) mm (x, y, z, world), height 157.8 mm, static margin 98.6 mm to the support polygon of the four G2 strips; a 2 deg compensated roll moves the COM by only 0.95 mm because the legs reconfigure. URDF mass 2.48 kg (the legacy power analysis cites 3.3 kg: unreconciled, repeated at x1.33).

| Case | Period [s] | Quasi-static margin [mm] | ZMP margin [mm] (approx.) | Frames with ZMP outside the support polygon | Peak COM horizontal accel [m/s^2] | Peak lower-leg torque [N m] | RMS lower-leg torque [N m] |
|---|---|---|---|---|---|---|---|
| WALK_357 | 1 | 1.55 | -97.02 | 32 | 16.6 | 1.06 | 0.62 |
| WALK_357 | 2 | 1.55 | 1.50 | 0 | 4.1 | 1.03 | 0.58 |
| WALK_357 | 4 | 1.55 | 1.54 | 0 | 1.0 | 1.03 | 0.58 |
| WALK_287 | 1 | 2.02 | -112.61 | 40 | 16.3 | 0.99 | 0.58 |
| WALK_287 | 2 | 2.02 | 1.98 | 0 | 4.1 | 0.96 | 0.55 |
| WALK_287 | 4 | 2.02 | 2.01 | 0 | 1.0 | 0.95 | 0.55 |

* **Static equilibrium** of the STAND is comfortable. **Quasi-static WALK** margins are only a few millimetres: the same arithmetic as the G4 report. A 1.5 mm margin equals the COM shift produced by 0.82 kg (the 3.3 kg vs 2.48 kg gap) displaced by about 6 mm, so URDF mass-property error alone can erase it; it must not be read as stability.
* **Dynamic behaviour (ZMP approximation, angular momentum neglected):** at T = 1 s the ZMP leaves the support polygon in a large part of the cycle (COM horizontal acceleration about 16 m/s^2, driven by the open-loop sway); at T = 2 s and 4 s the ZMP margin returns to the static value. The quasi-static WALK assumption therefore fails at T = 1 s and holds at T = 2 s (the threshold between them was not located), independent of any actuator limit. This is a research indicator, not a certification; TROT is not assessed.
* **Actuator feasibility** is a separate question (section 8).

## 8. Actuator feasibility

Limits used, with provenance:

| Quantity | Value | Provenance | Source |
|---|---|---|---|
| No-load speed, slowest of 16 healthy units | 4.315 rad/s | BENCH_NO_LOAD | ST3215 bench QC V6.1 (unloaded servos, bus ~11.0-11.1 V) |
| No-load speed, vendor | 4.712 rad/s (45 rpm) | VENDOR_NOMINAL | Feetech datasheet at 12 V |
| Speed budget of the URDF | 3.037 rad/s (29 rpm) | design budget | URDF README: conservative nominal-operation limit, not a validation |
| Torque, rated / stall / 80 % overload | 0.98 / 2.94 / 2.35 N m | VENDOR_NOMINAL | Feetech datasheet at 12 V; OverloadTorque 80 in the MATDOG profile |
| Effort budget of the URDF | 0.902 N m | design budget | URDF README |
| Acceleration of a commanded profile at Acc = 50 | 12.0 rad/s^2 | BENCH_NO_LOAD (one setting, not a capability) | QC move metrics |
| Acceleration capability for streamed targets | none | UNMEASURED | - |
| Any loaded velocity / torque / acceleration | none | UNMEASURED | - |

Verdicts of all requirement rows (WALK steady cycles at 0.5/1/2/4 s, STAND rise, G4.1 lifecycle peaks, torques):

| Limit | Verdicts of the requirement rows (margin 1.0) |
|---|---|
| bench no-load slowest healthy unit (2813 tick/s) | EXCEEDS_LIMIT: 2, PROVISIONALLY_SUPPORTED: 21 |
| vendor no-load 45 rpm @12 V | EXCEEDS_LIMIT: 2, PROVISIONALLY_SUPPORTED: 21 |
| loaded velocity limit on the assembled robot | REQUIRES_MEASUREMENT: 23 |
| acceleration capability (streamed targets, any Acc) | REQUIRES_MEASUREMENT: 23 |
| vendor overload protection 80 % of stall (2.35 N m) | PROVISIONALLY_SUPPORTED: 10 |
| loaded torque capability on the assembled robot | REQUIRES_MEASUREMENT: 10 |
| vendor rated torque 10 kg cm (0.98 N m) | PROVISIONALLY_SUPPORTED: 7 |
| loaded continuous torque on the assembled robot | REQUIRES_MEASUREMENT: 7 |

* **Joint speed.** Verified: nothing under load. WALK 357 at T = 1 s needs 3.160086587064039 rad/s: above the URDF speed budget (3.037 rad/s) and 73 % of the unloaded bench speed. Minimum periods:

| Case | Min period for the bench no-load speed [s] | Min period for the URDF speed budget [s] | Period at which peak acceleration equals the bench Acc=50 profile [s] |
|---|---|---|---|
| WALK_357 | 0.73 | 1.04 | 3.97 |
| WALK_287 | 0.63 | 0.89 | 3.68 |

* **Joint acceleration: REQUIRES MEASUREMENT.** The QC measured the acceleration of a commanded *profile* at Acc = 50 (about 12 rad/s^2); no measurement of what the servo can follow with streamed targets exists. The G4 WALK 357 peak (190 rad/s^2 at T = 1 s) is 16 times that profile; the peak equals it at a period of about 4 s.
* **Torque (rigid-body model, URDF masses).** Peak lower-leg torque of a WALK cycle is about 1.0 N m (1.4 N m at x1.33 mass): above the URDF effort budget (0.90 N m) and the vendor rated torque (0.98 N m) at peak, below the vendor 80 % overload level (2.35 N m); RMS about 0.6 N m. Static STAND worst torques (hip/upper/lower) are 0.17, 0.18, 0.47 N m (x1.33: 0.22, 0.24, 0.63). All PROVISIONAL at best: the model has no friction, compliance, backlash or cable loads, the load sharing among stance feet is an assumption, and the QC `Present Load` register is not calibrated to torque.
* **Joint range.** Required ranges lie inside the measured mechanical contacts for the STAND rise and for all four G4.1 lifecycles (worst margin 11.8 deg at RF lower); tilt compensation consumes it (section 5). Margins are not an approved envelope.
* **Timing and rate** (arithmetic from the bus and the QC data, not a measured 12-servo loop). A 12-servo sync write is about 1.04 ms at 1 Mbps; the QC reads one servo every 2 ms, so a sequential read-all is about 24 ms and unmeasured. The error of streaming targets (WALK 357):

| Update rate [Hz] | T = 1 s: hold error [ticks] | T = 1 s: linear-interpolation error [ticks] | T = 4 s: hold error [ticks] | T = 4 s: interpolation error [ticks] |
|---|---|---|---|---|
| 25 | 82.4 | 24.75 | 20.6 | 1.547 |
| 50 | 41.2 | 6.19 | 10.3 | 0.387 |
| 100 | 20.6 | 1.55 | 5.2 | 0.097 |
| 200 | 10.3 | 0.39 | 2.6 | 0.024 |
| 500 | 4.1 | 0.06 | 1.0 | 0.004 |

  Linear interpolation at 100 Hz keeps the error near one tick; holding targets at 50 Hz does not. Whether the servo interpolates between streamed targets is unmeasured.
* **Resolution and hysteresis.** 1 tick = 0.088 deg; measured directional hysteresis is 5 ticks (about 0.44 deg) per joint, so the smallest useful body-tilt correction is of that order unless dither is used; the stabilizer dead band must not be below it.
* **Latency.** First motion median 72 ms and settle median 145 ms (unloaded, QC steps). These define the loop delay `N` of section 4: it must be measured on the assembled robot.

## 9. Outstanding measurements

1. IMU mounting-level offset on the assembled robot, acquisition-to-command latency and jitter, accuracy under motion.
2. Loaded servo speed, acceleration (with streamed targets) and torque; thermal/current behaviour; tracking error under load; whether targets are interpolated.
3. A 12-servo read/write loop timing on the real bus.
4. Mass, COM and inertia of the assembled robot (battery, wiring, head/jaw) to replace the CAD URDF values.
5. Load sharing among stance feet (compliance) and ground friction.
6. Operational joint envelopes with margin to the measured stops; resolution of the LOWER MAX shortfall follow-up.
7. Calibration persistence (restoration) and q0 refinement follow-ups.

## 10. Validation on the final source (ALL PASSED)

Run by `validate.py` at HEAD `b6cad9f9ec8a1beb4ccec0eff40deeb6aa672e7b` (worktree clean before the run: True). The G4 independent oracle was re-run on the current source (3872 samples, 16 independently solved legs).

| Gate | Result | Summary |
|---|---|---|
| test_gait_build_strict_cpp17 | PASS |  |
| test_gait_run | PASS | GAIT_HOST = PASS: 148291 checks, 0 failures |
| test_contact_mode_build_strict_cpp17 | PASS |  |
| test_contact_mode_run | PASS | CONTACT_MODE_HOST = PASS: 68 checks, 0 failures |
| test_stabilization_build_strict_cpp17 | PASS |  |
| test_stabilization_run | PASS | STABILIZATION_HOST = PASS: 10111 checks, 0 failures |
| test_gait_asan_ubsan_build | PASS |  |
| test_gait_asan_ubsan_run | PASS | GAIT_HOST = PASS: 148291 checks, 0 failures |
| test_contact_mode_asan_ubsan_build | PASS |  |
| test_contact_mode_asan_ubsan_run | PASS | CONTACT_MODE_HOST = PASS: 68 checks, 0 failures |
| test_stabilization_asan_ubsan_build | PASS |  |
| test_stabilization_asan_ubsan_run | PASS | STABILIZATION_HOST = PASS: 10111 checks, 0 failures |
| g5a_tests | PASS | Ran 15 tests in 1.335s; OK |
| regenerate_independent_oracles | PASS | pitch_- {'max_tested_ok_deg': 5.5, 'first_failure_deg': 5.75, 'failed_leg': 'lf', 'cause': 'NO_CONVERGENCE', 'any_in_limit_solution_exists_independent': False, 'solves_if_edge_biased_contact_allowed': False} |
| regenerate_loop_simulations | PASS | {'gain_105_percent_of_bound_refused': True, 'fault_latch_time_s': 0.64, 'enable_refused_while_latched': True, 'final_state_after_reset': 'DISABLED'} |
| regenerated_evidence_identical_to_committed | PASS |  |
| g4_independent_oracle_current_source | PASS | "status": "PASS", |
| g4_artifact_tests | PASS | Ran 15 tests in 1.457s; OK |
| g4_artifact_manifest_check | PASS | ARTIFACT_MANIFEST OK 57 artifacts; historical generator revisions: 4 |
| g41_contact_tests | PASS | Ran 14 tests in 12.739s; OK |
| g41_artifact_manifest_check | PASS | ARTIFACT_MANIFEST OK 27 artifacts |
| g35_pose_audit_tests | PASS | Ran 28 tests in 15.851s; OK |
| g35_artifact_manifest_check | PASS | ARTIFACT_MANIFEST OK 80 artifacts; historical generator revisions: 1 |
| host_motion_runner_g1_to_g5a | PASS | STABILIZATION_HOST = PASS: 10111 checks, 0 failures |
| static_audit | PASS | STATIC_AUDIT = PASS |
| git_diff_check_worktree | PASS |  |
| git_diff_check_vs_g41_head | PASS |  |
| accepted_g1_g4_1_files_unchanged_vs_g41_head | PASS | only G5-A additions, the host-runner line and the host-oracle include allowlist changed; G1-G4.1 sources, geometry, calibration interfaces and evidence untouched |

## 11. Requirements for hardware commissioning (not authorized)

* A separate actuator safety layer owning all limits (position, speed, acceleration, torque/current, temperature, watchdog, torque-off path), with limits derived from loaded measurements; this milestone provides only a classifier.
* Persistence of calibration or an explicit per-boot q0 acquisition accepted by the calibration owner; approved operational envelopes.
* The measurements of section 9, an IMU level-reference procedure, and an explicit minimum-period/rate policy for WALK (a period of 1 s exceeds the URDF speed budget and the quasi-static assumption; 2 s satisfies both in this analysis).
* A tethered, torque-limited first stand in the validated joint range, with the stabilizer disabled first and enabled only after the level reference and latency are measured.
* Dynamics/state-estimation work before any TROT claim; independent safety review and explicit written authorization. Nothing in G5-A authorizes motion.
