# HANDOFF — MATDOG Motion Hardware Readiness

**A fixture micro: NO-GO. B autonomous STAND/WALK: NO-GO. Software integration and offline campaign: PASS.** No hardware operation was executed. Full Calibration was not repeated.

## Repository and immutable build identity

- Worktree: `/home/matteo-manicardi/.codex/worktrees/motion-enablement-20261012/robot-dog`.
- Branch: `matdog/motion-night-20261012`; origin: `https://github.com/MattRobotics/robot-dog.git`.
- Baseline: `4097ec7b49ae1670d488d15e172b09fd97e84026`, initially clean, required handoff/dossier/build/joint/paired artifacts present.
- Integration commit: `b4060f08c2eb40efd7f8fe60f4d2ae668da001cb`.
- Radio declaration correction and **compiled clean source commit**: `f62e89b1f482ebb2aee6cca5f5ec148d7509b383`.
- Draft PR: https://github.com/MattRobotics/robot-dog/pull/47. No merge on main.
- The delivery tip additionally contains this report, receipt/source inventory and a test print correction. Its exact SHA is registered after commit/push in `/home/matteo-manicardi/MATDOG/verification-artifacts/MOTION_HARDWARE_READINESS_20261013/HANDOFF_FINAL.json` and the PR description. Do not label that documentation tip as the compiled source.

Local preserved image:

`/home/matteo-manicardi/MATDOG/verification-artifacts/MOTION_HARDWARE_READINESS_20261013/candidates/f62e89b1f482ebb2aee6cca5f5ec148d7509b383/ROBOT_POWERED-ota0/MATDOG_Controller.ino.bin`

SHA256: `772b2658c63878b0d98ee8046cf3bd4bbb158460b0c79eb0af8b4361e4b0499f`.

1,690,304 bytes, ROBOT_POWERED, layout 16MiB/2×5MiB, ESP32 core 3.3.11, MOTION_AUTHORIZED=0, OTA_INGEST=0. Manifest and partition binary preserved beside it; exact verification in `BUILD_RECEIPT.json`. Installed baseline source identity `b08322dedc5564c080f691dc98047511ef2b953f` is distinct, has not been re-read today and is not replaced.

## Delivered state and remaining gates

| Area | Offline result | Remaining physical/method/software gate |
|---|---|---|
| Frozen prone geometry | 242 samples/251 segments preserved; 12 adjacent continuous PASS with no revolute exclusions; 120 nonadjacent pairs/segment ≥3.0064599155mm | Loaded assembly/fit/cables and simultaneous tube for encoder quantization, tracking and sequential writers |
| Runtime prone→150mm | Production MotionService→Coordinator→AdaptiveStand→Executor→Safe Actuator Layer; 3,142 simulated frames/37,704 mapped writes, independent geometric target comparison | Strict knot membership needs certified measured entry tube; four LOWER initial -92° incompatible with inset proposals; body/support/load external observation |
| RESTORE and 12 limits | Real restore integrity/identity/geometry/generation and accepted transforms; gen-1 reviewed separately from 1 October; 12 proposals with all phase margins | Current robot record/identity/installation not read; 0/12 limits approved; loaded rate/accel/tracking/thermal/electrical envelopes unavailable |
| Fixture micro | Real bounded one-joint quintic and 11 hold, distinct local session scope, prime/energize/arm/permit/revoke | Specific measured corridor and 12 workspace limits; fixture/kill/operator; bus WCET, output ceiling, prime/holding/STOP; reviewed local operator adapter and authorized image absent |
| Producers/ownership | 12 fresh raw/actual slots, BNO085 timestamps, BMS/power/thermal gates, exclusive ServoBus lifecycle, denials/revocation | Current freshness/latency/dropout/rail/thermal observations; MCU stall/reset and independent power-cut response not qualified |
| G4/FeedbackWalk | Preserved scheduler, runtime STL target adapter; 214 positions agree; entry/STOP/rearm and queued vx/vy/yaw transitions tested | Four derivative witness switches refused; continuous first-stride/velocity box swept and loaded stance/swing/dynamics unqualified |
| HTTP/Dashboard | 48 actions/twin/joystick preserved, actual/target and prime/arm distinct, gates visible, production handler/auth/CSRF tests, browser STOP offline ACK | Actual RUN_STA/TLS provisioning/radio/certificate and physical STOP untested; Web cannot grant authority |
| Build/regressions | Integrated PASS; immutable clean-source candidate and SHA/partition/source receipts | No install/flash or hardware qualification authorized |

Target clock and support requests are planner outputs. Encoder, quaternion and electrical telemetry do not prove feet carry load. No synthetic telemetry is represented as hardware. Simulation approvals are explicit test fixtures.

## Review evidence

- `README.md`: technical changes and limitations, including numerical certification methods.
- `TEST_RESULTS.json`: per-suite PASS/FAIL/BLOCKED, log path/hash, resolved development failures and physical denials.
- `SOURCE_MANIFEST.json`: 466 committed sources/artifacts, SHA256 and resolved Git LFS OID/size verification; no compiled firmware change after the candidate.
- `adjacent_axis.json`, `assembly_nesting.json`, `nonadjacent_3mm.json`, `operational_clearance.json`: frozen-route proofs and separation of lower bound from actual clearance.
- `JOINT_LIMITS.md`, `joint_limits_review.json`: 12 review rows, sources, margins/risks and per-phase extrema; no candidate promoted to approved.
- `stl_gait_runtime.json`: matched targets and derivative switch refusals.
- `FIRST_MICRO_MOTION.md`: A minimum gates and operator read-only/conditional movement procedures with abort/log sheet.
- `AUTONOMOUS_STAND_WALK.md`: B additional gates and staged procedures.

The integration log contains FAIL assertions from deliberately injected USB mutations; both historical defects are detected and the mutation suite passes. The static source-convergence selection defers historical Motion execution suites; the current Motion execution suites are explicitly run by `run_motion_enablement_tests.sh`. Do not report those historic deferred suites as freshly executed.

## Shortest next sequence

1. **Operator personally: local read-only session on externally supported robot with physical servo power cutoff.** Identify installed firmware and available commands first; record current calibration schema/CRC/generation/ACK/identity, 12 raw position/torque/GoalPosition/time/temperature/voltage, IMU quaternion/time, BMS and measured servo rail, fixture/kill and actual linkage pose. Use `FIRST_MICRO_MOTION.md`; no Full Calibration, write, grant or motion.
2. Review only the A workspace/envelope: qualify all 12 prime/hold targets, certify the selected micro corridor including actual uncertainty, establish scoped feedback/command/abort deadlines and current/torque/thermal/voltage ceilings. Fix the producer timeout strategy if measured WCET cannot satisfy them. Implement/review the explicit local operator adapter against that typed profile; it must not expose Web authority or a generic bypass. A does not wait for WALK derivative/dynamic gates.
3. After every A gate is satisfied, seek **separate explicit operator authorization** naming the reviewed firmware SHA, local presence, fixture, joint/δ/T, prime/arm/permit and power-cut abort. No previous permission for offline development authorizes these actions. A result must be logged before further movement.
4. For B, certify an inward entry from the measured LOWER pose without loosening contact margins, route membership/uncertainty tube, real ventral→feet load-transfer observations and 150mm support. Resolve STL contact witness derivative switches, then certify continuous runtime first stride and velocity transitions; qualify loaded STOP/rearm/dynamics. Only then review staged B authorization.

ADR-004: changes are original MATDOG runtime/test/certifier/documentation using already canonical project assets. No decompiled XGO source, secret, sensitive NVS backup, new third-party payload or compiled binary is committed. Build binaries/logs/browser screenshot remain local outside Git. No new worktree, destructive history operation, or main merge occurred.
