# MATDOG 1.0.1-dev.15 — joystick Motion, PRONE STOP recovery, runtime cost

**IMPLEMENTED NOT VALIDATED HW.** Offline candidate built from a clean commit; no flash, OTA, serial port, robot API, NVS/EEPROM write, push or merge. dev.14 (`4ff2aad`, APP `71028f95…`) remains the installed reference; its worktree and artifacts are unchanged.

Branch `matdog/dev15-joystick-motion`, worktree `~/MATDOG/worktrees/matdog-dev15-joystick-motion-20261105`, base `4ff2aadf4ed907d39f2b3ad4f9e6af54db5decee`.

## Candidate

| Item | Value |
|---|---|
| Version | `1.0.1-dev.15`, `ROBOT_POWERED`, OTA ingest 0, motion stack `MATDOG_SIMPLE_V1` |
| Source commit | `478b0f0512eef0a3b9603118b316f4c97f5d87d9`, CLEAN |
| APP SHA256 | `743c0e0c811a4ae83ee9c7cec7287779a10c35ad3627545233a1f8223bc8879d` (1,689,568 B, 32.2 % of the 5 MiB slot; dev.14 1,669,584 B) |
| Partition table | `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`, layout `MATDOG_16M_2x5M_NVS_V1` (identical to dev.14) |
| Merged image | `104a5e295931246b634bcd9ab5beff84816879d2d5d8d562014466d18463f0cd` |
| Artifacts | `~/MATDOG/build-output/dev15-joystick-motion-20261105/final-build-478b0f0/artifacts/ROBOT_POWERED-ota0/` (manifest, ELF, map, SHA256SUMS) |
| Rollback | dev.14 APP `71028f95…` unchanged on the robot and in its build output (not exercised) |

Regression on the candidate commit, all PASS: SimpleMotion (incl. joystick, PRONE STOP recovery, IMU slew), contextual STOP, operational host tests, dashboard DOM/contract/Twin/portal budget, HTTP fence contention, Motion HTTP transport, slack exemption, network static audit, STOP adapter, Twin assets, dev identity, release-candidate gate, motion convergence source, full `static_audit.py`. Viewer vitest 99/99. Pre-existing suites that already fail on dev.14 (missing sources/`jsdom` in their runners) unchanged.

## What changed and why

| Area | First divergence (evidence) | Change |
|---|---|---|
| Joystick | WALK law closed-form at constant velocity: a speed change would move planted feet | SE(2) `GaitTrack`: re-base at each change, planted anchors fixed, swing targets planned on the commanded ramp; constant parameters equal `gaitCartesian` bit for bit |
| Lateral / yaw | `CONTACT_INVALID` past 2° foot-axis tilt; `kHeight` HIP ±25 mrad | Tread fillet contact (closed form, STL agreement 0.03 mm, ≤12°, lower leg ≥2 mm off ground); `kSteer` FCL-certified 120/120 ≥3 mm (min 10.87 mm) + 12/12 adjacent; 12 mm sway and own IMU tilt envelope for steered walks only |
| Backward | Support −4.1 mm with the forward leg order | Time-reversed leg order for vx<0 (+1.2…+1.4 mm with reference contacts) |
| IMU heading | `compensateLevel` rebuilt the body without the gait yaw (latent in dev.14) | Correction composed with the gait heading; identical when yaw is 0 |
| PRONE STOP | dev.14 HW: refusal `STOP_RECOVERY_POSTURE_UNQUALIFIED` before any geometric check | Held RAW restarts the existing post-ENABLE route of the certified region holding held RAW and measured pose; torque ON, no ACQUIRE/ENABLE; also planned STAND_DEPLOY/LIFT |
| Runtime | dev.14 HW: max release 33–36 ms, COMPENSATE ≤20.5 ms, 22–59 missed | IK lazy verification, no seed FK, squared Newton residuals, one Gauss-Jordan condition inverse |
| IMU slew | 0.0004 rad per release (0.020 rad/s, slower with misses) | 0.040 rad/s × monotonic dt, capped at two periods; 0.060 rejected (native arrival regression) |
| Reset vista | camera placed on the IMU heading | robot-anchor yaw reference −yaw at click, canonical +X camera; quaternion/roll/pitch untouched |
| Diagnostics | HTTP lacked same-release timing records; aggregated IMU flag | status: MotionTiming max-work/first-miss partitions + work counters, IMU reason/requested/applied, thermal counters, walk law velocity; dashboard provenance FAST/CONTROL/writer |

## Measurements (offline)

Release cost proxy, libm calls per Motion release on production code with a tilted IMU (`scripts/offline_motion/run_motion_cost_probe.sh`), WALK45 dev.15 stage 2 → final: sin/cos 282 → 242, inverse trig 72 → 64, sqrt 118 → 94; plus 24 fewer divisions per condition estimate. Joint targets identical to the pre-change oracle (|Δq| = 0 at 1e-9 rad). **ESP32-S3 release time is not measured offline.**

Joystick envelope (`scripts/offline_motion/run_joystick_envelope.sh`, 6 directions, 8 octants, reversal, silent client, release in start, 8 random): no fault, all end STANDING, 0 IMU rays outside, compensated foot tilt ≤9.3°, non-foot ground clearance ≥2.26 mm. Static support with real fillet contacts and URDF COM: steady commands ≥ +0.35 mm (pure clockwise turn lowest); random multi-axis transients down to −0.90 mm. A sub-millimetre static margin is a model result, not a stability guarantee.

Stack, worst direct-call chain from `SimpleMotion::update` (Xtensa ELF, callx unresolved): dev.14 8832 B → dev.15 9104 B (+272 B).

## Not closed

- **Motion budget on the ESP32-S3**: the host proxy dropped ~14–20 %, not enough to claim 20 ms. HTTP 503 `MOTION_BUDGET_UNAVAILABLE` is the same cause (a command waits ≤100 ms for 5 ms of slack before the next release; with >15 ms of release work none arrives). Gate kept.
- **PRONE terminal torque OFF**: CAD rest model and dev.14 loads (RH/LH_LOWER 9.6/6.4 % in PRONE terminal) show residual rear-leg support; settling without torque is not predictable offline. Torque stays ON.
- Random joystick transients can drop the static margin below zero in the model; native DISENGAGE/ALIGN still end a STOP in SAFE_OFF (no synchronous reference); RH_HIP temperature spikes cause unknown (counters now exported).

## Minimum supervised hardware sequence (not executed)

1. Flash only after an explicit decision; verify APP SHA256, boot identity 1.0.1-dev.15, gen-1, SAFE_OFF 13/13.
2. STAND150 → WALK STANDARD 45 one cycle → WALK STOP: read `timing.max_work`/`first_miss`, `missed_releases`, `imu_correction` and stack high-water.
3. PRONE, STOP in PRONE_DEPLOY, STAND: STAND_FROM_STOPPED_PRONE to STANDING, torque never cycled.
4. Joystick on the fixture, then on the floor at 25 %: forward, backward, left, right, CCW, CW, release (WALK STOP), page close (1 s timeout), STOP in each.
5. Reset vista at several headings.
6. PRONE terminal: record loads and settle under an operator-held SAFE_OFF before any torque-OFF rule.
