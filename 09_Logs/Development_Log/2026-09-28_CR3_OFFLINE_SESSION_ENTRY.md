# MATDOG CR3 — offline completion session entry

Date: 2026-09-28
Branch: `feat/calibration-motion-full-cal-v1`
Entry HEAD: `2603ee789e7619b79c37e5f8bdadfbb06da8761e` (M0-M4 already landed; see
`2026-09-27_CR3_M0_M2_ENTRY.md`, `_M3_BACKEND.md`, `_Q0_EVIDENCE_PERSISTENCE.md`)

Scope: offline software completion only. No ESP32 flash, no live servo serial
access, no hardware motion. Worktree isolated at
`~/MATDOG/worktrees/robot-dog-calibration-cr3`, verified not to overlap the
parallel `feat/gait-engine-offline-v1` worktree or `robotics-reverse`/NormaCore.

## Pre-existing regressions found and fixed (commit `1b36f9d`)

Discovered while establishing a clean baseline before starting new work — all
three verified via `git diff` to be untouched by any change in this session
before being identified, so they predate this session:

1. `run_host_tests.sh` never linked `CalibrationTargetResolver.cpp` into
   `test_calibration_execution_engine`, even though
   `CalibrationExecutionEngine.cpp` calls `resolveUrdfQToRaw()`/
   `resolveDeltaFromQ0()`. Blocked the entire host suite outright
   (`set -euo pipefail`).
2. `SafeActuatorPolicy::evaluate()`'s CALIBRATION-owner motion-permit check
   ran before the session/replay-origin check, so a session-less request
   reported `REJECT_NO_CALIBRATION_MOTION_PERMIT` instead of the more
   fundamental `REJECT_NO_CALIBRATION_SESSION`. Diagnostic-precedence fix
   only — both are refusals either way, so no safety behavior changed.
3. Two test files had setup bugs exposed by fix #2 and by
   `CalibrationMotionPermit::check()`'s intentional revoke-on-mismatch
   behavior (a stale permit must require a fresh explicit grant, never
   resume) — see the commit message for detail.

## CR3 Priority 2 — uncertain Torque-ON / GoalPosition write safety (commit `385398a`)

`ServoBus::writeGoalPosition()` had no readback verification at all (pure
ACK/status), and `ServoBus::enableTorqueOn()` ANDed the independent
TorqueEnable readback with the ACK/status, which could under-report a
readback-confirmed-ON servo as `false`. Both now return the three-state
`servo::ServoWriteVerifyResult`, classified only by an independent register
readback (TorqueEnable, and the GoalPosition register itself — not
`present_position`, which lags a write by travel time). The uncertain state
is threaded up without collapsing: `ActuatorBackend` returns
`actuator::BackendWriteOutcome`, and `ActuatorRuntime::execute()` surfaces
`ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF`, distinct from
`BACKEND_REJECTED`. No production caller reaches these primitives yet
(`ActuatorRuntime` is still null-backend in `Controller.cpp`).

static_audit.py gained `check_goal_position_register_boundary` (read-only,
one-accessor rule for `SMS_STS_GOAL_POSITION_L`, mirroring the existing
`PositionOffset` rule) and `check_uncertain_write_propagation` (fails if
`UNCERTAIN` is ever folded into a verified outcome anywhere in the chain).

## Results as of `385398a`

- Host test suite: **22/22 binaries PASS** (`scripts/tests/run_host_tests.sh`),
  including 2 new UNCERTAIN-outcome cases added to `test_actuator_runtime.cpp`.
- Static audit: **PASS** (`python3 scripts/static_audit.py`), all checks
  including the two new ones and the existing mutation suites.
- Safe-actuator audit mutation suite: **PASS**
  (`python3 scripts/tests/test_static_audit_safe_actuator.py`).
- ESP32 firmware build: **PASS** (`scripts/build.sh`, FQBN
  `esp32:esp32:esp32s3:...`, `USB_ONLY` profile, no upload/no device contact).
  1,033,127 bytes (32%) program storage, 62,756 bytes (19%) dynamic memory.
- Hardware touched: **no**. No serial port opened, no ESP32 reset/flash, no
  servo/BMS command issued.

## Next

Priority 1 — production composition (real Geometry V5 binding, real
`ServoBusActuatorBackend`, `CalibrationMotionPermit` wired and refreshed every
tick, frozen q0 evidence promotable via an explicit guarded command) — see the
follow-up log entry for the wiring decisions and what was deliberately left
for the next hardware session.
