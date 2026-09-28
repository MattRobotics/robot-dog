# MATDOG CR3-M5 — fail-closed production actuator composition

Date: 2026-09-28
Commit: `5f57ff8`

## What changed

Replaced the CR3-M3/M4 fail-closed-by-**absence** composition (null backend,
unbound geometry, no `CalibrationMotionPermit` instance anywhere) with
fail-closed-by-**active gates**, per the CR3 brief's Priority 1:

- `geometry_profile_` bound to the real compiled Geometry V5 data at boot.
- `actuator_backend_` (`ServoBusActuatorBackend`) bound to the one
  Controller-owned `servo_bus_`, wired into `actuator_runtime_` as a real,
  non-null backend.
- `actuator_policy_` bound to the real geometry profile.
- `motion_permit_` (`CalibrationMotionPermit`) is a real Controller-owned
  member. `Controller::updateCalibrationMotionPermit()` runs unconditionally,
  first thing in `update()`, before `command_router_.update()` — every fact
  it feeds the permit (session state, geometry-bound, transform-completeness,
  authority, inhibit) is read fresh from its live source every tick, never
  cached. `check()` (never `grant()`) re-verifies an already-granted permit
  and revokes on any mismatch.
- `@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION`: new command,
  MAINTENANCE-gated, admits the frozen CR2-C q0 evidence into production via
  the already-existing `prepareCurrentQ0Evidence()` pipeline. RAM-only — no
  bus transaction, no authority acquisition, no EEPROM write.

## Why this stays fail-closed with real objects wired in

Three independent guarantees, so no single future one-line edit opens a path:

1. No command path anywhere reaches `plan()`/`commit()`/`execute()`/`abort()`
   on the actuator infrastructure (unchanged from CR3-M3/M4).
2. `operator_calibration_motion_authorized_` has no setter in this build →
   `CalibrationMotionPermit::grant()` can never be called → the permit can
   never become ACTIVE.
3. `calibration_.startSession()`/`.activate()`/`.submitPopulationEvidence()`
   have no caller in this Controller → no live session can ever exist →
   `session_active` is always false, independent of (2).

`check_actuator_infrastructure_wired_fail_closed()` (rewritten) and the new
`check_calibration_q0_promotion_wiring()` enforce all three mechanically.

## Verification

- Host suite: **22/22 PASS**.
- Static audit: **PASS**, including the rewritten/new checks.
- Safe-actuator mutation suite: **PASS**.
- **5 manual mutation spot-checks** against the two new/rewritten audit
  functions (not covered by the existing `test_static_audit_safe_actuator.py`
  suite, since that suite predates this gate) — each applied directly to the
  real source, confirmed to FAIL the audit with the expected reason, then
  reverted with `git checkout --` (working tree confirmed clean and
  re-passing afterward):
  1. `operator_calibration_motion_authorized_ = true` → caught.
  2. `actuator_runtime_.begin()` reverted to `nullptr` backend → caught.
  3. `updateCalibrationMotionPermit()` call removed from `update()` → caught.
  4. `calibration_.startSession(...)` call added to `Controller.cpp` → caught.
  5. Stray `transforms().admit(...)` added outside the one reviewed
     `@CALIBRATION Q0 PROMOTE` branch → caught.
- ESP32 build: **PASS** (`esp32:esp32:esp32s3`, `USB_ONLY` profile, no
  upload). 1,039,687 bytes (33%) program storage.
- Hardware touched: **no**.

## Deliberately left for the hardware session

- **Session-start orchestration.** `buildCurrentLegPopulationEvidence()` may
  only be called from `CalibrationQ0CaptureSession.cpp`
  (`check_calibration_population_evidence()` enforces this) — a new
  Controller-level orchestration cannot re-derive population evidence
  independently. The reviewed design: reuse the already-CR2-B-proven
  `q0_capture_.populationResult()` (after `@CALIBRATION Q0 CAPTURE` has run
  to `COMPLETE`) as the input to a new command that calls
  `calibration_.startSession(leg, mode, LIVE_SESSION)` →
  `calibration_.submitPopulationEvidence(q0_capture_.populationResult().evidence)`
  → `calibration_.activate()`. Needs its own
  `check_calibration_session_start_wiring()`-style audit function mirroring
  `check_calibration_q0_production_wiring()`.
- **Operator-authorization grant path.** A new MAINTENANCE-gated command
  (e.g. `@CALIBRATION MOTION PERMIT GRANT CONFIRM_...`) to set
  `operator_calibration_motion_authorized_ = true` and call
  `motion_permit_.grant(facts, &motion_permit_token_)`. Both of the above are
  intentionally bundled for the same session: neither is independently
  useful without the other, and both should be reviewed live with Matteo
  present, matching the CR3 brief's own STOP-CONDITION framing.
