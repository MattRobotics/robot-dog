# 2026-10-06 — ROBOT_POWERED boot census correction (v0.2.0-dev.2)

POWER ON
-> BOOTING / white LED
-> required component initialization
-> canonical read-only servo census
-> READY only after required hardware PASS

The startup census is not motion:
Ping only, incremental, no torque, no GoalPosition, no EEPROM.

v0.2.0-dev.1 remains preserved historical hardware evidence.

## Why

dev.1 on the real robot stayed `SYSTEM health=BOOTING` indefinitely:
ROBOT_POWERED marks the servo bus REQUIRED, and nothing observed the
population until an operator command ran. dev.2 makes the Controller arm
exactly one canonical census (11..55, 13 expected, 52..55 absent by design)
during `begin()`; `ServoBus::update()` then issues at most one Ping per tick.
The completed census verdict, not the last individual Ping/read, is the
servo subsystem health under ROBOT_POWERED. USB_ONLY is unchanged.

## Offline gate: what was actually wrong

- The reported `DEV2_OFFLINE_GATE=PASS` was false.
- `MOTION_CONVERGENCE = FAIL` (7 gates) and therefore `STATIC_AUDIT = FAIL`
  had one root cause: the gate ran without the pinned offline Python
  dependency tree on `PYTHONPATH` (`ModuleNotFoundError: trimesh`). The
  dev.1 dependency tree was re-verified (1148 files, 0 mismatches) and reused
  unchanged. Nothing functional, no provenance mismatch, no source change.
- That abort hid a real stale expectation in `test_dev_identity_build.py`
  (three `0.2.0-dev.1` literals). Updated to dev.2.
- No test disabled, no assertion weakened, no evidence regenerated.

Details, evidence paths and the standing limits:
`09_Logs/Validation_Reports/MATDOG_V0_2_DEV2_BOOT_SELFTEST_DELTA_AUDIT_2026-10-06.md`.
Committed-tree qualification and hardware results are appended there by a
later documentation-only commit.

## Outcome on the robot (2026-10-06, candidate d0ede36)

- Committed-tree qualification PASS (central static audit, motion 20/20,
  110/110 mutants, both sandboxed builds). ROBOT_POWERED 1,178,256 bytes
  `235e67be...`, USB_ONLY 1,174,368 bytes `1cc7aa7c...`.
- App-only flash over ROM no-stub PASS, exact read-back identical, no
  full-flash backup, table/otadata/nvs/matdog_nvs unchanged.
- First boot: `STARTUP_SERVO_CENSUS=PASS` 13/13 in 4.0 s, then
  `SYSTEM health=READY`. The dev.1 finding is closed.
- Fresh Q0 12/12 promoted; LF INITIAL RECOVERY 12/12; LF UPPER MIN measured.
- LF UPPER MAX stopped fail closed: thermal `REPEATED_ANOMALY` latch on bus 12
  after three spurious block-read temperature samples (95, 78, 77 against
  direct reads of 32..34 degC). Servos were never hot. SAFE_OFF 13/13 verified.
  No retry. RF/RH/LH, SAVE/ACK and persistence were not run.
- The artifact predates dev.2 (23 transients in the 2026-10-01 24/24 run);
  the per-servo 3-in-30 s latch came with the post-abort thermal rework. That
  combination, not dev.2, blocks a full calibration. It goes to central review.

Full results and residuals: section "Results" of the delta audit.
