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
