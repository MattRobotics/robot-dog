# MATDOG v0.2.0-dev.2 — Boot Self-Test Delta Audit
Date: 2026-10-06
Base: d7aa369631202aed70458d0c7ff2980534882058 (v0.2.0-dev.1, frozen, preserved)
Branch: fix/matdog-v0.2.0-dev2-boot-selftest
External evidence root:
`~/MATDOG/verification-artifacts/MATDOG_V0_2_DEV2_BOOT_SELFTEST_20261006`

## Hardware finding

v0.2.0-dev.1 booted and ran, but SYSTEM health remained BOOTING because
ROBOT_POWERED requires the servo subsystem while no automatic population
observation occurred (`SERVO detected=UNKNOWN`, `last_census=NOT_RUN`,
dev.1 post-power-cycle runtime log, 2026-10-06T17:38Z).

## Corrected boot contract

ROBOT_POWERED automatically schedules one canonical read-only servo census.

- canonical range: 11..55
- expected current population: 13
- 52..55 remain ABSENT_BY_DESIGN
- census is incremental
- at most one Ping per Controller tick
- no Torque ON
- no GoalPosition
- no EEPROM write
- no calibration motion
- no automatic SAFE_OFF write

Health:

- census pending -> BOOTING
- canonical census PASS -> servo OK
- population mismatch/incomplete -> FAULT
- census could not be armed -> FAULT (never BOOTING forever)
- READY requires all REQUIRED components to be healthy
- a later operator census replaces the population verdict when it completes;
  while it is RUNNING the previous completed verdict is retained

USB_ONLY is unchanged: no automatic census, health from `ServoBus::health()`.

## Source delta against the base

| File | Change |
|---|---|
| `src/config/BuildConfig.h` | `kFirmwareVersion` 0.2.0-dev.1 -> 0.2.0-dev.2 |
| `src/core/Controller.cpp` / `.h` | ROBOT_POWERED arms one `servo_census_.start()` in `begin()`; `update()` maps the completed census verdict to servo health; boot banner line `startup_servo_census : ENABLED_READ_ONLY_INCREMENTAL` replaces `startup_servo_scan : DISABLED` |
| `src/servo/ServoBus.cpp`, `src/servo/ServoCensus.h` | comments only (contract wording) |
| `scripts/static_audit.py` | `check_no_startup_servo_traffic` replaced by `check_startup_servo_selftest_wiring` (exactly one profile-gated `start()`, tracked result, no low-level servo primitive in `begin()`, verdict-to-health wiring, banner) |
| `scripts/tests/test_static_audit_startup.py` | new contract test + three rejected mutations |
| `scripts/tests/test_command_router_persistence.cpp` | stale `FW_VERSION=0.2.0-dev.1` expectation -> dev.2 |
| `scripts/tests/test_dev_identity_build.py` | three stale `0.2.0-dev.1` literals -> dev.2 (see below) |
| `scripts/tests/test_servo_population.cpp` | comments only |

No motion, gait, stabilization, calibration, persistence, OTA, network or
partition source changed. `ServoBus::begin()` still issues no bus traffic
(`check_no_auto_scan_on_boot` retained unchanged). The pure motion library
pins (`motion_convergence_sources.json`) are untouched and still match.

Documentation written before dev.2 that states `startup_servo_scan : DISABLED`
(CHANGELOG history, VALIDATION.md, G3_ROBOT_POWERED_VALIDATION_PLAN.md) is
historical evidence for the builds it describes and is deliberately not
rewritten; for dev.2 the line above supersedes it.

## Offline gate diagnosis (2026-10-06)

An earlier helper script printed `DEV2_OFFLINE_GATE=PASS`. That print was
false. The real state, from the preserved logs
(`00_diagnosis/as-found-motion-convergence-5lrz488q`):

1. `MOTION_CONVERGENCE = FAIL`, 7 of 20 gates: `fresh_g4`,
   `fresh_g5_independent`, `fresh_g5_perturbation`, `pose_audit_tests`,
   `gait_audit_tests`, `contact_audit_tests`, `stabilization_audit_tests`.
   Every one of them ends in the same exception:
   `ModuleNotFoundError: No module named 'trimesh'`.
   Root cause: ENVIRONMENT. The gate was started with the bare system Python.
   The dev.1 qualification ran it with
   `PYTHONPATH=<MATDOG_V0_2_DEV1_CONVERGENCE_20261005>/deps/python`, the
   offline-extracted pinned wheels (trimesh 5.1.0, python-fcl 0.7.0.11,
   rtree 1.4.1, networkx 3.7, Cython 3.3.0). That tree was re-verified
   against its own extraction manifest before reuse: 1148 files, 0 mismatches,
   0 extra files. It is not a functional failure, not a provenance failure
   and not a dirty-worktree/commit-identity failure
   (`MOTION_CONVERGENCE_SOURCE = PASS` throughout).
2. `STATIC_AUDIT = FAIL` was a pure consequence of (1): `check_host_tests` ->
   `run_host_tests.sh` -> `run_motion_convergence_tests.py`. The other 65
   direct static checks report 0 findings on the dev.2 tree.
3. A second, real failure was MASKED by (1): `run_host_tests.sh` aborted at the
   motion gate and never reached `test_dev_identity_build.py`, which failed
   3 of 9 tests because its fixture still pinned `0.2.0-dev.1` while the
   source under test identifies as 0.2.0-dev.2
   (`REFUSED=IDENTITY_MISMATCH DETAIL=FW_VERSION is missing from application
   bytes`). Same class as the stale expectation already corrected in
   `test_command_router_persistence.cpp`. Corrected by updating the three
   literals; the test still pins one exact version.

Not done: no test disabled, no assertion weakened, no golden or historical
evidence regenerated, no motion/gait/stabilization runtime touched, no pin
file edited.

## Qualification procedure for the committed candidate

Identical to the dev.1 qualification (`run_central.py`,
`run_remaining_offline.py`, `run_target_builds.py`), executed on the CLEAN
committed tree with the same environment: central `static_audit.py`
(full host suite, motion G1..G5-A 20 gates, nested mutation suites),
identity-isolation, manifest-identity, router-facade, release-session,
DALY guard, migration M0 (+ pinned writer), portal DOM, LED extras,
110 calibration behavioural mutants, then isolated USB_ONLY and
ROBOT_POWERED target builds (OTA ingest 0, `SOURCE_DATE_EPOCH` = commit time).

A pre-commit dry run of the central gate on the uncommitted tree is recorded
in `01_offline_gates/dryrun-dirty-tree` (2026-10-06T18:44:01Z..18:56:21Z,
exit 0: `STATIC_AUDIT = PASS`, `HOST_TESTS = PASS`, `MOTION_CONVERGENCE = PASS`
20/20). It is a dry run of the same sources on a DIRTY tree, started before
this commit's documentation files were finalized; it is not the qualification.

Results of the committed-tree qualification, the frozen binary identities and
the hardware validation are NOT claimed by this revision of the document.
They are appended below by a later documentation-only commit, after they
have actually been observed.

## Flash evidence policy for dev.2

Operator decision 2026-10-06:

NO additional 16 MiB full-flash backup will be taken.

Rationale:
- a complete pre-dev.1 flash backup is already preserved;
- v0.2.0-dev.1 application exact-byte readback is preserved;
- partition table and otadata were already validated;
- dev.1 source and binary remain frozen;
- dev.2 changes application firmware only.

Before dev.2 flashing use only bounded read-only identity/layout checks,
then app-only write and exact application readback.

No partition rewrite.
No generic erase.
No NVS erase.
No matdog_nvs erase.

## Standing limits (unchanged by dev.2)

MOTION_AUTHORIZED = 0. RESTORE = NOT_IMPLEMENTED. LOAD is not RESTORE; a
valid persisted record is not a motion authorization; a mechanical contact
is not an operational JointLimit approval. No stand, walk, trot, gait,
stabilizer, generic motion, EEPROM write, ID recode, PositionOffset write,
OTA ingest or `@SYSTEM SHUTDOWN`.
