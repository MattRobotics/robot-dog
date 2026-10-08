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
the hardware validation were not claimed by the candidate revision of this
document (commit d0ede36). They are recorded in the section "Results" at the
end, added by a later documentation-only commit after they were observed.

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

## Results (observed 2026-10-06, added after the candidate commit)

Candidate: `d0ede365dbbdbf3940a31929e8e2e132faf9d49f`
(`bac652f` source/tests + `d0ede36` documentation), tree CLEAN, not pushed.
Everything below is bound to that commit; this documentation commit changes
no source, test or tool. Evidence paths are relative to the external
evidence root named at the top.

### Offline qualification of the committed tree — PASS

`01_offline_gates/qualification-d0ede365dbbd/`

| Gate | Result |
|---|---|
| Central `static_audit.py` (full host suite, nested mutation suites) | PASS — `STATIC_AUDIT = PASS`, `HOST_TESTS = PASS (full current owners and G1 through G5-A motion)` |
| Motion convergence G1..G5-A, fresh G4/G5 oracles, manifests, ASan/UBSan | PASS 20/20, `integration_head` = candidate |
| identity-isolation / manifest-identity / router-facade | PASS / PASS / PASS |
| release-session, DALY guard, migration M0, pinned migration writer | PASS |
| portal DOM, LED preserved harness, LED policy ASan/UBSan | PASS |
| Calibration behavioural mutations | PASS — 110/110 caught |
| Selective motion port manifest (330 files) against this tree | 330/330 identical |
| USB_ONLY and ROBOT_POWERED sandboxed builds (no network, synthetic /dev), manifest verify | PASS / PASS |

The central gate and the remaining gates ran concurrently (dev.1 ran them in
sequence); the 110-mutant suite used a private copy of the dev.1
content-addressed host compile cache. Compiler warnings: 2 per profile, both
in the third-party SCServo library, both pre-existing.

### Frozen artifacts

`02_target_builds/FROZEN_CANDIDATE.json`, `ARTIFACT_SHA256SUMS`

| Profile | Bytes | SHA256 |
|---|---|---|
| ROBOT_POWERED, OTA ingest 0 | 1,178,256 | `235e67be0079f97d1575a96d1aa99464f8ae6d2d07027e9992911183640733ce` |
| USB_ONLY, OTA ingest 0 | 1,174,368 | `1cc7aa7cd1ed96f241f5ec85f2c627be55c0a23643bd352c156c51ef11c57bdc` |

BUILD_UTC `2026-10-06T18:56:49Z` (SOURCE_DATE_EPOCH = commit time), layout
`MATDOG_16M_2x5M_NVS_V1`, partition table
`8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`, sdkconfig
`534f3457aaf3ffbc82c45dc21d6b3ac57388c82d0d9a24bd2b8e6cadc3e48d9d` (identical to
dev.1). USB_ONLY was built and frozen only; it was not flashed.

### Application-only flash — PASS

`03_flash/01_app_only_flash_20261006T192747Z/DEV2_APP_FLASH_RECEIPT.json`

ROM `--no-stub`, reviewed esptool 5.3.1, no full-flash backup (operator
decision). Bounded read-only preflight: ESP32-S3, MAC 14:c1:9f:22:75:94,
16 MiB, secure boot and flash encryption disabled; partition sector byte-identical
to the dev.1 capture; otadata unchanged, stable, resolving app0 @ 0x010000,
equal to the running `SOURCE_SIGNATURE` partition; installed application
exactly the dev.1 image (`9827fcee...`, 1,177,904 bytes). One write of the
frozen ROBOT_POWERED image at 0x010000. Exact read-back of 1,178,256 bytes:
SHA256 `235e67be...`, byte-identical to the candidate. Partition sector,
otadata, `nvs` and `matdog_nvs` byte-identical before and after
(`matdog_nvs` `71189f7f...`, `nvs` `37bef8ca...`, the same hashes as the
pre-dev.1 captures). Bootloader, partition table, otadata and both NVS
regions were never written.

### Reset, runtime identity and boot self-test — PASS

`04_runtime/boot-after-flash-20261006T192950Z.log`, `RUNTIME_IDENTITY_VERDICT.json`

The reset used `--before default-reset --after hard-reset` and returned to
the application without a physical power cycle (the dev.1 session used
`--before no-reset`, which leaves DTR asserted and never produces a reset).
Complete boot banner captured from the first byte:

```
MATDOG Controller 0.2.0-dev.2 / build d0ede365dbbd / ROBOT_POWERED / app0 @ 0x010000
startup_motion : DISABLED   startup_torque : DISABLED
startup_servo_census : ENABLED_READ_ONLY_INCREMENTAL
STARTUP_SERVO_CENSUS=RUNNING canonical=17 expected_now=13 range=11..55
SYSTEM_BOOT_COMPLETE health=BOOTING power_state=RUN
STARTUP_SERVO_CENSUS=PASS verdict=PASS expected=13 present=13 missing=0 absent_by_design=4 absent_present=0 unexpected=0 elapsed_ms=4012 max_ping_us=21091
SYSTEM health=READY ... mode=MAINTENANCE authority=NONE
SERVO_POP canonical=17 expected_now=13 absent_by_design=4 last_census=PASS
```

`SOURCE_SIGNATURE`: FW_VERSION 0.2.0-dev.2, full GIT_SHA, GIT_DIRTY=NO,
ROBOT_POWERED, layout, schemas 1/2, OTA_INGEST=0, MOTION_AUTHORIZED=0,
RESTORE=NOT_IMPLEMENTED, BUILD_UTC as frozen. All 22 identity/boot/status
checks PASS. The boot contract of dev.2 is observed on the real robot:
BOOTING while the census is pending, READY only after 13/13.
Reset reason reported `OTHER` (USB-Serial/JTAG reset); no brownout, no panic,
no second reset during the session.

### Read-only validation — PASS for what was run

`05_hw_validation/01_readonly_status`, `02_servo_readonly`, `05_bms_imu_readonly`

- Operator `@SERVO CENSUS`: PASS 13/13, 4 absent by design, 0 unexpected.
- `@SERVO PREFLIGHT`: PASS 12/12 (physical unit, bus id, model 777,
  position_offset 0, persistent profile MATCH, torque_enable 0).
- DALY: ONLINE, 11.7 V, about -0.4 A, SOC 81% -> 80%, cells 3893..3912 mV,
  both MOS on, four alarm words 0000; `@BMS KEY READ` OK: key_logic=DISCHARGE
  (0x005A), sleep 3600 s. The runner's DALY guard stayed PASS during motion.
- BNO085: PASS, runtime_resets=0; counter deltas over 2.52 s give
  rotation/game/magnetic vector 50 Hz, accel/gyro about 10 Hz.
- LED: presentation READY, SOC valid, not charging.
- Wi-Fi INACTIVE (NO_CREDENTIALS), web server and TLS not started, OTA ingest
  disabled. No provisioning was attempted (standard NVS is not to be written).
- matdog_nvs: NO_RECORD / NEVER_INITIALIZED_OR_ERASED, MOTION_AUTHORIZED=0.

### Calibration — STOPPED at LF, fail closed

`05_hw_validation/03_calibration/`, `FAILURE_STOP_RECEIPT.md`

| Step | Result |
|---|---|
| Runner `prepare` (manifest, signature, MAINTENANCE, DALY guard, SAFE_OFF 13/13, READY, authority NONE) | PASS |
| Fresh Q0 capture 9/9, 12/12 candidates, spread 0, max abs delta vs CR2-C = 19 (stop at 82) | PASS |
| Q0 promote 12/12, geometry `3713f4ddc43b204e` (same as the 2026-10-01 24/24 run) | PASS |
| R2 reach, LF UPPER MIN, q0 2096: endpoint reach 1453, margin +15 vs the 1468 reference stop | PASS |
| LF session + permit, INITIAL RECOVERY 12/12, SAFE_OFF verified | PASS |
| LF ARMED plan vs the 2026-10-01 validated plan (probe signs, corridors, poses) | PASS |
| LF UPPER MIN contact: scout 1451, fine 1455 / 1454, witness accepted | measured (1 of 6) |
| LF UPPER MAX | FAILED: `UPPER_MAX_PROBE_FAILED`, `probe_failure=OVER_TEMPERATURE` |
| RF, RH, LH, export 24/24, SAVE CHECK, SAVE, ACK, persistence after reset | NOT_RUN |

Q0 (bus:tick): 11:2102 12:2096 13:1983 21:1971 22:2096 23:2030 31:2034 32:2061
33:2087 41:2092 42:2088 43:2027.

The over-temperature verdict is the repeated-anomaly latch of the thermal
confirmation, not heat: four block-read temperature samples of the moving
servo (76 on bus 42; 95, 78, 77 on bus 12) were each refuted by three direct
reads of 32..34 degC; the third transient on bus 12 inside 30 s latched
`REPEATED_ANOMALY` (published 71 > limit 70). All 13 servos read 32..35 degC
afterwards. The same block-read artifact is present in the earlier hardware
logs (23 transients in the 2026-10-01 PASS run, 19 on 2026-10-03); the
3-in-30 s per-servo latch arrived with the post-abort thermal rework and is
unchanged between dev.1 and dev.2. dev.2 touches no servo read or thermal
code. The firmware failed closed as specified. Nothing was weakened,
bypassed or retried.

SAFE_OFF 13/13 VERIFIED_OFF (runner de-escalation and an independent
re-check), torque=0 on all 13 read-backs, authority NONE, permit revoked, no
live session, MOTION_AUTHORIZED=0, matdog_nvs untouched. The controller was
not reset after the stop.

### Residuals

- R-1 (blocking for calibration): block-read PresentTemperature artifact on
  the moving servo versus the 3-in-30 s latch. Needs central review of the
  thermal trigger and a bus characterization (master runbook gate 7, audit
  H-1/H-2). Not a dev.2 regression; not corrected here.
- R-2: the robot is not in the q=0 pose. LF_UPPER stayed at raw 3423 and
  LH_UPPER at the rear park 2482, torque off. Bus 12 is latched until reboot,
  so no firmware recovery can pass in this boot; returning to q=0 is an
  operator physical action.
- R-3: LF UPPER MIN was found at 1454/1455, 1..2 ticks inside the endpoint
  reach (1453) for q0 2096. The reach margin of that contact is thin.
- R-4: persistence (SAVE/ACK/LOAD) was not exercised; a genuine power cycle
  was not performed.
- R-5: gates needing fixtures or operator action were not run: UART/thermal
  timing, charging LED presentation, Wi-Fi provisioning/portal/roaming,
  TLS/HMAC probes, heap/jitter instrumentation.
- R-6: under USB_ONLY the boot banner also prints
  `startup_servo_census : ENABLED_READ_ONLY_INCREMENTAL` although no census is
  armed in that profile. Cosmetic, USB_ONLY was not flashed; to correct in the
  next revision together with its audit token.
- R-7: `calibration_hw_session.py` leaves `@BMS STREAM ON` after a failed
  phase (its emergency stop drops the DALY guard before the final STREAM OFF).
  Turned off by hand at the end of this session.
- R-8: `elapsed_ms=4012` is measured from arming inside `begin()`. About
  3.0 s of it is the rest of `begin()` (census armed 19:29:52.73Z,
  `SYSTEM_BOOT_COMPLETE` 19:29:55.76Z) before the first `update()` tick; the
  45 Pings then take about 1 s. READY is reached about 5.6 s after reset.
