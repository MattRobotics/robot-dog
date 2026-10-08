# MATDOG v0.2.0-dev.3 — Thermal Verdict / Post-Abort Recovery Delta Audit
Date: 2026-10-06
Base: d0ede365dbbdbf3940a31929e8e2e132faf9d49f (v0.2.0-dev.2, hardware-qualified
boot self-test, preserved; its results are in the dev.2 delta audit, d67eaa1)
Branch: fix/matdog-v0.2.0-dev2-boot-selftest
External evidence root:
`~/MATDOG/verification-artifacts/MATDOG_V0_2_DEV3_THERMAL_20261006`

dev.2 and all its evidence stay as they are. This document is additive.

## Thermal root cause (hardware evidence)

2026-10-06, firmware d0ede36, LF UPPER MAX, bus 12:

| block-read value | direct reads 50 ms apart | decision in dev.2 |
|---|---|---|
| 95 | 32, 32, 32 | TRANSIENT |
| 78 | 32, 33, 32 | TRANSIENT |
| 77 | 34, 32, 32 | REPEATED_ANOMALY, published 71, OVER_TEMPERATURE, SAFE_OFF |

All 13 servos read 32..35 C afterwards. The same artifact is in every earlier
hardware log: 23 refuted samples in the 2026-10-01 24/24 PASS run, 18 plus one
confirmed-by-artifact (95, 117, 34) on 2026-10-03.

`ServoBus` reads go through `ValidatedServoRead` (bounded RX drain, exact
id/length/status/checksum, bounded timeout). The spurious values arrive in
frames that pass that validation: the servo itself returns them in the 15-byte
block read of registers 56..70 while it is moving. The byte is therefore not a
usable safety temperature during motion. The direct single-register read uses
the same validated transaction and is the value the servo reports when asked
for the temperature alone.

## Policy change (operator design decision 2026-10-06)

`src/calibration/ThermalConfirmation.{h,cpp}`:

| | dev.2 | dev.3 |
|---|---|---|
| Limit | 70 C | 70 C (unchanged) |
| Block-read value over the limit | counted as one over-limit sample | opens a confirmation, nothing else |
| CONFIRMED (over-temperature) | 3 over-limit samples including the block read | 3 DIRECT samples over the limit |
| Refuted block read | TRANSIENT, counted toward a latch | `BULK_TEMP_ARTIFACT_SUSPECT`, diagnostic counter only |
| 3 refuted in 30 s, or 8 per boot | latch until reboot, publish 71 | removed |
| Direct read fails / invalid | CONFIRMATION_READ_FAILED, publishes the block-read value | `THERMAL_TELEMETRY_FAULT`, publishes limit + 1 |
| Direct samples 2 over / 2 under | CONFIRMED | `THERMAL_TELEMETRY_FAULT` (incoherent) |
| Confirmation older than 300 ms, or for another servo | fail closed | fail closed (`THERMAL_TELEMETRY_FAULT`) |
| Sample spacing, pause of motion steps while pending | 50 ms, `monitorOnly` | unchanged |

Every direct-sample sequence that aborted in dev.2 still aborts in dev.3
(exhaustively: two direct over-limit samples before three normal ones end in
CONFIRMED or THERMAL_TELEMETRY_FAULT). The only sequences that no longer abort
are the ones the latch added: repeated refuted block-read values. A single
direct sample at or under 70 C never ends a confirmation early: three are
still required.

Controller change: one log field (`bulk_artifacts=`) on the existing
`CALIBRATION_THERMAL_CONFIRMATION` record, whose `samples=` still list the raw
block-read value first and the direct reads after it. No change to the
telemetry loop, the one-direct-read-per-tick budget, or `monitorOnly`.

Not changed: 70 C limit, hard-current abort, contact detection, corridors,
torque limit 500, speeds, accelerations, geometry, sequence plan, deadman,
SAFE_OFF path, any other read.

Known label: at executor level both abort verdicts still surface as
`probe_failure=OVER_TEMPERATURE` (the executor sees a temperature over the
limit). The `CALIBRATION_THERMAL_CONFIRMATION ... decision=` record printed in
the same tick carries the exact cause.

Not added: unconditional periodic direct sampling. The block-read value
remains the trigger of the direct confirmation, as in every hardware-validated
build; steady-state bus traffic is unchanged (no direct read at all in a run
without an over-limit block-read value). Detection coverage is the one of
dev.1/dev.2; the decision is direct-only.

## Post-abort recovery policy (runner)

`scripts/calibration_hw_session.py`, failure path of `legs` / `resume` / `all`:
SAFE_OFF 13/13, evidence export, then at most one automatic start of the
firmware's existing `@CALIBRATION POST_ABORT RECOVERY <leg>` when all hold:

- the firmware's own `PROBE_FINAL` record names an allow-listed search verdict
  (NO_CONTACT_BEFORE_GUARD, EARLY_STALL_OUTSIDE_CORRIDOR, TRACKING_FAILED,
  REPEATABILITY_FAILED, INSUFFICIENT_BASELINE, BACKOFF_CROSSES_HOME,
  BASELINE_PASSES_GUARD, SCOUT_MISSING) in UPPER_MIN/MAX or LOWER_MIN/MAX;
- no thermal verdict other than a refuted block read was printed in the run;
- 13/13 VERIFIED_OFF, export saved, authority NONE, link alive;
- the promoted q0 is the checkpointed acquisition of this boot and the export
  geometry equals the promoted geometry;
- fresh `@SERVO CENSUS` PASS 13/13 and `@SERVO PREFLIGHT` PASS 12/12;
- 13 direct reads: temperature 0..70 C, torque off;
- with `--require-daly`, a fresh DALY guard.

Then: session + permit, firmware recovery (one joint at a time, INITIAL
RECOVERY parameters, firmware gates unchanged), SAFE_OFF 13/13, 13 direct
reads within 10 ticks of q0 and torque off. A refusal or an unverified end is
de-escalated again and never retried. Never after: over-temperature, thermal
telemetry fault, hard current, stale/lost communication, torque/goal/limit
readback anomalies, servo status fault, refused/uncertain commands, motion
timeout, operator abort, an interruption, or any verdict not on the list.

This is same-boot only: the firmware recovery needs its RAM witness.

## Recovery across a reboot: not implemented, not attempted

The robot stands with LF_UPPER at raw 3423 (q0 2096) and LH_UPPER at 2482
(q0 2088) after the dev.2 stop. Flashing dev.3 reboots the controller and
discards the RAM q0 and witness. No command of the existing surface can move
those joints to the preserved q0:

- INITIAL RECOVERY refuses beyond 64 ticks from a q0 promoted in this boot;
- POST_ABORT RECOVERY refuses without the same-boot witness;
- STARTUP RECOVERY is one compiled route (RF, buses 21/22/32, reference
  2026-10-03) enforced in `ActuatorWritePolicy`, `CalibrationExecutionEngine`,
  the executor, the qualification and the static-audit certificate.

A host-only path through the existing command surface therefore does not
exist, and a new route means new motion authorization in the Safe Actuator
layer with its own clearance certificate. That was not written under time
pressure and is not claimed safe. The preserved session data that such a
route would need (12 q0 values, geometry, units, directions, hashes) is in
`MATDOG_V0_2_DEV2_BOOT_SELFTEST_20261006/07_pre_dev3_preservation`.

## Tests

- `test_thermal_confirmation.cpp`: A, B (the 2026-10-06 records, no fault),
  C (ten refuted spikes inside 30 s, and forty spread over a long run: never a
  verdict; a real overheat after them is still confirmed), D (three direct
  over-limit samples), E (direct read failed, negative, over 255, absent,
  port missing: fail closed over the limit), F (normal values, no direct
  read), G (70 / 5 / 3 / 3 / 50 ms pinned), incoherent 2/2, wrong servo,
  deadline.
- `test_full_leg_calibration_executor.cpp`: the real sequence with ten
  block-read spikes on the probed bus 12 inside 30 s ends 6/6 with exactly 30
  direct reads; a real overheat and a failed confirmation still abort; a run
  without spikes still issues no direct read.
- Behavioural mutants (compile and run, 113 total, 12 thermal): block read
  counted as direct evidence, refuted spikes latch again, over-temperature
  from the block read alone, incoherent samples cleared, clearing majority 2,
  fault published as safe, plus the retained ones.
- Static audit: direct-only tally, fault publication, incoherent branch and
  clearing majority pinned; any latch/threshold on refuted block reads is
  refused; six new audit mutations.
- `test_calibration_hw_session.py`: 8 new tests (recovery order and single
  attempt, resume after it, 15 non-recoverable verdicts, thermal verdicts and
  unsupported phases, six failed preconditions, refused/unverified recovery,
  opt-out, BMS stream off).
- H: no file under `src/` other than `ThermalConfirmation.{h,cpp}`,
  `Controller.cpp` (one log line) and `BuildConfig.h` (version) changed.

Qualification of the committed tree, binary identities, flash and hardware
results were not claimed by the candidate revision (b3fd945); they are in the
section "Results" at the end, added after they were observed.

## Standing limits

MOTION_AUTHORIZED = 0. RESTORE = NOT_IMPLEMENTED. No stand, walk, trot, gait,
stabilizer, EEPROM write, PositionOffset write, ID recode, OTA ingest.

## Results (observed 2026-10-06, added after the candidate commit)

Candidate: `b3fd945bdaf37d192d97b605ac0f59b67f1dba45` (62812cd thermal,
2f23247 runner, b3fd945 documentation), tree CLEAN, not pushed. This
documentation commit changes no source, test or tool. Paths are relative to
the dev.3 evidence root unless stated.

### Offline qualification of the committed tree — PASS

`01_offline_gates/qualification-b3fd945bdaf3/`

| Gate | Result |
|---|---|
| Central `static_audit.py` (host suite, nested mutation suites, runner suite) | PASS — `STATIC_AUDIT = PASS`, `HOST_TESTS = PASS` |
| Motion convergence G1..G5-A | PASS 20/20, `integration_head` = candidate |
| identity-isolation, manifest-identity, router-facade, release-session, DALY guard, migration M0 + pinned writer, portal DOM, LED extras | PASS |
| Calibration behavioural mutations | PASS — 113/113 caught (12 thermal) |
| Selective motion port manifest | 330/330 identical |
| USB_ONLY and ROBOT_POWERED sandboxed builds, manifest verify | PASS / PASS |

Central gate, remaining gates and builds ran concurrently on the same CLEAN
commit. Source delta against dev.2 under `src/`: `ThermalConfirmation.{h,cpp}`,
one log line in `Controller.cpp`, the version literal (`SRC_DELTA_VS_DEV2.txt`).

### Frozen artifacts

`02_target_builds/FROZEN_CANDIDATE.json`

| Profile | Bytes | SHA256 |
|---|---|---|
| ROBOT_POWERED, OTA ingest 0 | 1,178,272 | `5e2cbd17771712cc4e2ae10863edb33b59119754debdd5283551a1bb56f2c4be` |
| USB_ONLY, OTA ingest 0 | 1,174,384 | `e36d02a8262ae35c98f71029e76e09d1b80615cc842ea91047a62d49644179c2` |

BUILD_UTC `2026-10-06T20:22:48Z`, layout and partition table unchanged,
sdkconfig `534f3457...` (identical to dev.1 and dev.2). USB_ONLY not flashed.

### Preservation before reset/flash

`MATDOG_V0_2_DEV2_BOOT_SELFTEST_20261006/07_pre_dev3_preservation/`
(`PRESERVED_SESSION_MANIFEST.json`, `SHA256SUMS`): the dev.2 promoted q0 read
back from the running controller equals the checkpoint
(`q0_promoted.json` SHA256 `45d38a3c...`), geometry `3713f4ddc43b204e`, units,
directions, the failure export and the stop receipt.

### Application-only flash, reset, identity, boot self-test — PASS

`03_flash/01_app_only_flash_20261006T205943Z/DEV3_APP_FLASH_RECEIPT.json`,
`04_runtime/`

ROM `--no-stub`, no full-flash backup. Preflight: device identity, partition
sector and otadata identical to the dev.2 captures, running slot app0 @
0x010000, installed application exactly dev.2 (`235e67be...`). One write of the
frozen image; exact read-back of 1,178,272 bytes `5e2cbd17...`, byte-identical
(the read-back file keeps the tool's `dev2-app-exact-readback.bin` name).
Partition sector, otadata, `nvs`, `matdog_nvs` unchanged by the flash.
Boot: `MATDOG Controller 0.2.0-dev.3`, build `b3fd945bdaf3`,
`STARTUP_SERVO_CENSUS=PASS` 13/13 (4 absent by design, 0 unexpected,
elapsed 4023 ms), `SYSTEM health=READY`; runtime identity 22/22 checks PASS.

### q=0 pose after the dev.2 stop — restored by hand, verified, not a firmware recovery

`03_flash/00_pre_state/EXTERNAL_Q0_REPOSITIONING_OBSERVATION.md`,
`05_hw_validation/01_pose_vs_preserved_q0/Q0_POSE_VERIFICATION.json`

Between 20:05Z and 20:49Z, still under dev.2, buses 12 and 42 went from raw
3423 / 2482 to 2095 / 2092 with torque off on all 13, the actuator counters
unchanged (194 plans / 194 commits), authority never granted again and no
reset: they were moved by hand. No recovery motion was commanded by this
session and none exists for that case. After the dev.3 boot the pose was
verified by direct reads: 12/12 within 4 ticks of the preserved q0 (tolerance
10), torque off 13/13, temperatures 32..35 C, census 13/13, preflight 12/12.
`Q0_RECOVERY=PASS` is NOT claimed.

### Fresh Q0, calibration 24/24 — PASS

`05_hw_validation/03_calibration/`

- Runner `prepare`: manifest, signature, MAINTENANCE, DALY guard, SAFE_OFF
  13/13, READY, authority NONE.
- Fresh Q0 under dev.3: 9/9, 12/12, spread 0, max abs delta vs CR2-C 19;
  promoted 12/12, geometry `3713f4ddc43b204e`. Bus:tick 11:2102 12:2095 13:1983
  21:1971 22:2096 23:2030 31:2034 32:2061 33:2087 41:2092 42:2092 43:2028.
  R2 margin of LF UPPER MIN vs the 1468 reference stop: +16.
- INITIAL RECOVERY 12/12, then LF -> RF -> RH -> LH, 21:03:42Z..21:12:15Z:

| Leg | Verdict | Contacts | Seconds | ARMED plan vs 2026-10-01 |
|---|---|---|---|---|
| LF | HARDWARE_CONTACT_CALIBRATED | 6/6 | 130 | PASS |
| RF | HARDWARE_CONTACT_CALIBRATED | 6/6 | 132 | PASS |
| RH | HARDWARE_CONTACT_CALIBRATED | 6/6 | 115 | PASS |
| LH | HARDWARE_CONTACT_CALIBRATED | 6/6 | 115 | PASS |

`CALIBRATION_EVIDENCE_EXPORT=END legs_present=4 legs_contact_calibrated=4
legs_envelope_accepted=0 total_contacts_expected=24 total_contacts_accepted=24
all_contact_calibrated=1`. SAFE_OFF 13/13 verified after every leg and at the
end. `envelope_accepted=0`, `parameters_approved=0`: these are mechanical
contacts, not operational joint limits.

Fine contacts (fine1/fine2, raw):

| | UPPER MIN | UPPER MAX | LOWER MIN | LOWER MAX | HIP MIN | HIP MAX |
|---|---|---|---|---|---|---|
| LF | 1455/1453 | 3478/3476 | 3149/3149 | 1712/1712 | 2507/2507 | 1466/1464 |
| RF | 2730/2726 | 721/722 | 937/936 | 2355/2355 | 2546/2547 | 1498/1497 |
| RH | 2667/2665 | 659/659 | 992/991 | 2426/2424 | 1565/1564 | 2595/2595 |
| LH | 1481/1483 | 3484/3485 | 3133/3134 | 1702/1705 | 1498/1499 | 2542/2543 |

LF UPPER MIN absolute midpoint 1454 vs the 2026-10-01 reference 1461: -7
(budget +/-16).

### Thermal policy on the robot

`05_hw_validation/03_calibration/THERMAL_SUMMARY.json`

38 block-read values over the limit (71..150) in the run, on 11 servos, every
one refuted by direct reads and reported as `BULK_TEMP_ARTIFACT_SUSPECT`;
0 CONFIRMED, 0 THERMAL_TELEMETRY_FAULT. 115 direct confirmation reads. Up to
5 refuted values inside 30 s on one servo (buses 11 and 12), 3 or more on
seven servos: dev.2 would have latched on each of those seven.

One direct read was itself spurious: bus 32, block read 125, direct 32, 32,
86, 32. Three of four direct samples were normal, so it cleared; the direct
majority, not a single direct sample, is what decides.

The automatic post-abort recovery did not run: no leg failed.

### Persistence — PASS up to a reset; real power cycle not done

`05_hw_validation/04_persistence/`, `05_nvs_regions_after_ack/`, `04_runtime/`

| Step | Result |
|---|---|
| Export 24/24 re-verified, SAFE_OFF 13/13, authority NONE | PASS |
| SAVE CHECK | `CHECK_OK`, persistence status unchanged |
| SAVE | `WRITTEN_AWAITING_ACK generation=1 slot=A` |
| ACK 2 (wrong generation) | `WRONG_GENERATION`, persistence facts unchanged |
| ACK 1 | `OK`; `VALID_ACKNOWLEDGED`, record intact, `CALIBRATION_AVAILABLE=1`, `MOTION_AUTHORIZED=0` |
| Raw regions after ACK (ROM, read-only) | `matdog_nvs` `71189f7f...` -> `6d03b9dd804f470b26f0516b4093e733112b7bf58eb27989162798e6a00d3edd`; `nvs`, otadata, partition sector byte-identical |
| Reset (esptool run-mode reset), boot | `CALIBRATION_PERSISTENCE_BOOT nvs=READY verdict=VALID_ACKNOWLEDGED available=1 motion_authorized=0`; generation 1 intact |
| After the reset | `transforms_admitted=0`, authority NONE, no permit, `hardware_motion=BLOCKED`, `RESTORE=NOT_IMPLEMENTED` |
| Runner `verify-persistence` (read-only) | PASS, generation 1 |

LOAD is not RESTORE: nothing from storage was admitted to the transform
table. A genuine operator power cycle (master runbook gate 13) was not
performed; the reset reason is `OTHER`, not `POWERON`.

### Final state

SAFE_OFF 13/13 VERIFIED_OFF, torque off on 13 direct reads, authority NONE,
`SYSTEM health=READY`, MAINTENANCE, `MOTION_AUTHORIZED=0`, joints within a few
ticks of q0, servos 32..35 C, DALY 11.5 V / SOC 72 % / alarms 0, web server,
TLS and OTA ingest not started.

### Residuals

- T-1: the block-read artifact itself is not explained (servo-side, in valid
  frames). dev.3 makes it harmless; it does not remove it.
- T-2: a direct read can also be spurious (1 of 115 here, 1 on 2026-10-03).
  Two in one confirmation with two normal ones is `THERMAL_TELEMETRY_FAULT`:
  a fail-closed abort, improbable, not impossible.
- T-3: both abort verdicts still read `probe_failure=OVER_TEMPERATURE` at
  executor level; the decision record carries the cause.
- T-4: no unconditional periodic direct temperature sampling was added; the
  block-read value remains the trigger of the direct confirmation.
- P-1: no recovery to q0 across a reboot exists. After the dev.2 stop the pose
  was restored by hand. A firmware route would be a new certified motion
  program in the startup-recovery owner.
- P-2: the automatic post-abort recovery is covered by the offline runner
  suite only; it has not run on the robot.
- C-1: LF UPPER MIN was found at 1455/1453 against an endpoint reach of 1452
  for q0 2095: 1..3 ticks of margin.
- V-1: real power cycle not performed; fixture/operator gates not run
  (UART/thermal timing, charging LED, Wi-Fi/portal/roaming, TLS probes,
  heap/jitter instrumentation).
- The dev.2 residuals R-6 (USB_ONLY banner wording) and R-8 are unchanged.
