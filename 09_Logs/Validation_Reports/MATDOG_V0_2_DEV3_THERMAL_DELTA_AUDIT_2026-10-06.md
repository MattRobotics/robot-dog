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
results are not claimed by this revision; they are appended after they are
observed.

## Standing limits

MOTION_AUTHORIZED = 0. RESTORE = NOT_IMPLEMENTED. No stand, walk, trot, gait,
stabilizer, EEPROM write, PositionOffset write, ID recode, OTA ingest.
