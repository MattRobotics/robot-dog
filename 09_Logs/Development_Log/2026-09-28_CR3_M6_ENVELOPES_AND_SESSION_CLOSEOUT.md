# MATDOG CR3 — Priority 5/6 gates and offline session closeout

Date: 2026-09-28
Commits: `5286359` (Priority 5), `6c5c4fd` (Priority 6)
Final HEAD: `6c5c4fdf5f2a1a2959e033b57590aae44170d8c7`

## CR3 Priority 5 — contact-probe engine (commit `5286359`)

`src/calibration/ContactProbeEngine.{h,cpp}`: the state machine for the eight
executable UPPER endpoints' physical contact measurements. Drives
approach/backoff through `CalibrationExecutionEngine`'s existing
`CONTACT_PROBE` intent (its endpoint-plan/parking/travel-guard checks apply
unmodified); TorqueEnable goes through `SafeActuatorPolicy`/`ActuatorRuntime`
directly, the same split `FirstMotionExecutor` uses. The contact signal is
`MotionDeadmanMonitor`'s `STALLED` verdict (sustained position stall under
continued torque and healthy comms) — the only hardware-agnostic obstruction
signal available without inventing an uncharacterized current/load
threshold. Two independent approaches must stall within a caller-supplied,
never-defaulted repeatability tolerance before `CalibrationDomain.h`'s
existing `ContactWitness`/`ContactEvidence` vocabulary is populated;
arriving at the boundary with no stall on either pass is explicitly **not**
contact evidence. `MotionDeadmanMonitor` gained `lastProgressPosition()` to
support this. 15 tests, including false-positive resolution, repeatability
failure, and the travel guard being enforced by the policy layer.

## CR3 Priority 6 — operational-envelope builder (commit `6c5c4fd`)

`src/actuator/OperationalEnvelope.{h,cpp}`: a deliberately **separate** type
from `ActuatorWritePolicy.h`'s `JointLimit`/`ActuatorLimitTable` (the live
`POSITION_COMMAND` runtime gate) — nothing here admits into that table or is
consulted by `SafeActuatorPolicy`. Two builders:

- `buildContactDerivedEnvelope()` (UPPER): fails closed unless both sides
  carry real, current, `CONTACT_CONFIRMED`, witness-accepted evidence.
  `ContactEvidence` (recovered from the LF V25 oracle) carries no
  `GeometryProvenanceTag`, unlike the CR3-era `JointTransform`/`JointLimit`;
  rather than widen that historical type, the geometry tag each side was
  captured under is a required caller-supplied field on the request struct.
- `buildGeometryDerivedEnvelope()` (HIP/LOWER, which have **zero** executable
  contact endpoints inside the URDF — all 16 are
  `DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS`): derives a bound strictly inside
  the URDF domain from a caller-supplied required stand/gait workspace and a
  caller-supplied safety margin, neither invented here. Verified against
  both URDF directions (a dedicated test uses `LH_HIP`, `direction=-1`, to
  prove the min/max-tick ordering is not assumed positive). Has no path to
  any endpoint record — only to `GeometryJointRecord`'s own
  `urdf_lower`/`urdf_upper` — so it structurally cannot reach a diagnostic
  endpoint.

24 tests covering the happy path in both directions and every fail-closed
rejection reason on both builders.

## Final offline validation (this closeout)

- Host suite: **26/26 binaries PASS, 121,273 checks, 0 failures**
  (`scripts/tests/run_host_tests.sh`).
- Static audit: **PASS** (`python3 scripts/static_audit.py`, 138 source files
  scanned).
- Mutation suites, all **PASS**: safe-actuator
  (`test_static_audit_safe_actuator.py`), DALY (`test_static_audit_daly.py`,
  52/52 cases), LED (`test_static_audit_led.py`, 88/88 cases).
- ESP32 firmware build: **PASS** (`scripts/build.sh`, FQBN
  `esp32:esp32:esp32s3:...`, `USB_ONLY` profile, no upload). 1,039,747 bytes
  (33%) program storage, 62,820 bytes (19%) dynamic memory — **unchanged**
  from the Priority 1 (M5) build despite Priority 3/4/5/6 adding ~2,700 lines
  across 8 new source files, confirming the linker's dead-code stripping
  removes all four modules from the shipped binary entirely (nothing in
  `Controller.cpp` references them) — a stronger guarantee than "logically
  unreachable": they are **physically absent** from the image that would be
  flashed.
- Hardware touched across the entire CR3 offline session: **no**. No serial
  port opened, no ESP32 reset/flash, no servo/BMS command issued,
  `hardware_motion_authorized` remains `0` (compile-time default, audited).

## CR3 offline session summary

8 commits on `feat/calibration-motion-full-cal-v1` since the `2603ee7` entry
point, covering: 3 pre-existing regression fixes, Priority 2 (uncertain
write safety), Priority 1/M5 (fail-closed production composition), Priority
3 (first-motion executor), Priority 4 (telemetry/deadman monitor), Priority
5 (contact-probe engine), Priority 6 (operational-envelope builder). See the
individual gate log entries earlier in this directory (same date) for
per-gate detail. See the HANDOFF delivered directly to Matteo for the
remaining software work, the exact first physical validation sequence, and
where explicit current-session authorization will be required.
