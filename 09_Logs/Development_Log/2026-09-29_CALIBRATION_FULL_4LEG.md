# MATDOG — Full Calibration generalized to four legs (LF → RF → RH → LH, one build)

Date: 2026-09-29
Branch: `feat/calibration-full-4leg-v1`, based on main `4b6cf0e` (PR #31)
Status: OFFLINE-VALIDATED. No flash, no servo command, no hardware access in this work.

## Why

PR #31 shipped a Full Leg calibration that was LF-only in three places: the SESSION START
command, the FULL LEG command (bus 12, LH_UPPER as the mandatory auxiliary, a hard-coded
`-700000` re-approach point) and the parked-endpoint context in `Controller`. The evening
hardware session has to calibrate all four legs from ONE `ROBOT_POWERED` build, so the leg had
to become the only input and everything else had to be derived.

## What changed (existing engines reused; no new architecture)

| Piece | Change |
|---|---|
| `FullLegCalibrationExecutor` | `auxiliary_required` flag; a NOT_NEEDED MAX side skips `AUX_*`; FINAL_SAFE_OFF services the primary only, plus the auxiliary only where one was parked; never `safeOff(0)`. `endpointLeg()/endpointJoint()` accessors. |
| `FullLegCalibrationPlan` (new, pure) | `resolveFullLegPlan(profile, provenance, transforms, leg)`: canonical allocation → `JointIdentity` → Geometry V5 `bus_id` cross-check; MIN/MAX from `findEndpoint(leg, UPPER, side)`; auxiliary + park target from the MAX endpoint; the re-approach point from one geometry-verified rule (the LF `-700000` residue is gone). |
| `FullLegCalibrationFinalizer` (new, pure) | contacts recorded once each → UPPER/HIP/LOWER envelopes → limit validate/admit (approved parameters only) → `noteExecutionPhase(TORQUE_OFF)` → `completeSession()` → permit/authorization revoked → post-conditions (session terminal, authority NONE, permit off). Any refusal fails the session and still cleans up. RAM `FullLegEvidenceStore` (4 records) and deterministic `exportFullLegEvidence()`. |
| `SafeActuatorPolicy` | `validateOperationalLimit` / `admitOperationalLimit` (provenance-checked; transform looked up under `currentGeometryTag()`). |
| `CommandRouter` | one strict `matchLegCommand()` table (LF/RF/RH/LH) for `SESSION START` and `FULL LEG`; between-leg gates; session-leg cross-check; plan-driven FULL LEG; DIRECTION_VERIFY stays exact, LF-only; `@CALIBRATION EVIDENCE EXPORT` paced with `availableForWrite()` because the USB CDC TX ring is 3072 B with a 0 ms timeout. |
| `Controller` | parked-endpoint context follows the running request; `updateFullLegFinalization()` finalizes every terminal executor exactly once (also aborted/session-lost runs). |

## Parking matrix (host-tested against the compiled Geometry V5)

LF → LH_UPPER, RF → RH_UPPER, RH → none, LH → none. UPPER MIN needs no auxiliary on any leg.

## HARDWARE_CONTACT_CALIBRATED vs FINAL_OPERATIONAL_ENVELOPE_ACCEPTED

No approved stand/gait workspace exists (`first_stand_limit_rad` is null for all 12 joints), so
the envelope margins (UPPER 8 ticks, HIP/LOWER 50000 µrad over the full URDF domain) are
placeholders: `kFullLegOperationalParametersApproved = false`, audit-pinned. A hardware run
therefore ends at `HARDWARE_CONTACT_CALIBRATED` (validated UPPER MIN/MAX contacts, recorded
evidence, SAFE_OFF, session COMPLETED, authority NONE). Placeholder envelopes are reported and
never offered to the policy: **0 of 12 JointLimits are admitted** by a hardware run. Flipping the
flag is a deliberate, reviewed source change once an approved source exists.

## Verification (offline)

- Host suite: all binaries 0 failures (executor 695, plan 550, finalizer 2616 checks among them).
  Mandatory matrix covered: aux matrix; no-aux skips AUX_*; SAFE_OFF with 2/1 servos; canonical
  identity for buses 12/22/32/42 and HIP/LOWER; dynamic endpoint lookup; session-leg mismatch
  rejected; two contacts recorded once; envelopes READY/REFUSED; JointLimit admitted only with
  current provenance; `completeSession()` only after the full lifecycle; permit/authority cleanup
  after every leg; four legs sequential; one leg's failure does not falsify the others;
  deterministic four-leg export; earlier contention/abort invariants unchanged.
- Static audit PASS. New `check_full_leg_calibration_wiring` pins the strict four-token
  matcher, per-branch absence of leg/bus/backoff literals and parsers, one `start()`, arm-after-
  start, Controller finalization wiring and ordering, the finalizer's lifecycle order, the
  unapproved production parameters and the confinement of `admitOperationalLimit(`. The old exact
  `SESSION START LF` pin became the four-leg pin; nothing was weakened. Mutation suites (safe-
  actuator with 33 new cases, DALY, LED) PASS.
- USB_ONLY build and ROBOT_POWERED compile: exit 0, no MATDOG warnings.

## Not done / not claimed

- Never run on hardware. The q0 cross-check against CR2-C and the SAFE_OFF confirmations are
  operator steps in `FULL_CALIBRATION_4LEG_HARDWARE_RUNBOOK.md`; there is no approved q0
  repeatability threshold in the repository.
- No EEPROM/NVS/PositionOffset write exists in this path; all records are RAM and are lost at reset.
- The GAIT workstream (`feat/gait-engine-offline-v1`) is suspended and was not touched.
