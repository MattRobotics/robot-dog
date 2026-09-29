# Full Calibration — four-leg hardware runbook (LF → RF → RH → LH, one build, one session)

Date: 2026-09-29
Status: PROCEDURE — OFFLINE-VALIDATED, NEVER RUN ON HARDWARE
Firmware: ONE `ROBOT_POWERED` build from clean merged `main` (manifest `SOURCE_STATE=CLEAN`).

This document is a procedure, not an authorization. Building, flashing, energizing the servo
rail, every `@CALIBRATION` command below and every servo motion need the operator's explicit
go-ahead for that session. Nothing here writes EEPROM/NVS, `PositionOffset`, servo IDs or DALY
configuration; everything the run produces lives in RAM and is lost at reset — **export the
evidence before any power cycle.**

## What one build does

| Command (each exact, four-token) | Effect |
|---|---|
| `@CALIBRATION SESSION START <LF\|RF\|RH\|LH> CONFIRM_CURRENT_Q0` | live session for that leg from the current-boot q0 population evidence; no bus traffic |
| `@CALIBRATION MOTION PERMIT GRANT 16 CONFIRM_FIRST_MOTION` | fresh RAM permit for the live session (same command for every leg) |
| `@CALIBRATION FULL LEG <LF\|RF\|RH\|LH> CONFIRM_FULL_CALIBRATION` | UPPER MIN contact, (aux park), UPPER MAX contact, SAFE_OFF; then the evidence lifecycle |
| `@CALIBRATION FULL LEG STATUS` / `ABORT` | poll / abort the running sequence |
| `@CALIBRATION EVIDENCE EXPORT` | deterministic key=value dump of the RAM record of all four legs |
| `@CALIBRATION MOTION DIRECTION_VERIFY LF_UPPER +16 CONFIRM_FIRST_MOTION` | the first bounded diagnostic; **LF session only, once** |

The leg is the only free token. Bus ids, joint identities, MIN/MAX endpoints, the auxiliary
decision and its park target are derived by the firmware: canonical allocation → `JointIdentity`
→ cross-checked against Geometry V5 (`bus_id`), endpoints from `findEndpoint(leg, UPPER, side)`.

### Parking matrix (derived from the Geometry V5 MAX endpoint; pinned by host test)

| Leg | UPPER bus | MAX side needs | Parked auxiliary |
|---|---|---|---|
| LF | 12 | auxiliary | `LH_UPPER` (bus 42) → park 35.0° (610865 µrad), clear 1278983 µrad |
| RF | 22 | auxiliary | `RH_UPPER` (bus 32) → park 35.0° |
| RH | 32 | NOT_NEEDED | none |
| LH | 42 | NOT_NEEDED | none |

UPPER MIN needs no auxiliary on any leg. SAFE_OFF after a leg covers the primary, plus the
auxiliary only where one was used (never a `safeOff(0)`).

### Two levels of "calibrated" — never conflated

- **`HARDWARE_CONTACT_CALIBRATED`** — both UPPER contacts witnessed and recorded once each in the
  session, UPPER envelope READY, servos SAFE_OFF, session COMPLETED, permit revoked, authority NONE.
  **This is what a hardware run ends at today, and it is a PASS for this evening.**
- **`FINAL_OPERATIONAL_ENVELOPE_ACCEPTED`** — additionally UPPER/HIP/LOWER envelopes from
  *approved* parameters and all three JointLimits admitted and read back. It needs a reviewed
  stand/gait workspace and margin. None exists (`first_stand_limit_rad` is null for all twelve
  joints), so `kFullLegOperationalParametersApproved` is `false` (audit-pinned), placeholder
  envelopes (UPPER 8 ticks, HIP/LOWER 50000 µrad) are **reported but never admitted** to the
  actuator policy. **Expect `envelope_accepted=0`, `parameters_approved=0`, every `admission=
  NOT_ADMITTED_UNAPPROVED_PARAMETERS`, and 0 of 12 JointLimits admitted. That is correct, not a
  failure.** Do not treat a missing FINAL level as a reason to retry.

## Preconditions (before power on)

- Clean-merged-`main` `ROBOT_POWERED` binary, its manifest kept with the evening's evidence.
- Robot supported so every leg can sweep its full UPPER range and both rear UPPER joints can
  park without touching the ground or the frame; rail power switch within reach.
- Legs at the same nominal q0 pose used for CR2-C (`CR2C_Q0_HARDWARE_READONLY_RUNBOOK.md`).
- Serial log captured to a file for the whole session.

## Sequence

### 0. Flash and identity

1. Application-only flash of the exact manifest binary, verified digest (as in the LED V2 / CR2-C
   sessions). No EEPROM/NVS/bootloader change.
2. `@SYSTEM SOURCE_SIGNATURE` → the source SHA equals the merged `main` SHA in the manifest,
   `HARDWARE_PROFILE=ROBOT_POWERED`, no `-dirty`.
3. `@MODE MAINTENANCE`; `@MODE STATUS` → `MODE=MAINTENANCE`.
4. `@SERVO SAFE_OFF <id>` for **all 13 installed servos** — 11 12 13 21 22 23 31 32 33 41 42 43
   51 — each `SERVO_SAFE_OFF id=<id> result=VERIFIED_OFF`.

### 1. Fresh q0, 12/12, then promotion

1. `@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE`; wait for the unsolicited `CALIBRATION_Q0
   state=COMPLETE … candidates=12/12` and `CALIBRATION_Q0_POPULATION … observed=12/12`; no other
   servo command while it runs.
2. Compare every fresh q0 with CR2-C:

   | leg | LOWER | UPPER | HIP |
   |---|---|---|---|
   | LF | 11 = 2087 | 12 = 2100 | 13 = 1996 |
   | RF | 21 = 1985 | 22 = 2092 | 23 = 2030 |
   | RH | 31 = 2034 | 32 = 2042 | 33 = 2081 |
   | LH | 41 = 2073 | 42 = 2089 | 43 = 2035 |

   The repository has **no approved q0-repeatability threshold**
   (`CR3_Q0_ACCEPTANCE_PROMOTION_PLAN.md`); the operator judges each difference against a
   re-alignment they can account for. Hard stop for any |Δ| ≥ 82 ticks — the half-tooth
   ceiling (81.92): beyond it the difference is a different spline tooth, not noise.
3. `@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION` → `admitted=12/12`.

### 2. Per-leg cycle (LF, then RF, then RH, then LH — never concurrently)

For `<L>` in this order:

1. `@CALIBRATION SESSION START <L> CONFIRM_CURRENT_Q0` →
   `CALIBRATION_SESSION=ACTIVE leg=<L> …`. It is refused while the previous leg's run is not
   finalized, a session is live, a permit is active or authority is not NONE.
2. `@CALIBRATION MOTION PERMIT GRANT 16 CONFIRM_FIRST_MOTION` → `CALIBRATION_MOTION_PERMIT=ACTIVE`.
3. **LF only, once:** `@CALIBRATION MOTION DIRECTION_VERIFY LF_UPPER +16 CONFIRM_FIRST_MOTION`
   (bus 12, +16 ticks). Wait for its own SAFE_OFF, then `@SERVO SAFE_OFF 12` →
   `VERIFIED_OFF`. The command refuses in any other leg's session
   (`REASON=ACTIVE_SESSION_IS_NOT_LF`). Nothing revokes the permit when it finishes; if step 4
   nevertheless answers `NO_CURRENT_MOTION_PERMIT`, send the PERMIT GRANT again (it refuses with
   `PERMIT_ALREADY_ACTIVE_REVOKE_FIRST` while one is still active — that is fine).
4. `@CALIBRATION FULL LEG <L> CONFIRM_FULL_CALIBRATION` →
   `CALIBRATION_FULL_LEG=ARMED leg=<L> joint=UPPER bus=<12|22|32|42> … auxiliary=<LH_UPPER|
   RH_UPPER|NONE> aux_bus=<42|32|0>` — check the printed bus/aux against the matrix above.
5. Poll `@CALIBRATION FULL LEG STATUS` until the run ends (many seconds). Then
   `CALIBRATION_FULL_LEG_RESULT leg=<L> verdict=HARDWARE_CONTACT_CALIBRATED failure=NONE` prints
   on its own. `FULL LEG ABORT` / `SESSION ABORT` are always allowed.
6. Confirm the cleanup before the next leg:
   - `@SERVO SAFE_OFF <primary>` (and `<aux>` for LF/RF) → `VERIFIED_OFF`;
   - `@AUTHORITY STATUS` → owner NONE; `@CALIBRATION STATUS` → session COMPLETED (terminal);
   - `@CALIBRATION FULL LEG STATUS` → `CALIBRATION_FULL_LEG_RECORD leg=<L> … verdict=
     HARDWARE_CONTACT_CALIBRATED contact_calibrated=YES envelope_accepted=NO failure=NONE`.

A failed leg does not falsify the others: its record keeps `verdict=FAILED` with the reason, the
session is terminal, permit and authority are released, and the next leg (or a retry of the
same leg with a new `SESSION START` + `PERMIT GRANT`, `attempts` increments) is allowed once the
operator has judged the failure.

### 3. Close

1. `@SERVO SAFE_OFF <id>` for all 13 servos → 13/13 `VERIFIED_OFF`.
2. `@CALIBRATION EVIDENCE EXPORT` (refused while a run is in flight). It prints, paced so the
   USB CDC ring never drops a line: one `CALIBRATION_EVIDENCE_EXPORT=BEGIN … parameters_approved=0`,
   then per leg `…_LEG`, `…_AUX`, three `…_Q0`, two `…_CONTACT`, three `…_ENVELOPE` and three
   `…_LIMIT` lines, then `CALIBRATION_EVIDENCE_EXPORT=END legs_present=4
   legs_contact_calibrated=4 legs_envelope_accepted=0 all_contact_calibrated=1`.
3. Save the log with the export and the manifest. Only then power down.

Four-leg acceptance = `END` line above, all four `…_LEG` lines `verdict=HARDWARE_CONTACT_CALIBRATED
session_completed=1 permit_revoked=1 authority_released=1`, and 13/13 `VERIFIED_OFF`.

## Immediate stop (send `@CALIBRATION FULL LEG ABORT`, `@CALIBRATION SESSION ABORT`, then
## `@SERVO SAFE_OFF` on every servo)

- q0 mismatch the operator cannot account for, or any |Δ| ≥ 82 ticks; population ≠ 12/12;
- identity or Geometry V5 mismatch (`REASON=FULL_LEG_PLAN_…`, `CANONICAL_IDENTITY_MISMATCH`);
- any SAFE_OFF not `VERIFIED_OFF`;
- permit/session mismatch (`NO_ACTIVE_<LEG>_CALIBRATION_SESSION`, `SESSION_LEG_MISMATCH`);
- contact failure, stale telemetry, comms loss or deadman trip (`verdict=FAILED`);
- envelope build failure (`UPPER_ENVELOPE_NOT_READY`) or any admission failure;
- cleanup not clean: `authority_released=0` / `permit_revoked=0` / `session_completed=0`.

Do not continue to the next leg after a stop without an explicit decision.
