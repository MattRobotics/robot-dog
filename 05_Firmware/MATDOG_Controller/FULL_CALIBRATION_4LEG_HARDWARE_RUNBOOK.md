# Full Calibration — four-leg hardware runbook (LF → RF → RH → LH, one build, one session)

Date: 2026-09-29 (revised the same night for the staged endpoint search)
Status: PROCEDURE — OFFLINE-VALIDATED; the staged endpoint search has NEVER RUN ON HARDWARE
Firmware: ONE `ROBOT_POWERED` build, OTA ingest 0, manifest `SOURCE_STATE=CLEAN`. That is clean
merged `main` once the staged-search PR is merged. Its first hardware validation runs from the
PR's final commit (branch `fix/calibration-hw-session-20260929`) before merge.

This document is a procedure, not an authorization. Building, flashing, energizing the servo
rail, every `@CALIBRATION` command below and every servo motion need the operator's explicit
go-ahead for that session. Nothing here writes EEPROM/NVS, `PositionOffset`, servo IDs or DALY
configuration; everything the run produces lives in RAM and is lost at reset — **export the
evidence before any power cycle.**

## What one build does

| Command (each exact, four-token) | Effect |
|---|---|
| `@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE` | read-only, Torque-OFF 12/12 q0 capture of the current installation (nine samples per joint) |
| `@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION` | promotes **the twelve candidates of that capture** into the RAM transform table; replaces the previous q0 of every joint; no bus traffic |
| `@CALIBRATION SESSION START <LF\|RF\|RH\|LH> CONFIRM_CURRENT_Q0` | live session for that leg; refused unless the transform table holds exactly the current capture's q0 |
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

### How one side is measured — staged endpoint search (LF V25 oracle)

Hardware, 2026-09-29: a position-controlled ST3215 settles **4–5 ticks short of any goal**. LF_UPPER's
real MIN stop sits **~23 ticks past** the Geometry V5 contact / URDF limit (found by hand, torque
off). A single target at the model contact therefore never witnesses the stop. Each side is
searched the way the hardware-validated LF V25 calibrator did (full mapping in
`09_Logs/Development_Log/2026-09-29_FULL_CALIBRATION_V25_ORACLE_TRACEABILITY.md`):

1. **COARSE_TRANSIT** — 64-tick target steps at speed 160 / acc 8 (calibration-only profile) up to
   the corridor **entry = URDF limit − 64**.
2. **FINE_SEARCH** — 8-tick target steps through the corridor, at most to the
   **guard = URDF limit + 64**. Contact = V25's kinematic rule, all at once, 3 consecutive 20 ms
   samples:
   - ≥ 24 ticks travelled;
   - progress ≤ 2;
   - |speed| ≤ 10;
   - goal error > 10;
   - target still ahead.

   Current is **not** part of contact admission. A stall before the entry = `EARLY_STALL_OUTSIDE_CORRIDOR`.
   The step that would pass the guard is never sent = `NO_CONTACT_BEFORE_GUARD`.
3. **BACKOFF** — 96 ticks back toward q0. Current must fall back to the transit baseline.
4. **FINE_SEARCH pass 2** — 8-tick steps again. A candidate > 8 ticks short of pass 1 is a friction
   plateau and is stepped past. Pass-2 contact must be within **16 ticks** of pass 1.
5. SAFE_OFF (executor). Then (LF/RF) the auxiliary park before MAX, and the same search on MAX.

Hard aborts to SAFE_OFF:
- current ≥ 200 raw;
- > 70 °C;
- torque unexpectedly off;
- stale telemetry / failed reads for 3 s;
- tracking error > max(step + 4, 16) after 900 ms;
- backoff obstruction;
- policy refusal.

Every commanded target stays inside [opposite URDF limit, guard]. The policy refuses the corridor
to anything but `CALIBRATION_CONTACT_PROBE`, and the 160/8 profile to anything but the probe and
its auxiliary park.

Corridors for the 2026-09-29 22:05 q0 (they move 1:1 with the promoted q0; ARMED prints the live
ones as `CALIBRATION_FULL_LEG_SEARCH_CORRIDOR …`):

| Leg | bus | side | q0 | contact | URDF limit | entry | guard | guard past contact |
|---|---|---|---|---|---|---|---|---|
| LF | 12 | MIN | 2086 | 1493 | 1489 | 1553 | 1425 | 68 |
| LF | 12 | MAX | 2086 | 3473 | 3480 | 3416 | 3544 | 71 |
| RF | 22 | MIN | 2108 | 2701 | 2705 | 2641 | 2769 | 68 |
| RF | 22 | MAX | 2108 | 721 | 714 | 778 | 650 | 71 |
| RH | 32 | MIN | 2042 | 2635 | 2639 | 2575 | 2703 | 68 |
| RH | 32 | MAX | 2042 | 655 | 648 | 712 | 584 | 71 |
| LH | 42 | MIN | 2072 | 1479 | 1475 | 1539 | 1411 | 68 |
| LH | 42 | MAX | 2072 | 3459 | 3466 | 3402 | 3530 | 71 |

LF MIN is expected about **+23 past contact (~1470)**. Every other stop is unknown until
witnessed. Only UPPER is energized, plus the parked auxiliary for LF/RF MAX. **HIP and LOWER of the
leg are torque-off** (V25 held them at q0): watch that they stay out of the UPPER's path.

Every step prints one line:
```
CALIBRATION_SEARCH exec=<phase> side=MIN|MAX pass=1|2 stage=COARSE_TRANSIT|FINE_SEARCH|BACKOFF
  probe=<phase> target=<t> pos=<p> beyond_contact=<d> contact=… entry=… guard=… speed=…
  current=… baseline=<median>/<threshold> steps=… bypass=… p1=… p2=… failure=…
```

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
- Legs at the same nominal q0 pose used for CR2-C (`CR2C_Q0_HARDWARE_READONLY_RUNBOOK.md`); the
  fresh capture of this session, not CR2-C, becomes the promoted q0.
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

### 1. Fresh q0, 12/12, then promotion of THAT capture

The q0 the four legs are calibrated with is the one captured **in this boot**. The frozen CR2-C
values (2026-09-27) are a **comparison/reference only**: nothing in the production firmware reads
them any more, and promotion never falls back to them.

1. `@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE`; wait for the unsolicited `CALIBRATION_Q0
   state=COMPLETE … candidates=12/12` and `CALIBRATION_Q0_POPULATION … observed=12/12`; no other
   servo command while it runs.
2. Compare every fresh q0 with CR2-C (reference, not an input):

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
3. `@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION` →
   `CALIBRATION_Q0_PROMOTE=OK admitted=12/12 source=CURRENT_BOOT_CAPTURE capture_session=<n>`.
   The promoted `q0_tick` of every joint is the fresh capture's tick (the numbers compared in
   step 2), acceptance is the unchanged CR3 rule (≥ 9 samples, spread ≤ 16, |tick − 2048| ≤ 80,
   identity/bus/Geometry V5 bound) and any single bad candidate refuses all twelve
   (`REASON=REJECT_CANDIDATE|REJECT_PROMOTION|REJECT_RECORD_DUPLICATE`, `FAILED_RECORD_INDEX`).
   Refusals: `REASON=NOT_IN_MAINTENANCE_MODE`, `REJECT_FRESH_CAPTURE_NOT_COMPLETE` (no finished
   12/12 capture in this boot), `REJECT_CAPTURE_POPULATION_NOT_PASS`; `BUSY` with
   `CAPTURE_STILL_ACTIVE` or `CALIBRATION_SESSION_OR_MOTION_ACTIVE`.
4. Re-capturing and promoting again is allowed between legs' sessions: the new promotion
   **replaces** the previous q0 of the same twelve joints (RAM only; nothing is appended and
   nothing is written to a servo). A completed capture that has not been promoted, or that was
   superseded by a newer promotion, makes `SESSION START` refuse with
   `REASON=CURRENT_BOOT_Q0_NOT_PROMOTED hint=@CALIBRATION_Q0_PROMOTE`. All four legs of one
   evening run on ONE capture and ONE promotion.

### 2. Per-leg cycle (LF, then RF, then RH, then LH — never concurrently)

For `<L>` in this order:

1. `@CALIBRATION SESSION START <L> CONFIRM_CURRENT_Q0` →
   `CALIBRATION_SESSION=ACTIVE leg=<L> …`. It is refused while the previous leg's run is not
   finalized, a session is live, a permit is active or authority is not NONE.
2. `@CALIBRATION MOTION PERMIT GRANT 16 CONFIRM_FIRST_MOTION` → `CALIBRATION_MOTION_PERMIT=ACTIVE`.
3. *(Optional; not a FULL LEG prerequisite, and the one-shot runner does not send it.)*
   **LF only, once:** `@CALIBRATION MOTION DIRECTION_VERIFY LF_UPPER +16 CONFIRM_FIRST_MOTION`
   (bus 12, +16 ticks). Wait for its own SAFE_OFF, then `@SERVO SAFE_OFF 12` →
   `VERIFIED_OFF`. The command refuses in any other leg's session
   (`REASON=ACTIVE_SESSION_IS_NOT_LF`). Nothing revokes the permit when it finishes; if step 4
   nevertheless answers `NO_CURRENT_MOTION_PERMIT`, send the PERMIT GRANT again (it refuses with
   `PERMIT_ALREADY_ACTIVE_REVOKE_FIRST` while one is still active — that is fine).
4. `@CALIBRATION FULL LEG <L> CONFIRM_FULL_CALIBRATION` →
   `CALIBRATION_FULL_LEG=ARMED leg=<L> joint=UPPER bus=<12|22|32|42> … auxiliary=<LH_UPPER|
   RH_UPPER|NONE> aux_bus=<42|32|0>` — check the printed bus/aux against the matrix above,
   and the two `CALIBRATION_FULL_LEG_SEARCH_CORRIDOR` lines against the corridor table (shifted by
   the q0 change).
5. Poll `@CALIBRATION FULL LEG STATUS` until the run ends (tens of seconds; the offline model
   needs < 60 s per leg). Then
   `CALIBRATION_FULL_LEG_RESULT leg=<L> verdict=HARDWARE_CONTACT_CALIBRATED failure=NONE` prints
   on its own. **Only that exact record is a result.** The informational
   `CALIBRATION_FULL_LEG_NOTE HARDWARE_CONTACT_CALIBRATED = …` line never is.
   `FULL LEG ABORT` / `SESSION ABORT` are always allowed.
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

- q0 mismatch versus CR2-C the operator cannot account for, or any |Δ| ≥ 82 ticks; population ≠ 12/12;
  `Q0 PROMOTE` not `admitted=12/12 source=CURRENT_BOOT_CAPTURE`;
- identity or Geometry V5 mismatch (`REASON=FULL_LEG_PLAN_…`, `CANONICAL_IDENTITY_MISMATCH`);
- any SAFE_OFF not `VERIFIED_OFF`;
- permit/session mismatch (`NO_ACTIVE_<LEG>_CALIBRATION_SESSION`, `SESSION_LEG_MISMATCH`);
- contact failure, stale telemetry, comms loss or deadman trip (`verdict=FAILED`);
- envelope build failure (`UPPER_ENVELOPE_NOT_READY`) or any admission failure;
- cleanup not clean: `authority_released=0` / `permit_revoked=0` / `session_completed=0`.

- `EARLY_STALL_OUTSIDE_CORRIDOR`, `NO_CONTACT_BEFORE_GUARD`, `REPEATABILITY_FAILED`,
  `HARD_CURRENT_ABORT`, `CURRENT_NOT_RECOVERED`, `UNEXPECTED_STALL_DURING_BACKOFF`,
  `TRACKING_FAILED` in `CALIBRATION_FULL_LEG_PROBE_FINAL`. Report them, never retry blindly;
- LF MIN contact far from the hand-found stop (outside +7…+39 past contact);
- a torque-off HIP/LOWER segment moving into the UPPER's path, any unexpected noise or contact.

Do not continue to the next leg after a stop without an explicit decision.

## One-shot runner (`scripts/calibration_hw_session.py`)

It performs §0–§3 exactly as written above, and stops at the first failed check.
- Build/manifest check, then the app-only flash.
- Waits for USB re-enumeration, then a no-reset serial open.
- Signature, MAINTENANCE, SAFE_OFF 13/13.
- Q0 CAPTURE with the CR2-C table and the 82-tick stop, then PROMOTE (`transforms_admitted=12`).
- LF → RF → RH → LH, each with its cleanup checks.
- SAFE_OFF 13/13, then EVIDENCE EXPORT, verifying the `END` summary.

On any failure it sends FULL LEG ABORT and SESSION ABORT, does SAFE_OFF on all 13, and runs a
read-only export. It sends only operator commands that the firmware re-checks. Terminal detection
is the exact `CALIBRATION_FULL_LEG_RESULT` record. Everything is logged to
`<evidence-dir>/hw_session_<stamp>.log`.

```
python3 scripts/calibration_hw_session.py --evidence-dir <dir> \
    --backup <fresh full-flash backup .bin> --backup-sha256 <its authorized SHA256> \
    --confirm-q0-pose
```

- `--confirm-q0-pose` is the operator's statement that all four legs are at the manual q=0 pose.
- `--backup-sha256` is passed to `flash_app_only.sh` as `MATDOG_FLASH_BACKUP_SHA256`. A non-default
  backup is never accepted without it.
- `--legs LF` runs one leg.
- `--no-flash` is for when the board already runs the manifest build.
