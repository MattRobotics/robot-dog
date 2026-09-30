# TRUE Full Calibration — 24 contacts, four legs (LF → RF → RH → LH) — hardware runbook

Date: 2026-09-30 (replaces the 2026-09-29 UPPER-only procedure)
Status: PROCEDURE — OFFLINE-VALIDATED; the 24-contact sequence has NEVER RUN ON HARDWARE
Firmware: ONE `ROBOT_POWERED` build, OTA ingest 0, manifest `SOURCE_STATE=CLEAN`, from the PR's
final commit (branch `fix/full-calibration-24-contact-v25-generalized-v1`).

**TRUE FULL CALIBRATION = 4 legs × 3 joints (HIP, UPPER, LOWER) × MIN/MAX = 24 contacts.** A leg
passes only at 6/6; the robot only at 24/24 (`all_contact_calibrated=1`). There is no other
definition, and a 2/6 (UPPER-only) or 5/6 leg is a FAILED leg.

This document is a procedure, not an authorization. Building, flashing, energizing the servo
rail, every `@CALIBRATION` command below and every servo motion need the operator's explicit
go-ahead for that session. Nothing here writes EEPROM/NVS, `PositionOffset`, servo IDs,
`CalibrationOfs` or DALY configuration. TorqueLimit 500 is a RAM value restored by the next power
cycle. Everything the run produces lives in RAM and is lost at reset — **export the evidence
before any power cycle.**

## Commands (each exact)

| Command | Effect |
|---|---|
| `@CALIBRATION Q0 CAPTURE 9 16 CONFIRM_Q0_POSE` | read-only torque-OFF q0 capture, 12/12, nine samples per joint |
| `@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION` | promotes that capture into the RAM transform table (replaces any previous q0) |
| `@CALIBRATION SESSION START <LEG> CONFIRM_CURRENT_Q0` | live session for that leg |
| `@CALIBRATION MOTION PERMIT GRANT 16 CONFIRM_FIRST_MOTION` | fresh RAM permit for the live session |
| `@CALIBRATION INITIAL RECOVERY <LEG> CONFIRM_Q0_RECOVERY` | **motion**: every one of the 12 leg joints, one at a time, actively to its promoted q0 (prime at present → RAM TorqueLimit 500 → torque on → move → V25 settle gate → SAFE_OFF), then all 12 verified at q0 torque-off; ends `CALIBRATION_INITIAL_RECOVERY_RESULT verdict=PASS recovered=12/12 …`; session + permit stay live |
| `@CALIBRATION FULL LEG <LEG> CONFIRM_FULL_CALIBRATION` | **motion**: the TRUE Full-Leg run, 6 contacts (below) |
| `@CALIBRATION FULL LEG STATUS` / `ABORT` | poll / abort the running sequence (always allowed) |
| `@CALIBRATION STATUS` | includes `CALIBRATION_MOTION_PERMIT_STATE active=… operator_authorized=… token_valid=…` |
| `@CALIBRATION EVIDENCE EXPORT` | deterministic dump of the RAM record of all four legs |

The leg is the only free token. Bus ids, identities, corridors, prerequisite poses and the park
are derived by the firmware: canonical allocation → Geometry V5 → current promoted q0 →
geometry-validated sequence plan.

## What one Full-Leg run does

```
PREFLIGHT → INITIAL_RECOVERY (all 12 → q0, verified) → PARKING (front legs: rear UPPER → 35°, held)
→ UPPER_MIN → UPPER_MAX            (HIP@q0, LOWER@q0 held)
→ UPPER_HORIZONTAL (UPPER → 90°, held)
→ LOWER_MIN → LOWER_MAX            (HIP@q0, UPPER@90° held)
→ LOWER_FOLDED (LOWER → fold, held; UPPER → HIP-MIN clearance pose where it differs)
→ HIP_MIN → HIP_MAX                (UPPER@clearance pose of that side, LOWER@fold held)
→ DIAGNOSTICS (ordering, affine scale 850–1150‰, q0 shift ≤ 96)
→ RETURN_HIP → RETURN_LOWER_HELD → RETURN_UPPER → RESTORE_PARKING → CLEANUP (verified SAFE_OFF of all 12) → TORQUE_OFF
```

| Leg | HIP / UPPER / LOWER bus | Park | UPPER for LOWER | UPPER for HIP MIN / MAX | LOWER fold |
|---|---|---|---|---|---|
| LF | 13 / 12 / 11 | LH_UPPER (42) → 35° | 90° | 90° / 84.99° | −87.01° |
| RF | 23 / 22 / 21 | RH_UPPER (32) → 35° | 90° | 84.99° / 90° | −87.01° |
| RH | 33 / 32 / 31 | none | 90° | 90° / 90° | −39.99° |
| LH | 43 / 42 / 41 | none | 90° | 90° / 90° | −39.99° |

Every endpoint is searched with the LF V25 contact search (`measure_lf_contact_side_efficient`).
The sequence:
1. a 64-tick moving-current baseline;
2. the **coarse contact scout** in 64-tick steps through the corridor [URDF limit − 64, guard =
   URDF limit + 64]. Its steps short of the entry are free-space transit
   (`stage=COARSE_TRANSIT`); the rest are `stage=COARSE_SCOUT`;
3. release on the scout, backoff 96, fine pass 1 (8-tick steps), release, backoff 96, fine
   pass 2, release; repeatability |fine 1 − fine 2| ≤ 16.

   Each backoff counts as arrived only through V25's StableTargetGate: ≤ 12 ticks, |speed| ≤ 4,
   4 consecutive samples, ≥ 400 ms. Only then are the current-recovery check and the next pass
   allowed. Without new telemetry the search stops after 2 s (V25 `TELEMETRY_TIMEOUT`).

Both fine passes use the scout: they accept down to scout − 32, and step past a plateau more
than 8 ticks short of it. The scout is reference evidence only; the fine passes are the
measurement. Contact is kinematic; current only aborts. When the next 64-tick scout step would
pass the guard, one final partial step targets the guard itself (deviation D10, never beyond it).
Every stop down to guard − 18 (URDF + 46) is therefore found whatever q0's grid phase. A deeper stop
fails closed: `NO_CONTACT_BEFORE_GUARD`, SAFE_OFF. Stop there and keep the evidence. On `ARMED` the firmware
prints the six `CALIBRATION_FULL_LEG_SEARCH_CORRIDOR joint=… side=…` lines and the
`CALIBRATION_FULL_LEG_PREREQUISITE pose=…` lines for this q0. It also prints one
`CALIBRATION_SEQUENCE …` line per phase/step and one `CALIBRATION_SEARCH …` line per search step.

A held joint that loses torque, has its GoalPosition or TorqueLimit changed, drifts more than 10
ticks, moves faster than 40 raw on two samples, reports a status fault or 200 current, or goes 3 s
without telemetry ends the run at once in a verified SAFE_OFF of all 12 joints.

A PresentTemperature above 70 °C is confirmed first, as LF V25 did. Two direct reads of the same
servo, 50 ms apart, follow:
- ≥ 2 of the 3 readings over 70 °C: the run ends in SAFE_OFF;
- a single one: a transient, logged as `CALIBRATION_THERMAL_CONFIRMATION … decision=TRANSIENT`,
  and the run continues;
- a confirmation read that fails: SAFE_OFF. So does any bystander that gains torque or moves more than 16 ticks.

End records:
`CALIBRATION_FULL_LEG_CONTACTS leg=<L> expected=6 measured=6 accepted=6 diagnostics_accepted=YES`,
then `CALIBRATION_FULL_LEG_RESULT leg=<L> verdict=HARDWARE_CONTACT_CALIBRATED failure=NONE`.
**Only that exact RESULT line, with 6/6 on the CONTACTS line, is a leg PASS.**

`HARDWARE_CONTACT_CALIBRATED` is the honest end state: 6 contacts recorded, envelopes built,
servos SAFE_OFF, session COMPLETED. `FINAL_OPERATIONAL_ENVELOPE_ACCEPTED` needs an approved
stand/gait workspace that does not exist (`kFullLegOperationalParametersApproved = false`).
Expect `envelope_accepted=0`, `parameters_approved=0`. That is correct.

## Preconditions

- Charger **disconnected**; robot mechanically supported, every leg free to sweep its full
  UPPER, LOWER and HIP ranges and both rear UPPERs free to park; physical disconnect within reach;
  nobody inside a leg's motion envelope.
- The clean build and its manifest; serial log captured for the whole session (the runner does).

## Sequence (`scripts/calibration_hw_session.py`, one phase per invocation, no board reset)

1. `--phase prepare` — manifest (CLEAN, ROBOT_POWERED, ingest 0, SOURCE_COMMIT == HEAD, SHA256),
   application-only flash (`--backup … --backup-sha256 …`, or `--no-flash`), signature ==
   BUILD_ID, `MODE=MAINTENANCE`, SAFE_OFF 13/13 (11 12 13 21 22 23 31 32 33 41 42 43 51), health
   READY, authority NONE. **No motion.**
2. The operator places all four legs at the nominal q=0 calibration pose.
3. `--phase q0 --confirm-q0-pose` — raw positions printed; `Q0 CAPTURE` 9/9 passes, 12/12
   candidates, population PASS; every q0 against CR2-C (reference only):

   | leg | LOWER | UPPER | HIP |
   |---|---|---|---|
   | LF | 11 = 2087 | 12 = 2100 | 13 = 1996 |
   | RF | 21 = 1985 | 22 = 2092 | 23 = 2030 |
   | RH | 31 = 2034 | 32 = 2042 | 33 = 2081 |
   | LH | 41 = 2073 | 42 = 2089 | 43 = 2035 |

   **Hard stop at any |Δ| ≥ 82** (half a spline tooth). Then `Q0 PROMOTE` → `admitted=12/12
   source=CURRENT_BOOT_CAPTURE`, `transforms_admitted=12 geometry_bound=YES`. The promoted
   **fresh** q0 is authoritative from here on; CR2-C is only this plausibility reference, and 2048
   is never a q0.

   **R2 reach report (read-only, before any GO).** From the promoted LF UPPER q0, compute with the
   production resolver:
   - the LF UPPER MIN corridor (canonical contact, URDF limit, entry, guard);
   - the coarse-scout start depth, the deepest legal coarse target (the guard, via D10) and the
     endpoint reach (guard − 18);
   - the depth of the measured stop (raw 1468, 2026-09-30) and the predicted margin.

   If the margin is < 0, **stop before Full Calibration** and keep the evidence; nothing is
   widened. Recompute it after step 4 with the measured rest position of LF UPPER (bus 12).
4. `--phase recover` — LF `SESSION START` + `PERMIT GRANT`, then `INITIAL RECOVERY LF`: the 12
   targets must equal the promoted q0, and the run must end `verdict=PASS recovered=12/12`. Raw
   positions printed before and after. **Stops here:** LF session + permit live, every joint
   torque-off at q0, no Full-Leg motion yet.
5. **The operator's GO** (physically present, robot supported, charger disconnected, envelope
   clear, disconnect reachable).
6. `--phase legs --confirm-operator-go` — LF runs in the ready session. After each leg the
   cleanup is verified: SAFE_OFF 13/13, authority NONE, session COMPLETED,
   `CALIBRATION_FULL_LEG_RECORD leg=<L> … contacts_accepted=6/6 contact_calibrated=YES …`, six
   accepted contacts; the LF UPPER MIN contact must be +7…+39 past the canonical contact (the
   stop found by hand on 2026-09-29). RF, RH, LH each get a new session, permit and a verified
   INITIAL RECOVERY, then their Full-Leg run. Finally SAFE_OFF 13/13, authority NONE,
   `EVIDENCE EXPORT` saved and verified:
   `CALIBRATION_EVIDENCE_EXPORT=END legs_present=4 legs_contact_calibrated=4
   legs_envelope_accepted=0 total_contacts_expected=24 total_contacts_accepted=24
   all_contact_calibrated=1`, and for every leg `…_LEG … contacts_accepted=6 …`,
   `…_LEG_CLOSE … failure=NONE … session_completed=1 permit_revoked=1 authority_released=1 …`
   and six `…_CONTACT … recorded=1 measured=1 … witness_accepted=1`.

`--phase all` runs 1–6 in one invocation (both confirmations required).

## Immediate stop

The runner does this itself on the first failed check: `FULL LEG ABORT`, `SESSION ABORT`,
SAFE_OFF all 13, a read-only evidence export, no further leg, no retry. Stop conditions:
- any q0 |Δ| ≥ 82 or a population/promotion not 12/12;
- an `ARMED` plan (buses, park, 6 corridors, poses) that disagrees with the table above;
- INITIAL RECOVERY not `PASS 12/12`, or targets that are not the promoted q0;
- any leg not exactly 6/6 `HARDWARE_CONTACT_CALIBRATED`, or its cleanup not verified;
- any SAFE_OFF not `VERIFIED_OFF`, a lost link (use the physical disconnect), a hung run
  (watchdog);
- anything physical that looks wrong: stop by hand, first.

Known risk R1: V25's LF HIP MAX contact sat exactly on the corridor entry. A slightly shallower
stop fails `HIP_MAX_PROBE_FAILED` / `EARLY_STALL_OUTSIDE_CORRIDOR` after 5/6. Report it with the
evidence; no software change before the measured stop is reviewed.
