# TRUE Full Calibration = 24 contacts — LF V25 full-leg state machine generalized (2026-09-30)

Branch `fix/full-calibration-24-contact-v25-generalized-v1` (from `84bafcc`). Supersedes the
UPPER-only "Full Leg" of PR #34 (`fix/calibration-hw-session-20260929`), which stays in the
history unchanged and is **not** Full Calibration.

Status: **IMPLEMENTED / OFFLINE-VALIDATED. Hardware: NOT RUN** (the 24-contact sequence has never
moved a servo). Every hardware step needs the operator's explicit go-ahead for that session.

## 1. The scope error (what was wrong, and why it happened)

PR #34 shipped a "four-leg Full Calibration" that measured **two contacts per leg (UPPER MIN/MAX),
eight in all**, and called a leg `HARDWARE_CONTACT_CALIBRATED` on those two. That is not Full
Calibration.

**The one definition, from now on:**

```
TRUE FULL CALIBRATION = 4 legs x 3 joints (HIP, UPPER, LOWER) x 2 endpoints (MIN, MAX)
                      = 24 physical contact witnesses
a leg is HARDWARE_CONTACT_CALIBRATED only with 6/6; the robot only with 24/24.
```

Where the error came from. The Geometry Compiler V5 profile evaluates every endpoint from `q=0`
with **all other joints at `q=0`**. In that context the 16 HIP/LOWER contacts lie just outside
the URDF limits and are classified `DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS`, and only the 8 UPPER
endpoints are `EXECUTABLE_URDF_DOMAIN`. `DEVELOPMENT_GATES.md` and `CALIBRATION_READINESS.md §3` then
concluded "*Full operational calibration does **not** mean 24 contact motions: the eight upper
contacts are the current physical contact-calibration set*". That conclusion confused **a
property of V5's q=0 evaluation context** with **the calibration's scope**.

The only calibrator ever validated on hardware, LF V25 (`matdog.rs run_lf_state_machine`,
58/58 steps, **6/6 LF contacts**), never probed a HIP or LOWER from the V5 q=0 context. It held the
leg's other joints at reviewed prerequisite poses: UPPER horizontal while the LOWER is probed,
UPPER raised and LOWER folded while the HIP is probed. It moved between those poses in a fixed
order. The DIAGNOSTIC label never applied to that sequence. The correction keeps V5 as the
source of the canonical contact and the URDF domain, and adds a **second, geometry-validated
authorization object** for exactly the V25 sequence (§4).

Superseded statements, now corrected in place: `DEVELOPMENT_GATES.md` (Calibration gate,
GEOMETRY bullet), `CALIBRATION_READINESS.md §3`, `CALIBRATION_BOOTSTRAP.md` (the "8 upper
endpoints only" passages), `CALIBRATION_MAINTENANCE_REQUIREMENT.md`, `ROADMAP.md` (persistence
record: 24 contacts, not 8).

## 2. What the firmware does now (one leg)

`FullLegCalibrationExecutor` is the V25 state machine, generalized. Every target is resolved from
the current promoted q0, the transform direction, the Geometry V5 URDF domain and the validated
sequence plan (`FullLegCalibrationPlan.cpp`). Nothing is leg-specific.

| # | Phase | Moves | Held (torque on, GoalPosition = pose, ≤10 ticks) |
|---|---|---|---|
| 1 | PREFLIGHT | — | all 12 leg joints torque-OFF, fresh telemetry |
| 2 | INITIAL_RECOVERY | every leg joint of the robot → promoted q0, one at a time, then SAFE_OFF | — |
| 3 | PARKING | front legs: rear UPPER → 35° (Geometry V5's own UPPER MAX auxiliary) | park |
| 4 | UPPER_MIN | HIP → q0, LOWER → q0, then the staged search | park, HIP@q0, LOWER@q0 |
| 5 | UPPER_MAX | search from the MIN contact | park, HIP@q0, LOWER@q0 |
| 6 | UPPER_HORIZONTAL | UPPER → +90° (V25 UPPER_90) | park, HIP, LOWER |
| 7 | LOWER_MIN | search | park, HIP@q0, UPPER@90° |
| 8 | LOWER_MAX | search from the MIN contact | park, HIP@q0, UPPER@90° |
| 9 | LOWER_FOLDED | LOWER → folded; UPPER → HIP-MIN clearance pose where it differs | park, UPPER, LOWER |
| 10 | HIP_MIN | search | park, UPPER@clearance(MIN), LOWER@folded |
| 11 | HIP_MAX | where the per-side clearance differs (LF, RF): HIP → q0, UPPER → MAX pose; then search | park, UPPER@clearance(MAX), LOWER@folded |
| 12 | DIAGNOSTICS | V25 derive_joint_evidence: ordering, affine scale 850–1150‰, q0 shift ≤ 96 | |
| 13–16 | RETURN_HIP → RETURN_LOWER_HELD → RETURN_UPPER → RESTORE_PARKING | back to q0 in V25 order | |
| 17–18 | CLEANUP → TORQUE_OFF | verified SAFE_OFF of all 12, leg at rest ≤ 16 ticks of q0 | |

COMPLETE only with **6/6 contacts + accepted diagnostics + verified rest**. A safety failure goes
straight to the verified SAFE_OFF of every leg joint. A diagnostics rejection first returns the
leg through the reviewed RETURN phases, then ends FAILED.

`@CALIBRATION INITIAL RECOVERY <LEG> CONFIRM_Q0_RECOVERY` runs phases 1–2 alone, then the verified
SAFE_OFF (recovery-only run). It gives the controller-verified q0 baseline the operator requires
after a fresh q0 promotion, before any leg moves. It uses the same session and permit, and the
leg run that follows starts from PREFLIGHT.

## 3. Gap analysis against LF V25, and every deliberate deviation

Full item-by-item mapping:
[`2026-09-30_FULL_CALIBRATION_24_CONTACT_V25_TRACEABILITY.md`](2026-09-30_FULL_CALIBRATION_24_CONTACT_V25_TRACEABILITY.md).

Ported unchanged: phase order; held sets (`prerequisites_for`); `hip_upper_clearance_delta`
(LF 90/85, RF 85/90, rear 90/90); UPPER_90 (1024 ticks), UPPER_85 (967); the V25 StableTargetGate
(≤10 ticks, |speed| ≤ 4, 4 samples, ≥ 400 ms); held drift ≤ 10; passive corridor ±32;
non-participant drift ≤ 16; rest ≤ 16; 12 s motion timeout + travel at 80 ticks/s; telemetry ≤ 3 s;
TorqueLimit 500 before TorqueEnable; the V25 contact search stage for stage (moving-current
baseline, **coarse contact scout** at 64, release, backoff 96 arrived through V25's
StableTargetGate, fine 1 and fine 2 at 8 judged against the scout — adaptive corridor scout − 32,
one-fine-step lag bypass, kinematic plateau —, telemetry timeout 2 s,
repeatability fine-to-fine ≤ 16; guard/entry ±64, travel ≥ 24, settle 900 ms, current only as
abort); `stop_pressure` after every accepted approach; affine diagnostics from the fine passes;
return order; verified global torque-off on every exit; TorqueLimit never restored (RAM, until
power cycle).

Deviations, each with its reason:

| # | V25 | Now | Reason |
|---|---|---|---|
| D1 | `normalize_all_matdog_joints_to_q0` skipped a joint already within 10 ticks of home | **every one of the 12 is actively commanded to q0**, prime → limit → torque → move → settle → SAFE_OFF | operator requirement 2026-09-30: a capture next to the pose is not a controller-verified baseline |
| D2 | recovery wrote GoalPosition = HOME with torque OFF, then enabled torque (the servo drives itself home at the torque-on instant) | GoalPosition := **present** (torque off), TorqueLimit, torque on (no motion), then a reviewed move to q0 | no torque-on instant ever has a goal away from the present position; the stale-goal hazard is structurally absent |
| D3 | rear LOWER fold `LOWER_FOLDED_DELTA = −990` (−87.01°) (V25 never ran a rear leg) | rear legs **−455 ticks (−39.99°)**; front legs keep −990 | nominal CAD: at −87° the folded RH/LH lower leg passes the body at **0.04 mm** during the HIP sweep, inside mesh/assembly tolerance, so a false contact is plausible. At −455 the worst interior clearance is **2.02 mm** (§4) |
| D4 | rear UPPER parked at `UPPER_30_DELTA` (30°) | **35.000°** | the current Geometry V5 compiler's own parking plan for LF/RF UPPER MAX; the pre-reset 30° is superseded |
| D5 | held joints checked for drift only | also **fail at \|speed\| > 40 raw on two consecutive samples** | operator requirement 2026-09-30 |
| D6 | coarse contact scout, fine passes accept down to scout − 32 | **CLOSED (corrective commit on 873a121).** Was: no coarse scout, pass 1 a fine pass, pass 2 judged against pass 1 (inherited from PR #34). Now the V25 sequence: baseline → coarse scout (64) → release → backoff (StableTargetGate) → fine 1 → backoff → fine 2, both fine passes judged against the scout (§3b). The remaining implementation differences are listed in the traceability log | — |
| D7 | one LF-only state machine, station-mediated RAM writes | one generic executor behind `SafeActuatorPolicy`; each move is policy-authorized against the phase table and **re-derived by the policy** | architecture |
| D8 | LF hardware run (`lf_hip_sequence_profile`): UPPER 90° for **both** HIP sides | V25's generic `hip_upper_clearance_delta`: LF 90/85, RF 85/90, rear 90/90 (HIP → q0, UPPER → the MAX pose between the sides) | Geometry V5: at UPPER 90° on both sides the folded lower leg hits `base_link` before the URDF limit (LF HIP MIN→MAX, RF HIP MIN: `PATH_OBSTRUCTION_BEFORE_URDF_LIMIT`); the per-side poses are `CLEAR_TO_END` (traceability, § HIP clearance pose). A mechanical-installation/geometry deviation, not a search change: on LF/RF the HIP MAX search starts at q0 instead of the HIP MIN contact, so its raw scout-grid phase differs from historical LF V25 |
| D10 | the scout ends when the next 64-tick step would pass the guard (grid-phase blind gap before it) | **ONE final bounded partial coarse step targets the existing guard** (0 < Δ < 64, never beyond, never repeated); same detector and safety rules; guard, step size, corridor, fine passes unchanged | measured 2026-09-30: the real LF UPPER MIN stop (raw 1468, URDF + 25 at q0 2090) lay in V25's gap (margin −3…+1). NOT V25 behaviour: a current-installation robustness adaptation; phase-independent reach now guard − 18 |
| D9 | the configured temperature limit (EEPROM 0x0D) re-read with every observation | verified by the persistent-profile preflight (0x0D = 70); the present temperature is checked on every sample against 70 °C | LOW, intentional: no EEPROM reads in the hot calibration loop |

**Risk R1 (hardware, flagged — not changed without evidence).** V25's LF HIP MAX contact was
+39.375° = **448 ticks, exactly the corridor entry** (URDF 512 − 64). V25 accepted it with zero
margin. On this installation a contact a tick or two shallower (q0 placement, assembly) fails
`HIP_MAX_PROBE_FAILED / EARLY_STALL_OUTSIDE_CORRIDOR` after 5/6 contacts. RF HIP MIN mirrors it.
This is pinned by the executor suite (`V25 LF hardware contact set replayed` passes at 448;
`one tick before the entry` fails closed). If it happens on hardware: stop, keep the evidence,
and decide a corridor change from the measured stop, not before. (Geometry V5 suggests V25's
448 was a lower-leg/body contact at UPPER 90°; the plan probes LF HIP MAX at UPPER 85°, so the
real HIP stop may lie deeper. That is a hypothesis to check on hardware, not evidence.)

**Risk R2 (hardware) — closed by D10 on measured evidence.** V25's scout reach was the deepest
legal 64-tick grid target − 11, so it depended on the fresh q0 (the stop is fixed in raw ticks while
the corridor and grid move with q0). On 2026-09-30 the measured LF UPPER MIN stop (raw 1468) at the
fresh q0 2090 had a V25-grid margin of −3…+1, and the run was stopped before any motion. With D10
the endpoint reach is guard − 18 for every q0, a margin of +21 for that stop. The original analysis
is kept below for the record.

The known LF UPPER MIN stop (found by hand 2026-09-29, raw ≈ 1470, depth 616 at that boot's q0
2086) lies at depth q0 − 1470 against a reach of about start depth + 619…623. The margin is
positive near q0 2086 and negative near 2100 (CR2-C).

Rule: after the fresh Q0 PROMOTE, compute a read-only reach report from the promoted q0 and refine
it after INITIAL RECOVERY. If the margin is < 0, stop before Full Calibration. Nothing is widened.
A miss on hardware is `UPPER_MIN_PROBE_FAILED / NO_CONTACT_BEFORE_GUARD` at 0/6 and SAFE_OFF
(traceability, § Reach).

**Installation fact.** The current ST3215s were recentred near their raw mid-range (~2048) before
mounting, PositionOffset = 0, and q=0 lies near 2048 but not at it. 2048 is a servo/provisioning
fact only, never q0, a target or a search origin. Every calibration target is the fresh promoted
q0 + direction · q (proven for two installations and all 24 corridors by
`test_recentred_installation_translation`). Historical LF V25 raw numbers are never targets.

### 3b. The coarse contact scout (D6 closed, 2026-09-30)

Every one of the 24 endpoint searches now runs V25 `measure_lf_contact_side_efficient` stage for
stage in the one generic `ContactProbeEngine`. The steps: a 64-tick moving-current baseline from
the present pose, then the coarse contact scout. The scout steps 64 ticks from the baseline end
through the static corridor, never clamped. Its pre-corridor steps are free-space transit, where
a stall is only ever an EARLY_STALL. The scout tick is stored (`ContactEvidence.coarse_tick`) as
reference evidence and never used as metrology. Then GoalPosition := the scout (verified) and
backoff 96. The backoff counts as arrived only through V25's StableTargetGate: ≤ 12 ticks,
|speed| ≤ 4, 4 consecutive samples, ≥ 400 ms, reset by any bad sample. Only then are the current
recovery and the next pass allowed. Then fine pass 1 (8), GoalPosition := fine 1, backoff 96 (same
gate), fine pass 2 (8), GoalPosition := fine 2, and repeatability |fine 1 − fine 2| ≤ 16. The
search fails after V25's TELEMETRY_TIMEOUT (2 s, was 3 s) without telemetry. Both fine passes
use the scout as V25 did: the corridor is extended HOME-ward to scout − 32
(`ADAPTIVE_FINE_SCOUT_TICKS`). A candidate lagging the scout by more than one fine step
(`FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS = 8`) is a friction/chamfer plateau and is stepped past.
After a settle window with a large error, the V25 kinematic-plateau confirmation applies. The
diagnostics use the midpoint of the two fine passes and the envelope uses the second fine pass;
the scout never enters either. No fine pass runs without an accepted scout
(`SCOUT_MISSING`), and the executor records no evidence without one. The exact
mapping, the remaining narrowings and the reach property R2 are in the traceability log.

Unchanged: TorqueLimit 500; speed 160 / acceleration 8; guard/entry ±64; backoff 96; fine step 8;
hard-current abort 200; telemetry, GoalPosition and TorqueLimit readback on every sample (now
also in the baseline and the release verification); SAFE_OFF fail-closed; every phase, held set,
geometry pose, the park and the rear fold. The scout's 64-tick step can leave the target up to
one coarse step past a stop (V25 behaviour), bounded by TorqueLimit 500, the 200-raw abort and
the guard.

## 4. Geometry validation of all 24 searches

`06_Software/Matdog_Core/calibration/matdog_full_calibration_sequence_geometry_v5.py` evaluates
every segment of every leg's sequence on the SHA-pinned URDF (`3890a3f0…`) and collision meshes
(manifest `60fff604…`), using the same V5 scene/kernel. The segments are parking, both probe
corridors up to the guard, every prerequisite transition, return and restore. Each is swept at
0.5° and bisected to 1e-4 rad. Any collision other than the probed joint's own modelled stop
fails the leg. The start pose of each segment must also be collision-free.

Result (artifacts: `09_Logs/Validation_Reports/Full_Calibration_Sequence_Geometry_2026-09-30/`):

| Leg | Segments | Verdict | Held poses (UPPER for LOWER / HIP MIN / HIP MAX, LOWER fold) | Park |
|---|---|---|---|---|
| LF | 15 | all CLEAR_TO_END | 90° / 90° / 84.99° / −87.01° | LH UPPER 35° |
| RF | 16 | all CLEAR_TO_END | 90° / 84.99° / 90° / −87.01° | RH UPPER 35° |
| RH | 11 | all CLEAR_TO_END | 90° / 90° / 90° / −39.99° | — |
| LH | 11 | all CLEAR_TO_END | 90° / 90° / 90° / −39.99° | — |

`CalibrationSequencePlanData.h` is generated from those four artifacts by the tool
(`--export-header`). `static_audit.py` re-exports it and requires the committed header to match
exactly, and every segment to be `CLEAR_TO_END` from a clear start.

Evidence of the rear-fold decision (2-D scan UPPER 80–110° × fold −87…−30°, both HIP sides):
at UPPER 90°, fold −87.01° gives an interior minimum of 0.04–0.14 mm (base ↔ lower leg at
|hip| 21–31°); fold −40° gives 2.02–2.24 mm, limited by the upper leg approaching the body just
before the modelled stop. The front legs keep V25's fold. A front fold of −40° brings the lower
leg within 0.014 mm of the body at −21° on the MIN side, and V25 proved −87° on hardware.

Correction made during validation: the first report's `base_static` field evaluated a transition
segment's moving joint at q=0 instead of at the segment's start. That flagged a HIT for a pose
the sequence never visits (UPPER 0° with LOWER folded). The tool now checks the real start pose
with the segment's own stop excluded, and a HIT fails the leg. The re-run is the committed evidence.

## 5. TorqueLimit 500 (V25 `prepare_motor`)

Ported. `ServoBus::writeReviewedRamTorqueLimit(id)` writes RAM register 48 (`SMS_STS_TORQUE_LIMIT_L`)
= the compile-time `kReviewedRamTorqueLimit = 500` and returns the verdict of an **independent
readback**. It is the only raw register write in ServoBus (audit-pinned). The EEPROM Max Torque is
never written, and nothing restores the value (V25 semantics). It returns at the next power cycle.
The executor writes it before every TorqueEnable, verifies torque/limit/goal on the next sample,
and checks the TorqueLimit readback on **every** sample of every held and probed joint.

## 6. Machine-enforced invariants (fail closed)

Every tick, before any write: session/permit/authority/mode continuation. Then every held joint
must be torque on, GoalPosition equal to its pose, TorqueLimit 500, drift ≤ 10, not above 40 raw
speed on two samples, status 0, current < 200, temperature ≤ 70 °C, and telemetry fresher than 3 s.
One round-robin bystander is checked per tick: torque off, within 16 of where recovery left it
(limp participants: within 32 of q0). A probe starts only when the held set is **exactly** V25's
set at the plan ticks. The policy refuses a probe unless the executor reports it verified, and
refuses any move not in the phase table, re-deriving the tick itself.

## 7. Offline evidence (this branch)

- host suites: `run_host_tests.sh` exit 0, 0 warnings, failures = 0, including:
  - `test_contact_probe_engine` (33,419 checks), the V25 search case by case, [T1]–[T15]: the
    scout in every search, distinct from transit, stored as reference, never metrology,
    followed by the backoff, 8-tick fine passes judged against it, the plateau bypass, fine
    pass 2 independent and scout-referenced, fine-only repeatability, no scout = no endpoint,
    the guard, early friction, every per-sample violation in every stage, and the V25 rule
    functions against V25's own test values. [H1] covers the V25 backoff StableTargetGate rule by
    rule: in the band while moving, one good sample, 4 samples inside 400 ms, oscillation and
    speed resets, current recovery judged only once settled, no fine pass before settling, and
    fail closed on timeout. [P1]–[P11] cover the final partial scout step (D10): full steps stay
    64, one partial step < 64 to the guard and never repeated, never past the guard, stops in the
    old grid gap found, the scout still reference only, fine passes unchanged (8-tick grid,
    phase-dependent in the last 18 ticks), no stop = NO_CONTACT_BEFORE_GUARD, MIN/MAX, and every
    per-sample fault during the partial step;
  - `test_thermal_confirmation` (145 checks): the LF V25 runtime PresentTemperature
    confirmation (§8b);
  - `test_cr3_q0_fresh_promotion` (1,459 checks): fresh q0 supersedes CR2-C, and the recentred
    installation — two installations, different per-joint offsets, 12/12 URDF→raw commands,
    24/24 corridors and every pose and park translated by exactly Δq0;
  - `test_full_leg_calibration_executor` (18,963 checks): the full V25 sequence on
    LF/RF/RH/LH with the V25 search trace of all 24 endpoints and the evidence mapping, the
    recovery-only run and ~30 adversarial cases, plus all 24 endpoints with stops at guard − 18
    (22 of 24 reached only through the partial step);
  - `test_full_leg_calibration_plan` (the 24-profile matrix + phase table);
  - `test_full_leg_calibration_finalizer`: 2/6 and 5/6 fail; 6/6 passes; 23/24 and 8/24 are not
    Full Calibration; only 24/24 sets `all_contact_calibrated=1`;
- `static_audit.py` PASS, including the Safe Actuator suite (orchestration, TorqueLimit,
  plan-data and coarse-scout mutations), DALY 52/52, LED 98/98 and the runner suite (24);
- `test_calibration_search_behaviour_mutations.py`: **89/89** behaviour mutations caught (18
  staged search + 35 coarse scout + 9 backoff StableTargetGate / telemetry timeout + 4 final
  partial scout step + 6 thermal confirmation + 17 orchestration); every one must fail the host
  tests.

## 8. LED charging presentation (separate change, same build)

Live 2026-09-30, attended charge with KEY OFF, fresh DALY telemetry: `state=CHARGING`,
SOC 55.0 %, charge + discharge MOS ON, `alarms=0000 0000 0000 0010`. The LED showed
`CHARGING_FAULT` because the Controller treated any non-zero alarm word as a charging fault. The
DALY app names word 3 bit 0x0010 "GPS or soft switch turn off MOS". It appears on every
KEY-OFF charge.

Change, **presentation only**: `status::dalyAlarmBlocksChargingPresentation()` treats exactly
word 3 bit 0x0010 as non-blocking. Words 0–2 ≠ 0, or any other bit of word 3, still show
`CHARGING_FAULT`. Unchanged: the raw alarm words (`@BMS STATUS` still prints `0000 0000 0000
0010`), `DalyBms` `alarms_clear` (the KEY/MOS write gate still needs all four words zero), DALY
protection, MOS state and every non-LED gate. The audit pins the helper's exact expression, the
0x0010 constant, its confinement to LED presentation and the unchanged DALY gate. The LED
mutation suite catches reverting to the any-word test, widening the mask and ignoring a word.
Expected live: 6 fixed green + the 7th breathing at SOC 55 %; 11 fixed + the last breathing at a
reported 100 % (never FULL from SOC).

## 8b. Hardware finding: single-sample thermal abort → LF V25 thermal confirmation (2026-09-30)

**What happened.** The first TRUE LF run, on build `4241a39eab60` with fresh q0 and verified INITIAL
RECOVERY 12/12, ended at 7.1 s in PARKING with `OVER_TEMPERATURE`, before any contact search. M42
(LH UPPER, the rear park) was moving 2072 → 2470. One PresentTemperature sample of M42 exceeded
70 °C; about a second later all 12 leg servos read 31–34 °C (M42: 32 °C). The run failed closed:
FULL LEG ABORT, SESSION ABORT, SAFE_OFF 13/13 verified, export 0/24. That aborted run is evidence
only.

**Why.** The port compared each raw sample against 70 °C and aborted on the first one over it. The
LF V25 hardware stack did not.

**The oracle, audited** (`LF_V25_Hardware_Oracle/source/software/drivers/st3215/src/port.rs`):
- constants: `MATDOG_EXPECTED_TEMPERATURE_LIMIT_C = 70`, `MATDOG_THERMAL_CONFIRMATION_READS = 3`,
  `MATDOG_THERMAL_CONFIRMATION_DELAY = 50 ms` (L36–41);
- `apply_matdog_direct_temperature` (L491): a PresentTemperature reading above the limit is followed
  by two more `read_motor_temperature_direct` reads of the **same motor** (L422: a 1-byte
  PresentTemperature read), **each preceded by a 50 ms sleep**;
- `classify_matdog_direct_temperature_samples` (L57): of the three values, ≥ 2 over the limit is
  `Confirmed` (torque forced off, the highest over-limit value published), exactly 1 is
  `Transient` (the last normal value published, `MATDOG_THERMAL_DIRECT_TRANSIENT` logged), and 0 is
  `Normal`;
- a failed confirmation read returns an error, and `scan_motors` stops the bus worker (L659–666). No
  further observation is published, so the calibration dies on telemetry loss: fail closed.

Two different things, not to be confused:
- **A. runtime PresentTemperature transient confirmation** — the bug, now ported;
- **B. persistent MaxTemperature configuration (EEPROM 0x0D = 70)** — verified by the
  servo-profile preflight (D9), unchanged.

**The port** (`calibration/ThermalConfirmation.*`, applied by `Controller::updateFullLegCalibration`
to every Full-Leg sample before the executor sees it):
- a sample > 70 °C triggers two fresh `ServoBus::readPresentTemperatureDirect` reads of the same
  servo (operational timeout), each after 50 ms;
- ≥ 2 of 3 over 70 °C gives CONFIRMED: the monitors see the highest value and abort, as before;
- exactly 1 gives TRANSIENT: the monitors see the last normal value and the run continues;
- a confirmation read that does not answer gives CONFIRMATION_READ_FAILED: the over-limit trigger
  stays published and the run aborts. That is the original behaviour, and stricter than V25's
  2-s telemetry timeout;
- every non-normal decision prints `CALIBRATION_THERMAL_CONFIRMATION bus= decision= samples= published=`.

The one difference from V25: V25 took its first reading from a direct read every 500 ms (its bulk
temperature byte was always overwritten). Here the first reading is the normal per-tick
observation, whose acquisition is unchanged by the operator's instruction. The confirmation reads,
the timing and the rule are V25's.

**Unchanged:** every other fail-closed check — communication, servo status, TorqueEnable,
TorqueLimit, GoalPosition, hard current, telemetry age, held joints, guards, authority, permit — has
no confirmation or debounce. The contact search, q0, geometry, D10, backoff, poses and TorqueLimit
are also unchanged.

**Tests:**
- `test_thermal_confirmation`, through the real classification with a scripted direct-read port:
  - the oracle constants;
  - ≤ 70 °C reads nothing;
  - >70 / normal / normal → TRANSIENT; >70 / >70 / normal, >70 / normal / >70 and >70 / >70 /
    >70 → CONFIRMED;
  - the same servo, fresh direct reads, a 50 ms wait before each;
  - exactly two reads;
  - read failures and a missing port fail closed;
  - today's M42 (255, then 32, 32) → TRANSIENT.
- The executor suite, through the Controller-equivalent path: today's M42 transient during PARKING
  completes 6/6, a probed-joint transient mid-search completes 6/6, a real overheat and an
  unconfirmable sample abort to SAFE_OFF, and a normal run makes no confirmation read.
- Audit pins, 4 audit mutations and 6 behaviour mutations: reads removed, 1 of 3, another servo, a
  cached value, no wait, a failed read no longer fail-closed.

## 9. Next

Hardware, in order, each step on the operator's go-ahead: clean build → application-only flash
→ SAFE_OFF 13/13 → operator places the legs at q=0 → fresh Q0 CAPTURE/PROMOTE → INITIAL
RECOVERY 12/12 → LF session + permit → **stop for the operator's GO** → LF 6/6 → RF → RH → LH →
export (24/24). Procedure: `05_Firmware/MATDOG_Controller/FULL_CALIBRATION_4LEG_HARDWARE_RUNBOOK.md`.
