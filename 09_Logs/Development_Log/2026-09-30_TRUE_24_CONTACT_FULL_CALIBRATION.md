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
TorqueLimit 500 before TorqueEnable; the staged search (coarse 64, fine 8, guard/entry ±64,
backoff 96, travel ≥ 24, repeatability 16, settle 900 ms, kinematic contact, current only as
abort); `stop_pressure` release; affine diagnostics; return order; verified global torque-off on
every exit; TorqueLimit never restored (RAM, until power cycle).

Deviations, each with its reason:

| # | V25 | Now | Reason |
|---|---|---|---|
| D1 | `normalize_all_matdog_joints_to_q0` skipped a joint already within 10 ticks of home | **every one of the 12 is actively commanded to q0**, prime → limit → torque → move → settle → SAFE_OFF | operator requirement 2026-09-30: a capture next to the pose is not a controller-verified baseline |
| D2 | recovery wrote GoalPosition = HOME with torque OFF, then enabled torque (the servo drives itself home at the torque-on instant) | GoalPosition := **present** (torque off), TorqueLimit, torque on (no motion), then a reviewed move to q0 | no torque-on instant ever has a goal away from the present position; the stale-goal hazard is structurally absent |
| D3 | rear LOWER fold `LOWER_FOLDED_DELTA = −990` (−87.01°) (V25 never ran a rear leg) | rear legs **−455 ticks (−39.99°)**; front legs keep −990 | nominal CAD: at −87° the folded RH/LH lower leg passes the body at **0.04 mm** during the HIP sweep, inside mesh/assembly tolerance, so a false contact is plausible. At −455 the worst interior clearance is **2.02 mm** (§4) |
| D4 | rear UPPER parked at `UPPER_30_DELTA` (30°) | **35.000°** | the current Geometry V5 compiler's own parking plan for LF/RF UPPER MAX; the pre-reset 30° is superseded |
| D5 | held joints checked for drift only | also **fail at \|speed\| > 40 raw on two consecutive samples** | operator requirement 2026-09-30 |
| D6 | coarse contact scout, fine passes accept down to scout − 32 | no coarse scout: pass 1 is a fine pass with the static corridor `[limit − 64, guard]`, pass 2 accepts down to pass 1 − 32 | PR #34 (staged search), unchanged here — see risk R1 |
| D7 | one LF-only state machine, station-mediated RAM writes | one generic executor behind `SafeActuatorPolicy`; each move is policy-authorized against the phase table and **re-derived by the policy** | architecture |

**Risk R1 (hardware, flagged — not changed without evidence).** V25's LF HIP MAX contact was
+39.375° = **448 ticks, exactly the corridor entry** (URDF 512 − 64). V25 accepted it with zero
margin. On this installation a contact a tick or two shallower (q0 placement, assembly) fails
`HIP_MAX_PROBE_FAILED / EARLY_STALL_OUTSIDE_CORRIDOR` after 5/6 contacts. RF HIP MIN mirrors it.
This is pinned by the executor suite (`V25 LF hardware contact set replayed` passes at 448;
`one tick before the entry` fails closed). If it happens on hardware: stop, keep the evidence,
and decide a corridor change from the measured stop, not before.

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

- host suites: all pass (34 binaries, failures = 0), including
  `test_full_leg_calibration_executor` (full V25 sequence on LF/RF/RH/LH, the recovery-only
  run, ~30 adversarial cases), `test_full_leg_calibration_plan` (the 24-profile matrix + phase
  table), `test_full_leg_calibration_finalizer` (2/6, 5/6 fail; 6/6 passes; 23/24 and 8/24 are
  not Full Calibration; only 24/24 sets `all_contact_calibrated=1`);
- `static_audit.py` PASS, including the Safe Actuator (new orchestration / TorqueLimit /
  plan-data mutations), DALY and LED audit mutation suites and the runner suite;
- `test_calibration_search_behaviour_mutations.py`: staged-search + 17 orchestration behaviour
  mutations, each must fail the host tests.

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

## 9. Next

Hardware, in order, each step on the operator's go-ahead: clean build → application-only flash
→ SAFE_OFF 13/13 → operator places the legs at q=0 → fresh Q0 CAPTURE/PROMOTE → INITIAL
RECOVERY 12/12 → LF session + permit → **stop for the operator's GO** → LF 6/6 → RF → RH → LH →
export (24/24). Procedure: `05_Firmware/MATDOG_Controller/FULL_CALIBRATION_4LEG_HARDWARE_RUNBOOK.md`.
