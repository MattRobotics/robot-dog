# Full Calibration staged search — LF V25 hardware-oracle traceability (2026-09-29)

Status: **OFFLINE-VALIDATED, HARDWARE VALIDATION PENDING.** This maps every contact-search
mechanic of the new staged endpoint search (`ContactProbeEngine` and friends, branch
`fix/calibration-hw-session-20260929`) onto the only MATDOG calibrator ever validated on hardware:
the LF V25 station-mediated calibrator archived under
[`09_Logs/Historical/NormaCore_MATDOG_Archive/LF_V25_Hardware_Oracle/`](../Historical/NormaCore_MATDOG_Archive/LF_V25_Hardware_Oracle/)
(`source/software/drivers/st3215/src/auto_calibrate/matdog.rs`, "V25" below; line numbers refer
to that file).

V25 is **evidence, not code to copy**. It is the oracle for hardware *behaviour*. Its
station-mediated architecture (RAM register writes from a host process, one hard-coded LF state
machine, no policy layer) is not reintroduced.

## Why the search changed tonight (hardware evidence, 2026-09-29)

| # | Evidence | Consequence |
|---|---|---|
| 1 | One slow GoalPosition (speed 40) to the Geometry V5 contact timed out ~480 ticks into a ~590-tick move (fixed 12 s budget) | travel-aware budgets (`b646631`) |
| 2 | With the budget fixed, the joint "stalled" 5 ticks short of the target (1500 vs 1495), then "arrived" 4 short (1499) | contact+16 allowance (`e5c0a3c`) → URDF-clamped to +4 (`67cd3cb`) |
| 3 | Clamped target 1489: the joint arrived at 1493 (4 short) — it always ended 4–5 ticks short of ANY goal | the 1499/1500/1493 "stalls" were servo settling, not a stop |
| 4 | Operator, by hand, torque OFF: the real LF_UPPER MIN hard stop is **~2° (~23 ticks) past** the canonical contact / URDF limit | a single target at/near the model contact can never witness it; "sustained no progress" is not contact |

`b646631`, `e5c0a3c` and `67cd3cb` stay in the history as those experiments. The staged-search
commit supersedes the contact+16 / URDF-clamp mechanism (removed) and keeps the travel-aware
deadman (still used by the backoff and the auxiliary park).

## Architecture now implemented

```
per side (MIN, then MAX), pass 1:
  TorqueEnable
  COARSE_TRANSIT   64-tick target steps @ speed 160 / acc 8, up to the corridor entry
  FINE_SEARCH      8-tick target steps through [entry, guard]; V25 kinematic detector
  contact #1       persistent stall inside [entry, guard]  (outside: EARLY_STALL = anomaly)
  BACKOFF          96 ticks back toward q0 @ 160/8; current must recover to baseline
pass 2:
  FINE_SEARCH      8-tick steps from the backoff point
  contact #2       must reproduce #1 (a candidate > 8 ticks short of #1 = friction plateau, stepped past)
  repeatability    |#1 - #2| <= 16
SAFE_OFF (executor), then the auxiliary park (LF/RF MAX only) before the MAX side, finalization.
```

`guard = URDF limit + 64` and `entry = URDF limit − 64` along the probe direction
(`actuator::resolveCalibrationSearchCorridor`). The canonical Geometry V5 contact and the URDF
domain are **not** changed. The corridor is a calibration-search bound only: the policy grants it
to `CALIBRATION_CONTACT_PROBE` search steps alone (`REJECT_CALIBRATION_SEARCH` everywhere else),
and the V25 speed profile only to the probe and its auxiliary park (`REJECT_MOTION_PROFILE`
everywhere else — never `POSITION_COMMAND`, stand or gait). For LF_UPPER at q0 2086 the MIN
corridor is entry 1553, contact 1493, URDF limit 1489, **guard 1425**. The hand-found stop (~1470)
lies well inside it.

## Item-by-item traceability

Columns: **V25** = what the oracle did (source); **Why** = its reason (V25 comments / evidence);
**Now** = where it lives in the current generic four-leg architecture; **Δ** = deliberate
difference and reason (— = ported unchanged).

| Item | V25 | Why (V25) | Now | Δ |
|---|---|---|---|---|
| GOAL_SPEED 160 | `GOAL_SPEED=160` (l.35), written to RAM GoalSpeed per motor (`prepare_motor` l.3875) | V38: "all six LF contacts have completed supervised hardware passes… use a still bounded but materially faster production-calibration envelope" | `ServoBus::kSearchEnvelopeSpeed=160`, reachable only as `MotionProfile::CALIBRATION_SEARCH` via policy/runtime/backend | — (per-write WritePosEx speed instead of a RAM register write; same value) |
| ACCELERATION 8 | `ACCELERATION=8` (l.36) | same | `kSearchEnvelopeAcceleration=8` | — |
| TorqueLimit 500 | RAM TorqueLimit 500 (l.34), verified on every observation (`ensure_observation_safe`) | "still only half of the ST3215 command range": bounds contact force | **not ported** — servos run their EEPROM MaxTorque 1000 | Δ: would be a NEW RAM write primitive outside the reviewed actuator surface. Contact force is bounded instead by the fine step (the commanded target is never more than 10+8 = 18 ticks past a blocked joint, test-pinned) plus the unchanged 200-raw hard-current abort. **Risk:** at TorqueLimit 1000 a genuine contact could trip HARD_CURRENT_ABORT — a safe failure; the per-step `current=` evidence line will show it, and porting TorqueLimit 500 is then the reviewed fix. |
| Coarse step 64 | `COARSE_STEP_TICKS=64`; the coarse scout pass steps INTO contact (`approach_with_scout(64)`) | fast discovery of the stop; the scout depth later rejects fine-pass friction plateaus | `kSearchCoarseStepTicks=64`, **transit only**: coarse steps stop (clamped) at the corridor entry | Δ: operator's staged spec (bounded contact energy): contact is only ever met by an 8-tick step, never with up to 64 ticks of error at TorqueLimit 1000. Pass 1 takes over the scout's role as depth reference (see "adaptive"/"plateau"). |
| Fine step 8 | `FINE_STEP_TICKS=8`; target advances from the previous TARGET, not the position | small increments bound each motion | `kSearchFineStepTicks=8`; `next = target + 8`; test: every fine step is exactly 8 target ticks | — |
| Guard overshoot 64 | `guard = URDF limit + probe_sign × 64` (`build_profile` l.415); the step that would pass it is an error ("travel guard reached without contact", `passed_guard`) | real stops lie slightly beyond the URDF limit (V25 LF UPPER MIN −53.525° vs URDF −52.5°) | `kCalibrationSearchGuardOvershootTicks=64` in `resolveCalibrationSearchCorridor`; probe fails `NO_CONTACT_BEFORE_GUARD` before issuing that step; the policy independently refuses any search target past the guard | — (the operator named Geometry-contact + 64 as default "unless V25 proves a different semantic": V25's is URDF-limit + 64, ≈ contact + 68 MIN / + 71 MAX, and is what is used) |
| Backoff 96 | `BACKOFF_TICKS=96`; `crossed_home` refused; arrival tolerance STATIC+2 = 12; current must recover (`backoff_and_verify` l.4211) | release pressure, re-approach from a known clear point | `kSearchBackoffTicks=96`, `BACKOFF_CROSSES_HOME`, backoff deadman arrival 12, `CURRENT_NOT_RECOVERED` vs baseline threshold | — |
| Minimum contact travel | `MINIMUM_CONTACT_TRAVEL_TICKS=24` from the pass start | a stall right at the start is not contact | `kSearchMinContactTravelTicks=24` | — |
| Endpoint/contact corridor | acceptance `[URDF − 64, guard]` (`contact_acceptance_bounds` l.800); CONFIRMED inside, EarlyStall outside | contact far from the model is an anomaly, fail closed | `searchCorridorAccepts`; detector returns `EARLY_STALL` outside → `EARLY_STALL_OUTSIDE_CORRIDOR` (SAFE_OFF) | — |
| Adaptive fine scout | fine passes accept up to 32 ticks HOME-ward of the coarse scout (`adaptive_contact_acceptance_bounds`, `ADAPTIVE_FINE_SCOUT_TICKS=32`) | the coarse pass may find an earlier real stop | pass 2 accepts up to 32 ticks HOME-ward of pass-1's contact | Δ reference = pass 1 (no coarse scout) |
| Plateau / friction handling | fine candidate lagging the coarse scout by > 8 = "friction plateau bypass", keep stepping (`fine_contact_reproduces_coarse_depth`, `FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS=8`); `confirm_kinematic_plateau` after a settle window; outside-corridor settle 16 (V36 HIP evidence) | friction/chamfer plateaus are not the endpoint | pass-2 candidate lagging pass 1 by > 8 → bypassed (`plateau_bypass_count`); outside-corridor settle 16 ported; within 10 of target = "arrived" (the 4–5 tick settling is never contact) | Δ: `confirm_kinematic_plateau` not ported — a large tracking error with no confirmed contact fails closed (`TRACKING_FAILED`). **Known limitation** (test-pinned): without a coarse scout, a plateau inside the corridor that holds against a full fine step of error (≥18 ticks) on BOTH passes reads as contact. Tonight's hardware showed only 4–5 tick settling, which is handled. |
| Low-progress detection | per-sample progress ≤ 2 (`max_progress_ticks`) | stopped joint | `kSearchMaxProgressTicks=2` | — |
| Low-velocity detection | `speed & 0x7FFF ≤ 10` (`max_velocity_raw`) | stopped joint | `kSearchMaxVelocityRaw=10`, same magnitude mask; an unread speed never counts as low | — |
| Persistence / sample rules | first 4 samples after a new target ignored (`TARGET_STARTUP_SAMPLES`); 3 consecutive (`persistence_samples`); target must still be AHEAD; goal error > 10 inside / > 16 outside the corridor | a real stop is where the target runs away from the joint | `ContactSearchDetector::observe` ports `HybridContactDetector::observe` rule for rule (detector unit test covers each rule) | — |
| Telemetry cadence | bus poll every 20 ms (`port.rs` l.267); rules are in samples | — | detector and baseline consume at most one sample per 20 ms (`kSearchSampleIntervalMs`) whatever the Controller tick rate; test: confirmation needs ≥ 6×20 ms at 2/5/10 ms ticks | Δ representation only (keeps V25's meaning on a faster loop) |
| Contact settle window | 900 ms per step (`CONTACT_SETTLE_WINDOW`); after it: error ≤ max(step+4, 16) → next step, else fail | bounded lag vs failure | `kSearchSettleWindowMs=900`, same tracking-limit rule | — |
| Repeatability | fine1 vs fine2 ≤ 16 (`repeatability_spread`) | two independent approaches | `repeatability_tolerance_ticks=16` (plan), `REPEATABILITY_FAILED` | — |
| Two-pass confirmation | coarse scout + fine 1 + fine 2 | — | fine 1 + fine 2 (pass 1 alone never completes; test: stop removed before pass 2 → fails closed) | Δ as coarse step |
| Current / load semantics | moving baseline median + MAD over 64 ticks of travel (`BaselineStats`); **`_current_supports_contact` is computed and NOT used** in the contact decision; current used for the backoff recovery check and the hard abort | contact admission is kinematic | same: baseline from the first 64 ticks of pass-1 travel (≥ 6 moving samples, else `INSUFFICIENT_BASELINE`); used only for `CURRENT_NOT_RECOVERED`; **no current threshold is part of contact admission**; load not used (as V25) | Δ baseline gathered during transit instead of a separate 64-tick baseline move |
| HARD_CURRENT_ABORT_RAW | `current ≥ 200` → HardAbort / global torque-off (l.69, `ensure_observation_safe`) | electrical/mechanical protection | `kSearchHardCurrentAbortRaw=200` on Present Current (reg 0x45, same register V25 read), magnitude-masked → `HARD_CURRENT_ABORT` (SAFE_OFF) | see TorqueLimit row |
| Temperature | abort if temp > configured limit, limit must equal 70 | thermal protection | `OVER_TEMPERATURE` above 70 °C | Δ the EEPROM limit itself is not re-read per sample (the C018 profile preflight owns EEPROM values) |
| Goal/status readback per sample | HardAbort if goal register ≠ commanded, status ≠ 0, driver error | detect foreign writes / servo faults | GoalPosition read back at every write (`classifyServoWriteVerify`), torque-off per sample (`TORQUE_UNEXPECTEDLY_OFF`) | Δ status/goal not read per sample (not in `RuntimeState`); a servo fault that drops torque is caught; one that keeps torque is not |
| Motion timeout (long moves) | `max(12 s, distance/80 + 5 s)` (`motion_timeout_for_distance`, `MIN_EXPECTED_MOTION_TICKS_PER_SECOND=80`) | long returns exceeded 12 s at the old speed | travel-aware deadman: `12 s + distance/80` for the backoff and the park (`kSearchMinExpectedTicksPerSecond=80`) | — (≥ V25's budget) |
| Held-target settling | `StableTargetGate`: ≤ 10 ticks and speed ≤ 4 for 4 samples over 400 ms before a held move counts | a moving joint is not "arrived" | park arrival within 10 (`kSearchStaticToleranceTicks`), backoff within 12, deadman ARRIVED | Δ no 400 ms stability window (the move is followed by a monitored step, which re-checks) |
| SAFE_OFF semantics | verified global torque-off on every abort and at the end | — | executor SAFE_OFF on every terminal path (primary, + auxiliary where used, never bus 0); runner SAFE_OFF 13/13 before, after, and on any failure; `@SERVO SAFE_OFF` stays ungated and outside the policy | Δ UPPER is SAFE_OFF'd between MIN and MAX (V25 held it with `stop_pressure`) — the reviewed executor contract |
| Parking / prerequisites | LF: LH_UPPER held at +30° (`UPPER_30_DELTA`) for both sides; **LF HIP and LF LOWER actively held at q0** (`prerequisites_for`); non-participating joints monitored for drift (16) | keep the modelled collision geometry | parking from Geometry V5 only: LF MAX → LH_UPPER 42, RF MAX → RH_UPPER 32 at the compiler's 35° (610865 µrad); RH/LH none; MIN needs none | Δ **HIP/LOWER are not held** (torque-off, placed at q0 by the operator) and other joints are not drift-monitored — the reviewed four-leg architecture energizes only the probed UPPER and the named auxiliary. The runner's operator note asks for a stop if a limp segment moves into the UPPER's path. The compiler's 35° park (vs V25's 30°) is Geometry V5's own result. |
| Sequencing | LF UPPER, LOWER, HIP; MIN then MAX with no SAFE_OFF between | full leg | UPPER MIN → SAFE_OFF → (park) → UPPER MAX → SAFE_OFF; HIP/LOWER envelopes are geometry placeholders (unapproved) | scope of HARDWARE_CONTACT_CALIBRATED today |
| LF witness vs reference | contacts within 24 of LF reference ticks (`LF_CONTACT_WITNESS_TOLERANCE_TICKS`) | LF-only evidence | not ported (class D, historical LF-only); the runner cross-checks LF MIN against tonight's hand-found stop (+7…+39 past contact) before any other leg | Δ |

## Offline evidence

- `test_contact_probe_engine` 10 148 checks: detector rules one by one; all four UPPER joints ×
  MIN/MAX with stops at contact −5/0/+23/+50; step bounds (coarse ≤ 64 and never into the
  corridor, fine exactly 8, backoff exactly 96); guard never passed; commanded target never > 18
  past the stop; settling 4/5/9 ticks never contact; brief (100 ms) stop, yielding friction, 30 t/s
  slowdown never contact; early stall outside the corridor; pass-2 plateau bypass; non-repeatable
  contact; pass 1 alone never completes; backoff obstruction = anomaly; speed noise 1-in-4
  slots found / every-other slot fails closed; hard current, torque drop, over-temperature, stale
  telemetry, comm loss, policy refusal; cadence at 2/5/10 ms ticks; the plateau limitation pinned.
- `scripts/tests/test_calibration_search_behaviour_mutations.py`: 18 source mutations, and each
  one must make a host-test suite report failures. A mutant that merely fails to compile does not
  count. The mutations:
  - early stall accepted as contact;
  - guard +128;
  - fine step 16;
  - backoff 64;
  - settle band 2;
  - persistence 1;
  - min travel 0;
  - hard-current abort off;
  - 20 ms cadence removed;
  - unread speed counted as low;
  - guard check removed;
  - plateau bypass removed;
  - repeatability off;
  - pass 1 alone completes;
  - current-recovery check off;
  - backoff obstruction treated as arrival;
  - search corridor granted to every operation;
  - V25 speed granted to POSITION_COMMAND.
- `test_full_leg_calibration_executor` 4 288 checks: four legs complete at the physical stops in
  < 60 s simulated; exact park; dual SAFE_OFF on MAX failure, aux stall, aux torque uncertainty,
  prerequisite loss in MIN/AUX/MAX, operator abort; never bus 0; READY envelope.
- `test_calibration_execution_engine`: corridor = URDF ± 64 for every leg/side; guard / opposite
  limit admitted, one tick past refused; search flag and speed profile refused for TORQUE_ENABLE,
  POSITION_COMMAND (also under MOTION owner), DIRECTION_VERIFY, AUX (flag) and corrupted values.
- `test_calibration_hw_session.py` (runner, fake Controller in the firmware's record formats):
  NOTE line never terminal; decorated / wrong-leg RESULT never terminal; nothing but STATUS polls
  mid-run; failed leg stops the session with ABORT + SAFE_OFF 13 + export; q0 half-tooth stop
  before PROMOTE; wrong build stops before motion; SESSION START needs a promoted current-boot
  q0; link loss sends nothing; watchdog ABORT; LF MIN cross-check. Two runner mutations were
  re-verified 2026-09-29 and each fails the suite: `terminal.search` instead of `fullmatch`, and a
  terminal regex that accepts another leg's RESULT. The audit pins `terminal.fullmatch(text)`.
- Static audit `check_calibration_search_boundaries` pins every V25 constant above, the corridor,
  the calibration-only gates and the callers; 17 new mutation cases, all caught.

## What only hardware can answer

1. Whether contact at TorqueLimit 1000 stays under the 200-raw abort with ≤ 18 ticks of error.
2. The real stops of RF/RH/LH and of every MAX side (only LF MIN was located by hand).
3. Whether limp HIP/LOWER stay out of the UPPER's path during the sweep.
