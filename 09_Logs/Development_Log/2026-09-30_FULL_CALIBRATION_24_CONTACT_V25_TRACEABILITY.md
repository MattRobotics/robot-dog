# 24-contact Full Calibration — LF V25 hardware-oracle traceability (2026-09-30)

Maps every phase and mechanic of `FullLegCalibrationExecutor` (the LF V25 full-leg state machine
generalized to four legs) to the oracle:
[`09_Logs/Historical/NormaCore_MATDOG_Archive/LF_V25_Hardware_Oracle/`](../Historical/NormaCore_MATDOG_Archive/LF_V25_Hardware_Oracle/),
`source/software/drivers/st3215/src/auto_calibrate/matdog.rs` (line numbers below). The contact
search is mapped stage by stage in **§ Contact search** below; it supersedes the search section of
[`2026-09-29_FULL_CALIBRATION_V25_ORACLE_TRACEABILITY.md`](2026-09-29_FULL_CALIBRATION_V25_ORACLE_TRACEABILITY.md)
(which described the no-scout PR #34 search). Deviations D1–D10 are explained in
[`2026-09-30_TRUE_24_CONTACT_FULL_CALIBRATION.md`](2026-09-30_TRUE_24_CONTACT_FULL_CALIBRATION.md) §3;
D6 (no coarse scout) is **closed**; D5 (a post-V25 held-joint speed abort) is **retired**
(§ Held-role supervision). Where this log says "ported", the V25 rule is reproduced; every
place where the port differs, even slightly, is listed under the table it belongs to. "Exact" is
not claimed anywhere.

V25 is evidence of hardware **behaviour**, not code to copy. Nothing below reintroduces its
station-mediated architecture or an LF-only table.

## Phases

| V25 (`run_lf_state_machine`, L2850) | Now (`FullLegCalibrationExecutor`) | Δ |
|---|---|---|
| "Verified global torque OFF once at session entry" (L2854) | `PREFLIGHT`: every leg joint torque-OFF with fresh telemetry, else `PREFLIGHT_TORQUE_ON` | — |
| `normalize_all_matdog_joints_to_q0` (L3698) | `INITIAL_RECOVERY` (`stepRecover`): one joint at a time, prime at present → TorqueLimit → torque → move to q0 → StableTargetGate → SAFE_OFF; then all 12 verified at q0 torque-off | D1, D2 |
| `STARTUP_HOME_RECOVERY_LIMIT_TICKS = 64` (L63) | `kSequencePrimeMaxDistanceTicks = 64`: a joint farther from q0 is not moved (`INITIAL_RECOVERY_OUT_OF_RANGE`) | — |
| `Parking`: "Park LH upper M42 once for the complete LF session" | `PARKING`: the rear UPPER of a front leg → park, HELD for the whole leg | D4 (35°, Geometry V5) |
| `UpperMin` / `UpperMax` | `UPPER_MIN` (HIP, LOWER energized and held at q0 first) / `UPPER_MAX` | — |
| `UpperHorizontal`: "M12 directly from MAX contact to horizontal hold" | `UPPER_HORIZONTAL`: UPPER → `upper_for_lower` (UPPER_90), HELD | — |
| `LowerMin` / `LowerMax` | `LOWER_MIN` / `LOWER_MAX` | — |
| `LowerFolded`: "M11 directly from MAX contact to HIP parallel hold" | `LOWER_FOLDED`: LOWER → `lower_folded`, HELD; UPPER → HIP-MIN clearance pose where it differs from UPPER_90 | D3 (rear fold) |
| `HipMin` / `HipMax` | `HIP_MIN` / `HIP_MAX` (per-side UPPER pose: HIP → q0, UPPER → MAX pose, then probe, where the poses differ) | D8 |
| `Diagnostics`: "endpoint and affine q0 diagnostics from all fine contacts" | `DIAGNOSTICS` (`deriveFullLegJointDiagnostics`, port of `derive_joint_evidence` L2094) | — |
| `ReturnHip` → `ReturnLowerHeld` → `ReturnUpper` → `RestoreParking` | `RETURN_HIP` → `RETURN_LOWER_HELD` → `RETURN_UPPER` → `RESTORE_PARKING`, same order | — |
| "Final verified global torque OFF" (L3058, and on every failure path L2585/L4243) | `CLEANUP` / `TORQUE_OFF`: SAFE_OFF of all 12, each confirmed by an independent readback, retried every tick until verified; on success also rest ≤ 16 ticks of q0 | — |
| `transition` (L1066) — forward-only, one step | `nextPhase()` — one phase per update, V25 order; the session refuses any other order (`PHASE_REPORT_REJECTED`) | — |

## Held prerequisites

| V25 `prerequisites_for` (L372) | Now (`stepProbe` held-set check + `sequencePlanTargetAllowed`) |
|---|---|
| LF / RF: rear UPPER parked (`UPPER_30_DELTA`) | front legs: rear UPPER at the Geometry V5 park pose (35°), held through every probe (D4) |
| UPPER probe: HIP = 0, LOWER = 0 | UPPER probe: HIP@q0, LOWER@q0 |
| LOWER probe: HIP = 0, UPPER = `UPPER_90_DELTA` | LOWER probe: HIP@q0, UPPER@`upper_for_lower` (90°) |
| HIP probe: UPPER = `hip_upper_clearance_delta(leg, side)` (L363), LOWER = `LOWER_FOLDED_DELTA` | HIP probe: UPPER@`upper_for_hip_min` / `_max` (LF 90/85, RF 85/90, rear 90/90), LOWER@`lower_folded` |
| `validate_transition_entry` (L1109) | a probe starts only if the held set is **exactly** these slots at exactly these ticks (`HELD_SET_MISMATCH` otherwise); the policy refuses the probe unless the executor reports the prerequisites verified |

## Constants

| V25 | Value | Now | |
|---|---|---|---|
| `TORQUE_LIMIT` (L34) | 500 | `ServoBus::kReviewedRamTorqueLimit`, `kFullLegCalibrationTorqueLimit` | RAM 48, readback-verified, checked every sample |
| `GOAL_SPEED` / `ACCELERATION` (L35–36) | 160 / 8 | `ServoBus::kSearchEnvelopeSpeed/Acceleration` (`MotionProfile::CALIBRATION_SEARCH`) | every sequence write |
| `STATIC_TOLERANCE_TICKS` (L40) | 10 | `kSequenceStaticToleranceTicks` | settle gate, held drift, recovery verify |
| `PROBE_HOME_TOLERANCE_TICKS` (L53) | 16 | `kSequenceRestToleranceTicks` | final rest |
| `PROBE_PASSIVE_RESTORE_DRIFT_TICKS` (L57) | 32 | `kSequencePassiveCorridorTicks` | limp participants |
| `NON_PARTICIPATING_MAX_DRIFT_TICKS` (L117) | 16 | `kSequenceBystanderDriftTicks` | bystanders |
| `LF_HELD_MAX_SPEED_RAW` (L116) | 4 | `kSequenceSettleMaxSpeedRaw` | StableTargetGate (promotion to held) and INITIAL_RECOVERY settle; **not** the supervision of an already-held joint (as V25) |
| `LF_TRANSITION_SETTLED_SAMPLES` / `_WINDOW` (L114–115) | 4 / 400 ms | `kSequenceSettledSamples` / `kSequenceSettleWindowMs` | StableTargetGate (L887) |
| `MOTION_TIMEOUT` / `MAX_TELEMETRY_AGE` (L74–75) | 12 s / 3 s | `kSequenceMotionTimeoutMs` / `kSequenceMaxTelemetryAgeMs` | + travel at 80 ticks/s |
| `UPPER_90_DELTA` / `UPPER_85_DELTA` (L96–97) | 1024 / 967 | sequence plan poses (URDF q), resolved per q0 and direction | geometry-validated |
| `LOWER_FOLDED_DELTA` (L98) | −990 | front −990, rear −455 | D3 |
| `AFFINE_SCALE_MIN/MAX_PERMILLE` (L107–108) | 850 / 1150 | `kAffineScaleMin/MaxPermille` | |
| `MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS` (L113) | 96 | `kModelZeroMaxShiftTicks` (shift from the promoted q0) | |
| — | — | `kHeldSpeedTransientReportRaw = 40`, `kHeldSpeedTransientEventCap = 32` | diagnostic only (`CALIBRATION_HELD_SPEED_TRANSIENT`), never an abort; replaces the retired D5 abort |

All of the above are pinned to exactly these values by `static_audit.py`
(`check_full_calibration_sequence`, `check_calibration_search_boundaries`,
`check_servo_id_write`).

## Held-role supervision — `validate_lf_role_observation` (L1378), `LfMotorRole::ActivelyHeld`

| V25 | Now (`FullLegCalibrationExecutor::monitorHeld`, every held joint, every tick) |
|---|---|
| telemetry age > `MAX_TELEMETRY_AGE` | no usable sample for ≥ 3 s → `STALE_TELEMETRY` |
| `has_driver_error` / `status != 0` | `SERVO_STATUS_FAULT` |
| `current >= HARD_CURRENT_ABORT_RAW` | `HARD_CURRENT_ABORT` |
| `validate_matdog_temperature` | `OVER_TEMPERATURE`, after the V25 over-limit confirmation (dev log §8b) |
| `validate_lf_active_readback`: torque enabled, `TORQUE_LIMIT`, `goal_position == target_tick` | `HELD_JOINT_READBACK` |
| `circular_distance(position, target) > STATIC_TOLERANCE_TICKS` (10) | `HELD_JOINT_DRIFT` (`|present − target| > 10`; the installation never wraps) |
| — (no speed check on an `ActivelyHeld` joint) | none. A held joint above 40 raw inside its hold is only logged: `CALIBRATION_HELD_SPEED_TRANSIENT`, rising edge, ≤ 32 per run |

`LF_HELD_MAX_SPEED_RAW = 4` is V25's settling criterion, and the port uses it the same way:
- `StableTargetGate::observe_at` (L887): within tolerance **and** |speed| ≤ 4 for 4 samples over
  ≥ 400 ms. This is `stepMove`'s settle, before `Op::HOLD` promotes the joint to held, and the
  contact search's backoff gate;
- `lf_initial_recovery_needed` (L914): recovery quiescence. The port recovers every joint (D1),
  and each recovery settles through the same |speed| ≤ 4 gate.

The retired D5 was an additional post-V25 rule: an already-held joint failed at |speed| > 40 raw
on two consecutive samples. On 2026-09-30 it aborted LF UPPER MAX 66 ms into a coarse step while
every held joint was inside its 10-tick hold (dev log §8c). A held-role failure now prints
`CALIBRATION_HELD_ROLE_FAILURE`, which names the motor and carries its full readback and the
active probe's state.

## `prepare_motor` (L3875) — the energize order

V25: GoalPosition := present (verified) → TorqueLimit 500 (verified) → Acc 8 → GoalSpeed 160 →
TorqueEnable (verified). Now (`stepEnergize`, and the recovery prime): `PRIME_AT_PRESENT` goal
write (policy: only in an energizing phase, only within 64 ticks of q0) → `CALIBRATION_TORQUE_LIMIT`
(readback-verified) → `TORQUE_ENABLE` (readback-verified). The next sample must show torque on,
TorqueLimit 500 and GoalPosition equal to the prime, else `ENERGIZE_NOT_VERIFIED`. Speed and
acceleration go with every GoalPosition write (`WritePosEx`, the V25 envelope). The host suite
checks the order on every bus against a servo model that drives to a **stale** GoalPosition at
torque-on.

## Contact search — `measure_lf_contact_side_efficient` (L3236), all 24 endpoints

One generic `ContactProbeEngine` runs this for every one of LF/RF/RH/LH × HIP/UPPER/LOWER ×
MIN/MAX. The executor owns exactly one, and the engine names no leg or joint (both audit-pinned).
The MAX side starts where the MIN side's released contact left the joint, as in
`measure_lf_joint_pair_efficient` (L3198), except front-leg HIP MAX (D8, below).

### Ported from V25

| V25 (matdog.rs) | Now (`ContactProbeEngine`) | Port |
|---|---|---|
| **Baseline**: `acquire_moving_current_baseline_forward` (L3294). ONE 64-tick move from the present position, guard-checked. A sample counts while the position changed or speed > 0. It ends at ≤ 10 ticks with ≥ 6 samples. `MOTION_TIMEOUT` is 12 s; a deadline with ≥ 6 samples proceeds un-arrived, < 6 samples is an error. Stats are median and MAD (L1733) | `BASELINE_PENDING/MONITORING`: the same move, rules, deadline and statistics | ported; sample store capped at 32 |
| **Coarse contact scout**: `approach_with_scout(64, None)` (L3962). `next = target + 64` from the baseline end; `passed_guard(next)` (L4926) is an error. Settle window 900 ms, arrival at ≤ 10, tracking limit 68, detector with the **static** bounds (L800) | pass 0: the same steps, never clamped at the entry. The step past the guard is never issued (`NO_CONTACT_BEFORE_GUARD`). Steps short of the entry are labelled `COARSE_TRANSIT`, the rest `COARSE_SCOUT`; this is only a label (one detector, one pass). V25 has no separate transit | ported, **plus the final partial step to the guard (D10, below)** |
| **Coarse reference**: the scout tick is logged "discarded" and is not metrology | `scout_tick` / `ContactEvidence.coarse_tick`: reference only | ported |
| `EarlyStall` → `stop_pressure`, error; tracking failure without a scout → error | `EARLY_STALL_OUTSIDE_CORRIDOR`, `TRACKING_FAILED` → SAFE_OFF | ported; the error path differs (below) |
| **Release**: `stop_pressure` (L4249) after every accepted approach, `set_motor_goal_verified` | `RELEASE_PENDING` → `RELEASE_VERIFYING`: GoalPosition := the contact, read back | ported |
| **Backoff**: `backoff_and_verify` (L4211). Contact − 96, `crossed_home`. `move_motor_to` in the LF session is arrived only through the **StableTargetGate** (L887, used at L4290): ≤ 12 ticks, \|speed\| ≤ 4, 4 consecutive observations, ≥ 400 ms, reset by any non-qualifying one. Then the current must be ≤ median + max(4·MAD, 5) | `BACKOFF`: the same target and home check. The deadman's in-band sample is not arrival: `SearchSettleGate` must hold (same band, speed, count, window and reset). Only then are the current recovery and the next fine pass allowed | ported; the deadline differs (below) |
| **Fine pass 1 / fine pass 2**: `approach_with_scout(8, Some(scout))` twice, each from its own backoff | pass 1 and pass 2 | ported |
| **Adaptive fine scout**: `adaptive_contact_acceptance_bounds(Some(scout))` (L814) extends the corridor home-ward to scout − 32, never toward the guard | `searchAdaptiveAcceptanceEntryDepth()`, both fine passes | ported (V41 LF HIP MAX case replayed) |
| **Lag rule**: `fine_contact_reproduces_coarse_depth` (L852), lag ≤ 8 = `FINE_STEP_TICKS`, else a friction/chamfer plateau bypass → next step | `searchFineContactReproducesScout()`, against the scout on both fine passes | ported (V23 values and the M11 chamfer replayed) |
| **Plateau handling**: `confirm_kinematic_plateau` (L4164). 3 new samples in the adaptive bounds, target ahead, speed ≤ 10, within 32 of the scout, span ≤ 3 | `observeKinematicPlateau()`, only when a scout exists | ported |
| **Repeatability**: `repeatability_spread(first, second)` (L4875) ≤ 16 | \|fine 1 − fine 2\| ≤ 16 after the fine-2 release | ported |
| Metrology: `contact_result_tick()` (L2213) is the midpoint of the fine passes | diagnostics: midpoint(fine_tick_1, fine_tick_2); the envelope uses fine_tick_2; the scout never enters either | ported |
| `HybridContactDetector::observe` (L1838) | `ContactSearchDetector::observe`, rule for rule | ported |
| `TELEMETRY_TIMEOUT` (L73) is 2 s for every new observation | `kSearchTelemetryTimeoutMs` 2000 (search stages) and the backoff deadman's telemetry age 2000 | ported |
| `TORQUE_LIMIT` 500, `GOAL_SPEED` 160, `ACCELERATION` 8 | the constants table above | ported |
| the full-leg state machine | the Phases table above | ported |

**D10 — final bounded partial coarse-scout step (deliberate current-installation deviation, NOT
V25).**
- *What V25 does:* V25 ends the scout when the next 64-tick step would pass the guard. That leaves a
  grid-phase-dependent gap between the last full target (minus the 11-tick detection margin) and
  the guard.
- *Evidence:* on 2026-09-30 the measured LF UPPER MIN hard stop (raw 1468, depth 622 at the fresh q0
  2090) fell in that gap. The V25-grid margin was −3…+1.
- *Now:* when the next 64-tick step would pass the guard and no coarse contact has been confirmed,
  ONE final partial step targets the existing guard itself. It is never beyond the guard, is 0 < Δ <
  64 ticks, and is never repeated. Once the target is the guard, the next step is refused with
  `NO_CONTACT_BEFORE_GUARD`.
- *Same rules as any coarse step:* the same detector, readback, current and safety rules, and the
  tracking limit of V25's `probe_tracking_error_limit(Δ)`. A contact it finds is the coarse scout
  reference like any other.
- *Unchanged:* q0, the Geometry contact, the URDF limits, the guard (URDF + 64), the 64-tick step,
  speed 160 / acceleration 8, TorqueLimit 500, the fine 8-tick passes, backoff 96, the
  StableTargetGate, repeatability and the operational limits.
- *Safety argument:* the step is never larger than a V25 coarse step and is clamped to the already
  reviewed guard. It removes the grid-phase blind spot without enlarging the corridor.
- *Resulting reach, whatever the grid phase:* the coarse scout finds any stop down to guard − 11. The
  whole endpoint is found down to guard − 18 (URDF + 46), limited by the unchanged 8-tick fine
  grid, which needs a fine target 11…18 ticks past the stop before the guard. A stop in the last 10
  ticks before the guard has no kinematic contact signature and fails closed.

Implementation differences inside the search (the algorithm is otherwise V25's):
- every V25 error return (early stall, tracking failed, travel guard, repeatability, current not
  recovered, …) ends in SAFE_OFF_REQUIRED, the caller's verified torque-off, instead of V25's
  GoalPosition := present followed by an error;
- the contact detector consumes samples at V25's 20 ms bus-poll cadence whatever the Controller tick
  rate; the backoff settle gate counts every sample, and its 400 ms bound applies unchanged;
- the backoff deadline is 12 s + travel at 80 ticks/s = 13.2 s for 96 ticks. V25's is
  max(12 s, travel + 5 s) = 12 s. A settle that takes more than 12 s ends 1.2 s later here,
  still `MOTION_TIMEOUT`;
- at most 32 moving-current baseline samples are kept (V25: an unbounded `Vec`);
- a release readback mismatch is `GOAL_READBACK_MISMATCH` at once, never a retry;
- LOW, intentional (D9): V25 re-read the servo's configured temperature limit (EEPROM 0x0D) with
  every observation. Here the persistent-profile preflight verifies it (0x0D = 70 among the 20
  profile registers). The hot loop does not read EEPROM.
- Runtime PresentTemperature over-limit **confirmation — ported from V25 `port.rs`** (2026-09-30,
  after a single-sample false abort on hardware). A sample > 70 °C is re-read directly twice,
  50 ms apart, on the same servo. ≥ 2 of 3 over the limit aborts; exactly 1 is a transient and
  the run continues; a failed confirmation read aborts. The first reading is the normal per-tick
  observation (V25: a 500 ms direct read); the confirmation is V25's (dev log §8b).

### Current-installation deliberate differences

These come from the current mechanism and calibration contract, not from the search algorithm:
- **Fresh measured q0.** The authoritative q0 is the current-boot, manually positioned capture,
  promoted into `JointTransformTable`, never the historical LF V25 raw q0. Historical LF raw q0 or
  contact numbers are never targets. Every raw target is `q0_tick + direction · q`:
  - `resolveUrdfQToRaw`, and `resolveCalibrationSearchCorridor` (`home_tick = q0`; contact and URDF
    limits = q0 + direction · q; entry/guard = URDF limit ∓/± 64);
  - the plan poses and the rear park (`resolveFullLegPlan`);
  - the INITIAL_RECOVERY targets.

  There is no modulo or wrap: an out-of-range raw target is refused. `test_recentred_installation_translation`
  proves this for two installations whose q0 differ by a different offset per joint: every
  identical URDF command, all 24 corridors (home, contact, URDF limits, entry, guard, depth
  geometry invariant), every prerequisite pose and the park all move by exactly Δq0.
- **Servos physically recentred near raw 2048, PositionOffset = 0.** The mounting puts q=0 near
  2048 but not at it, so 2048 is a servo/provisioning fact only. It serves as the profile sanity
  value and the ±80-tick q0 plausibility window at promotion. It is never q0, a target or a
  search origin; the audit refuses it in every calibration target unit. PositionOffset is only
  ever read (audit-pinned).
- All 12 INITIAL_RECOVERY joints actively commanded (D1).
- GoalPosition prime at present before TorqueEnable (D2).
- Geometry-required side-specific HIP prerequisite poses (D8, below).
- Rear LOWER folded pose −455 ticks (D3).
- 35° rear park (D4).
- (D5, a post-V25 held-joint speed abort, is retired: § Held-role supervision.)
- Temperature-limit register verified by preflight, not per sample (D9, LOW).

### Reach of the coarse scout (risk R2) — closed by D10 on measured evidence

V25 steps the scout on a 64-tick grid from the baseline end and never issues a target past the
guard; there is no clamp. A stop is scouted only if a grid target that still fits before the guard
lies more than the 10-tick band beyond it:

- baseline end depth B ≈ start depth + 54…58 (the first 20 ms sample within 10 ticks of the
  64-tick baseline target, moving 3–4 ticks per sample);
- deepest legal scout target T = B + 64·⌊(guard − B)/64⌋;
- reach = T − 11;
- margin = reach − stop depth.

V25's reach depended on the grid phase, because the stop is fixed in raw ticks while the corridor
and the grid move with q0. The port reproduced this without narrowing it. D10 removes the phase
dependence: the coarse reach is now guard − 11 and the endpoint reach guard − 18 (URDF + 46) for
every q0.

For LF UPPER MIN (guard depth 661, 9 grid steps), reach ≈ start depth + 619…623. The stop was found
by hand on 2026-09-29 at about raw 1470; at that boot's q0 2086 that is depth 616 (corridor: entry
1553, contact 1493, URDF 1489, guard 1425). With a fresh q0 its depth is q0 − 1470. The margin is
therefore positive at q0 ≈ 2086 and negative near 2100 (the CR2-C value).

Measured 2026-09-30 (read-only, torque off, 10 samples, spread 0): the LF UPPER MIN hard stop is
at **raw 1468**. At the fresh q0 2090 that is depth 622 (URDF + 25). The V25 grid alone gave a
margin of −3…+1, so the run was stopped before any motion. With D10 the endpoint reach is depth 643,
a margin of **+21**, independent of the grid phase.

**Decision rule (kept):** after every fresh Q0 PROMOTE and before any Full Calibration GO, a read-only
LF UPPER MIN reach report is computed from the promoted q0 with the production resolver. If the
margin is < 0, the run stops before Full Calibration. Nothing is widened. `test_final_partial_scout_step`
and `test_partial_scout_step_all_24_endpoints` pin the step, its bounds and its reach.

## `stop_pressure` (L4249)

After **each** accepted approach (the coarse scout, fine 1, fine 2): GoalPosition := that contact,
verified by readback (`RELEASE_PENDING` → `RELEASE_VERIFYING`), so the joint rests ON the stop
without pressing. After the scout and fine 1 the backoff follows; after fine 2 the probe is
COMPLETE and the next phase moves the joint away from there.

## HIP clearance pose (D8) — a geometry deviation, not a search change

V25's LF **hardware** run used `lf_hip_sequence_profile` (L465): LF UPPER held at `UPPER_90` for
**both** HIP sides, with no UPPER change between HIP MIN and HIP MAX. V25's generic profile
(`prerequisites_for` → `hip_upper_clearance_delta`, L363) uses LF 90/85, RF 85/90, rear 90/90; the
executor uses the latter. Geometry V5 decides between them.

With UPPER 90° on both sides (same tool, scene and URDF;
`~/MATDOG/evidence/full_cal_24contact_geometry/v25_upper90_both_hip_sides_{lf,rf}.*`), the folded
lower leg hits `base_link` before the URDF limit: `PATH_OBSTRUCTION_BEFORE_URDF_LIMIT` in LF
`HIP_MIN_TO_MAX_SEARCH` and in RF `HIP_MIN_SEARCH`. The side-specific poses are all `CLEAR_TO_END`
(§4 of the dev log).

What changes: on LF/RF the HIP returns to q0 after HIP MIN, the UPPER moves to the HIP-MAX pose,
and the HIP MAX search starts at q0 instead of at the HIP MIN contact. So its baseline start,
travel and raw scout-grid phase differ from historical LF V25. The contact-search algorithm,
corridor and request are identical. The model is consistent with V25's LF HIP MAX contact sitting
exactly on the corridor entry (R1): at UPPER 90° it predicts a body contact before the HIP stop.
That is a hypothesis for hardware review, not a measured fact.
