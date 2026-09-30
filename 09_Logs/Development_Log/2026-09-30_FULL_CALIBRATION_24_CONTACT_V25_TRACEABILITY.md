# 24-contact Full Calibration — LF V25 hardware-oracle traceability (2026-09-30)

Maps every phase and mechanic of `FullLegCalibrationExecutor` (the LF V25 full-leg state machine
generalized to four legs) to the oracle:
[`09_Logs/Historical/NormaCore_MATDOG_Archive/LF_V25_Hardware_Oracle/`](../Historical/NormaCore_MATDOG_Archive/LF_V25_Hardware_Oracle/),
`source/software/drivers/st3215/src/auto_calibrate/matdog.rs` (line numbers below). The staged
contact search itself is mapped in
[`2026-09-29_FULL_CALIBRATION_V25_ORACLE_TRACEABILITY.md`](2026-09-29_FULL_CALIBRATION_V25_ORACLE_TRACEABILITY.md)
and is unchanged here except where noted. Deviations D1–D7 are explained in
[`2026-09-30_TRUE_24_CONTACT_FULL_CALIBRATION.md`](2026-09-30_TRUE_24_CONTACT_FULL_CALIBRATION.md) §3.

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
| `HipMin` / `HipMax` | `HIP_MIN` / `HIP_MAX` (per-side UPPER pose: HIP → q0, UPPER → MAX pose, then probe, where the poses differ) | — |
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
| `LF_HELD_MAX_SPEED_RAW` (L116) | 4 | `kSequenceSettleMaxSpeedRaw` | settle gate |
| `LF_TRANSITION_SETTLED_SAMPLES` / `_WINDOW` (L114–115) | 4 / 400 ms | `kSequenceSettledSamples` / `kSequenceSettleWindowMs` | StableTargetGate (L887) |
| `MOTION_TIMEOUT` / `MAX_TELEMETRY_AGE` (L74–75) | 12 s / 3 s | `kSequenceMotionTimeoutMs` / `kSequenceMaxTelemetryAgeMs` | + travel at 80 ticks/s |
| `UPPER_90_DELTA` / `UPPER_85_DELTA` (L96–97) | 1024 / 967 | sequence plan poses (URDF q), resolved per q0 and direction | geometry-validated |
| `LOWER_FOLDED_DELTA` (L98) | −990 | front −990, rear −455 | D3 |
| `AFFINE_SCALE_MIN/MAX_PERMILLE` (L107–108) | 850 / 1150 | `kAffineScaleMin/MaxPermille` | |
| `MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS` (L113) | 96 | `kModelZeroMaxShiftTicks` (shift from the promoted q0) | |
| — | — | `kSequenceHeldSpeedAbortRaw = 40` × 2 samples | D5 |

All of the above are pinned to exactly these values by `static_audit.py`
(`check_full_calibration_sequence`, `check_calibration_search_boundaries`,
`check_servo_id_write`).

## `prepare_motor` (L3875) — the energize order

V25: GoalPosition := present (verified) → TorqueLimit 500 (verified) → Acc 8 → GoalSpeed 160 →
TorqueEnable (verified). Now (`stepEnergize`, and the recovery prime): `PRIME_AT_PRESENT` goal
write (policy: only in an energizing phase, only within 64 ticks of q0) → `CALIBRATION_TORQUE_LIMIT`
(readback-verified) → `TORQUE_ENABLE` (readback-verified). The next sample must show torque on,
TorqueLimit 500 and GoalPosition equal to the prime, else `ENERGIZE_NOT_VERIFIED`. Speed and
acceleration go with every GoalPosition write (`WritePosEx`, the V25 envelope). The host suite
checks the order on every bus against a servo model that drives to a **stale** GoalPosition at
torque-on.

## `stop_pressure` (L4249)

After each accepted two-pass contact: GoalPosition := the pass-2 contact, so the joint rests ON
the stop without pressing (`ContactProbePhase::RELEASE_PENDING`). The next phase then moves the
joint away from there.
