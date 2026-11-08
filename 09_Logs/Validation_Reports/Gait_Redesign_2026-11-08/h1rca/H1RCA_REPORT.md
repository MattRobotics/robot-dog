# H1-RCA 1.0.2-dev.3 — NATIVE_TARGET_NOT_REACHED reproduced and explained — 2026-11-08

Installed firmware unchanged (1.0.2-dev.3, 94a260cd3b4e). Acquisition read-only (HTTP 1 Hz + USB qualified no-reset opener); no command to actuators from the tools; no reset (uptime continuous 282 s → 751 s).

## Sequence (http.jsonl, device timestamps from MOTION_DIAG)
1. STAND 120 → HOLD: correction saturates at roll/pitch +32.5/−32.8 mrad, imu `WORKSPACE` (benign).
2. Joystick start: HEIGHT unwind from +29.3/−29.6 → 0 completes; WALK_JOYSTICK (envelope ~±25 mrad).
3. Full pull back: `JOYSTICK_REVERSING` → WALK STOP → WALK_SETTLE (18:44:44.96 UTC): correction grows to +41.3/−44.1 then +63.7/−66.5 mrad (steered walk: fillet edge contact model, limit 12°; body held by the fixture).
4. Page restart at STANDING → `stand(0.12)` → HEIGHT (≈ device 462.51 s). imu `WORKSPACE`, wanted 0, applied **constant +63.7/−66.5 for 16 s**. Last goal change `goal_sent_us=462 259 634` (before HEIGHT): **no reference was ever issued in HEIGHT**.
5. FAULT at `at_us=477 842 858` ≥ `deadline_us=477 832 841` = stand time + 12 s + 66.5/20 s. Exit `phase=HEIGHT state=MOVING`, commanded RAW identical to trace (38) within 7 ticks.

## Exit snapshot (motion_diag_at_fault_184501Z.txt)
All 12 joints: valid, torque 1, speed 0, status 0, |cmd−raw| ≤ 7 ticks (LH_HIP 7, LH_UPPER 5), feedback age ≤ 102 ms, loaded joints RF_UPPER/LOWER, RH_HIP, LH ×3 (load ≤ 64/1000). `max_gap_us` 46 586 (< 100 ms), missed 23 (during the walk, none in HEIGHT). MOTION_TIMING: no FIRST_MISS in the armed window, MAX_WORK 8.8 ms in HEIGHT (read 4.1, compensate 2.6, convert 1.8 ms). No tracking, bus, timing or electrical anomaly.

## First verifiable divergence — the refusing condition
In HEIGHT the pending branch calls `compensateLevel(holdNominal_, {}, now, unwind=true)` → `compensationTarget(f, body, seed, nominalStrip=true, edgeTilt=0)` (edgeTilt is the gait's only in WALK/WALK_HOLD). A target whose foot-axis tilt exceeds the nominal strip (`nominalTiltRad` 0.0349 rad − ½ tick) is **invalid**, so the **first test of `compensationSafe()`, `pointSafe(q)` (`!q.valid`), fails**: `WORKSPACE`, goal retained, correction unchanged, next release identical → deadline.
Offline on the HOLD 120 mm nominal (check_unwind_contact_model.out): the whole unwind path from (+63.7, −66.5) down to (+35.0, −36.6) mrad is INVALID (worst foot-axis tilt 0.0696 → 0.0368 rad); valid from (+31.8, −33.2). The first step (0.8 mrad) is invalid, so progress is impossible. The first HOLD saturated at (+32.5, −32.8) for the same reason (next step +33.3/−33.6 invalid).
The later tests (box, decoded RAW, loaded residual + age lag room, kLift fallback, STOP proof) are never reached; the exit data show nothing that would fail them.

## Classification
- **Logic error, not a physical limit.** The correction was accepted in WALK_SETTLE under the fillet edge contact model (steered gait, `edgeTiltRad` 12°) and then must be unwound under the narrower HOLD/HEIGHT nominal-strip model, step by step. The target (zero correction, canonical stand) is valid and physically reachable; servos track within 7 ticks. Only joystick walks are exposed (WALK STANDARD has `edgeTiltRad` 0). PRONE from that HOLD takes the same branch (PRONE_HEIGHT) and would fault the same way.
- Concause: the large WALK_SETTLE correction (fixture: body held, open-loop levelling with servo sag). On the ground the same occurs if the settle correction exceeds ~2° (slope).
- Code unchanged since v1.0.1 (latent); dev.3's automatic restart only makes it immediate.

## Proposed minimal patch (NOT implemented — awaiting authorization)
Unwind steps are solved in the contact model that accepted the correction: in `compensateLevel`, pass the fillet edge model (`kEdgeContactMaxTiltRad`) when `unwind` is true. Offline the edge model equals the nominal model wherever the latter is valid (0 rad difference along the path) and is valid along the whole unwind path; each step only reduces the tilt; boxes, decoded RAW, loaded/lag room, STOP proof, slew, deadline unchanged. Regression: bench with a steered walk ending with a correction beyond the strip (test probe), then STAND 120 and PRONE must complete without NATIVE_TARGET_NOT_REACHED; existing suites unchanged.
