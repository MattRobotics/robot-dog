# MATDOG Issue #52 — Real-time consolidation + ST3215-C018 dynamic optimisation — 2026-10-15

Labels: **IMPLEMENTATO** (code in the worktree) · **TESTATO OFFLINE** (production code, host tests) · **SIMULATO** (production code + simulated servo/bus/loop) · **STIMATO** (arithmetic / model estimate) · **NON VERIFICATO HARDWARE**. No flash, OTA, servo command, NVS/EEPROM write, push or merge in this session.

## 1. Issue #52 refactor — actual state

| Item | State |
|---|---|
| 64-bit monotonic clock + absolute-deadline scheduler (`core/MotionClock.h`) | IMPLEMENTATO, TESTATO OFFLINE (32-bit wrap, overrun/skip, no catch-up) |
| Time-driven WALK (`FeedbackWalk::streamUpdate`, HOLD → measured STAND) | IMPLEMENTATO, SIMULATO (10 cycles × 4 servo models, period exact) |
| 12-joint frame transaction SafeActuator → ActuatorRuntime → backend → ServoBus SYNC_WRITE | IMPLEMENTATO, TESTATO OFFLINE (zero bytes on any refusal; packet byte-equal to SCServo 1.0.2). NON VERIFICATO HARDWARE |
| Asynchronous per-joint feedback, encoder stamps ≠ heartbeat ≠ support | IMPLEMENTATO, SIMULATO (1 Hz heartbeat → 49.9 frames/s) |
| Priority STOP lane, `SUPERSEDED_BY_STOP`, SAFE_OFF sync burst + per-id verify | IMPLEMENTATO, TESTATO OFFLINE (HTTP probe, DOM). Latency NON VERIFICATO HARDWARE |
| dev.3 thermal artifact: block over-temperature confirmed by a direct read before invalidating a joint | IMPLEMENTATO (Controller), compile-verified |

Suites (this session, final code unless noted): operational motion **PASS 8,810,358 checks**; motion service/executor 2459/0; HTTP transport 76/0; Dashboard DOM PASS; host tests PASS (all suites); static audit: PASS on the pre-STAND/FAST state, final rerun recorded in the handoff; lifecycle/restore/commissioning/adaptive/safety-boundaries PASS earlier on the Issue #52 code.

## 2. Runtime motion profiles (one table, `motion/OperationalWalkProfile.h`)

ST3215 RAM registers only: ACC (SRAM 41, ×100 steps/s²) and GoalSpeed (SRAM 46/47, steps/s) are commands to the servo's own profile generator; 1 step = 1.534 mrad. EEPROM `MATDOG_C018_V1`, PID, protections, torque limit unchanged; no provisioning.

| Profile | Geometry | Planner envelope | Drive pair (register ceiling) | Default |
|---|---|---|---|---|
| STAND (operational) | certified paired route, junction-limited time law | 0.29 rad/s, 2.0 rad/s² (planner 1.0), 20 ms frames, step ≤ 6 mrad | 300 / 20 = 0.46 rad/s, 3.07 rad/s² | yes (STAND) |
| WALK STANDARD | 20/30/40 mm, 7 s, lift 15, duty 0.8 | 1.0 rad/s, 4.5 rad/s² | 1000 / 50 = 1.53 rad/s, 7.67 rad/s² | yes (30 mm) |
| WALK FAST — EXPERIMENTAL | 60 mm, 3 s, lift 10, duty 0.75 | 1.55 rad/s, 13.0 rad/s² | 1200 / 100 = 1.84 rad/s, 15.3 rad/s² | never; explicit Dashboard opt-in labelled SPERIMENTALE |

Consistency is compiled: executor ceilings 1.6 rad/s / 13.5 rad/s² ≥ every profile (static_assert); ServoBus accepts exactly the table pairs (static_assert); the Controller latches the STAND pair when ARMED and the running walk profile's pair while walking; the drive monitor accepts only table pairs. Explored and NOT adopted: ACC 150 / GoalSpeed 1500 (needed only by 60 mm / 2.4 s = 25 mm/s, see §3).

## 3. WALK — best demonstrated offline

Sweep (production IK, steady cycle at 1 kHz, `scripts/tests/run_gait_envelope_sweep.sh`, STIMATO kinematics): 216 valid candidates (stride 30–60 mm × T 2.4–7 s × lift 10–15 × duty .75–.8), all ≥ 0.14 rad from the historical gen-1 limits.

| Candidate | v_eq | peak qd | peak qdd | jerk | Drive needed (×1.25) |
|---|---|---|---|---|---|
| 60/4/10/.75 | 15 mm/s | 0.98 | 6.0 | — | 1000/50 |
| **60/3/10/.75 (FAST)** | **20 mm/s** | **1.310 rad/s** | **10.69 rad/s²** | 267 rad/s³ | 1068/88 → 1200/100 |
| 60/2.7/10/.75 | 22.2 mm/s | 1.456 | 13.2 | — | 1500/150 |
| 60/2.4/10/.75 | 25 mm/s | 1.638 | 16.7 | — | 1500/150 |

End-to-end (SIMULATO, production firmware path, 10 continuous cycles + WALK STOP → measured STAND):

| Model | Result | Period | v_eq | Frames | Monitor peak | Lead to latest goal p99 / max |
|---|---|---|---|---|---|---|
| FAST ideal servo | PASS | 3.000 s | 20.0 mm/s | 49.5 Hz | 0.77 mrad | — |
| FAST τ30 ms, 1200/100 saturation, RR feedback 2/loop | PASS | 3.000 s | 20.0 mm/s | 49.5 Hz | 0.76 mrad | 49 / 67 mrad |
| FAST τ60 ms, loop 5 ms ±2 ms, RR 1/loop | PASS | 3.000 s | 20.0 mm/s | 48.5 Hz | 0.76 mrad | 82 / 109 mrad |
| FAST on the STANDARD drive 1000/50 | **stops at 0.49 cycle, TRACKING_LAG_PERSISTENT** | — | — | — | — | servo cannot follow: the higher pair is necessary |
| STANDARD 20/30/40 mm, ideal / τ30 / τ60 / 40 ms stalls | PASS | 7.000 s | 2.9 / 4.3 / 5.7 mm/s | 47.8–49.8 Hz | ≤ 5.3 mrad (stalls) | — |

Coordinated stop on FAST at 0.5 / 1.0 / 1.37 / 2.0 / 2.62 / 3.0 cycles (ramp-in = acceleration, swing, stance, all-foot instants): all reach measured STAND (7.6–9.4 s). Emergency STOP (swing and stance), heartbeat loss, feedback loss, stuck joint, slow drive, 150 ms stall, refused / incomplete frame: SAFE_OFF with zero frames after the latch (SIMULATO).

**Best equivalent speed demonstrated offline: 20 mm/s (fixture, SIMULATO); best sustainable period 3.0 s.** 25 mm/s (T = 2.4 s) needs 1.64 rad/s and 16.7 rad/s², i.e. the 1500/150 pair; not implemented because 150 is beyond what can be justified without measuring ACC behaviour under load. Fixture equivalent speed is NOT ground speed.

Geometry: FAST box certified (`operational_walk_fast_region_certificate.json`): 120/120 non-adjacent pairs, 12/12 adjacent revolutes, certified lower bound **6.158 mm** (base_link / rf_lower_leg_link; required 6 mm — thin margin). STANDARD box: 6.890 mm. Both certificates carry the sha256 of the current `OperationalWalkProfile.cpp`.

## 4. PRONE → STAND

Baseline: rest-to-rest quintic on each of 251 segments (60.03 s × route_time_scale 2 = 120 s planned), commissioning drive 20/5 (0.031 rad/s servo ceiling), single writes, 4 s entry stage floor.

Issue #52 (IMPLEMENTATO): the SAME polyline (max off-path 9e-14 rad, TESTATO OFFLINE) with a junction-limited look-ahead time law (`motion/PhaseTimedRoute.h`): infinity-norm arclength, trapezoidal speed (|s''| ≤ a), rest only at the 5 support transitions, corner speed ≤ a·dt / |direction jump| → derived route time **10.86 s** (phases 0.46/2.22/0.51/0.11/7.55 s), |qd| ≤ 0.290 rad/s, |qdd| ≤ 1.0 rad/s², finite-difference incl. corners 1.50 rad/s² ≤ envelope 2.0. Entry stages derived from the same rates (1.02 + 1.02 s). Each frame is still tracked before the next (6 mrad tube = the route certificate's assumption); frames are one SYNC transaction.

| Model (SIMULATO) | PRONE fold offset | Legacy | Issue #52 | Final error |
|---|---|---|---|---|
| τ30 ms, RR 2/loop | 0° / +0.8° / −2.0° | 175.9 s (0°) | **21.1 / 21.1 / 21.2 s** (8.4×) | 0.74 mrad, STAND_VERIFIED |
| ideal servo | 0° / +0.8° / −2.0° | — | 20.7 / 20.7 / 20.9 s | 0.76 mrad |
| τ60 ms, loop 5 ms, RR 1/loop | 0° | — | 59.1 s | 0.74 mrad |

The gap between 13 s planned and 21 s achieved is the per-frame confirmation (12 fresh samples after each frame at ~2 samples per loop). A faster STAND would need a region certificate for streamed (non-confirmed) STAND frames — the paired route is certified as a tube around the path, not as a box, so this is not claimed.

## 5. Dynamics and power (STIMATO)

URDF `matt_robodog_rev00.urdf`: masses, COM and inertia tensors present for all 17 links (2.48 kg total), joint effort limit 0.902 N·m, velocity 3.04 rad/s. Not in the URDF: battery/electronics (complete robot 3.3 kg per `01_Docs/01_Analysis/ST3215_Quadruped_3S_Power_Load_Analysis.md`), gearbox friction/efficiency, cables.

`06_Software/Matdog_Core/motion/dynamics/leg_torque_estimate.py` (RNEA + J^T F):

| Gait | Fixture (legs in air) peak hip/upper/lower | Ground quasi-static, 3 stance feet, 3.3 kg |
|---|---|---|
| 30 mm / 7 s | 0.13 / 0.11 / 0.02 N·m | — |
| 60 mm / 3 s (FAST) | 0.14 / 0.12 / 0.02 N·m | 0.37 / 0.62 / 0.60 N·m |
| 60 mm / 2.4 s | 0.14 / 0.12 / 0.02 N·m | ≈ same (inertial terms negligible) |

Leg inertial torques are negligible at these rates; gravity dominates. Ground peak 0.62 N·m = 63 % of the ST3215 rated 10 kg·cm at 12 V (datasheet), ≈ 84 % of the rated value scaled to 9 V. Stall (30 kg·cm) is NOT used as a continuous budget. Current (Kt 11 kg·cm/A, datasheet): ≈ 0.6 A on the most loaded joints, a few amperes for 12 servos quasi-static; impacts, uneven load sharing, the BMS limit, powerboard, connectors and wiring drops are NOT modelled here — see the existing power analysis; the 120 C battery rating is not a system current capability. **Conclusion: on the fixture the limiting factor is the servo profile generator (GoalSpeed/ACC) and tracking, not torque.**

## 6. Tracking (SIMULATO)

Delay-aligned monitor (distance from the goals active in the last 150 ms) ≤ 0.8 mrad except Controller stalls (5.3 mrad); fault threshold 6 mrad × 3 samples. Time lag (|q − newest goal|) is large at FAST (p99 49 mrad τ30, 82 mrad τ60); geometric safety comes from the certified box (measured joints must stay inside box ± 6 mrad, immediate stop otherwise), not from the tube around each instant. Feedforward/anticipation was not added: the simulated servo model is not a measured C018 response; compensation must be tuned on fixture data.

## 7. Missing before a flash candidate / physical trial

1. Fixture characterisation (separate authorization): SYNC_WRITE acceptance on the installed C018 units; real lag/τ per joint at 300/20, 1000/50, 1200/100; ACC register behaviour under load; feedback round time with Wi-Fi active; STOP latency metrics (`MOTION_SAFE_OFF=VERIFIED …`).
2. FAST remains EXPERIMENTAL until 1. is measured; thin 6.158 mm certified margin.
3. Ground walking: no load/contact/IMU-based stability qualification; nothing here is a ground claim.
4. Commit and builds of this exact state (see handoff).
