# Gait performance matrix — Issue #52 — 2026-10-15

Labels: SIMULATO = production firmware code (MotionService → Coordinator → FeedbackWalk.streamUpdate → Executor STREAMING → SafeActuator frame transaction → ActuatorRuntime) driven by a simulated servo/bus/loop (`scripts/tests/test_operational_motion.cpp`, `StreamDriver`). NON MISURATO HARDWARE: no robot was touched. Fixture equivalent speed is NOT ground speed.

## 1. Before / after the timing rework (same 30 mm preset, T = 7 s, lift 15 mm, duty 0.8)

| Pipeline | Steady cycle | Equivalent speed | Source |
|---|---|---|---|
| Baseline feedback-paced, ideal servo | 7.42 s | 4.04 mm/s | audit (SIMULATO) |
| Baseline, loop 2 ms / feedback 10 ms | 8.71 s | 3.44 mm/s | audit (SIMULATO) |
| Baseline, loop 5 ms / feedback 20 ms | 13.50 s | 2.22 mm/s | audit (SIMULATO) |
| Baseline, real Controller with 1 Hz Dashboard poll | ≈1 frame/s (140 frames in 140 s) | — | audit heartbeat probe (MISURATO HOST) |
| **Time-driven, every model below** | **7.000 s** (reference follows the clock) | **4.29 mm/s** | this work (SIMULATO) |

The rework removes the artificial slowdown: the cycle is exactly the commanded period whatever the servo lag, loop jitter or Web cadence; if a servo cannot follow, the walk stops for that physical reason (it is never stretched).

## 2. 10 continuous cycles + WALK STOP → measured STAND (SIMULATO, all PASS)

`OPERATIONAL_MOTION PASS checks=8571821 failures=0`. `walk_s` = START→10 cycles (includes the 7 s ramp); `period_s` printed by the test averages every cycle boundary seen while walking, ramp-in and the two decelerating stop cycles included (7.63 s); the steady cycles are 7.000 s by construction.

| Model | Servo | Loop / reads | Frames | Frame rate | Overruns / skipped | Monitor peak | Lead to latest target p99 / max |
|---|---|---|---|---|---|---|---|
| A ideal ×3 presets (20/30/40 mm) | target reached at delivery | 2 ms / 2 per loop | 4725 | 49.8 Hz | 0 / 0 | 0.77 mrad | — |
| B τ 30 ms, GoalSpeed 1.534 rad/s, ACC 7.67 rad/s², 1-tick quantization | first order + saturation | 2 ms / 2 RR | 4725 | 49.8 Hz | 0 / 0 | 0.77 mrad | see log |
| C τ 60 ms | idem | 5 ms + 0–2 ms jitter / 1 RR | 4725 | 49.3 Hz | 0 / 0 (max late 6.96 ms) | 0.79 mrad | 52 / 60 mrad (lower) |
| D τ 30 ms + 40 ms Controller stall every 1 s | idem | 2 ms / 2 RR | 4537 | 47.8 Hz | 94 / 188 (max late 40 ms) | 5.30 mrad | see log |

"Monitor" = delay-aligned error (distance from the goals active during the last 150 ms) — the fault criterion (> 6 mrad for 3 samples). "Lead to latest target" = |q − newest goal|: with τ = 60 ms the joints trail the reference by up to 60 mrad in time-lag terms. That is not hidden by the monitor; it is why the region certificate covers the whole box ± tube rather than a tube around each instant. Per-joint RMS/p95/p99/max for every model are the `STREAM_TRACK` lines of the test log.

Fault injection (SIMULATO, all PASS): emergency STOP in LF swing/stance → SAFE_OFF, 0 frames after the latch; heartbeat lost → deadman SAFE_OFF ≤ 3.1 s; feedback lost → `STREAM_FEEDBACK_STALE` 90 ms; stuck LF LOWER → `TRACKING_LAG_PERSISTENT` 0.54 s; commissioning drive 20 steps/s → stops 3.6 s, never time-dilated; 150 ms Controller stall → `STREAM_STARVED`; backend refusal → `FRAME_REFUSED_ZERO_BYTES`; incomplete TX → `FRAME_TX_UNCERTAIN`.

## 3. Issue #52 SLO (20–25 mm/s, 2–4 s, 30–60 mm, lift 10–20 mm) — NOT reachable in the current envelope

Kinematic peaks from the audit sweep (`gait_matrix.csv`, production IK, SIMULATO/STIMATO):

| Constraint set | Best equivalent speed | Gait |
|---|---|---|
| Walk envelope 1.0 rad/s, 4.5 rad/s² (current, certified presets) | 8.6 mm/s | 60 mm / 7 s / lift 10–20 / duty 0.75–0.8 (outside the certified box) |
| Executor ceilings 1.2 rad/s, 6.0 rad/s² | 10 mm/s | 40 mm / 4 s / lift 10 / duty 0.75 |
| Drive register hypothesis 1000 steps/s (1.534 rad/s), ACC 50 (7.67 rad/s²) | 15 mm/s | 60 mm / 4 s / lift 10 / duty 0.75 (6.01 rad/s²) |
| SLO 20 mm/s: 60 mm / 3 s / lift 10 / duty 0.75 | — | 1.31 rad/s, 10.7 rad/s² > ACC hypothesis |
| 40 mm / 2 s / lift 15 / duty 0.8 | — | 2.93 rad/s, 45.2 rad/s² |

Cause: the swing phase (20–25 % of T) dominates acceleration (∝ 1/T²); 20 mm/s needs ≈10.7 rad/s² peak, above the reviewed ACC register value and the certified walk envelope. Reaching it requires, separately: a higher reviewed ACC/GoalSpeed pair qualified under load on the fixture, a new preset family, a new dense-sampled region box and certificate (the tube stays 6 mrad), and — for duty 0.75 — a verified all-foot instant for the coordinated stop (duty 0.8 provides it today). None of this is done or claimed here.

## 4. Certificate

Re-pinned box (union of 50 ms targets and 1 kHz time-driven samples) — `operational_walk_region_certificate_issue52_final.json`: 120/120 non-adjacent pairs, 12/12 adjacent revolutes, certified lower bound 6.890 mm (base_link / rf_lower_leg_link, required 6 mm); 13.79 mm is a cell-centre distance, not the bound. The previous box bound was 6.951 mm.

## 5. Measurements left for the fixture (separate decision)

Period and tracking under load at 50 Hz; SYNC_WRITE acceptance; real τ/lag per joint; STOP latency metrics; Wi-Fi-induced loop stalls; ACC/GoalSpeed behaviour vs register hypothesis.
