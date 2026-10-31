# MATDOG — Motion realtime architecture (Issue #52) — 2026-10-15

Scope: the execution timing contract only (clock/scheduler, executor, frame
transaction, feedback, STOP). Offline work: **no flash, no OTA, no servo command,
no hardware session, no NVS/EEPROM, no push**. Labels: MISURATO HOST (computed on
the Asus host by the production code), SIMULATO (production code driven by a
simulated servo/bus/loop), STIMATO (arithmetic budget), NON MISURATO HARDWARE.

## 1. Bottlenecks found by the Codex audit and what replaced them

| # | Baseline (b064ef5 + WIP) | Effect | New contract |
|---|---|---|---|
| A | `Controller.cpp:1251` stamped the observation with the heartbeat time; `checkTracking` needs stamp ≥ frame+50 ms | gait advanced ≈1 frame per Dashboard poll (1 Hz) | Observation `stampS` = OLDEST per-joint encoder sample (real read completion, 64-bit µs clock). Heartbeat = `supportStampS` + presence only. Assessment stamp = clock now. |
| B | FeedbackWalk clock advanced only after the pending frame was tracked; executor wrote one joint per update, then waited for all 12 fresh and inside the tube | stop-and-wait; 7.4–13.5 s per 7 s cycle (SIMULATO, audit) | Time-driven reference (`FeedbackWalk::streamUpdate`) on absolute deadlines (`PeriodicScheduler`, `next_release += period`, no catch-up); one coordinated frame per release; asynchronous per-joint tracking monitor. |
| C | stop endpoint `2(u−I(u))` cancelled to terminal+1 ULP after 10 cycles; `ceil(cycles)` of an integer-up-to-rounding phase scheduled an extra cycle | coordinated stop → IK_FAILURE (reported, not a collision) | residual `s⁴(5−6s+2s²)`, s=1−u (≥0, exactly 0 at u=1); ceil with 8-ULP tolerance. Terminal instant clamped; HOLD until measured settle. |
| D | 12 unicast WritePosEx + 10-byte readback per joint | ≥12 bus transactions per frame | ONE SYNC_WRITE (104 B) of the 12 canonical ids, SRAM 41..47, through a single-use SafeActuator frame transaction. |
| E | STOP button waited for `sendBusy`; mailbox request could hold ≤3 s; SAFE_OFF one joint per tick with write+readback | STOP serialized behind ordinary traffic | Dashboard STOP has its own lane; firmware one-slot STOP lane consumed first in `Controller::update`; STOP epoch makes earlier commands `SUPERSEDED_BY_STOP`; SAFE_OFF = one SYNC TorqueEnable=0 to the 12 ids, then per-id readback verification (+ unicast OFF retry). |

## 2. Temporal contracts

* **Clock**: `core::MonotonicClock64` extends `micros()` to 64 bit (wrap-safe, sampled every loop). The planner, scheduler, encoder stamps and walk commands use ONE time base (`MotionCoordinator::bindClock`). Unit test: 32-bit wrap, 10 s across 2³² µs (MISURATO HOST).
* **Scheduler**: absolute deadlines, `next_release += period`; a late poll releases ONCE, dropped instants are counted (`overruns`, `skipped`, `max_lateness_us`), never replayed.
* **Reference**: phase = f(clock − start), common to the four legs; targets generated for the actual release instant; checked per frame for joint limits, continuity/rates over the REAL interval (vmax·dt), analytic velocity/acceleration envelope, support, and membership in the certified per-joint box. A gap > 100 ms (5 periods) is refused (walk stops), never planned through.
* **Executor sequences**: `validated_sequence` (whole frame checked) → `tx_sequence` (all bytes left the UART, TX-complete; NOT applied) → `observed_sequence` (every joint sampled after that TX) → `settled_sequence` (HOLD target reached by fresh samples inside the tube with zero measured speed).
* **Tracking monitor**: per joint, per NEW sample at its own instant `t_s`: error = distance of `q_meas` from the range of goals that were active during `[t_s − 150 ms, t_s]`. > 6 mrad for 3 consecutive samples → `TRACKING_LAG_PERSISTENT` (physical failure to follow). Immediate faults kept: limits/mapping, torque, status/temperature/voltage (producer), stale sample (100 ms), measured joint outside certified box ± tube.
* **Geometry argument**: the certificate covers every configuration in the per-joint box `[min−tube, max+tube]`, each joint independent; every target is inside `[min,max]`; every measured joint is inside the box ± tube or the executor stops. Tracking lag inside the box is therefore a following/health question, not a collision question.
* **HOLD/STAND**: the final all-foot target is emitted once; STAND is declared only when the executor reports it settled (observed after its TX, inside tube, measured speed = 0) AND the observation is stationary.
* **STOP**: zero-velocity WALK STOP = coordinated deceleration over two periods to all-foot STAND with torque. MOTION.STOP / heartbeat deadman / any fault = SAFE_OFF (`executor.stop`), after which `stream()` refuses every frame (state ≠ STREAMING) — no target after revocation.

## 3. State machine (executor)

```
OFF --arm--> READY --submit--> WRITING --12 writes--> TRACKING --tracked--> READY      (prime / entry / STAND: finite segments, unchanged)
READY --beginStream--> STREAMING --stream(final)--> HOLD --settled + endStream--> READY
STREAMING/HOLD --monitor fault | stop() | gate--> SAFE_OFF_REQUIRED --all 12 verified OFF--> OFF
```

## 4. Dataflow (one Controller loop)

```
micros() -> MonotonicClock64
STOP lane (HttpTransport::serviceStopLane, admin lease + session + CSRF) -> MotionService STOP -> session.revoke -> executor.stop
  -> emergencyTorqueOffBurst: SYNC TorqueEnable=0 (12 ids, 32 B)              [before any poll]
... command router, IMU, BMS, census/preflight, readiness ...
updateOperationalMotion:
  fast reads 56..70 round-robin (<=3 per loop, 2 ms budget), slow 40..49 one joint / 100 ms
  -> OperationalFeedback (per-joint q, speed, sampled_at_us)
  -> observation (stamp = oldest joint sample; support = declaration while present)
  -> MotionCoordinator.update(now_us):
       executor.monitor (async tracking)  -> scheduler.poll -> FeedbackWalk.streamUpdate
       -> executor.stream: whole-frame validation -> SafeActuator planFrame/commitFrame
       -> ActuatorRuntime.executeFrame -> ServoBusActuatorBackend -> ServoBus SYNC_WRITE (104 B, TX-complete)
  SAFE_OFF verification: readback per joint, unicast OFF retry
HttpTransport.update (ordinary mailbox; commands older than the STOP epoch -> SUPERSEDED_BY_STOP)
```

## 5. Modules preserved (not modified in behaviour)

CAD REV01, FK/IK, Gait/derivatives, direction/identity mapping, calibration transforms and gen-1 limits, PRONE recognition, measured-start entry and STAND route (segment mode unchanged; STAND end-to-end ×3 PASS), automatic RESTORE gen-1, OperationalReadiness, dynamic Motion Authority, Android Twin (read-only), CSRF/auth, heartbeat 3 s window, zero torque at boot.

## 6. Not done / open

* SYNC_WRITE on the installed C018 servos is NOT hardware-qualified (packet bytes are identical to the library's; no device was touched).
* Phase scaling under tracking degradation is not implemented: persistent lag stops the walk (observable reason), the reference is never stretched.
* Gait geometry/rates (SLO 20–25 mm/s, 2–4 s cycles) are NOT changed in this rework; see GAIT_PERFORMANCE_MATRIX.md.
