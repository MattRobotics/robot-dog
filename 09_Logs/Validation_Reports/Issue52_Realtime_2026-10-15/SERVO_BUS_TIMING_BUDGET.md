# Servo bus timing budget — Issue #52 — 2026-10-15

All numbers are **STIMATO** (1 Mbaud, 8N1 = 10 µs/byte) unless labelled. **NON MISURATO HARDWARE**: nothing here was measured on the robot in this session.

## Packets (bytes from src/servo/ServoPacket.h; equal to SCServo 1.0.2 — MISURATO HOST, `run_servo_packet_tests.sh`)

| Transaction | Bytes on wire | Wire time | Reply |
|---|---|---|---|
| SYNC_WRITE goal frame, 12 ids, SRAM 41..47 | 104 | 1.04 ms | none (no ACK exists) |
| SYNC_WRITE TorqueEnable=0, 12 ids | 32 | 0.32 ms | none |
| Fast read 56..70 (15 B) | 8 req + 21 reply | 0.29 ms + servo return delay + turnaround | status packet, parsed with header/id/len/checksum |
| Slow read 40..49 (10 B) | 8 + 16 | 0.24 ms + delay | idem |
| Old per-sample read (40..49 + 56..70) | 53 | 0.53 ms + 2× delay | — |
| Old per-joint write (WritePosEx + 10 B readback) | 15 + 6 + 8 + 16 | ≥0.45 ms ×12 per frame | ACK + readback |

## Per-frame budget at 50 Hz (20 ms)

* Goal frame TX: 1.04 ms (+ `HardwareSerial::flush` until the shifter is empty; recorded as `tx_complete_us`).
* Feedback: ≤3 fast reads per loop within a 2 ms budget. With ~0.45 ms per read (wire 0.29 ms + return delay/turnaround, STIMATO) and a 2 ms loop: ≈1.5 joints/ms → a full 12-joint round in ≈8–10 ms (STIMATO). With the old 1 joint / 2 ms poll the round was ≥30.36 ms (audit), ≈60 ms at a 5 ms loop.
* Slow control block: 1 joint / 100 ms → each joint's torque/goal/profile/limit re-verified every 1.2 s (plus the arm-time readbacks, unchanged). A fast sample is refused if that joint's control block is older than 1.5 s.
* Bus occupancy at 50 Hz: 1.04 ms TX + ~13 ms of reads per 20 ms → ≈70 % (STIMATO). 100 Hz frames would leave ≈4 ms per period for reads (≈8 joints/period): feasible only with a shorter loop; 20 Hz halves TX load but coarsens the reference. 50 Hz is the first hypothesis, to be measured on the fixture.

## STOP budget (offline targets to qualify, NOT measured values)

| Metric (Controller prints `MOTION_SAFE_OFF=VERIFIED …`) | Budget |
|---|---|
| latch → last target TX (no new frame after latch; `stream()` refuses outside STREAMING) | ≤ 10 ms (one loop) |
| latch → SYNC TorqueEnable=0 TX complete (32 B, before any poll in that loop) | ≤ 10 ms; all OFF attempts ≤ 50 ms on a healthy bus |
| latch → all 12 VERIFIED_OFF (readback per id, unicast OFF retry) | 12 × (read ≈0.3–8 ms timeout) → ≤ 100 ms healthy; an absent servo stays UNVERIFIED and is retried each round; the physical cut remains the fallback |

## HTTP / Web

* Dashboard STOP is sent on its own lane (not behind `sendBusy`); the firmware STOP lane is a separate one-slot handoff consumed first in the loop, budget 600 ms for the HTTP reply; the ordinary mailbox keeps its 1.2 s / 3 s budgets.
* Web polling (status ≈1 Hz) and the heartbeat (presence 3 s) no longer gate the gait (HEARTBEAT_1HZ test: 49.9 frames/s with a 1 Hz heartbeat — SIMULATO).

## To measure on the fixture (separate decision, not authorized here)

1. SYNC_WRITE acceptance by the installed C018 units (goal latch readback after a frame, all 12 ids).
2. Servo return delay / turnaround per read; loop period distribution with Wi-Fi AP active.
3. STOP latency metrics above with a saturated bus and with one servo unpowered.
4. GoalSpeed/ACC real profile under load (ACC ×100 steps/s² is a register hypothesis).
