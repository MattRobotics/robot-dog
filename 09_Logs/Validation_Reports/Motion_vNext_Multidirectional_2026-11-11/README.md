# Motion vNext — steered joint boxes of STANDARD and FAST (2026-11-11)

Provenance of `src/simple/SteerStandardCollisionData.h` (`kSteerStandard`) and
`src/simple/SteerFastCollisionData.h` (`kSteerFast`). Offline geometry only: no
device, no dynamics, no hardware qualification.

| File | Content |
|---|---|
| `steer_envelope_standard.txt`, `steer_envelope_fast.txt` | Per-joint envelopes written by `scripts/offline_motion/joystick_envelope.cpp` (`JOY_ENVELOPE`), one row per scenario: nominal targets and IMU-corrected targets at the rays of the steered envelope `kSteerTilt` |
| `steer_standard_certificate.json`, `steer_fast_certificate.json` | `scripts/offline_motion/certify_steer_box.py` certificates (`--tube-rad .040`, clearance 3 mm, ventral +3 mm). Their SHA256 is the `kSteerStandardProof` / `kSteerFastProof` of the headers; `envelope_sha256` matches the envelope files here |
| `collect_steer_envelope.sh` | The collection run: the 45 gate scenarios of `run_joystick_envelope.sh` plus a 52-direction grid (26 directions x 0.5 and 1.0 of the limits), 97 scenarios per mode |
| `collection_measurement_patch.diff` | Measurement-only patch of the scratch copy used for the collection: STANDARD/FAST steered walks evaluated on the gen-1 joint limits (`walkBox`, `regimeHandover`, `pointSafe`), so that no walk ends before its envelope is recorded. Never in the firmware |

Collection limits, per gait cycle of each mode's own law: steered fore/aft 45 mm,
lateral 20 mm, yaw 0.16 rad, sway 10 mm (the dev.6 SLOW values). The firmware
limits (`JoystickSpec`) are smaller (STANDARD 20 mm / 0.12 rad / sway 9 mm, FAST
12 mm / 0.08 rad / sway 7.5 mm): their walks lie inside the collected envelope,
and the joystick gate re-checks every frame and IMU ray against the boxes.

| Box | Scenarios | Non-adjacent pairs | Adjacent pairs | Minimum clearance | Time |
|---|---|---|---|---|---|
| `kSteer` (SLOW, dev.15) | 68 | 120/120 | 12/12 | 10.87 mm | 122 s |
| `kSteerStandard` | 97 | 120/120 | 12/12 | 11.08 mm | 115 s |
| `kSteerFast` | 97 | 120/120 | 12/12 | 10.99 mm | 117 s |

Not modelled (as for `kSteer`): fixture, cables, battery, reprinted cover beyond
the ventral allowance, servo horn hardware.
