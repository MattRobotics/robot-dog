# Steered SLOW box `kSteer` re-certified for `random_3` (2026-11-18)

Provenance of `src/simple/SteerCollisionData.h` (`kSteer`, `kSteerProof`).
Geometry only: no device, no dynamics, no hardware qualification.

## The defect

The joystick envelope gate failed in SLOW (`JOY_MODE=0`, scenario `random_3`).
The cause is pre-existing: 30c894a fails the same way, and it fails before the FP32 change.

- RF UPPER exceeded the `kSteer` upper bound (0.96346 rad) by 1–2 RAW ticks.
- This happened at the `kSteerTilt` pitch-up vertex (+55 mrad of IMU correction).

Bisection of the SLOW law after the dev.15 certification (`9a6d105`):

| Commit | `random_3` margin |
|---|---|
| `9a6d105` | +41.5 mrad |
| `d36ce04` | +6.3 mrad |
| `5472b65` | +1.7 mrad |
| `23577ad` | +0.1 mrad |
| `b4ce6d5` and later | −3.0 mrad |

The SLOW steered law evolved after the certification, and the box was never re-certified.

On the robot, `compensationSafe` would refuse that correction (`WORKSPACE`) or stop the walk (`WALK_IK_OR_WORKSPACE`). This is not a collision: 3 mrad moves the thigh by ≤ 0.6 mm, against a certified clearance of ≥ 3 mm.

## Procedure

Same tools and criteria as `kSteer` (dev.15) and `kSteerStandard`/`kSteerFast` (`Motion_vNext_Multidirectional_2026-11-11`).

1. **Envelope collection.**
   - The candidate `05e1214` (P1 + FP32) ran the 45 gate scenarios of `run_joystick_envelope.sh` plus a 52-direction grid: 97 scenarios, SLOW limits.
   - It ran on a scratch copy whose steered walks used the gen-1 limits (measurement only, as on 11/11).
   - A measurement-only patch of the envelope tool recorded, per scenario, the frames walked in the steered regime: `walkBox() = kSteer`, nominal targets and the RAW-rounded IMU-ray targets.
   - Result: `steer_envelope_slow_steered.txt`, 79 scenarios with steered frames.
2. **Comparison with `kSteer`.**
   - Every joint of that envelope lies inside `kSteer`, except the RF UPPER upper bound: 0.964874 rad, 1.42 mrad above.
3. **Minimal box.**
   - `kSteer` with only that bound moved to the envelope + 40 mrad (1.004874 rad). The other 23 bounds are unchanged.
   - Input file: `steer_slow_certificate_input.txt`, one row equal to `kSteer` narrowed by the tube, except RF UPPER at the envelope maximum.
   - `certify_steer_box.py` adds back `--tube-rad .040`.
4. **Certificate.** `certify_steer_box.py --tube-rad .040`, clearance 3 mm (+3 mm ventral). File: `steer_slow_certificate.json`.

| Box | Non-adjacent pairs | Adjacent pairs | Certified lower bound (required 3 mm) | Minimum cell-centre distance |
|---|---|---|---|---|
| `kSteer` dev.15 (`3cd54de1…`) | 120/120 | 12/12 | 3.035 mm | 10.87 mm |
| `kSteer` this certificate (`06c38f8d…`) | 120/120 | 12/12 | 3.085 mm | 10.87 mm |

The SHA256 of `steer_slow_certificate.json` is `kSteerProof`. Its `envelope_sha256` is the SHA256 of `steer_slow_certificate_input.txt`.

## Verification

- The joystick envelope gate passes in SLOW with this box (`random_3` included).
- `kSteerTilt`, `compensationSafe`, the STOP proof and every other box are unchanged.

Not modelled (as for `kSteer`): fixture, cables, battery, reprinted cover beyond the ventral allowance, servo horn hardware.
