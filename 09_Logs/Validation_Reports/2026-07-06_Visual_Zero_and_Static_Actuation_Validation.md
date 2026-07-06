# MATDOG — Visual-Zero Calibration and Static Actuation Validation

**Date:** 2026-07-06
**Status:** APPROVED AS VISUAL-ZERO AND STATIC-ACTUATION BASELINE
**Calibration status:** `VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION`

## Scope

This report records the first hardware-calibration checkpoint for the 12 ST3215 leg servos. It approves the visual-zero reference, encoder-to-URDF conversion, static hold and controlled return-to-zero operation. It does **not** approve unsupported standing, gait, dynamic locomotion, final mechanical limits or `zero_encoder_final`.

## Visual-Zero Capture

The robot was manually placed in the intended URDF visual-zero pose. A read-only Station capture acquired 18 samples for every joint on bus `5B14114953`. Circular encoder spread was **0 tick for all 12 joints**.

| Joint | Servo | Visual-zero tick |
|---|---:|---:|
| lf_hip_joint | M13 | 3541 |
| lf_upper_leg_joint | M12 | 2819 |
| lf_lower_leg_joint | M11 | 2069 |
| rf_hip_joint | M23 | 4070 |
| rf_upper_leg_joint | M22 | 1072 |
| rf_lower_leg_joint | M21 | 4070 |
| rh_hip_joint | M33 | 1492 |
| rh_upper_leg_joint | M32 | 1011 |
| rh_lower_leg_joint | M31 | 4038 |
| lh_hip_joint | M43 | 1221 |
| lh_upper_leg_joint | M42 | 2941 |
| lh_lower_leg_joint | M41 | 140 |

Evidence artifact: `09_Logs/Calibration_Sessions/2026-07-06_205031_visual_zero_capture.result.yaml`.

## Validation Completed

- Calibration YAML, servo mapping, direction signs, URDF axes, joint groups and canonical URDF hash passed validation.
- For all 12 joints, `encoder == zero_encoder_visual` produced `q = 0 rad`.
- Circular encoder wrap-around and `encoder -> rad -> encoder` round-trip passed.
- Live read-only telemetry was verified on LF M11, M12 and M13; the expected URDF sign was observed for both encoder directions.
- Controlled micro-probes passed on M11 and M12. M13 moved and returned exactly; its strict target residual was 9 ticks and remains an item for later load review.
- Static hold passed for each leg individually and for all 12 servos together.
- Guarded all-servo move to visual zero passed after a preflight limit of ±30 ticks.

## Coordinated Visual-Zero Result

Final errors from saved target:

| Servo | Error |
|---|---:|
| M13 | 12 ticks |
| M12 | 2 ticks |
| M11 | 2 ticks |
| M23, M22, M21 | 0 ticks |
| M33, M32 | 0 ticks |
| M31 | 1 tick |
| M43, M42, M41 | 0 ticks |

All 12 servos were inside the configured 12-tick integration threshold. Cleanup disabled torque on all 12 servos.

## Approved Boundary

Approved next work:

1. Live forward kinematics from Station telemetry for one leg.
2. Physical comparison of calculated foot position.
3. Single-leg inverse kinematics.
4. Supported low-stand planning.

Open items:

- `zero_encoder_final` remains unset.
- M13 tracking must be revisited under controlled load.
- Physical safe limits and first-stand limits remain unset.
- Unsupported stand, gait and dynamic movement remain prohibited.
