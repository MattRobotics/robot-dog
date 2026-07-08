# MATDOG C3-B — Live FK read-only validation

Date: 2026-07-08

## Scope

Validation of the generalized MATDOG live FK read-only tool after C3-A.

The tool was used only through NormaCore Station telemetry:

- no direct serial access
- no torque enable/disable
- no goal command
- no speed command
- no accel command
- no stand pose
- no gait command

## Result

PASS.

The live FK read-only pipeline was validated for all four legs:

| Leg | Servo order | Tip link | Result |
| --- | --- | --- | --- |
| LF | M13, M12, M11 | lf_foot_link | PASS |
| RF | M23, M22, M21 | rf_foot_link | PASS |
| RH | M33, M32, M31 | rh_foot_link | PASS |
| LH | M43, M42, M41 | lh_foot_link | PASS |

## Observed static offsets from visual zero

The robot was not perfectly re-aligned to visual zero during this read-only session.

Approximate observed visual-zero errors:

| Leg | Max observed error |
| --- | --- |
| LF | ~6 ticks |
| RF | ~37 ticks |
| RH | ~19 ticks |
| LH | ~41 ticks |

These offsets do not invalidate this validation step, because C3-B validates the live telemetry → encoder → radians → URDF FK chain.

They do mean that strict `--require-visual-zero-tolerance 10` should not be expected to pass on every leg unless the robot is manually re-aligned closer to visual zero.

## Gate status

C3-B live FK read-only validation is complete.

Next gate before any commanded stand:

1. design offline safe stand candidate
2. validate all 12 joint angles against URDF limits
3. validate against MATDOG safe limits once available
4. validate contact geometry
5. validate self-collision / ground-collision assumptions
6. design rest-to-stand trajectory
7. only then consider a restrained/suspended commanded test
