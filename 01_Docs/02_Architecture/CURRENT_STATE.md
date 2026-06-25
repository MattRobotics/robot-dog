# MATDOG Current State

## Confirmed

- Twelve ST3215 servos are detected and responsive through the Waveshare Bus Servo Adapter.
- The custom distribution board and branch wiring are validated for the current development phase.
- The current runtime architecture remains `Station → Waveshare → ST3215`.
- ESP32 integration is deferred until the robot has completed stand, IK and walking validation.
- The canonical leg order is `[LF, RF, RH, LH]`.
- Trot diagonal pairs are `[LF, RH]` and `[RF, LH]`.

## Canonical Servo Mapping

| Leg | Hip | Upper | Lower |
|---|---:|---:|---:|
| LF — Front Left | M13 | M12 | M11 |
| RF — Front Right | M23 | M22 | M21 |
| RH — Rear Right | M33 | M32 | M31 |
| LH — Rear Left | M43 | M42 | M41 |

## Geometry Known Today

- Front-to-rear hip-axis spacing: `225 mm`
- Left-to-right hip-axis spacing: `95 mm`
- Hip-to-knee nominal segment: `90 mm`
- Knee-to-foot nominal segment: `110 mm`
- Target body height in stand: `150 mm`

## Not Yet Validated

- Final CAD-derived hip-axis height relative to `base_link`.
- Final URDF mesh geometry and each joint origin.
- Foot-contact frame offsets caused by the eccentric rubber feet.
- Encoder zero values for the mechanical zero pose.
- Joint direction signs and software motion limits.
- Stand pose, leg IK, gait generation and MATDOG Station integration.

## Next Single Technical Objective

Create and validate the first CAD-derived MATDOG URDF with the canonical joint tree and names.

## Safety Rule

No automated multi-servo pose, gait or body-velocity command may be enabled until calibration and joint limits are documented and validated.
