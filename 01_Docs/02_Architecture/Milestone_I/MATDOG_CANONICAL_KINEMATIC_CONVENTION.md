# MATDOG canonical kinematic convention

Status: `CURRENT_CANONICAL` for Milestone I.1.

## Model identity and order

The canonical model is the REV00 URDF at the hash recorded in the source
manifest. It contains 12 revolute joints and four fixed foot joints. Canonical
leg order is `LF, RF, RH, LH`; joint order within each leg is `hip, upper,
lower`. The complete parent/child, origin, axis, limits, servo mapping, and
evidence are machine-readable in `joint_registry.csv`.

The mapping is:

| Leg | Hip | Upper | Lower |
|---|---:|---:|---:|
| LF | M13 | M12 | M11 |
| RF | M23 | M22 | M21 |
| RH | M33 | M32 | M31 |
| LH | M43 | M42 | M41 |

Encoder-to-URDF-q signs supported by hardware direction probes are:

| Leg | Hip | Upper | Lower |
|---|---:|---:|---:|
| LF | -1 | +1 | -1 |
| RF | -1 | -1 | +1 |
| RH | +1 | -1 | +1 |
| LH | +1 | +1 | -1 |

For unwrapped local differences,
`q_rad = direction * signed_tick_delta(displayed_tick, 2048) * 2*pi/4096`.
This formula does not authorize a wrapped command. `GOAL_POSITION` remains an
unsigned `0..4095` value.

## Geometry and limits

The URDF uses metres and radians. Hip axes are `+X`; upper and lower axes are
`+Y`. Hip limits are +/-0.785398163397 rad, upper limits are
[-0.916297857297, 2.138028333693] rad, and lower limits are
[-1.605702911835, 0.654498469498] rad. Effort and velocity fields are preserved
in the joint registry as URDF model parameters, not hardware validation.

The hip-to-knee joint offset is 0.09 m. The knee-to-nominal-foot-frame chain
is represented by the lower link and fixed foot origin; no absent CAD value
is inferred. Front hip origins are 0.0200 m higher than rear hip origins in
`base_link`. This asymmetry is intentional model data and is not normalized.

## Proof boundary

No new FK or IK is defined here. Existing repository kinematics are evidence
inputs only. URDF joint limits are model constraints, not measured contacts or
safe limits. Collision uses detailed mesh geometry for offline inspection;
per-joint compliance cannot establish configuration-level collision safety.

The URDF custom `motorDirection` values disagree with multiple hardware-probe
signs. Their semantics are therefore `DECISION_REQUIRED`; the hardware-backed
calibration direction is the canonical encoder-to-q sign, without editing the
URDF in this milestone.
