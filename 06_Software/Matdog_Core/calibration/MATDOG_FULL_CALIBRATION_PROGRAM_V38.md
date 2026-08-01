# MATDOG Full Calibration Program V38

Date: 2026-08-01  
Status: implementation and offline verification in progress  
Primary runtime: NormaCore Station native ST3215 calibrator

## Objective

Move from individually armed contact pilots to a single operator action that:

1. verifies both mechanical contacts of every joint;
2. compares the measured contacts with the canonical REV00 URDF;
3. derives a calibrated software `q = 0` without changing encoder scale;
4. places the calibrated leg, and later the complete robot, at the accepted HOME;
5. finishes with verified global torque OFF.

The rollout remains evidence-driven:

```text
LF full sequence
→ LH rear-leg geometry and hardware sequence
→ RF mirrored front sequence
→ RH mirrored rear sequence
→ LF → RF → RH → LH complete 12-joint orchestration
```

## Completed LF evidence

All three LF joints have completed supervised MIN and MAX contact acquisition:

| Joint | Servo | MIN | MAX | Repeatability |
|---|---:|---:|---:|---:|
| upper | M12 | 1443 / 1443 | 3443 / 3442 | 0 / 1 ticks |
| lower | M11 | 3094 / 3092 | 1664 / 1666 | 2 / 2 ticks |
| hip | M13 | 2530 / 2530 | 1595 / 1595 | 0 / 0 ticks |

The combined HIP program completed `Done 20/20`, restored the LF geometry, executed verified global torque OFF and released the serial port.

## Why repeatability is not enough

A repeated contact can still be a repeated servo-settle point rather than the fully seated mechanical stop. This is particularly relevant to the M13 MAX observation, which was repeatable but had low current and stopped at 1595 ticks while the URDF limit is 1536 ticks.

The production calibrator therefore applies two independent gates:

```text
contact repeatability
+
URDF endpoint consistency
```

A contact pair is not allowed to redefine HOME solely because its spread is small.

## V38 motion envelope

The first pilots used a deliberately slow and low-energy motion envelope. Since all six LF contacts and the prerequisite poses have now been exercised physically, V38 uses:

```text
TorqueLimit = 500
GoalSpeed   = 160
Acc         = 8
coarse step = 64 ticks
fine step   = 8 ticks
settle      = 900 ms
```

Unchanged protections:

- model-derived contact corridors;
- 64-tick guard beyond each URDF endpoint;
- status and driver-error aborts;
- hard-current abort at raw 200;
- unsigned ST3215 `GoalPosition`;
- Station-only serial ownership;
- global torque OFF on success and failure.

The extra torque and speed are intended to remove the previous under-seating ambiguity. They do not authorize travel beyond the existing guard.

## Model-derived q=0

For each joint the encoder scale and direction remain fixed. For each endpoint:

```text
zero_candidate = measured_contact - direction × URDF_endpoint_delta_ticks
```

This produces:

```text
zero_from_MIN
zero_from_MAX
```

The calibrated software HOME is accepted only when:

```text
circular_distance(zero_from_MIN, zero_from_MAX) <= 24 ticks
and
circular_distance(estimated_zero, 2048) <= 96 ticks
```

The estimate is the circular midpoint of the two zero candidates.

No joint-specific encoder scale is fitted. The fixed ST3215 conversion remains the kinematic contract.

## Current LF model-zero diagnosis

The evidence already collected produces:

| Joint | zero from MIN | zero from MAX | disagreement | Decision |
|---|---:|---:|---:|---|
| M12 upper | 2040 | 2048 | 8 | accepted candidate 2044 |
| M11 lower | 2046 | 2092 | 46 | re-acquire |
| M13 hip | 2018 | 2107 | 89 | re-acquire |

This confirms the operator observation: M13 MAX, and to a lesser extent M11 MAX, must be repeated with a firmer and faster approach before replacing HOME.

## One-click LF sequence

Explicit arm token:

```text
LF_LEG_FULL_V38
```

One press of **Auto Calibrate** executes:

```text
M12 MIN → backoff → repeat → HOME → torque OFF
M12 MAX → backoff → repeat → HOME → torque OFF
M11 MIN → backoff → repeat → HOME → torque OFF
M11 MAX → backoff → repeat → HOME → torque OFF
M13 MIN → backoff → repeat → HOME
M13 MAX → backoff → repeat → HOME → torque OFF
compute three URDF model-zero estimates
```

When all three estimates pass:

```text
M13 → calibrated q=0
M12 → calibrated q=0
M11 → calibrated q=0
verify all three holds
global torque OFF
Done
```

When one estimate fails:

```text
MODEL_ZERO_INCONSISTENT
contacts remain logged
HOME is not replaced
global torque OFF
Failed
```

This is one Station-owned sequence; it is not a chain of external Python servo commands.

## EEPROM policy

The 2026-07-10 digital-zero Position Offset calibration remains preserved as historical hardware configuration. V38 does not write:

- Position Offset;
- EEPROM LOCK;
- reset or ResetCalibration;
- RegWrite or Action;
- Save or Freeze.

The post-contact HOME is initially a **software model-zero target**. An EEPROM migration may be evaluated only after all 12 joints have consistent two-endpoint evidence and the regenerated four-leg FK closes against the URDF.

## RF transfer

RF is the mirrored front-leg case. The same logical sequence is directly reusable with the canonical mapping:

```text
RF: hip M23, upper M22, lower M21
parking joint: RH upper M32
```

The code must instantiate the mirrored joint directions and existing RF profile table; it must not copy LF tick signs manually.

## LH rear-leg development gate

LH is the next hardware sequence requested after LF. Its mapping is:

```text
LH: hip M43, upper M42, lower M41
```

UPPER and LOWER horizontal/parallel placement remains the expected HIP prerequisite. A front-leg clearance pose must be selected by an offline collision sweep before LH hardware is enabled. The likely mechanism is parking LF through M12, but no exact target is accepted merely from visual intuition.

Required LH gates:

1. enumerate candidate LF M12 parking angles;
2. validate all LH prerequisite paths and both contact corridors against collision meshes;
3. select the smallest sufficient parking movement;
4. lock the target and reverse recovery order in tests;
5. run the complete LH sequence with one explicit arm token;
6. verify model-zero consistency and final HOME.

## Complete four-leg program

The final operator program will use one arm token and one Auto Calibrate action:

```text
LF → RF → RH → LH
```

Each leg must finish its own contact and model-zero gates before the next leg starts. Between legs the program performs verified global torque OFF and validates the exact 12-servo topology.

The full program becomes hardware-eligible only after:

- LF V38 PASS with all three model-zero estimates accepted;
- LH rear-clearance sequence PASS;
- mirrored RF and RH offline profile tests PASS;
- four-leg final-HOME collision and FK audit PASS.

## Canonical files

- `MATDOG_LF_CONTACT_EVIDENCE_2026-08-01.yaml`
- `matdog_model_zero_solver.py`
- `tests/test_matdog_model_zero_solver.py`
- `MATDOG_JOINT_CALIBRATION.yaml`
- canonical URDF: `03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf`

NormaCore implementation branch:

```text
MattRobotics/norma-core
matdog/full-calibration-v38
```

MATDOG record branch:

```text
MattRobotics/robot-dog
matdog/full-calibration-program-v38
```
