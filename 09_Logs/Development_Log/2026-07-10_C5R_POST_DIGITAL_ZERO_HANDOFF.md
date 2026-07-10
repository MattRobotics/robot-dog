# MATDOG — C5-R Post-Digital-Zero Handoff

## Repository

```text
repository: ~/MATDOG/github/robot-dog
branch: main
```

Record the pushed commit with:

```bash
git rev-parse HEAD
```

## Completed state

The 12-servo ST3215 digital-zero calibration is complete.

Final verified state:

```text
target displayed mechanical q=0: 2048 ticks
all 12 planned EEPROM offsets: PASS
EEPROM lock: 1 on all 12 servos
torque: 0 on all 12 servos
maximum raw deviation from q0 capture: 3 ticks
```

Final audit:

```text
09_Logs/Calibration/C5_R_digital_recenter/
2026-07-10_145457Z_final_12_offset_readback.json
```

SHA-256:

```text
15619d23ddcb17651ba729a0d69309b5b56befeb3377f123ff7f131582fcf8ec
```

Canonical software:

```text
06_Software/Matdog_Core/calibration/
matdog_digital_zero_calibration.py
```

Canonical digital-zero procedure:

```text
06_Software/Matdog_Core/calibration/
MATDOG_DIGITAL_ZERO_CALIBRATION.md
```

## Final digital-zero offsets

```text
M11 +101    M12 +859    M13 -505
M21 -1986   M22 -891    M23 -1687
M31 -2021   M32 -953    M33 -470
M41 -1824   M42 +979    M43 -740
```

## Interpretation

Digital zero is finished and must not be repeated now.

It maps the validated physical mechanical `q = 0` pose to displayed encoder
position `2048`. It does not measure mechanical end stops or establish the final
safe travel envelope.

## Next objective

Design and validate a MATDOG-specific automatic mechanical end-stop calibration
for all 12 joints, with a single operator action and a guarded internal
one-joint-at-a-time sequence.

Planning document:

```text
06_Software/Matdog_Core/calibration/
MATDOG_MECHANICAL_ENDSTOP_CALIBRATION_PLAN.md
```

The next phase must:

1. audit the elrobot-style automatic calibration architecture;
2. define a MATDOG collision-safe dependency order;
3. define low-energy speed, acceleration, current/effort and timeout guards;
4. detect contact using encoder progress, velocity, current and repeatability;
5. back off after contact and repeat the measurement;
6. populate `measured_contact_rad` and `safe_limit_rad` only after validation;
7. preserve all digital-zero EEPROM offsets;
8. complete a new read-only joint-state/FK closure;
9. regenerate and audit all C5 encoder targets;
10. run a new supervised C5 stand only after explicit authorization.

## C4 and C5 status

C4-A through C4-E remain valid as offline geometric, kinematic, collision,
timing and stability references.

C4-F requires a post-limit-calibration hardware preflight update.

The previous C5 executor is retained only as a pre-recenter engineering
reference. It must not be reused directly because all angle-to-encoder target
conversion and hardware-envelope checks must be regenerated against:

- the new digital zero;
- the measured mechanical contact limits;
- the derived conservative safe limits.

## Non-negotiable constraints

- Station is the sole serial owner.
- No direct `pyserial`.
- No generic NormaCore `AutoCalibrate`.
- No signed `GoalPosition`.
- Use normal unsigned ST3215 goal semantics.
- No hardware movement without explicit authorization.
- Keep the robot supported.
- Calibrate one joint at a time.
- Abort on ambiguous contact, unexpected current, missing telemetry,
  self-collision risk or prerequisite-joint drift.
