# MATDOG LF calibration V25 — final canonical record

**Date:** 2026-08-04  
**Leg:** LF  
**Status:** hardware validated, EEPROM frozen, persistent profile saved  
**NormaCore PR:** `MattRobotics/norma-core#11`  
**Frozen source branch:** `release/matdog-lf-calibrator-v25`  
**Reviewed source head:** `f87dd1fbc7e8100d275c74f9af448642f3429680`

## 1. Final result

The supervised V25 run completed the entire LF sequence:

```text
step count: 58/58
runner: PASS
global torque OFF: verified
Station shutdown: graceful
serial adapter: released
stage: LF_STAGED
EEPROM freeze: LF_FROZEN
persistent profile: LF_FROZEN
residual process/listener/serial owner: none
```

This closes the LF mechanical calibration milestone. The sequence must not be repeated unless LF mechanics, mounting, servo, URDF or calibration state changes.

## 2. Final contact evidence

| Joint | Servo | Coarse/final behaviour | Accepted MIN | Accepted MAX | Fine repeatability |
|---|---:|---|---:|---:|---:|
| LF hip | M13 | bounded 13-tick directional settle handled before MAX contact | 2535 | 1600 | 2 ticks at MAX |
| LF upper | M12 | cable obstruction removed; contact returned to supervised witness | 1439 | 3443 | 1 tick at MAX |
| LF lower | M11 | chamfer/friction plateau crossed using coarse-depth evidence | 3093 | 1658 | repeatable |

The final M13 MAX sequence reached the real contact after the V24 false abort:

```text
coarse: 1595
fine #1: 1599
fine #2: 1601
spread: 2 ticks
```

The final M12 MAX returned to the expected physical region:

```text
coarse: 3446
fine #1: 3443
fine #2: 3444
spread: 1 tick
```

## 3. URDF comparison and affine q0

| Joint | URDF range | Measured mechanical range | Difference | Affine q0 before EEPROM |
|---|---:|---:|---:|---:|
| M13 hip | 90.00° | 82.18° | −7.82° | 2067 |
| M12 upper | 174.99° | 176.13° | +1.14° | 2040 |
| M11 lower | 129.55° | 126.12° | −3.43° | 2074 |

The calibrated affine model is conceptually:

```text
raw_tick = q0_affine + direction × calibrated_scale × q_URDF
```

The q0 values were not obtained from a simple midpoint for the asymmetric upper and lower joints. They were derived from both measured endpoints and the respective URDF MIN/MAX angles.

Endpoint residuals after affine alignment:

```text
M13: approximately +0.05° / +0.04°
M12: approximately +0.02° / −0.07°
M11: approximately −0.07° / −0.01°
```

## 4. EEPROM provisioning result

| Motor | Previous Position Offset | Frozen Position Offset | Final displayed position |
|---|---:|---:|---:|
| M11 lower | 101 | 127 | 2046 |
| M12 upper | 859 | 851 | 2051 |
| M13 hip | −505 | −486 | 2048 |

Final readback contract:

```text
EEPROM LOCK = 1
torque enabled = 0
q0 displayed residual = 0..3 ticks
```

The transaction occurred only after measurement PASS, URDF/witness gate PASS, Station shutdown and serial release.

## 5. V25 general rules

### 5.1 Static-joint validation

A raw speed spike is not independent proof that a held or torque-off joint moved. Static-role validation remains fail-closed on:

- real position drift beyond the role tolerance;
- goal modification;
- unexpected torque state;
- hard-current, status, thermal or telemetry-integrity failure.

### 5.2 Friction/chamfer-aware fine contact

The coarse scout establishes a previously demonstrated reachable depth. If a fine pass reports contact more than one fine step before that depth, it is treated as a bounded friction/chamfer plateau and the approach continues in controlled fine increments.

This does not widen:

- the outer URDF guard;
- the contact-current envelope;
- hard-current abort;
- timeout;
- repeatability;
- model/witness gates.

### 5.3 Bounded tracking lag

The fine approach loop uses the same global 16-tick directional-settle floor already accepted by the detector:

```text
13 ticks → continue
17 ticks → fail closed
```

The rule is general and not specific to M13.

### 5.4 Uniform hardware witness

All six contacts must remain within one uniform supervised-LF witness band. This rejected the cable-obstructed M12 MAX around 3397 and accepted the unobstructed result around 3443.

### 5.5 Affine authority

The affine q0/scale gate and hardware witness are authoritative for staging and persistent provisioning. Fixed nominal encoder-scale disagreement remains visible as diagnostics and is not silently discarded.

## 6. Architecture boundaries

```text
Station/native Rust state machine
→ sole motion authority and serial owner

external runner
→ orchestration, latest-only evidence, live progress and shutdown verification

EEPROM provisioner
→ starts only after Station exits and the serial adapter is released
```

The provisioner performs:

```text
backup
→ EEPROM unlock
→ staged RegWrite operations
→ Action
→ readback
→ EEPROM relock
→ rollback of already-modified LF offsets on failure
```

## 7. Permanent constraints

- `GoalPosition` is unsigned standard `0..4095`.
- Signed-wrap is forbidden.
- The digital-home commissioning program remains separate.
- Only explicitly involved joints may be commanded.
- Global torque OFF must be verified on success and every failure path.
- No later leg implementation may introduce per-motor exceptions to bypass a general detector problem.
- RF/RH/LH development must start from the merged V25 architecture and use leg-specific geometry, directions, prerequisites and witnessed evidence.

## 8. Next milestone

```text
merge NormaCore PR #11
→ preserve release/matdog-lf-calibrator-v25
→ generalize V25 to RF without changing LF evidence
→ supervised RF six-contact calibration
→ RF affine gate and transactional freeze
```

LF is now a frozen reference leg for future cross-leg validation.
