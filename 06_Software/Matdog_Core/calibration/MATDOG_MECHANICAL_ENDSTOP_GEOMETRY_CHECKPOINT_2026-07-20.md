# MATDOG — Mechanical End-Stop Geometry Checkpoint

**Date:** 2026-07-20  
**Status:** GEOMETRIC PREREQUISITES VALIDATED OFFLINE — NO HARDWARE EXECUTION YET

## Purpose

This document freezes the results of the offline geometric investigation performed after the final ST3215 digital-zero recentering.

No motor movement, Station command, serial access, or EEPROM write was performed during this investigation.

The mechanical end-stop calibration remains:

- planned;
- not yet command-eligible;
- dependent on implementation of telemetry validation, hybrid contact detection, timeout, immediate stop, controlled backoff, repeatability checks, abort handling and complete audit logging.

## Immutable calibration baseline

- Digital zero completed and verified for all 12 ST3215 servos.
- Displayed q=0 is near 2048 ticks.
- Position Offset EEPROM readback: PASS 12/12.
- EEPROM LOCK=1.
- TORQUE=0 at final audit.
- Maximum observed raw drift from acquired q=0: 3 ticks.
- ST3215 `GOAL_POSITION` remains normal unsigned.
- Signed-wrap commands are permanently forbidden.
- Old raw targets and old C4/C5 trajectories are not command-eligible after EEPROM recentering.
- Geometric poses in radians must be regenerated through the post-recenter conversion and revalidated.

Final digital-zero artifact:

`09_Logs/Calibration/C5_R_digital_recenter/2026-07-10_145457Z_final_12_offset_readback.json`

Expected SHA256:

`15619d23ddcb17651ba729a0d69309b5b56befeb3377f123ff7f131582fcf8ec`

## Canonical servo mapping

- LF: hip M13, upper M12, lower M11
- RF: hip M23, upper M22, lower M21
- RH: hip M33, upper M32, lower M31
- LH: hip M43, upper M42, lower M41

## Canonical URDF limits

- hip: `[-0.785398163397, +0.785398163397] rad` = `[-45°, +45°]`
- upper: `[-0.916297857297, +2.138028333693] rad`
- lower: `[-1.605702911835, +0.654498469498] rad` ≈ `[-92°, +37.5°]`

## Joint directions

- LF: hip -1, upper +1, lower -1
- RF: hip -1, upper -1, lower +1
- RH: hip +1, upper -1, lower +1
- LH: hip +1, upper +1, lower -1

## Static tracking tolerance

- 10 ticks
- 0.0153398078788564 rad
- 0.87890625°

This applies to static hold, micro-probe, return validation and controlled static-pose checks. It does not define mechanical contact limits or dynamic gait acceptance.

## Collision-analysis method

The audit used the canonical URDF collision STL meshes.

Successive checks included:

1. transformed exact AABB broad phase;
2. convex-hull feasibility with SciPy;
3. BVH-accelerated triangle/triangle surface-intersection narrow phase;
4. synthetic self-tests of the triangle-intersection kernel;
5. trajectory sampling, generally at 1°;
6. rear parking-path sampling at 0.5°.

Rules used:

- AABB overlap alone was never accepted as proof of collision.
- Convex-hull separation was accepted as proof that the enclosed real meshes were also separated.
- Where convex hulls intersected, original STL triangle surfaces were checked with narrow phase.

## Validated HIP calibration prerequisite

For every leg:

```yaml
hip_rad: 0.0
upper_rad: 0.8726646259971648   # +50°
lower_rad: 0.0
```

This pose permits the HIP path:

```text
0° → -45° → 0° → +45° → 0°
```

The complete path was sampled at approximately 1° and passed internal/body collision checks for LF, RF, RH and LH.

### Rejected HIP prerequisite

`upper=+90°, lower=0°` is not a valid common HIP prerequisite for the anterior legs.

At the exact URDF hip extremes it produces real triangle-surface collision:

- LF at hip `+45°`: `base_link ↔ lf_lower_leg_link`
- RF at hip `-45°`: `base_link ↔ rf_lower_leg_link`

The opposite anterior extremes were clear:

- LF hip `-45°`: clear
- RF hip `+45°`: clear

RH and LH were clear at both hip extremes with upper `+90°`, but a common `+50°` HIP prerequisite was selected for all legs.

## Validated LOWER calibration prerequisite

For every leg:

```yaml
hip_rad: 0.0
upper_rad: 1.5707963267948966   # +90°
lower_rad: 0.0
```

At this prerequisite, the complete LOWER path:

```text
0° → -92° → 0° → +37.5° → 0°
```

passed the internal/body narrow-phase collision audit for all four legs.

The apparent `hip_link ↔ lower_leg_link` overlap near lower `-92°` was a convex-hull false positive. The original STL triangle surfaces were separated.

## Complete single-leg path audit

For every leg, the following sequence passed exact mesh collision checking with approximately 1° sampling:

1. Home to HIP prerequisite:
   - hip `0°`
   - upper `0° → +50°`
   - lower `0°`

2. HIP negative approach and return:
   - upper `+50°`
   - lower `0°`
   - hip `0° → -45° → 0°`

3. HIP positive approach and return:
   - upper `+50°`
   - lower `0°`
   - hip `0° → +45° → 0°`

4. Transition to LOWER prerequisite:
   - hip `0°`
   - upper `+50° → +90°`
   - lower `0°`

5. LOWER negative approach and return:
   - hip `0°`
   - upper `+90°`
   - lower `0° → -92° → 0°`

6. LOWER positive approach and return:
   - hip `0°`
   - upper `+90°`
   - lower `0° → +37.5° → 0°`

Results:

- LF: PASS
- RF: PASS
- RH: PASS
- LH: PASS

## Cross-leg collision finding

With all non-active legs at q=0/home:

- RH and LH active calibration paths passed cross-leg broad-phase checks.
- LF and RF cannot directly execute the complete path while their ipsilateral rear leg remains at home.

The direct front-leg transition:

```text
upper +50° → +90°
hip=0°
lower=0°
```

causes real collisions from approximately `upper=+74°` through `+87°`:

- LF foot against LH foot/lower
- RF foot against RH foot/lower

The mesh surfaces become clear again around `+88°`, but the continuous path through `74°...87°` is invalid and must not be commanded.

Trying to perform the anterior LOWER calibration with upper values from `+50°` through `+73°` did not solve the problem across the full lower range. Real cross-leg collisions remained.

## Validated rear parking prerequisite for front-leg calibration

Before calibrating an anterior leg, park its ipsilateral rear leg at:

```yaml
hip_rad: 0.0
upper_rad: 0.5235987755982988   # +30°
lower_rad: 0.0
```

Mapping:

- calibrating LF requires LH parked at upper `+30°`
- calibrating RF requires RH parked at upper `+30°`

The rear parking path:

```text
rear upper 0° → +30° → 0°
rear hip=0°
rear lower=0°
```

passed convex-hull separation checks at 0.5° sampling against:

- base/body;
- its own non-adjacent links;
- all other legs at home.

With the ipsilateral rear leg parked at `+30°`, the complete anterior calibration path passed convex-hull separation for both sides:

- LF with LH parked: PASS
- RF with RH parked: PASS

The search evaluated the full front path over 542 sampled configurations.

Rear parking candidates from `+30°` through `+90°` all passed. `+30°` was selected because it requires the smallest departure from home.

## Canonical intended leg order

```text
LF → RF → RH → LH
```

Additional prerequisites:

- before LF, park LH at upper `+30°`;
- restore LH to home after LF;
- before RF, park RH at upper `+30°`;
- restore RH to home after RF;
- RH and LH calibration do not require an additional cross-leg parking pose based on the current geometry audit.

## Intended per-leg calibration sequence

1. Upper:
   - approach first direction;
   - immediate stop on suspected contact;
   - controlled backoff;
   - verify recovery;
   - repeat;
   - approach opposite direction;
   - stop/backoff/repeat;
   - return home.

2. Move upper to HIP prerequisite:
   - hip `0°`
   - upper `+50°`
   - lower `0°`

3. Hip:
   - min contact/backoff/repeat;
   - return zero;
   - max contact/backoff/repeat;
   - return zero.

4. Move upper to LOWER prerequisite:
   - hip `0°`
   - upper `+90°`
   - lower `0°`

5. Lower:
   - min contact/backoff/repeat;
   - return zero;
   - max contact/backoff/repeat;
   - return zero.

6. Return complete leg to safe/home.

7. For an anterior leg, restore the parked rear leg to home.

Only the probing joint may advance toward its expected contact during each contact-search step. Prerequisite joints are commanded to static validated poses and then monitored.

## Hybrid sensorless contact detector requirement

Contact must not be inferred from current alone.

Required evidence fusion:

- command advancing toward expected limit;
- measured encoder progress below expected;
- low velocity or persistent loss of progress;
- present current rises relative to the adaptive moving baseline;
- fresh telemetry;
- valid status with no servo error;
- pose remains inside a model-derived travel guard;
- maximum travel and timeout not exceeded.

Detector states:

```text
FREE_MOTION
CONTACT_SUSPECTED
CONTACT_CONFIRMED
CONTACT_REPEATABLE
AMBIGUOUS_CONTACT
HARD_ABORT
```

After suspected contact:

1. stop immediately;
2. perform controlled backoff;
3. verify encoder recovery;
4. repeat the approach;
5. compare contact positions;
6. accept only repeatable results.

## Telemetry requirements

- NormaCore Station remains sole serial owner.
- Do not use direct pyserial access.
- `PresentCurrent` register 0x45 is currently returned as raw u16; absolute physical units must be verified before defining absolute thresholds.
- `PresentSpeed` is currently returned as raw u16; independent circular encoder velocity must be computed.
- Station polling is sequential and timestamps differ between motors.
- HashSet iteration order is not stable.
- Unknown-ID scanning can create gaps around hundreds of milliseconds.
- Never assume a fixed frame rate or a fixed number of telemetry frames.
- After a command, wait for `CommandSuccess` or result timestamp, then require an active-motor telemetry timestamp newer than `CommandSuccess`.
- Enforce sample-age and telemetry-gap limits.
- Monitor non-active joints and abort if they leave their static tolerance.
- Every approach must have maximum travel, timeout, stop and retract behavior.

## Configuration-schema warning

`MATDOG_JOINT_CALIBRATION.yaml` currently declares:

```yaml
schema_version: 3
```

Historical utilities still explicitly require schema version 2, including:

- `matdog_capture_visual_zero.py`
- `matdog_apply_visual_zero.py`
- `matdog_calibration_validate.py`

The historical validator also requires `first_stand_limit_rad` and `safe_limit_rad` to remain null.

Therefore:

- do not add the prerequisite poses blindly to `MATDOG_JOINT_CALIBRATION.yaml`;
- first update or supersede the stale schema-2 validator contract;
- preserve `measured_contact_rad` and `safe_limit_rad` as null until real repeatable hardware contact measurements exist;
- geometry-derived prerequisite poses are not measured mechanical limits.

## Current acceptance state

### Completed

- Digital zero and EEPROM readback: PASS
- Canonical unsigned encoder contract: confirmed
- URDF collision meshes loaded and audited
- HIP prerequisite `upper=+50°`: geometrically validated
- LOWER prerequisite `upper=+90°`: geometrically validated
- Complete per-leg calibration paths: geometrically validated
- Cross-leg interference discovered and characterized
- Rear parking pose `upper=+30°`: geometrically validated
- Rear parking home-to-pose and pose-to-home paths: validated

### Still required before hardware execution

- Resolve schema-version mismatch and define canonical configuration fields.
- Implement the mechanical end-stop calibrator.
- Implement Station-mediated command and telemetry barriers.
- Decode or independently derive usable velocity.
- Characterize current baselines without copying thresholds from another robot.
- Implement stale telemetry, gap and status guards.
- Implement maximum-travel guards derived from URDF and validated prerequisites.
- Implement immediate stop, controlled backoff and automatic retract.
- Implement global torque-off/hard-abort path.
- Implement repeated-contact acceptance and ambiguity rejection.
- Produce dry-run artifacts for every planned command.
- Run suspended HIL validation before any unsupported stand.
- Do not unblock C5 until new post-recenter limits and trajectories are generated and audited.

## Resume point

The next software task is not further geometric exploration.

Resume from:

1. preserve this checkpoint in Git;
2. inspect and update the calibration schema/validator contract;
3. define canonical fields for:
   - HIP prerequisite pose;
   - LOWER prerequisite pose;
   - rear parking pose;
   - front-to-rear parking dependency;
   - geometry-validation metadata;
4. implement and test the model-based hybrid end-stop calibrator;
5. keep hardware execution blocked until all software safety gates pass.

