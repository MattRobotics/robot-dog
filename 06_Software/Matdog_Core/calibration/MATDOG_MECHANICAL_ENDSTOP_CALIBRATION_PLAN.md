# MATDOG — Native NormaCore Mechanical End-Stop Calibrator Plan

**Status:** IMPLEMENTATION FOUNDATION — HARDWARE BLOCKED UNTIL NATIVE PILOT TESTS PASS
**Canonical implementation:** NormaCore ST3215 driver, Rust
**Pilot:** LF upper joint, servo M12, URDF minimum
**EEPROM writes:** forbidden

## 1. Decision

The MATDOG mechanical end-stop calibrator shall be implemented as a robot-specific native sequence inside NormaCore, following the same architectural pattern used by SO101 and ElRobot:

```text
software/drivers/st3215/src/auto_calibrate/matdog.rs
```

Python command-capable end-stop prototypes are retired and are not command-eligible.

## 2. Robot identification

Auto-calibration may start only when the discovered motor set exactly equals:

```text
11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43
```

Missing or unexpected IDs fail before torque is enabled.

## 3. Permanent constraints

- Station is the sole serial owner.
- Normal unsigned ST3215 GoalPosition semantics.
- No direct serial access.
- No servo reset.
- No Position Offset modification.
- No EEPROM unlock/write/lock cycle.
- No calibration freeze command.
- One probing joint at a time.
- All prerequisite joints are monitored.
- Global torque-off cleanup is mandatory and must be read back.

## 4. MATDOG geometry

MATDOG is not front/rear symmetric. Front hip axes are 20 mm higher than rear hip axes. Four explicit leg profiles shall be retained.

Joint map and directions:

```text
LF: M13 hip -1, M12 upper +1, M11 lower -1
RF: M23 hip -1, M22 upper -1, M21 lower +1
RH: M33 hip +1, M32 upper -1, M31 lower +1
LH: M43 hip +1, M42 upper +1, M41 lower -1
```

Order:

```text
LF → RF → RH → LH
```

Prerequisites:

```text
HIP search:   hip 0°, upper +50°, lower 0°
LOWER search: hip 0°, upper +90°, lower 0°
LF search:    park LH upper at +30°
RF search:    park RH upper at +30°
```

## 5. Native primitives

Add RAM-only primitives to the common calibrator or a MATDOG-specific helper:

- verified RAM write with command result and register readback;
- torque-state verification;
- position command with progress watchdog;
- fresh motor observation extraction;
- hybrid limit approach;
- immediate pressure stop;
- controlled backoff and recovery verification;
- repeated-contact comparison;
- verified return home;
- verified global torque-off cleanup.

Existing SO101/ElRobot functions that reset servos or write EEPROM must not be called.

## 6. Hybrid observation

Each active-motor observation contains:

- individual monotonic timestamp;
- present position;
- present speed;
- present current;
- goal position;
- torque enable;
- torque limit;
- status/error;
- last command and result.

Current is a relative load/torque proxy. It is not reported as physical torque without characterization.

## 7. Contact policy

Contact requires persistent agreement of:

```text
commanded motion toward expected limit
+ sufficient previous travel
+ insufficient encoder progress
+ low velocity
+ current rise over direction/pose-specific moving baseline
+ fresh telemetry
+ status OK
+ prerequisite stability
+ travel/time guards respected
```

States:

```text
FREE_MOTION
CONTACT_SUSPECTED
CONTACT_CONFIRMED
CONTACT_REPEATABLE
AMBIGUOUS_CONTACT
HARD_ABORT
```

A hard absolute-current threshold triggers immediate retract. A separate adaptive current delta participates in normal contact confirmation.

## 8. M12 MIN pilot

Sequence:

1. exact 12-ID preflight;
2. global torque OFF and readback;
3. verify M12 and all prerequisite states;
4. prime M12 at current position;
5. configure RAM torque limit, speed and acceleration;
6. verify RAM readback;
7. enable M12 torque and verify readback;
8. return to home 2048 if required;
9. acquire moving-current baseline;
10. coarse approach toward URDF minimum within a model guard;
11. confirm hybrid contact;
12. stop pressure and back off;
13. fine second approach;
14. require repeatability;
15. return to 2048 within 10 ticks;
16. global torque OFF and readback;
17. publish progress and persistent report.

## 9. Pilot acceptance

PASS only when:

- no EEPROM command exists in the executed path;
- M12 demonstrably moves;
- RAM writes and torque state are read back;
- all observations are fresh and status is zero;
- two contacts are repeatable;
- backoff restores tracking and lowers current;
- M12 returns home within 10 ticks;
- all motors finish torque OFF.

## 10. Full calibration

After pilot PASS:

```text
LF upper min/max
LF hip min/max at upper +50°
LF lower min/max at upper +90°
restore LF and LH parking

RF upper min/max
RF hip min/max at upper +50°
RF lower min/max at upper +90°
restore RF and RH parking

RH upper/hip/lower
LH upper/hip/lower
```

Each side uses two low-energy approaches. Measurements populate `measured_contact_rad` only after repeatability passes. `safe_limit_rad` is derived using a documented inward margin.

## 11. Post-calibration gate

Before a new physical stand:

```text
24 contact results
→ safe-limit YAML
→ read-only joint/FK closure
→ regenerate HOME→LOW_STAND→STAND trajectory
→ collision/contact/support/timing audit
→ supervised suspended stand
→ gradual load transfer
```

Old raw C5 targets remain non-command-eligible.
