# MATDOG — Automatic Mechanical End-Stop Calibration Plan

## Status

```text
STATUS: PLANNED — NOT YET COMMAND-ELIGIBLE
HARDWARE EXECUTION: NOT AUTHORIZED BY THIS DOCUMENT
```

## Objective

Create a MATDOG-specific automatic calibration workflow for all 12 ST3215 leg
joints, with an operator experience comparable to pressing a single calibration
control on a production robot.

The workflow must discover the real mechanical contact range of every joint,
derive conservative safe limits and store the validated results in the
canonical MATDOG calibration record.

This phase is separate from digital-zero calibration. Digital zero is already
complete and maps mechanical `q = 0` to displayed encoder position `2048`.
Mechanical end-stop calibration must not rewrite those Position Offset values.

## Canonical joints

```text
LF: M13 hip, M12 upper, M11 lower
RF: M23 hip, M22 upper, M21 lower
RH: M33 hip, M32 upper, M31 lower
LH: M43 hip, M42 upper, M41 lower
```

## Mandatory architecture

- NormaCore Station remains the sole serial owner.
- No direct `pyserial` access is allowed.
- The generic NormaCore `AutoCalibrate` operation is not authorized.
- The tool must be MATDOG-specific and use the canonical servo map.
- Normal unsigned ST3215 `GoalPosition` semantics must be preserved.
- Signed `GoalPosition` experiments are forbidden.
- EEPROM digital-zero offsets must remain unchanged.
- The robot must remain supported throughout calibration.
- Hardware motion requires a fresh explicit operator authorization.

## Required calibration strategy

The final tool should expose a single top-level calibration action but execute
a guarded internal sequence.

Each joint must be calibrated separately:

```text
preflight
    -> place adjacent joints in a collision-safe prerequisite pose
    -> apply conservative speed, acceleration and effort/current limits
    -> approach one mechanical direction
    -> detect contact
    -> stop immediately
    -> back off
    -> validate repeatability
    -> approach the opposite direction
    -> detect contact
    -> stop immediately
    -> back off
    -> validate repeatability
    -> derive measured contact and safe operating limits
    -> continue only after PASS
```

The 12-joint order must not be chosen only by servo ID. It must be derived from
a collision-safe dependency graph because the reachable range of one joint can
depend on the position of the other two joints in the same leg.

Example dependency to preserve: a hip joint must not be driven toward a limit
when the upper or lower leg position can cause self-contact.

## Contact detection

Mechanical contact must not be inferred from a single signal.

A valid detector should combine:

- commanded versus measured position progress;
- low measured velocity or repeated lack of encoder progress;
- motor current rise relative to the moving baseline;
- elapsed-time and maximum-travel guards;
- servo status/error flags;
- agreement across repeated low-energy approaches.

Contact must be rejected when evidence is ambiguous.

Absolute current thresholds must be established experimentally for MATDOG and
must not be copied blindly from another robot.

## Safety controls

The implementation must include:

- low initial speed and acceleration;
- conservative torque/effort or current limits where supported;
- per-joint travel bounds narrower than any unknown full revolution;
- per-approach timeout;
- immediate stop and controlled backoff;
- operator abort;
- global torque-off abort path;
- one active joint at a time;
- live verification that all non-active joints remain within their safe
  prerequisite windows;
- audit logging after every approach;
- no continuation after a failed or ambiguous joint.

## Data to record

For every joint, record at minimum:

```text
servo ID
joint name
digital-zero offset
mechanical q=0 raw encoder
contact position in both directions
repeatability spread
approach direction
current baseline and contact evidence
speed and acceleration used
backoff distance
derived safe margin
measured_contact_rad
safe_limit_rad
result and reason
```

The canonical destination is:

```text
06_Software/Matdog_Core/calibration/
MATDOG_JOINT_CALIBRATION.yaml
```

The existing fields `measured_contact_rad` and `safe_limit_rad` must be populated
only after validated hardware measurements.

## Acceptance criteria

The mechanical end-stop phase is complete only when:

- all 12 joints pass repeated contact acquisition;
- no self-collision or unintended leg-to-body contact occurs;
- every joint has two validated contact limits or a documented intentionally
  restricted side;
- conservative safe margins are justified and recorded;
- the safe limits are compatible with the canonical URDF limits;
- digital-zero offsets are unchanged;
- the robot returns to a known supported pose;
- all EEPROM locks and torque states are verified as required;
- a final read-only joint-state and four-leg FK closure passes;
- C5 encoder targets are regenerated and audited against the measured limits.

## Required work before hardware execution

1. Audit the existing MATDOG and elrobot calibration implementations.
2. Extract only the architectural ideas that are safe and applicable.
3. Define the MATDOG collision-safe joint dependency order.
4. Define low-energy motion parameters and abort thresholds.
5. Build an offline/dry-run state machine and audit format.
6. Review the complete command path before enabling motion.
7. Obtain explicit operator authorization for the first supervised run.

## Relationship to C5

The previous C5 executor and its trajectory remain useful engineering
references, but they are not command-eligible after the digital recenter.

Before a new physical stand:

```text
mechanical end-stop calibration
    -> canonical safe limits
    -> read-only joint/FK closure
    -> regenerate C5 encoder targets
    -> wrap, limit, collision and frame-delta audit
    -> hardware preflight
    -> new supervised C5 stand
```
