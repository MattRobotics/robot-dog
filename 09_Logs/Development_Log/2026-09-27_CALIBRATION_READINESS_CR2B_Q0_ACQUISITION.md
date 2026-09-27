# MATDOG Calibration Readiness — CR2-B same-session read-only q0 acquisition

**Date:** 2026-09-27  
**Branch:** `feat/calibration-readiness-v1`  
**Scope:** production wiring for read-only q0 evidence acquisition  
**Status:** IMPLEMENTED / OFFLINE + FIRMWARE BUILD VALIDATED  
**Hardware action performed in this gate:** NONE  
**Flash performed:** NONE  
**Motion:** NONE  
**Torque ON:** NONE  
**GoalPosition:** NONE  
**EEPROM / PositionOffset / CalibrationOfs write:** NONE  
**Actuator authority acquired by q0 capture:** NONE

## 1. Entry evidence

CR2-A was synchronized and validated locally at:

```text
5daefc73f824797d3052c75002a495bed21b58f3
```

Observed on the ASUS K53SV:

```text
STATIC_AUDIT = PASS
Scanned 111 source files

CALIBRATION_Q0_BOOTSTRAP_TESTS = PASS
checks_run=52 failures=0

all remaining host suites = PASS
git diff --check = clean
working tree = clean
```

This closed CR2-A offline. It did not create formal persisted H1, capture live q0, authorize motion
or flash hardware.

## 2. Architectural decision: q0 acquisition is not a motion session

`CalibrationManager::startSession(LIVE_SESSION)` remains intentionally refused while:

```text
MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED = 0
hardware_motion_authorized = false
```

Read-only q0 acquisition must not weaken that gate. Therefore CR2-B does **not** use
`CalibrationManager`, does not request `ActuatorAuthority::CALIBRATION`, and does not introduce a
second authority concept.

CR2-B is an evidence-acquisition transaction only.

## 3. Existing components reused

No second bus, scan, preflight or register layer was created.

CR2-B reuses:

```text
Controller-owned ServoBus
Controller-owned ServoCensus
Controller-owned ServoPreflight
CalibrationPopulationEvidence (CR1)
CalibrationQ0Bootstrap (CR2-A)
CalibrationGeometryProfileData (current generated Geometry V5)
```

The q0 sampling read is the existing:

```cpp
ServoBus::readRuntimeState(...)
```

and consumes only:

```text
PresentPosition
TorqueEnable
```

No new ST3215 register accessor was added.

## 4. Shared semantic identity mapping

CR1 already contained the authoritative current conversion:

```text
CanonicalServo allocation row
  -> leg
  -> joint kind
  -> expected physical-unit label
  -> JointIdentity
```

That parser was exposed as:

```cpp
semanticIdentityFromCanonical(...)
```

rather than duplicated in CR2-B. Physical-unit identity remains configuration/assembly evidence;
the ST3215 does not report a unit serial.

## 5. Pure acquisition coordinator

Added:

```text
src/calibration/CalibrationQ0CaptureSession.h
src/calibration/CalibrationQ0CaptureSession.cpp
scripts/tests/test_calibration_q0_capture_session.cpp
```

The coordinator owns no hardware object. Its states are:

```text
IDLE
  -> NEED_CENSUS_START
  -> WAIT_CENSUS
  -> NEED_PREFLIGHT_START
  -> WAIT_PREFLIGHT
  -> SAMPLING
  -> COMPLETE | FAILED
```

A configuration is refused unless:

- sample count is 3..32;
- stability budget was explicitly supplied;
- stability budget is below the meaningless half-turn value 2048;
- the operator explicitly confirmed nominal URDF q=0.

The coordinator generates a nonzero acquisition-session id for each attempt.

## 6. Same-session population evidence

CR2-B starts a **fresh existing census** and then a **fresh existing preflight**.

Only after both were sequenced by the same coordinator does it create:

```cpp
PopulationEvidenceBuildContext {
    current_observation_bundle = true,
    session_ms = capture_start_ms,
}
```

and invoke the existing CR1 producer.

Sampling cannot begin unless CR1 returns a formal current population PASS.

The earlier stand-alone `@SERVO PREFLIGHT 12/12 PASS` remains untouched and is not retroactively
renamed formal H1 evidence.

## 7. Round-robin q0 observations

Sampling order is deliberately:

```text
pass 0: all 12 leg joints once
pass 1: all 12 leg joints once
...
pass N: all 12 leg joints once
```

rather than taking every sample of one joint consecutively.

Each observation must match the expected current bus slot and must satisfy:

```text
read_ok = true
TorqueEnable = 0
raw tick in 0..4095
```

One observation is acquired per Controller update, using the existing `readRuntimeState()`.
No loop in CR2-B iterates over the hardware bus synchronously.

After the final full pass, each 12-joint sample set is reduced by CR2-A. All twelve must become
`Q0BootstrapStatus::CANDIDATE` or the whole acquisition ends FAILED.

## 8. Candidate-only output

Success means:

```text
formal current population evidence: PASS for this capture transaction
12 q0 records: CANDIDATE
accepted: NO
promoted: NO
JointTransform admitted: NO
hardware motion authorized: NO
```

CR2-B has no persistence path.

## 9. Controller wiring

`Controller` owns exactly one:

```cpp
CalibrationQ0CaptureSession q0_capture_;
```

`Controller::updateQ0Capture()` may perform only:

1. start the existing census;
2. observe its completion;
3. start the existing preflight;
4. observe its completion;
5. perform exactly one existing runtime-state read per tick;
6. feed that observation into the pure coordinator.

Leaving `MAINTENANCE` fails the acquisition. There is no authority to release because none was
acquired.

## 10. USB command surface

Added:

```text
@CALIBRATION Q0 STATUS
@CALIBRATION Q0 ABORT
@CALIBRATION Q0 CAPTURE <samples 3..32> <stability_ticks 0..2047> CONFIRM_Q0_POSE
```

### CAPTURE gates

It is refused unless:

- current operating mode is `MAINTENANCE`;
- compiled profile declares servo rail available (`ROBOT_POWERED`);
- no q0 capture is already active;
- no ordinary servo scan/census/preflight transaction is active;
- argument count/ranges are valid;
- exact pose-confirmation token is present.

This does not mean the supplied stability number is the final q0 acceptance/plausibility
tolerance. It only rejects an internally moving/unstable sample cloud before a candidate is
formed. CR3 still owns acceptance policy.

### STATUS

Cache-only. It performs no bus read. On success it prints all 12 candidate identities, raw q0
ticks, measured spreads and CANDIDATE state, followed by explicit:

```text
accepted=NO
promoted=NO
transform_admitted=NO
motion_authorized=NO
```

### ABORT

Ends the CR2-B transaction and prevents subsequent q0 runtime samples. If an existing census or
preflight service is already in progress, that read-only service may finish because those
pre-existing services have no cancellation primitive. Abort sends no actuator command.

## 11. Single diagnostic owner during acquisition

While CR2-B is active, these operator commands are refused:

```text
@SERVO SCAN
@SERVO CENSUS
@SERVO PREFLIGHT
@SERVO READ
```

so they cannot interleave transactions into the evidence bundle.

`@SERVO SAFE_OFF` is the deliberate exception and remains reachable in every state. A safety
de-escalation is not subordinated to calibration evidence acquisition.

## 12. Static audit

Added/extended audit rules enforce:

- one CR2-B coordinator instance, owned by `Controller`;
- pure coordinator: no Arduino, transport, authority or actuator writes;
- no `CalibrationManager` dependency;
- exact same-session census→preflight→population sequence;
- current Geometry V5 identity;
- round-robin 12-joint sampling;
- Torque-OFF and raw-domain gates;
- candidate-only output;
- Controller q0 wiring contains only existing read-only services;
- no q0 auto-start at boot;
- command requires MAINTENANCE + powered profile + exact pose token;
- competing servo diagnostics are blocked during capture;
- q0 STATUS is cached-only;
- SAFE_OFF does not consult q0 state.

The pre-existing audits still enforce:

```text
ActuatorRuntime backend = nullptr
hardware motion default = 0
no production Torque ON
no production GoalPosition
PositionOffset write forbidden
SAFE_OFF independent
```

## 13. Validation closure

The required local validation completed successfully on 2026-09-27.

At functional firmware SHA
`2e5cbfa43378ed8d0e76e1c2a942886ef34a89db`:

```text
CALIBRATION_Q0_CAPTURE_SESSION_TESTS = PASS
checks_run=467 failures=0

all remaining host suites = PASS

USB_ONLY firmware build = PASS
ROBOT_POWERED firmware build = PASS
final resting artifact restored to USB_ONLY
```

The first static-audit run exposed two audit false positives: CR1/CR2-A public function
prototypes in their own headers were being mistaken for production call sites. No firmware or
calibration implementation changed. The audit was narrowed to mask only those two declarations.

At audit-fix SHA
`9ff046f0302928dfd2833d848206f3d66ca75228`:

```text
Scanned 114 source files
STATIC_AUDIT = PASS
git diff --check = clean
working tree = clean
```

Because the only delta from `2e5cbfa...` to `9ff046f...` is
`scripts/static_audit.py`, the successful firmware builds remain valid evidence for the same
CR2-B implementation.

CR2-B is therefore **OFFLINE + BUILD VALIDATED**.

This closure does **not** constitute:

```text
hardware validation
live q0 capture
evidence acceptance or promotion
JointTransform admission
hardware motion authorization
permission to flash or move the robot
```

A separately authorized read-only hardware gate is next.
