# CR3 - q0 acceptance and promotion entry plan

Date: 2026-09-27
Status: DESIGN ENTRY - NO ACCEPTANCE / NO PROMOTION / NO MOTION
Input evidence: 09_Logs/Validation_Reports/Calibration_Q0_CR2C_2026-09-27/

## Facts now available

CR2-C produced twelve current-installation q0 candidates in one formally current population
session. Every joint supplied nine samples with TorqueEnable=0 and spread=0 ticks. The largest
absolute diagnostic distance from raw centre 2048 is 63 ticks.

These facts prove static within-session stability and current identity/provenance. They do not by
themselves quantify independent manual re-alignment repeatability, linkage backlash across
re-positioning, or operator placement error.

## Existing hard ceiling

The bootstrap contract already derives 163.84 ticks per spline tooth and therefore 81.92 ticks as
the widest possible half-tooth discrimination window. Any q0 plausibility tolerance must remain
strictly below 81.92 ticks.

CR2-C observed:

    max |q0 - 2048| = 63 ticks
    half-tooth ceiling = 81.92 ticks
    remaining discrimination margin = 18.92 ticks

This is useful evidence but does not justify selecting an acceptance threshold by fitting one
manually established dataset.

## CR3-A acceptance-policy requirements

Before a CANDIDATE may become ACCEPTED, policy must require:
1. exact current semantic joint + physical-unit identity;
2. exact current Geometry V5 provenance;
3. formal current 12/12 population evidence associated with acquisition;
4. estimator MANUAL_ZERO_POSE;
5. explicit nominal-q0 operator confirmation;
6. Torque OFF on every underlying sample;
7. raw domain 0..4095;
8. reviewed sample-count policy;
9. within-session spread within reviewed stability budget;
10. explicit q0 plausibility rule strictly below half-tooth ceiling;
11. no historical LF q0 substitution and no PositionOffset compensation.

## What CR2-C does and does not justify

CR2-C justifies retaining the twelve records as current hardware evidence. It does not yet justify
choosing a universal q0 plausibility threshold from one manually established pose.

The next design decision is how independent pose-placement repeatability is measured before fixing
that final threshold. Preferred evidence is repeated, independently re-established nominal-q0
captures, not repeated samples while the mechanism remains untouched; static repeatability was
already measured by CR2-C at zero spread.

No further hardware run is authorized by this document.

## Promotion/persistence boundary

    CANDIDATE
      -> reviewed acceptance policy
    ACCEPTED
      -> explicit persistence/currentness transaction
    PROMOTED
      -> eligible to construct JointTransform

Persistence must remain outside ST3215 EEPROM. No PositionOffset, CalibrationOfs or one-key-middle
write may be introduced. Exact project/runtime persistence mechanism remains TO_DESIGN and must
preserve physical-unit identity plus geometry provenance.

## Motion state

CR3 starts with:

    hardware_motion_authorized = false
    ActuatorBackend             = nullptr
    accepted transforms         = 0
    promoted transforms         = 0
    motion authority            = NONE

CR3 work must not change those facts merely to make later calibration motion possible.
