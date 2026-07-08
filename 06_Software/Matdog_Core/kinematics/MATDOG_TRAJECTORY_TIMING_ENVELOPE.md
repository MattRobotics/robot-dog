# MATDOG — Offline Trajectory Timing Envelope

## Purpose

This document records the C4-D offline timing and servo-envelope validation for the C4-C contact-locked rest-to-stand trajectory.

C4-D does not command the robot. It only evaluates whether the already validated offline trajectory can be executed with a conservative timing profile.

## Source trajectory

C4-D uses the C4-C contact-locked IK rest-to-stand trajectory:

- feet locked to the C4-A footprint;
- contact references on `world Z = 0`;
- `base_link` parallel to the ground;
- body height ramp from 100 mm to 150 mm;
- 51 sampled frames;
- C4-B collision/contact policy passed at every frame.

## Rejected trajectory type

Direct joint-space interpolation from visual-zero `q = 0` to the C4-A stand candidate remains rejected.

The accepted trajectory is contact-locked IK, not visual-zero interpolation.

## Timing candidate

The first conservative timing envelope is:

- duration: 10.0 s;
- samples: 51;
- sample interval: 0.2 s;
- maximum allowed speed: 10.0 deg/s;
- maximum allowed acceleration: 25.0 deg/s².

## Validated result

The archived C4-D report validates:

- maximum joint range: 47.071 deg;
- maximum per-sample step: 1.291 deg;
- maximum measured speed: 6.454 deg/s;
- maximum measured acceleration: 0.946 deg/s²;
- speed envelope passed;
- acceleration envelope passed.

The most demanding joints are the front lower-leg joints:

- `lf_lower_leg_joint`;
- `rf_lower_leg_joint`.

## Command eligibility

C4-D is still offline-only.

The trajectory remains not command-eligible until later gates validate:

- static stability / support polygon;
- hardware safe-mode preflight;
- supervised execution path;
- explicit operator approval;
- emergency stop / abort procedure.
