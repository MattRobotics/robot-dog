# MATDOG — Offline Static Stability / Support Polygon Policy

## Purpose

This document records the C4-E offline static stability validation for the C4-C/C4-D rest-to-stand trajectory.

C4-E does not command the robot. It only checks whether the projected body reference remains inside the four-foot support polygon during the full offline trajectory.

## Source trajectory

C4-E uses:

- the C4-C contact-locked IK rest-to-stand trajectory;
- the C4-D conservative 10 s timing envelope;
- the achieved foot contact references from each sampled frame.

## Support polygon

For every sampled frame, the support polygon is computed as the convex hull of the four achieved foot contact references:

- LF contact reference;
- RF contact reference;
- RH contact reference;
- LH contact reference.

## COM proxy

For this first offline gate, the projection of `base_link` origin is used as a conservative center-of-mass proxy.

The tested region is not a single point only. C4-E checks:

- the proxy center;
- four corners of a ±20 mm uncertainty box in X/Y around the proxy.

This does not replace a future CAD-derived center-of-mass model.

## Validated result

The archived C4-E report validates:

- 51 safe samples out of 51;
- worst support margin: 74.000 mm;
- COM proxy uncertainty box fully inside the support polygon;
- no Station, serial, torque, target, speed, accel, stand or gait command.

## Command eligibility

C4-E is still offline-only.

The trajectory remains not command-eligible until later gates validate:

- hardware safe-mode preflight;
- supervised execution path;
- explicit operator approval;
- emergency stop / abort procedure;
- first physical test at very low speed.
