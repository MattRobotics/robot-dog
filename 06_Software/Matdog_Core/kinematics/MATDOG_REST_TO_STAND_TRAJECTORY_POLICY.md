# MATDOG — Offline Rest-to-Stand Trajectory Policy

## Purpose

This document records the C4-C offline rest-to-stand trajectory policy.

The goal is to define a safe offline trajectory candidate before any automatic stand command is allowed.

## Rejected approach

Direct joint-space interpolation from visual-zero `q = 0` to the C4-A stand candidate is rejected.

Reason:

- `q = 0` is a visual/mechanical reference, not a valid four-foot contact stance;
- LF/RF and RH/LH do not close on the same world ground plane at visual zero;
- the C4-C1 direct interpolation probe produced lower-leg ground penetration during intermediate samples;
- the final C4-A pose is valid, but the direct path toward it is not.

## Accepted approach

C4-C uses a contact-locked IK trajectory:

- `base_link` remains parallel to the ground;
- all foot contact references remain locked to the C4-A footprint;
- all foot contact references remain on `world Z = 0`;
- body height ramps from `100 mm` to `150 mm`;
- IK is solved for every sampled frame;
- C4-B collision/contact policy is evaluated at every sampled frame.

## MATDOG-specific clearance interpretation

The expected distal lower-leg fork clearance around the rigid TPU 90D foot cylinder remains a review condition, not an automatic failure, if the lower-leg mesh remains above `world Z = 0`.

The critical condition is knee / lower_leg_joint clearance relative to the foot contact reference.

## C4-C validated result

The archived C4-C trajectory report validates:

- 51 safe samples out of 51;
- no non-foot ground penetration;
- expected lower-leg fork review only;
- positive knee/contact clearance across the full trajectory;
- foot contact references held on `world Z = 0`;
- no Station, serial, torque, target, speed, accel, stand or gait command.

## Command eligibility

C4-C is still offline-only.

The trajectory is not command-eligible until later gates validate:

- servo speed and acceleration limits;
- trajectory timing;
- static stability / support polygon;
- supervised hardware safe mode;
- explicit operator approval.
