# MATDOG — Offline Collision / Contact Policy

## Purpose

This document records the MATDOG-specific collision/contact interpretation used before any automatic rest-to-stand command is allowed.

The policy exists to avoid a wrong generic interpretation of the lower-leg geometry.

## Material assumption

The foot cylinder is solid TPU, Shore D 90.

For the current C4 offline validation phase it is treated as practically rigid.

No compression model is used in C4-B.

## Foot / lower-leg architecture

Each `*_foot_link` is a cylindrical TPU foot mounted eccentrically at the distal end of the corresponding `*_lower_leg_link`.

The distal fork / arms of the lower leg intentionally surround the TPU foot cylinder. Therefore, low ground clearance of the distal lower-leg fork near the foot is expected and must not be classified as an automatic failure if the mesh remains above `world Z = 0`.

The analytical foot-contact model remains responsible for stable contact geometry. The STL collision meshes remain responsible for collision/contact validation.

## Correct interpretation

Allowed:

- `foot_link` at ground contact.

Expected review, not automatic failure:

- distal `lower_leg_link` fork close to the rigid TPU foot cylinder while still above `world Z = 0`.

Failure:

- any non-foot collision mesh below `world Z = 0`.

Critical kinematic risk:

- `lower_leg_joint` / knee descending toward or below the foot contact reference.

This is the real rest-to-stand danger case: if the lower joint closes upward before the upper leg has moved backward enough, the knee can descend too far relative to the foot contact point.

## Static C4-B checks

For the C4-A stand candidate, the policy checks:

- mesh ground clearance for all collision links;
- knee / `lower_leg_joint` world Z;
- foot contact reference world Z;
- knee-to-contact vertical clearance.

The current C4-B result does not make the pose command-eligible.

## Future C4-C requirement

The same invariants must be checked across the full rest-to-stand trajectory.

For every sampled trajectory frame, validate:

- foot contact policy;
- lower-leg distal fork review;
- non-foot mesh ground penetration;
- knee-to-contact clearance;
- URDF limits;
- support mode.

Only after C4-C/C4-D trajectory validation and supervised hardware safety gates can an automatic stand test be considered.
