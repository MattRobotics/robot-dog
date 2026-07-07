# MATDOG — Quadruped Leg Kinematic Contract

## Scope

This document defines the common software contract used to query forward
kinematics and contact geometry for LF, RF, RH and LH.

The implementation must always read the canonical URDF chain for the selected
leg. It must not create mirrored sign rules outside the URDF.

## Canonical leg order

LF, RF, RH, LH

## Joint topology

Each leg has three actuated joints followed by one fixed foot joint:

    <leg>_hip_joint
    → <leg>_upper_leg_joint
    → <leg>_lower_leg_joint
    → <leg>_foot_joint (fixed)
    → <leg>_foot_link

At visual-zero, all four foot-link frames are aligned with `base_link`.

## Joint-axis contract

All four canonical URDF chains define:

    hip joint:   local X axis
    upper joint: local Y axis
    lower joint: local Y axis

The robot is not made symmetric in software by multiplying arbitrary signs.

Differences between LF, RF, RH and LH are already represented by:

- the canonical URDF joint origins;
- the front / rear hip elevation difference of +20 mm;
- the left / right geometry;
- fixed foot-joint lateral offsets of -1.5 mm for LF/LH and +1.5 mm for RF/RH;
- the canonical encoder-to-radian direction mapping outside this FK layer.

## Contact policy

At visual-zero every leg shall produce:

    NOMINAL_STRIP_CONTACT

A positive hip probe can produce `EDGE_BIASED_CONTACT`. This is a valid
offline geometric classification, not a first-stand authorization.

## Non-regression rule

A generic all-leg solver must use:

    selected leg ID
    → exact canonical URDF chain
    → exact canonical URDF limits
    → exact FK
    → common local foot contact model

It must not copy LF positions, limits, offsets or sign assumptions into the
other legs.
