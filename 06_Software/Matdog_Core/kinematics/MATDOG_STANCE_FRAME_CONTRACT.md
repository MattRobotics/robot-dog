# MATDOG — Body Geometry and Reference-Stance Contract

## Purpose

This document separates the repeatable visual-zero calibration pose from the
future four-foot reference stance used for standing, planning and gait.

MATDOG must not assume that a joint-space calibration pose is automatically a
flat, safe or command-eligible stance on the ground.

## Intentional front / rear body geometry

MATDOG is intentionally designed with the front hip axis elevated by:

```text
+20.0 mm relative to the rear hip axis
```

This is a deliberate mechanical decision.

The intended effects are:

- improve the forward stability margin when the robot later includes a head,
  neck, sensors and an object held by the mouth;
- produce a more animal-like, feline posture;
- avoid a visually and mechanically flat table-like quadruped body.

The design intent does not by itself prove that the robot cannot tip. Actual
stability must later be evaluated using mass properties, centre of mass,
support polygon, payload, terrain, friction and trajectory constraints.

## Visual-zero is not a standing pose

At URDF visual-zero:

```text
LF / RF foot contact reference Z in base_link: -0.0934 m
RH / LH foot contact reference Z in base_link: -0.1134 m
front minus rear contact height:              +0.0200 m
```

Therefore:

```text
visual-zero != flat four-foot ground stance
```

This is expected and intentional.

Visual-zero remains the correct reference for:

- encoder calibration;
- URDF joint convention validation;
- FK / IK repeatability;
- read-only live validation;
- mechanical assembly checks.

It must not be used as a shortcut for a first stand command.

## Geometric separation

```text
base_link
→ hip / upper / lower joints
→ fixed foot_joint with side-specific ±1.5 mm eccentricity
→ foot_link
→ local cylindrical rubber-foot contact model
```

The intentional front / rear `+20 mm` body geometry is represented by the
canonical URDF joint origins. The side-specific `±1.5 mm` foot offset remains
a distinct lower-leg-to-foot construction compensation.

Neither value may be duplicated inside the local cylinder contact model.

## Future reference stance

The future reference stance will solve, offline:

```text
world Z = 0
+ body translation
+ body pitch / roll when required
+ LF / RF / RH / LH joint targets
+ four validated foot contact references
+ URDF limits
+ contact-mode policy
+ collision policy
```

Only after that offline stance is validated can it become an input to a
supervised, low-risk physical stand procedure.
