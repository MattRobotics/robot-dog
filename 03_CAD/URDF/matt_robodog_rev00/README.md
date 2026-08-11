# MATDOG — URDF REV00 Kinematic Baseline

## Status

Archived and validated kinematic baseline.

This directory is the canonical REV00 robot description used as the starting point for inverse kinematics, stand-pose development, gait generation and later simulation work.

## Canonical Files

- `matt_robodog_rev00.urdf`
  Canonical URDF model.

- `meshes/*.stl`
  Detailed STL visual meshes used by the URDF.

- `meshes/collision/*.stl`
  Dedicated, simplified STL collision meshes used by the URDF.

- `textures/`
  URDF Studio texture assets, when present.

- `reference/Coordinate_Cinematiche_matt_robodog_rev00.xlsx`
  Authoritative table for joints, frames, limits, servo IDs, mesh transforms, masses and material notes.

- `SHA256SUMS.txt`
  Integrity manifest for the canonical URDF assets.

## Kinematic Topology

- 1 base link
- 4 legs
- 17 links
- 16 joints
- 12 revolute leg joints
- 4 fixed foot joints

Each leg follows:

base_link
→ hip_joint
→ hip_link
→ upper_leg_joint
→ upper_leg_link
→ lower_leg_joint
→ lower_leg_link
→ foot_joint
→ foot_link

## Coordinate Convention

- X: forward
- Y: left
- Z: up
- Units: metres and radians
- Right-handed coordinate system

## Joint Limits Consolidated in REV00

Hip joints:

- Minimum: -45.0 degrees
- Maximum: +45.0 degrees

Upper-leg joints:

- Minimum: -52.5 degrees
- Maximum: +122.5 degrees

Lower-leg joints:

- Minimum: -92.0 degrees
- Maximum: +37.5 degrees

The lower-leg closing limit was reduced to -92.0 degrees to avoid mechanical interpenetration.

## Mesh Convention

The four upper-leg STL meshes have their final orientation baked directly into the mesh vertices.

Therefore, the URDF visual and collision mesh origins for upper-leg links are intentionally:

origin xyz="0 0 0" rpy="0 0 0"

No compensating mesh transform should be reintroduced without updating the canonical reference workbook.

## Collision Status

The REV00 model keeps the detailed meshes under `meshes/` for `<visual>` and
uses the approved dedicated meshes under `meshes/collision/` for
`<collision>`. Both sets retain their canonical link filenames; the directory
separates their roles.

The dedicated collision baseline approved on 2026-08-10 contains 17 meshes,
119,696 triangles and 5,986,228 bytes. It reduces the historical 472,020-triangle
collision set by 74.64%. Collision origins and `scale="0.001 0.001 0.001"`
remain unchanged, so this representation change does not create a new
mechanical revision.

## Physical Data Status

Mass and material references are maintained in the Excel workbook.

Link center-of-mass coordinates and inertia tensors are not yet authoritative. They must be extracted from Solid Edge before dynamic simulation is considered valid.

## Mechanical Segment and Foot-Contact Frame

- `hip_to_knee`: `90.0 mm`, measured between the relevant joint axes.
- `knee_to_foot_interface_center`: `110.0 mm`, measured from the knee axis
  to the center of the lower-leg endpoint where the eccentric rubber foot
  attaches.
- `knee_to_foot_contact_frame`: `118.1 mm`, measured from the knee axis to
  the URDF `foot_joint`, which represents the nominal ground-contact point
  of the eccentric rubber foot.

The lower-leg mechanical interface center and the URDF `foot_joint` are
intentionally different because the rubber foot is eccentric. Use the
`foot_joint` contact-frame distance for IK, stand-pose and gait planning.

## ST3215 URDF Effort and Velocity Semantics

All twelve revolute leg joints currently use:

    effort   = 0.902244 N*m
    velocity = 3.03687289847 rad/s = 29 rpm

These are conservative MATDOG nominal-operation limits for the intended 3S
operating point. They are not an official Feetech torque-speed curve at
exactly 11.1 V.
