# ADR-002 — URDF REV00 Kinematic Baseline

Date: 2026-06-30

## Status

Accepted

## Context

MATDOG required a canonical robot description before inverse kinematics, stand-pose development and gait generation could begin.

The CAD model was decomposed into rigid URDF links, meshes were simplified for browser-based URDF editing, and all joint frames, axes, limits, servo mappings and mesh transforms were validated in URDF Studio.

## Decision

The canonical robot description is stored at:

`cad/urdf/matt_robodog_rev00/`

The canonical source of truth is:

1. `matt_robodog_rev00.urdf`
2. Final STL meshes in `meshes/`
3. `Coordinate_Cinematiche_matt_robodog_rev00.xlsx`

## Kinematic Decisions

- ROS convention: X forward, Y left, Z up.
- All joint origins are expressed in the parent-link frame.
- Four legs follow the same kinematic topology:
  base → hip → upper → lower → foot.
- Upper-leg limits are -52.5° to +122.5°.
- Lower-leg limits are -92.0° to +37.5°.
- Foot joints are fixed.
- The four upper-leg mesh transformations are baked into the STL geometry.
- Upper-leg visual and collision origins are zeroed in the final URDF.

## Collision Decision

The same mesh is currently used for visual and collision geometry.

This is accepted for REV00 because the immediate goal is kinematic validation and gait development, not full rigid-body dynamics.

## Physical Properties

Mass and material data are recorded in the REV00 workbook.

Center of mass and inertia tensors are intentionally deferred until they are extracted from the CAD assemblies.

## Consequences

All future IK, gait and simulation work must start from this revision.

Any modification to:

- joint origins;
- axes;
- limits;
- baked STL orientation;
- mesh filenames;
- servo mapping;

must update both the URDF and the canonical workbook in the same commit.
