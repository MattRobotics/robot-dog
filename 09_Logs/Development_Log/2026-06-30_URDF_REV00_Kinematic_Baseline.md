# MATDOG URDF REV00 Kinematic Baseline Completed

Date: 2026-06-30

## Milestone

The first complete MATDOG URDF kinematic baseline was completed and archived.

## Completed Work

- Imported final low-poly STL meshes into URDF Studio.
- Built the complete 17-link, 16-joint robot hierarchy.
- Defined parent-relative joint origins.
- Defined joint axes, limits, servo IDs and directions.
- Added visual and collision mesh references.
- Corrected upper-leg orientation.
- Baked the upper-leg mesh transform into the four STL files.
- Removed compensating upper-leg mesh transforms from the final URDF.
- Corrected lower-leg mechanical closing limit to -92.0°.
- Consolidated kinematic, mass and material reference data in the REV00 workbook.
- Archived the canonical URDF package in GitHub.

## Deliverables

- Canonical URDF
- Final STL mesh set
- Kinematic, mass and material reference workbook
- Architecture decision record
- Kinematic validation report
- Asset integrity manifest

## Next Engineering Phase

1. Extract center of mass and inertia tensors from Solid Edge.
2. Add authoritative URDF inertial blocks.
3. Implement one-leg forward and inverse kinematics.
4. Validate stand pose with physical ST3215 servos.
5. Implement gait generation.
6. Perform first walking tests.
