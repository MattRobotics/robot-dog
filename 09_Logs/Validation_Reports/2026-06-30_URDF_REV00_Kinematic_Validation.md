# URDF REV00 Kinematic Validation

Date: 2026-06-30

## Objective

Validate the first complete MATDOG URDF model before beginning inverse kinematics and gait implementation.

## Scope

- URDF topology
- Link and joint naming
- Parent-child frame relationships
- Joint axes
- Joint limits
- Servo IDs and directions
- Visual mesh placement
- Collision mesh placement
- Upper-leg baked mesh orientation
- Lower-leg mechanical closing limit

## Results

Validated:

- 17 rigid links
- 16 joints
- Four complete leg chains
- Correct parent-child hierarchy
- Mesh origins aligned to their local link frames
- Visual and collision meshes overlapping correctly
- Upper-leg mesh orientation baked into STL files
- Upper-leg range set to -52.5° to +122.5°
- Lower-leg range set to -92.0° to +37.5°
- Lower-leg closing interference removed at the revised -92.0° limit

## Collision Configuration

REV00 uses the same STL geometry for visual and collision meshes.

This configuration is accepted for CAD-oriented collision checking and URDF Studio validation.

## Not Yet Validated

The following items remain outside the REV00 validation scope:

- CAD-derived center of mass for each link
- Inertia tensors
- Dynamic simulation stability
- Real servo zero calibration
- Servo direction validation on powered hardware
- IK solver implementation
- Gait generation
- Ground-contact simulation

## Status

APPROVED AS KINEMATIC BASELINE FOR IK AND GAIT DEVELOPMENT
