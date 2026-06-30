# MATDOG — CAD and Robot Description

This directory contains the mechanical representation of MATDOG.

## Scope

- Solid Edge master CAD exports
- URDF robot descriptions
- STL visual meshes
- STL collision meshes
- Mechanical reference data
- Kinematic reference workbooks

## Current Canonical Package

The first complete robot description is:

[MATDOG URDF REV00](URDF/matt_robodog_rev00/)

REV00 contains the validated 17-link and 16-joint kinematic baseline used for:

- joint calibration
- forward kinematics
- inverse kinematics
- stand-pose development
- gait development
- future simulation

## Organisation

03_CAD/
├── README.md
└── URDF/
    ├── README.md
    └── matt_robodog_rev00/

The Solid Edge CAD assemblies remain the geometric source of truth.

The URDF is the canonical kinematic description derived from the CAD master.

Any canonical change to joint origins, axes, limits, mesh orientation or foot-contact frames must update both the URDF package and its kinematic reference workbook.
