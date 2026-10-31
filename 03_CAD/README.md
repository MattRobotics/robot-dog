# MATDOG — CAD and Robot Description

This directory contains the mechanical representation of MATDOG.

## Scope

- Solid Edge master CAD exports
- URDF robot descriptions
- STL meshes: one canonical STL per link (17), shared by `<visual>` and `<collision>`
- Mechanical reference data
- Kinematic reference workbooks

## Current Canonical Package

The operational robot description is **REV01-FIX** (integrated in `main` by PR #57, 2026-10-31, verified offline only):

[MATDOG URDF package](URDF/matt_robodog_rev00/) — the directory and file names keep the `rev00` label on purpose, for compatibility with existing tools and paths. The revision is identified by the SHA-256 values in its `SHA256SUMS.txt`.

REV01-FIX keeps the REV00 17-link and 16-joint kinematics (axes, limits, motor IDs unchanged) and adds the new `base_link`, the four HIP Z offsets raised by 2.2 mm, the Solid Edge masses/CoM/inertias, and a single STL per link shared by visual and collision. It is the model used for:

- joint calibration
- forward kinematics
- inverse kinematics
- stand-pose development
- gait development
- future simulation

The original REV00 inputs that the hardware calibration chain is bound to (URDF, manifest, old `base_link`) are preserved, unchanged and clearly historical, in [`URDF/historical_calibration_basis/rev00/`](URDF/historical_calibration_basis/rev00/README.md). They are not the operational model.

## Organisation

03_CAD/
├── README.md
└── URDF/
    ├── README.md
    ├── historical_calibration_basis/rev00/   (historical REV00 calibration basis)
    └── matt_robodog_rev00/                   (operational REV01-FIX package)
        ├── matt_robodog_rev00.urdf
        ├── meshes/collision/                 (17 canonical STL)
        └── reference/                        (single 7-sheet workbook)

The Solid Edge CAD assemblies remain the geometric source of truth.

The URDF is the canonical kinematic description derived from the CAD master.

Any canonical change to joint origins, axes, limits, mesh orientation or foot-contact frames must update both the URDF package and its kinematic reference workbook.
