# MATDOG URDF Packages

This directory contains versioned MATDOG robot-description packages.

## Active Baseline

[MATDOG URDF package — REV01-FIX](matt_robodog_rev00/)

The operational model is **REV01-FIX** (PR #57, 2026-10-31, verified offline; no hardware validation of the new geometry). The directory and file names still say `matt_robodog_rev00` for compatibility; identify the revision by the SHA-256 in `matt_robodog_rev00/SHA256SUMS.txt`. It has one canonical STL per link, shared by visual and collision, and the Solid Edge inertial properties.

## Historical calibration basis

[REV00 calibration basis](historical_calibration_basis/rev00/README.md) preserves the REV00 URDF, manifest and `base_link` mesh to which the hardware calibration chain (NVS gen-1, geometry tag `3713f4ddc43b204e`) is bound. It is historical provenance, not the operational model; do not edit it.

## Revision Rule

Do not overwrite the canonical package for experimental work.

Create a new revision directory only when a geometry, joint-frame, calibration or mesh-orientation change becomes canonical.
