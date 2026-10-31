# MATDOG Bill of Materials

This directory contains the bill of materials and component records for MATDOG.

## Scope

- actuators
- electronics
- mechanical components
- fasteners
- sensors
- batteries and power electronics
- manufacturing consumables
- supplier references and costs

## Current Status

The mechanical and electronic architecture is still evolving.

The canonical kinematic mass and material references for the current robot body and legs are stored in:

[MATDOG REV01-FIX reference workbook](../03_CAD/URDF/matt_robodog_rev00/reference/Coordinate_Cinematiche_matt_robodog_rev00.xlsx) (file name keeps the `rev00` label for compatibility)

That workbook is not a procurement BOM. It is the single canonical 7-sheet engineering reference: joint table, per-link masses, CoM and inertias (Solid Edge, 2026-10-30, nominal configuration including the Jetson Orin: 17 links 2.928 kg vs 2.937 kg assembly), mass reconciliation with the 2.807 kg weighing (historical, without Jetson), and the preliminary ST3215 3S speed/torque reference (40 RPM design reference, not qualified; the URDF limit stays 29 RPM). The sheets that existed before the 2026-10-31 consolidation remain in Git history.

## Future Structure

This directory will contain versioned BOM files when the following subsystems are frozen:

- mechanical assembly
- servo and wiring system
- power system
- battery and BMS
- head and sensor package
- onboard computing hardware
