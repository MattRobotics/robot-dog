# MATDOG Electronics

This directory contains the electrical architecture and hardware integration records for MATDOG.

## Current Validated Control Chain

    Asus Ubuntu
    → NormaCore Station
    → Waveshare Bus Servo Adapter
    → custom power-distribution board
    → 12 × Feetech ST3215

The current servo bus, branch wiring and power-distribution board are validated for the locomotion-development phase.

## Current Contents

- Servo_Mapping/
  Canonical mapping between semantic MATDOG joints and physical ST3215 IDs.

## Current Servo Mapping

    LF: hip M13, upper M12, lower M11
    RF: hip M23, upper M22, lower M21
    RH: hip M33, upper M32, lower M31
    LH: hip M43, upper M42, lower M41

## Future Contents

- wiring diagrams
- power-distribution-board documentation
- battery and BMS design
- charging and docking system
- IMU integration
- head sensors
- Jetson and coprocessor integration
- electrical validation measurements

Raw ST3215 bus diagnostics remain part of NormaCore Station.
MATDOG-specific wiring, mappings and electrical design belong here.
