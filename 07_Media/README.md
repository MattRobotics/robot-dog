# MATDOG Media

This directory contains media created specifically for the MATDOG project.

## Scope

- photographs of assemblies and prototypes
- CAD renders
- URDF and simulation screenshots
- calibration photographs
- test videos
- locomotion demonstrations
- public project graphics
- media assets used for documentation

## Organisation Rule

Engineering evidence should remain close to the related technical record when practical.

Examples:

- a servo-bus validation video may be referenced from 09_Logs/Validation_Reports/
- a calibration photograph may be referenced from its calibration procedure
- a public render or project image belongs here

Large raw video files should not be committed to Git unless they are essential engineering evidence.


## Dashboard closeout — 2026-10-11

[Dashboard assets and provenance](Dashboard/2026-10-11/README.md) contains the two
original generated mockups: [Maintenance](Dashboard/2026-10-11/01_mockup_maintenance_illustrative.png)
and [Diagnostics](Dashboard/2026-10-11/02_mockup_diagnostics_illustrative.png).
They are illustrative, not hardware evidence; their PNG bytes are unchanged.
The real STA screenshot and sensitive Wi-Fi mockup remain local and excluded from Git.
The [hardware closeout](../09_Logs/Validation_Reports/MATDOG_HW_NON_MOTION_CLOSEOUT_2026-10-11.md)
owns validation facts; the [Motion handoff](../09_Logs/Development_Log/2026-10-12_MOTION_ENABLEMENT_HANDOFF.md)
owns the next operational gates.
