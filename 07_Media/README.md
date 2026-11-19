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

## Responsibilities (added 2026-11-19)

| Material | Where |
|---|---|
| Photos, videos, renders, screenshots and visual assets made for the project | This directory, e.g. `Dashboard/<date>/` |
| Screenshots or videos that **are** evidence of a dated test | Next to that test's report in `09_Logs/Validation_Reports/<folder>/`, as in `Dev17_HW_2026-11-07/screenshots/`. They are linked from here when useful, never copied twice |
| Raw recordings of hardware sessions (serial logs, status traces, large videos) | Kept locally on the operator's workstation; the session's evidence manifest lists their hashes |

The Stable `v1.0.2` hardware qualification of 19/11/2026 produced no media, only logs and reports ([evidence](../09_Logs/Validation_Reports/Motion_1314bc4_HW_2026-11-19/README.md)). The test index is [`08_Tests`](../08_Tests/README.md).
