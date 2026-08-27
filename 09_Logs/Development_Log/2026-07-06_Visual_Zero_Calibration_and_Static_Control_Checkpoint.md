# 2026-07-06 - Visual-Zero Calibration and Static Control Checkpoint

> ## ⚠️ SUPERSEDED DOCUMENTATION — STATION-MEDIATED PHASE
>
> This document describes the **Station-mediated, 12-servo** architecture and the pre-2026-08-27
> physical installation. It is **historical evidence**, not current operational truth.
>
> - The **ESP32-S3 coprocessor** now owns the ST3215 bus, not NormaCore Station.
> - The robot has **17 servos** (12 leg + 5 head/jaw), not 12.
> - **All calibration was RESET** on 2026-08-27; 14 of 17 servos were recoded to new bus IDs.
>
> Current: [ARCHITECTURE.md](../../01_Docs/02_Architecture/ARCHITECTURE.md) ·
> [calibration reset](../../09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md) ·
> [historical archive](../../09_Logs/Historical/NormaCore_MATDOG_Archive/README.md)


## Milestone

MATDOG completed the transition from direction-only encoder mapping to a stored visual-zero reference and a validated static control path for all 12 leg servos.

## Completed Work

- Captured the intended URDF visual-zero pose through read-only Station telemetry.
- Acquired 18 samples per joint; every joint had a circular encoder spread of 0 ticks.
- Applied the 12 captured values to `zero_encoder_visual`.
- Advanced calibration state to `VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION`.
- Validated YAML-to-URDF contract, encoder wrap handling and encoder-to-radian-to-encoder round trips.
- Validated live telemetry-to-URDF angle conversion on LF M11, M12 and M13.
- Validated guarded micro-probes on M11, M12 and M13.
- Validated static hold on every leg and on all 12 leg servos together.
- Commanded all 12 servos to the saved visual-zero pose through guarded preflight and torque-off cleanup.

## Evidence

- Capture session: `09_Logs/Calibration_Sessions/2026-07-06_205031_visual_zero_capture.result.yaml`
- Calibration YAML: `06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml`
- Full report: `09_Logs/Validation_Reports/2026-07-06_Visual_Zero_and_Static_Actuation_Validation.md`

## Open Risk

M13 returned reliably to its initial position but showed a larger strict target residual than M11 and M12 during the micro-probe. This is not a calibration failure, but it must be reviewed before supported standing or higher-load validation.

## Next Technical Milestone

`Station telemetry -> calibrated joint angles -> single-leg forward kinematics -> physical foot-position comparison -> single-leg inverse kinematics -> supported low-stand planning`

## Not Yet Approved

- `zero_encoder_final`
- physical safe limits
- unsupported standing
- gait generation
- dynamic locomotion
