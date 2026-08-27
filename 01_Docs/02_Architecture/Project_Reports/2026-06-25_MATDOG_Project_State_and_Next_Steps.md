# MATDOG — Historical project-state report

> ## ⚠️ SUPERSEDED DOCUMENTATION — STATION-MEDIATED PHASE
>
> This document describes the **Station-mediated, 12-servo** architecture and the pre-2026-08-27
> physical installation. It is **historical evidence**, not current operational truth.
>
> - The **ESP32-S3 coprocessor** now owns the ST3215 bus, not NormaCore Station.
> - The robot has **17 servos** (12 leg + 5 head/jaw), not 12.
> - **All calibration was RESET** on 2026-08-27; 14 of 17 servos were recoded to new bus IDs.
>
> Current: [ARCHITECTURE.md](../../../01_Docs/02_Architecture/ARCHITECTURE.md) ·
> [calibration reset](../../../09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md) ·
> [historical archive](../../../09_Logs/Historical/NormaCore_MATDOG_Archive/README.md)


**Original date:** 25 June 2026  
**Status:** historical snapshot, superseded

This document originally recorded the first MATDOG repository split, hardware bring-up and pre-URDF development plan. It is no longer an operational project-state document.

For the current project state use:

```text
README.md
REPOSITORY_VERIFICATION_INDEX.md
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
09_Logs/Development_Log/2026-08-04_LF_V25_AND_REPOSITORY_CLEANUP.md
```

## Historical scope retained

The June 2026 phase established:

- the MATDOG/NormaCore repository boundary;
- the twelve-servo ST3215 bus and canonical servo mapping;
- the CAD/URDF-first development strategy;
- Station as the sole serial owner;
- semantic high-level commands instead of raw encoder targets;
- the staged roadmap from calibration to stand, gait and autonomy.

## Current correction

The project has progressed beyond this snapshot:

```text
REV00 CAD/URDF: complete
12-servo digital-home commissioning: complete
four-leg read-only FK: complete
LF mechanical calibration: V25 hardware validated and frozen
RF/RH/LH mechanical calibration: pending
complete all-leg persistent profile: pending
stand and locomotion: pending post-calibration regeneration
```

No complete all-leg calibration program is currently hardware validated. Future work starts from the merged LF V25 architecture rather than from historical versioned programs.

Private research material is intentionally excluded from the public MATDOG project baseline.
