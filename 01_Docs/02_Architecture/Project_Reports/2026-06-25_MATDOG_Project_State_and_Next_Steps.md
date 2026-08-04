# MATDOG — Historical project-state report

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
