# Digital Zero snapshots — 2026-08-14

> ## ⚠️ HISTORICAL EVIDENCE — SUPERSEDED
>
> **These values are NOT active truth.**
> All MATDOG calibration was **RESET on 2026-08-27** when the 17 servos were removed, bench-provisioned
> and remounted. See [MATDOG_CALIBRATION_RESET_2026-08-27.md](../MATDOG_CALIBRATION_RESET_2026-08-27.md).

## Status

| Field | Value |
|---|---|
| Captured | 2026-08-14 |
| Scope | 12 leg servos (M11–M43) — the 5 head servos did not exist yet |
| Current status | **HISTORICAL / SUPERSEDED — PENDING FULL RECALIBRATION** |
| Safe to import as current state? | **No** |
| Safe to command hardware from? | **No** |

## Why superseded

Every servo represented here has since been:

1. physically removed from the robot;
2. bench-provisioned to `PositionOffset = 0` with physical RAW centre 2048 ±1;
3. in several cases **recoded to a different bus ID**;
4. remounted into its linkage.

The `PositionOffset` values recorded in these snapshots are therefore no longer present on any
servo — **all 17 units now hold `PositionOffset = 0`**. The mechanical q0 relationship they
describe belongs to an installation that no longer exists.

Several units also changed identity. For example M22 now answers on ID 13 as `LF_HIP`, and M31 now
answers on ID 51 as `NECK_ROTATION`. Reading these files by bus ID against the current robot will
produce **wrong joint associations**. Current mapping:
[`MATDOG_SERVO_ALLOCATION.yaml`](../../../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml).

## Why kept

This is preserved evidence of the state measured on 2026-08-14, retained deliberately for
provenance and for comparison after recalibration. **Do not delete.**

## Contents

| File | Description |
|---|---|
| `2026-08-14_135649Z_digital_zero_plan.json` | planned digital-zero operation |
| `2026-08-14_135649Z_mechanical_q0_snapshot.json` | mechanical q0 as measured then |
| `2026-08-14_135649Z_st3215_pre_recenter_backup.json` | full ST3215 register backup before recentering |

Each has a `.sha256` companion. All three verified intact during the 2026-08-27 audit.

## Related

- [Calibration reset — 2026-08-27](../MATDOG_CALIBRATION_RESET_2026-08-27.md)
- [ST3215 provisioning campaign](../../Validation_Reports/ST3215_Provisioning_2026-08-27/README.md)
- [C5_R digital recenter — 2026-07-10](../C5_R_digital_recenter/README.md) (also historical)
