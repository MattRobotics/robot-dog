# MATDOG joint and servo contract

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


Status: `CURRENT_CANONICAL` for the offline foundation.

The normative row sets are `joint_registry.csv` and
`servo_mapping_registry.csv`. Their exact IDs are frozen in
`foundation_expectations.json`. They contain exactly 12 joints, 12 unique
servo IDs, four legs and three actuated joints per leg. Each row links to a
source ID and, for direction, its individual versioned probe result.

Each joint row is checked against REV00 for name, type, parent, child, origin,
RPY, axis, lower/upper limits, effort, velocity, `motorId`, `motorDirection`,
`motorType`, `armature`, and units. The hardware-backed encoder-to-q direction
remains separate from the historical custom `motorDirection`; the eight
differences require open conflict rows.

## Encoder state

- `raw_q0_tick` is the unoffset physical reading used to compute the EEPROM
  Position Offset; it is not a command after recenter.
- `digital_zero_offset_i16` is the signed value written during the separately
  completed 2026-07-10 recenter procedure.
- `zero_tick=2048` is displayed q=0 after recenter. Final readbacks range from
  2048 to 2051 and are preserved per servo.
- Static tracking tolerance is 10 ticks only for the stated static tests. It
  is not a zero, mechanical, dynamic, collision, or safe-limit tolerance.
- Visual-zero and old C5 raw targets are historical after recenter.

The 12-servo digital-zero result is `MATDOG_VERIFIED`; each direction is
supported by its own `PASS_DIRECTION_TEST` hardware record. Those facts do
not prove a motion envelope.

## Calibration-profile association

Each joint has MIN and MAX native contact profile IDs, producing 24 distinct
profiles in exact NormaCore `all_profiles()` order. Software availability is
not physical validation: all 24 are
`software-ready` in pinned NormaCore main; only `LF_UPPER_M12_MIN` and
`LF_UPPER_M12_MAX` are `validated` as repeatable mechanical-contact
observations. The other 22 are `hardware-pending`. No profile has a proven
operational safe limit.

## Prohibitions

This contract does not authorize servo commands, torque, EEPROM writes,
Station startup, a serial owner, first stand, calibration execution, or any
hardware interaction. It provides data for later separately gated work.
