# MATDOG flash layout safety notice — 2026-10-08

**Documentation only.** This notice changes no source, script, partition table, test, evidence or
hardware configuration, and it authorizes nothing: no flash, no migration, no calibration, no
motion. `MOTION_AUTHORIZED=0`.

## Rule

> **Do not build from, or flash, `main` to update the current robot.**
>
> `main` still carries the legacy flash layout. The current robot runs a different one.
> Updating the robot from `main` risks restoring the old partition table and losing the layout
> that holds the persisted calibration record.

## What differs

| | `main` (this branch's base, `b65e75d`) | dev.3 candidate line |
|---|---|---|
| Partition scheme | `PartitionScheme=app3M_fat9M_16MB` — the legacy Arduino scheme, pinned in `scripts/build.sh`, `scripts/upload.sh` and `scripts/flash_app_only.sh` | `PartitionScheme=custom`, table from `partitions.csv`, `LAYOUT_ID=MATDOG_16M_2x5M_NVS_V1` |
| Application slots | two 3 MiB slots (`app0` at `0x10000`, `app1` at `0x310000`, FFAT at `0x610000`) | two 5 MiB slots (`app0` at `0x10000`, `app1` at `0x510000`) |
| `matdog_nvs` partition (`0xFE0000`, 64 KiB) | absent | present |
| Calibration Persistence V1 | not in the tree | implemented; `RESTORE` is `NOT_IMPLEMENTED` |
| `scripts/upload.sh` | full `arduino-cli upload`: writes bootloader (`0x0`), **partition table (`0x8000`)**, `boot_app0`/otadata (`0xE000`) and the application, every time | refusing stub: performs no hardware operation, exits 1 |
| `scripts/flash_app_only.sh` | writes only the active application slot; no reference to the layout or to `matdog_nvs` | adds a flash-layout gate; refuses a device that still has the legacy table |

The `main` column is read from the files of this repository at `b65e75d`. The dev.3 column is read
from commit
[`b764c25`](https://github.com/MattRobotics/robot-dog/tree/b764c25c9530350331dbaa59ea2d84ddab3a3f69)
([`partitions.csv`](https://github.com/MattRobotics/robot-dog/blob/b764c25c9530350331dbaa59ea2d84ddab3a3f69/05_Firmware/MATDOG_Controller/partitions.csv),
[`scripts/upload.sh`](https://github.com/MattRobotics/robot-dog/blob/b764c25c9530350331dbaa59ea2d84ddab3a3f69/05_Firmware/MATDOG_Controller/scripts/upload.sh),
[`scripts/flash_app_only.sh`](https://github.com/MattRobotics/robot-dog/blob/b764c25c9530350331dbaa59ea2d84ddab3a3f69/05_Firmware/MATDOG_Controller/scripts/flash_app_only.sh)).
The legacy slot offsets are quoted from the convergence audit (below), not re-derived from a
device.

## What the robot runs

The convergence audit records that the robot's partition table was written to
`MATDOG_16M_2x5M_NVS_V1` on 2026-10-03 (table SHA-256
`8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`), and the dev.2 and dev.3
reports record firmware running on it: `matdog_nvs` was empty under dev.2 and held an
acknowledged calibration record (generation 1) after the dev.3 SAVE/ACK. The migration execution
report itself and the raw evidence are off-repository and are **not** published here; this notice
does not reproduce them.

## Why `main` must not be used on the current robot

1. `main`'s `scripts/upload.sh` rewrites the partition table on every run (its own header says so).
   The dev.3 stub documents the consequence for layout V1: a full rewrite "can put the old
   partition table back, or move/erase the persistent MATDOG NVS partition".
2. `main`'s `scripts/flash_app_only.sh` has no layout gate, there is no `partitions.csv` on
   `main`, and the `main` firmware sources contain no `matdog_nvs` or Calibration Persistence
   code.
3. The legacy scripts remain in `main` as historical tooling. Their documented use (bring-up of a
   replacement or blank board, or a deliberate partition-scheme change) requires separate explicit
   operator authorization and is not authorized by this notice.

## dev.3 status

| Item | Value |
|---|---|
| Firmware provenance commit | [`b3fd945bdaf37d192d97b605ac0f59b67f1dba45`](https://github.com/MattRobotics/robot-dog/commit/b3fd945bdaf37d192d97b605ac0f59b67f1dba45) (`0.2.0-dev.3`) |
| Branch tip | `b764c25c9530350331dbaa59ea2d84ddab3a3f69` — adds documentation only, no source |
| Branch | `backup/matdog-dev3-preservation-20261008` — a preservation branch, not a merge candidate as named |
| Hardware Validation 2026-10-07 | execution **COMPLETE**, acceptance **BLOCKED** |
| Release status | **candidate, not accepted.** Not an authorized release baseline. Not on `main`. |

The 2026-10-07 status is stated by the project owner. Its validation report, result matrix and
defect record are **not yet in this repository**; they will be added in a later documentation
change and are not reconstructed here. The latest dev.3 evidence published on GitHub is the
[2026-10-06 delta audit](https://github.com/MattRobotics/robot-dog/blob/b764c25c9530350331dbaa59ea2d84ddab3a3f69/09_Logs/Validation_Reports/MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md).

Standing limits: `MOTION_AUTHORIZED=0`; `RESTORE=NOT_IMPLEMENTED`; stand and gait hardware
`BLOCKED`; no operational envelope approved.

## Historical records are unchanged

The frozen `matdog-controller-v0.1.0` release, the 2026-10-01 development firmware `dfcecb670d05`
and its 24/24 Full Calibration report, and every dated report remain valid evidence for their own
date and scope. They are not rewritten. The 2026-10-01 snapshot in the root README is not updated
by this notice; a full status synchronization is a separate change.

## Sources

- Convergence audit (branch `audit/matdog-v0.2.0-dev1-claude-handoff`, evidence only, not for
  merge): [`MATDOG_FIRMWARE_CONVERGENCE_AUDIT_V0_2.md`](https://github.com/MattRobotics/robot-dog/blob/b6d99573755f85993c189227b6a3f890fb15fe20/_audit/MATDOG_V0_2_DEV1_CLAUDE_HANDOFF/MATDOG_FIRMWARE_CONVERGENCE_AUDIT_V0_2.md)
  (legacy scheme and offsets, 2026-10-03 table write).
- dev.3 delta audit:
  [`MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md`](https://github.com/MattRobotics/robot-dog/blob/b764c25c9530350331dbaa59ea2d84ddab3a3f69/09_Logs/Validation_Reports/MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md).
- This repository: [`scripts/build.sh`](scripts/build.sh), [`scripts/upload.sh`](scripts/upload.sh),
  [`scripts/flash_app_only.sh`](scripts/flash_app_only.sh).
