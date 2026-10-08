# MATDOG — Flash layout divergence: documentation mitigation (2026-10-08)

**Scope:** documentation only. No firmware, script, partition table, test, evidence or hardware
change. `MOTION_AUTHORIZED=0`.

## Finding

`main` and the robot use different flash layouts.

| | Layout |
|---|---|
| `main` at `b65e75d72f404499b37be055899280fc4187d5f9` | legacy `PartitionScheme=app3M_fat9M_16MB`; no `partitions.csv`; no `matdog_nvs` |
| Robot (written 2026-10-03, per the convergence audit) | `MATDOG_16M_2x5M_NVS_V1`, partition table SHA-256 `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7` |

The divergence is recorded in the convergence audit on the evidence branch
`audit/matdog-v0.2.0-dev1-claude-handoff`. It was found to be unmitigated in the public
documentation during the read-only repository convergence audit of 2026-10-08: nothing on `main`
told a reader that `main` is not flashable on the current robot.

## Risk

`main`'s `05_Firmware/MATDOG_Controller/scripts/upload.sh` is a full `arduino-cli upload`. Its own
header states that it writes the bootloader, the partition table, `boot_app0`/otadata and the
application every time. For layout V1 the dev.3 line documents the consequence: a full rewrite
"can put the old partition table back, or move/erase the persistent MATDOG NVS partition".
`main`'s `scripts/flash_app_only.sh` has no layout gate, and `main` contains no Calibration
Persistence code. A reader following `main` could therefore flash the robot with the wrong layout.

## Mitigation

[PR #37](https://github.com/MattRobotics/robot-dog/pull/37) adds, as documentation only:

- [`FLASH_LAYOUT_SAFETY_NOTICE.md`](../../05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md):
  the rule (do not use `main` to update the current robot), a `main`-versus-dev.3 comparison and
  its sources;
- short notices linking to it in the root [`README.md`](../../README.md),
  [`05_Firmware/README.md`](../../05_Firmware/README.md), the
  [Controller README](../../05_Firmware/MATDOG_Controller/README.md) and the
  [`ROADMAP.md`](../../01_Docs/02_Architecture/ROADMAP.md);
- the root README snapshot of 2026-10-01 relabelled *Historical snapshot*, content unchanged;
- this log.

The notice is bound to `b65e75d`; PR-1 must update it when layout V1 is integrated.

## What did not change

No firmware source, `upload.sh`, `flash_app_only.sh`, `build.sh`, partition table, test or
configuration was modified. No evidence file and no branch other than the PR #37 branch was
touched. No flash, serial, NVS, EEPROM, OTA or motion operation was performed.

## dev.3 status

| Item | Value |
|---|---|
| Provenance commit | `b3fd945bdaf37d192d97b605ac0f59b67f1dba45` (`0.2.0-dev.3`) |
| Branch tip | `b764c25c9530350331dbaa59ea2d84ddab3a3f69`, documentation only |
| Hardware Validation 2026-10-07 | execution **COMPLETE**, acceptance **BLOCKED** |
| Release status | candidate; **not an accepted release**; not on `main` |

The 2026-10-07 status is stated by the project owner. Its validation report, result matrix and
defect record are not yet in this repository and are not reconstructed here. The latest dev.3
evidence on GitHub is the
[2026-10-06 delta audit](https://github.com/MattRobotics/robot-dog/blob/b764c25c9530350331dbaa59ea2d84ddab3a3f69/09_Logs/Validation_Reports/MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md).

## Standing limits

`MOTION_AUTHORIZED=0`. `RESTORE=NOT_IMPLEMENTED`. Stand and gait hardware are blocked and no
operational envelope is approved. This log authorizes no flash, migration, calibration or motion.

## Open

- PR-1 updates the safety notice when layout V1 reaches `main`.
- The 2026-10-07 validation report, result matrix and defect record are to be published in a
  later documentation change.
- A full status synchronization of the README, ROADMAP and VALIDATION remains a separate change.
