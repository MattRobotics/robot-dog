# MATDOG flash layout safety notice — 2026-10-08 (updated for PR-3)

**Documentation only.** This notice authorizes nothing: no flash, no migration, no calibration, no
motion. `MOTION_AUTHORIZED=0`.

## Rule

> **Do not build from, or flash, `main` to update the current robot.**
>
> `main` carries the correct flash layout, `MATDOG_16M_2x5M_NVS_V1`, since PR-1, a selective part
> of the dev.1 line since PR-2 and the dev.2/dev.3 source delta since PR-3. Its sources correspond to
> the dev.3 candidate on the robot, but a build of `main` is **not** the installed image, and dev.3
> is not an accepted release. Integrating code is not an authorization to flash and not a hardware
> acceptance.

## Scope of this notice

This version describes `main` **after PR-3**. PR-1 merged the Persistence V1 line at
[`7b258b36c257bd455f135aee2667d035c4929544`](https://github.com/MattRobotics/robot-dog/commit/7b258b36c257bd455f135aee2667d035c4929544);
PR-2 added the boundary commit `13da04a290fc4e06d05636b5fd553476ed2d51e5` with its original SHAs plus
three recreated commits ([PR-2 log](../../09_Logs/Development_Log/2026-10-08_PR2_SELECTIVE_DEV1_INTEGRATION.md)).
PR-3 is built on `main`
[`e622a80d539c75a0cde4747419822fbdca7e34b5`](https://github.com/MattRobotics/robot-dog/commit/e622a80d539c75a0cde4747419822fbdca7e34b5)
and adds seven recreated commits from the dev.2/dev.3 candidate
([PR-3 log](../../09_Logs/Development_Log/2026-10-08_PR3_DEV2_DEV3_SELECTIVE_INTEGRATION.md)).
If PR-3 is not merged, `main` is the PR-2 state: identity `0.2.0-dev.1`, without the dev.2 boot
census and the dev.3 thermal verdict.

Any further integration must update this notice in the same change, together with the short
notices in the root `README.md`, `05_Firmware/README.md`, `05_Firmware/MATDOG_Controller/README.md`
and `01_Docs/02_Architecture/ROADMAP.md`.

## What `main` contains after PR-3

| Item | Value on `main` after PR-3 | How it was checked |
|---|---|---|
| Partition table source | `partitions.csv`, `LAYOUT_ID=MATDOG_16M_2x5M_NVS_V1` | blob identical to `7b258b3` and to the dev.3 tip `b764c25` |
| Rows | `nvs` 0x9000+0x5000 · `otadata` 0xE000+0x2000 · `app0` 0x10000+0x500000 · `app1` 0x510000+0x500000 · `ffat` 0xA10000+0x5D0000 · `matdog_nvs` 0xFE0000+0x10000 · `coredump` 0xFF0000+0x10000 | `scripts/matdog_layout.py contract` |
| Binary table SHA-256 | `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7` (3072 bytes) | `gen_esp32part.py` of `esp32:esp32 3.3.11` on `partitions.csv`; same hash from an independent encoder; equal to the pinned `EXPECTED_TABLE_SHA256` |
| `scripts/build.sh` | `PartitionScheme=custom` | FQBN line; `matdog_layout.py check-fqbn` accepts it and refuses `app3M_fat9M_16MB` |
| `scripts/upload.sh` | refusing stub, no hardware operation, exit 1 | executed: prints `REFUSE`, exits 1 |
| `scripts/flash_app_only.sh` | application slot only, with a flash-layout gate that refuses a device still on the legacy table and any write range touching `matdog_nvs` | script header; host tests |
| Calibration Persistence V1 | record, codec, A/B NVS store, SAVE/ACK/RECONCILE, boot LOAD; `RESTORE` not implemented (LOAD never admits a transform) | source; host tests |
| Firmware version literal | `kFirmwareVersion = "0.2.0-dev.3"` (PR-3); `0.2.0-dev.1` after PR-2, `0.1.0` after PR-1 | `src/config/BuildConfig.h` |
| dev.2 / dev.3 content | read-only boot servo census at ROBOT_POWERED startup; thermal verdict from direct reads only; host runner starts the post-abort recovery at most once | PR-3 log |
| Correspondence with dev.3 | every non-Markdown file under `src/` and `scripts/` is byte-identical to the dev.3 tip `b764c25` except the two PR-2 test-gate scripts and the PR-2 manifest; deferred files are absent | PR-3 log |
| dev.1 content | recovery, DALY/release stages, Wi-Fi/OTA V3 (OTA ingest default 0), charging priority, pure motion library (`src/motion`, unwired) | PR-2 log |
| Deferred (not on `main`) | motion execution suites, oracles, G35/G4/G4.1/G5-A evidence, third-party XGO material | `motion_integration_manifest_pr2.json`, [ADR-004](../../09_Logs/Architecture_Decisions/ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md) |

## Why `main` must still not be used on the current robot

1. **It is not the installed image.** The dev.3 report records the robot running `0.2.0-dev.3`,
   build `b3fd945bdaf3`, built from commit `b3fd945`. A build of `main` has another source commit
   and build ID. Source equivalence does not imply an identical binary, and no binary comparison
   was made.
2. **dev.3 is not accepted.** The Hardware Validation of 2026-10-07 has execution COMPLETE and
   acceptance BLOCKED; its report is not in the repository. No release is approved.
3. **No hardware acceptance comes with `main`.** The hardware results (calibration 24/24, SAVE/ACK
   and LOAD across a reset, 2026-10-06) were obtained with the dev.3 candidate, not with a `main`
   build, and a real power cycle was not performed. PR-1, PR-2 and PR-3 are code integrations
   checked offline.
4. **The motion library is not an authorization.** It is compiled and unwired; `MOTION_AUTHORIZED=0`
   and `RESTORE=NOT_IMPLEMENTED` hold, and stand and gait hardware stay blocked.
5. **Any flash needs its own authorization.** Even with the correct layout, a write needs explicit
   operator authorization, a fresh verified backup and the application-only procedure with all
   its gates. `scripts/upload.sh` no longer performs any upload.

## Before PR-1: `main` at `b65e75d` and `a09cb76`

Kept for traceability. This is what `main` contained before PR-1.

| Item | `main` before PR-1 |
|---|---|
| Partition scheme | legacy `PartitionScheme=app3M_fat9M_16MB` in `scripts/build.sh`, `scripts/upload.sh` and `scripts/flash_app_only.sh`; no `partitions.csv` |
| Application slots | two 3 MiB slots (`app0` at `0x10000`, `app1` at `0x310000`, FFAT at `0x610000`), quoted from the convergence audit |
| `matdog_nvs` / Calibration Persistence | absent |
| `scripts/upload.sh` | full `arduino-cli upload`: bootloader, **partition table**, `boot_app0`/otadata and application on every run |
| `scripts/flash_app_only.sh` | active application slot only; no layout gate |

A full upload from that tree can put the legacy table back on a layout-V1 device. The dev.3 stub
says it "can put the old partition table back, or move/erase the persistent MATDOG NVS partition".

## What the robot runs

The convergence audit records that the robot's partition table was written to
`MATDOG_16M_2x5M_NVS_V1` on 2026-10-03 (table SHA-256 `8f756ecb…`).
- The application flashed then was built from `be0c129`, whose firmware sources are identical to
  `7b258b3`.
- The dev.2 and dev.3 reports record later firmware on the same layout. Under dev.2 `matdog_nvs`
  was empty. After the dev.3 SAVE/ACK it held an acknowledged calibration record (generation 1).
- The migration execution report and the raw evidence are off-repository and are not reproduced
  here.

## dev.3 status

| Item | Value |
|---|---|
| Firmware provenance commit | [`b3fd945bdaf37d192d97b605ac0f59b67f1dba45`](https://github.com/MattRobotics/robot-dog/commit/b3fd945bdaf37d192d97b605ac0f59b67f1dba45) (`0.2.0-dev.3`) |
| Branch tip | `b764c25c9530350331dbaa59ea2d84ddab3a3f69` — adds documentation only, no source |
| Branch | `backup/matdog-dev3-preservation-20261008` — a preservation branch, not a merge candidate as named |
| Hardware Validation 2026-10-07 | execution **COMPLETE**, acceptance **BLOCKED** |
| Release status | **candidate, not accepted.** Not an authorized release baseline. Not on `main`. |

The 2026-10-07 status is stated by the project owner. Its validation report, result matrix and
defect record are **not yet in this repository** and are not reconstructed here. The latest dev.3
evidence published on GitHub is the
[2026-10-06 delta audit](https://github.com/MattRobotics/robot-dog/blob/b764c25c9530350331dbaa59ea2d84ddab3a3f69/09_Logs/Validation_Reports/MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md).

Standing limits: `MOTION_AUTHORIZED=0`; `RESTORE=NOT_IMPLEMENTED`; stand and gait hardware
`BLOCKED`; no operational envelope approved.

## Historical records are unchanged

The frozen `matdog-controller-v0.1.0` release, the 2026-10-01 development firmware `dfcecb670d05`
and its 24/24 Full Calibration report, and every dated report remain valid evidence for their own
date and scope. They are not rewritten. The 2026-10-01 snapshot in the root README is kept as a
historical snapshot. A full status synchronization is a separate change.

## Sources

- Convergence audit (branch `audit/matdog-v0.2.0-dev1-claude-handoff`, evidence only, not for
  merge): [`MATDOG_FIRMWARE_CONVERGENCE_AUDIT_V0_2.md`](https://github.com/MattRobotics/robot-dog/blob/b6d99573755f85993c189227b6a3f890fb15fe20/_audit/MATDOG_V0_2_DEV1_CLAUDE_HANDOFF/MATDOG_FIRMWARE_CONVERGENCE_AUDIT_V0_2.md)
  (legacy scheme and offsets, 2026-10-03 table write, flashed `be0c129` application).
- dev.3 delta audit:
  [`MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md`](https://github.com/MattRobotics/robot-dog/blob/b764c25c9530350331dbaa59ea2d84ddab3a3f69/09_Logs/Validation_Reports/MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md).
- PR-3 integration log:
  [`2026-10-08_PR3_DEV2_DEV3_SELECTIVE_INTEGRATION.md`](../../09_Logs/Development_Log/2026-10-08_PR3_DEV2_DEV3_SELECTIVE_INTEGRATION.md).
- PR-2 integration log:
  [`2026-10-08_PR2_SELECTIVE_DEV1_INTEGRATION.md`](../../09_Logs/Development_Log/2026-10-08_PR2_SELECTIVE_DEV1_INTEGRATION.md).
- PR-1 integration log:
  [`2026-10-08_PR1_PERSISTENCE_LAYOUT_V1_INTEGRATION.md`](../../09_Logs/Development_Log/2026-10-08_PR1_PERSISTENCE_LAYOUT_V1_INTEGRATION.md).
- This repository: [`partitions.csv`](partitions.csv), [`scripts/matdog_layout.py`](scripts/matdog_layout.py),
  [`scripts/build.sh`](scripts/build.sh), [`scripts/upload.sh`](scripts/upload.sh),
  [`scripts/flash_app_only.sh`](scripts/flash_app_only.sh).
