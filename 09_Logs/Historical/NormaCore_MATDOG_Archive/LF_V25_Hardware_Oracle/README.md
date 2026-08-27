# LF V25 — historical hardware oracle

> # ⚠️ HISTORICAL HARDWARE ORACLE
> # STATION-MEDIATED ARCHITECTURE
> # PRE-2026-08-27 PHYSICAL INSTALLATION
> # NOT CURRENT MATDOG RUNTIME
> # NOT CURRENT CALIBRATION
> # DO NOT COMMAND HARDWARE FROM THIS ARCHIVE

Complete archive of the **LF V25 Station-mediated calibrator** — the only MATDOG full-leg
calibration flow ever genuinely hardware-validated. Archived from `MattRobotics/norma-core` before
its branch was retired.

---

## Provenance

Machine-readable: [`PROVENANCE.json`](PROVENANCE.json)

| Field | Value |
|---|---|
| Source repository | `MattRobotics/norma-core` |
| Original branch | `release/matdog-lf-calibrator-v25` |
| Head commit | `f87dd1fbc7e8100d275c74f9af448642f3429680` |
| Head tree | `b1a18b52b381e64549f485ceb67e0dfd0198f93b` |
| Merge-base with `main` | `32e3222c87016b7f5d7c1c1da497a4cea3e7b80a` |
| `main` at archive time | `4a8ed6337261553b79c928975808d294c9ca723b` |
| Unique commits on branch | **92** |
| `main` ahead by | 19 |
| Fully merged into `main` | **No** |
| Files changed vs merge-base | 16 (all MATDOG-specific) |
| Archived | 2026-08-27 |

Head SHA was verified directly against the live remote with `git ls-remote` at archive time — not
taken from a summary.

## Contents

```text
source/                            16 files at branch head, byte-for-byte
lf_v25_vs_merge_base.patch         full diff vs merge-base (11 278 insertions, 532 deletions)
unique_commits.txt                 all 92 unique commits with dates and subjects
file_inventory.txt                 per-file diffstat
PROVENANCE.json                    machine-readable provenance
SHA256SUMS                         integrity manifest
../bundles/lf_v25_unique.bundle    git bundle carrying the 92 commits
```

### Archived source files

```text
.github/workflows/matdog-lf-freeze-artifact.yml
.github/workflows/matdog-native-calibrator-check.yml
.github/workflows/matdog-native-observer-check.yml
.github/workflows/matdog-v42-pinned-station.yml
software/drivers/st3215/src/auto_calibrate/matdog.rs
software/drivers/st3215/src/auto_calibrate/matdog_test.rs
software/drivers/st3215/src/bin/matdog_lf_freeze.rs
software/drivers/st3215/src/port.rs
tools/matdog/matdog_headless_auto_calibrate.py
tools/matdog/matdog_lf_profile.py
tools/matdog/matdog_native_observer_contract.py
tools/matdog/matdog_v42_pinned_launcher.py
tools/matdog/test_matdog_headless_auto_calibrate.py
tools/matdog/test_matdog_lf_profile.py
tools/matdog/test_matdog_native_observer_contract.py
tools/matdog/test_matdog_v42_pinned_launcher.py
```

Only the MATDOG-specific delta is archived. The rest of NormaCore is upstream code and is
deliberately **not** copied.

## Reconstruction

The archive is readable **without** the deleted branch. Three independent paths:

**1. Direct** — `source/` holds every file at branch head, byte-for-byte. Verify with
`sha256sum -c SHA256SUMS`.

**2. Patch** — apply against the merge-base:

```bash
git checkout 32e3222c87016b7f5d7c1c1da497a4cea3e7b80a
git apply lf_v25_vs_merge_base.patch
```

**3. Bundle** — restore the full commit history:

```bash
git fetch ../bundles/lf_v25_unique.bundle \
  refs/remotes/origin/release/matdog-lf-calibrator-v25:refs/heads/lf-v25-restored
```

> The bundle is *thin*: it requires base commit `32e3222c…`, which is reachable from
> `MattRobotics/norma-core` `main`. `main` is retained permanently, so the bundle stays usable.
> If `main` were ever unavailable, `source/` and the patch remain fully self-contained.

## Hardware validation — what LF V25 actually proved

Executed 2026-08-04 on the **previous** physical installation:

```text
58/58 sequence complete
6/6 LF mechanical contacts accepted
URDF affine gate PASS
supervised hardware-witness gate PASS
RAM q0 staging PASS
Station shutdown and serial release PASS
transactional EEPROM freeze PASS
persistent LF profile PASS
global torque OFF verified
```

Final hardware contacts:

```text
HIP   MIN -42.803°   MAX +39.375°
UPPER MIN -53.525°   MAX +122.607°
LOWER MIN -91.846°   MAX +34.277°
```

Frozen `PositionOffset` values at that time: M11 `127`, M12 `851`, M13 `−486`.

## ⚠️ Why none of that is current state

| Then | Now |
|---|---|
| Station was sole ST3215 serial owner | **ESP32-S3 coprocessor** owns the bus |
| M11/M12/M13 mounted in the LF leg | all 17 servos removed, provisioned, **remounted** |
| `PositionOffset` 127 / 851 / −486 | **`PositionOffset = 0` on all 17 units** |
| M11 = LF lower, ID 11 | **M11 is now ID 52, `NECK_PITCH`** |
| 12 servos | **17 servos** (12 leg + 5 head/jaw) |

Reading this archive by bus ID against the current robot will associate the **wrong joint**.
Current mapping: [`MATDOG_SERVO_ALLOCATION.yaml`](../../../../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml).

## Toolchain and runtime assumptions

| Assumption | Status |
|---|---|
| NormaCore Station running and owning the serial port | **no longer the architecture** |
| Station-mediated command path | superseded |
| Rust toolchain + `cargo` build of the `st3215` driver | still buildable from `source/` |
| Python host tools under `tools/matdog/` | require the Station Python bindings |
| GitHub Actions workflows | reference `norma-core` CI, not `robot-dog` |

## What remains genuinely valuable

The **algorithms and engineering decisions**: contact-detection state machine, bounded evidence
gates, affine q0 derivation with URDF cross-check, supervised hardware-witness gating,
transactional EEPROM freeze with backup/readback/rollback, and torque-OFF-on-every-exit
discipline. These informed the ESP32-S3 provisioner design and remain the reference for what a
correct calibration result looks like.

## Related

- [NormaCore MATDOG archive index](../README.md)
- [Canonical architecture](../../../../01_Docs/02_Architecture/ARCHITECTURE.md)
- [Calibration reset](../../../Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md)
- [Generic V25 superseded WIP](../Generic_V25_Superseded_WIP/README.md)
