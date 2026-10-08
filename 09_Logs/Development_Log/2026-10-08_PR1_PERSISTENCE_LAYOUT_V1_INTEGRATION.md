# MATDOG — PR-1: Calibration Persistence V1 and flash layout V1 integration (2026-10-08)

**Scope:**
- Code integration by merge plus documentation, checked offline in a cloud container.
- No hardware access, no flash, no serial, NVS, EEPROM or OTA operation, no motion.
- `MOTION_AUTHORIZED=0`.
- **Do not flash `main`** on the current robot; see
  [`FLASH_LAYOUT_SAFETY_NOTICE.md`](../../05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md).

## Git integration

| Item | Value |
|---|---|
| Base `main` | `a09cb76b428f46f0c6a77c049ab906da4dc4fed3` (merge of PR #37) |
| Merged line | `feat/calibration-persistence-record-store-v1` at `7b258b36c257bd455f135aee2667d035c4929544` (13 commits, `4a46b87` … `7b258b3`) |
| Branch | `integration/pr1-persistence-layout-v1` |
| Merge commit | `c358a3e5a44a7f79f3b815a3492368917a9860aa` (`--no-ff`; no squash, rebase or force-push) |
| Conflicts | none: PR #37 and the Persistence line touch disjoint files |
| Tree check | `diff a09cb76..c358a3e` equals `diff b65e75d..7b258b3`, and `diff 7b258b3..c358a3e` equals PR #37's diff (patch-id identical) |
| Evidence | no pre-existing file under `09_Logs/`, `09_Backups/` or `03_CAD/` is modified, renamed or deleted |

All 13 original SHAs, `7b258b3` and `a09cb76` are ancestors of the merge commit. Documentation
for PR-1 is a separate commit on top. It changes Markdown files only.

Not integrated:
- the v0.2.0 integration line (`d7aa369`) and the dev.2/dev.3 line (`b764c25`);
- Wi-Fi PHY diagnostic `a06314e`, WIP M0.4 P2 `43fa92a`, divergent gait line `e170471`;
- no branch, tag or backup ref was modified.

## Flash layout verification (merge commit)

- **`partitions.csv`:**
  - blob identical to `7b258b3` and to the dev.3 tip `b764c25`;
  - `LAYOUT_ID=MATDOG_16M_2x5M_NVS_V1`;
  - rows: `nvs` 0x9000+0x5000, `otadata` 0xE000+0x2000, `app0` 0x10000+0x500000, `app1`
    0x510000+0x500000, `ffat` 0xA10000+0x5D0000, `matdog_nvs` 0xFE0000+0x10000, `coredump`
    0xFF0000+0x10000.
- **Binary table:** `gen_esp32part.py` of `esp32:esp32 3.3.11` gives 3072 bytes, SHA-256
  `8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7`. That equals
  `EXPECTED_TABLE_SHA256` in `scripts/matdog_layout.py`. An independent encoder written for this
  check gives the same hash.
- **`scripts/build.sh`:** FQBN uses `PartitionScheme=custom`. `matdog_layout.py check-fqbn`
  accepts it and refuses `app3M_fat9M_16MB` (`FQBN_LEGACY_SCHEME`).
- **`scripts/upload.sh`:** executed; prints `REFUSE`, performs no hardware operation, exits 1.
- **`scripts/flash_app_only.sh`:** carries the P2.3 layout gate (legacy table refused, write range
  may not touch `matdog_nvs`).

## Firmware facts relevant to `main`

- The firmware sources of `7b258b3` are identical to `be0c129`. The convergence audit records an
  application built from `be0c129` being flashed on 2026-10-03, during the layout migration.
- The persistence record, store, NVS backend and save-marker sources are identical between
  `7b258b3` and the dev.3 tip `b764c25`.
- The firmware version literal is still `0.1.0`, the same number as the frozen
  `matdog-controller-v0.1.0` tag; only the build id distinguishes a `main` build. Not changed
  here: it is firmware source.

## Test results (first run, working tree at the merge commit)

Toolchain installed in the container, not in the repository:
- `arduino-cli 1.5.1` (commit `01f3d4f2b`, as recorded in `SOURCE_PROVENANCE.md`);
- `esp32:esp32 3.3.11`;
- the pinned libraries Adafruit BNO08x 1.2.5, Adafruit BusIO 1.17.4, Adafruit Unified Sensor
  1.1.15, SCServo 1.0.2, Adafruit NeoPixel 1.15.5;
- `scipy`, Git LFS objects materialized, and esptool 5.3.1 with pyserial 3.5 in a separate venv.

| Check | Result |
|---|---|
| `scripts/tests/run_host_tests.sh` | PASS — exit 0, every suite 0 failures; the USB-framing mutants are reported `DETECTED` as intended |
| `test_matdog_layout.py` | OK (0 skipped with the toolchain installed) |
| `test_build_manifest.py` | OK |
| `test_migration_m0.py`, `test_ota_partition_logic.py`, `test_command_router_framing_mutations.py` | OK / OK / PASS |
| `test_migration_m0_write.py` (esptool 5.3.1, pyserial 3.5) | OK, 40 tests (2 skipped) |
| `MATDOG_PROFILE=USB_ONLY scripts/build.sh` | PASS; app 1,120,304 bytes (21.4 % of the 5 MiB slot); layout gate `MATDOG_16M_2x5M_NVS_V1`, table `8f756ecb…`; OTA ingest 0 |
| `MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh` | PASS; app 1,123,408 bytes (21.4 %); same layout gate; OTA ingest 0 |
| Compiler warnings | 2 per profile, all in the third-party SCServo library |

Two environment-only failures were observed and resolved in the container:
- missing `scipy`, and STL files left as Git LFS pointers;
- missing `gen_esp32part.py` before the ESP32 core was installed.

Without these, `static_audit.py` reported 4 findings and then 1. No repository file was changed
to resolve them. The full `static_audit.py` re-run in a clean worktree with the complete
toolchain is recorded in the follow-up section below.

Build binaries are not committed. Their hashes identify this container's builds only; they are
not release artifacts and were not compared with any flashed image.

## Documentation updated in this PR

Root `README.md`, `05_Firmware/README.md`, the Controller `README.md`,
`FLASH_LAYOUT_SAFETY_NOTICE.md`, `ROADMAP.md`, the Controller `CHANGELOG.md` and this log.
- **Kept:** the general **DO NOT FLASH MAIN** notice. The quoted 2026-10-01 README snapshot and
  every historical report are unchanged.
- **Stated in every notice:** the layout integration is neither a flash authorization nor a
  hardware acceptance.

## Residual risks and open items

- **dev.3:** the robot runs the dev.3 candidate. `main` after PR-1 does not contain dev.1–dev.3.
  The Hardware Validation of 2026-10-07 is execution COMPLETE, acceptance BLOCKED (stated by the
  owner). Its report is not in the repository.
- **No hardware acceptance:** no `main` build has been accepted on hardware. A real power cycle
  of the persistence path has not been performed with any firmware.
- **Stale comment:** `scripts/matdog_layout.py` still says the legacy table is the one "the device
  runs today". That was true on 2026-10-02 and is false since the 2026-10-03 migration. It is
  source and is not edited here.
- **Divergence from the dev.3 line:** the dev.3 line modifies `CHANGELOG.md`, `VALIDATION.md`
  and the Controller `README.md` further. The next integration (PR-2) must merge those on top of
  these PR-1 documentation edits.
