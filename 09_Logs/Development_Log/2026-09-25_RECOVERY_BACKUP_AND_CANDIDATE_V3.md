# Recovery-backup gate hardening + candidate V3 (real Wi-Fi/OTA credentials)

**Date:** 2026-09-25/26 · **Branch:** `feat/controller-nextgen-integration-v1` @ `ad08db6d175e3a01ecd21186ec62870a62b14cfb`

Powered, no-motion hardware session with the operator physically present. Covers: a fresh
full-flash backup (the two prior stub-based attempts failed identically; a `--no-stub` ROM-loader
read succeeded), a source fix so `flash_app_only.sh` can authorize that fresh backup without
weakening its fail-closed default, and a new candidate build now carrying real Wi-Fi/OTA
credentials. **No flash performed. No motion. No servo/DALY write. No merge to `main`.**

## Fresh full-flash backup

Two ordinary stub-based `esptool read-flash` attempts failed deterministically at the same byte
offset (`0x0002d000`, 184,320 bytes), first at esptool's default baud, then again at a conservative
115200 baud — ruling out baud as the cause. Per operator authorization, a `--no-stub` (ROM
bootloader) full read was attempted instead and succeeded completely:

```text
BACKUP_PATH    /home/matteo-manicardi/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-25_205220_nostub.bin
BACKUP_SIZE    16777216
BACKUP_SHA256  8758be21fea080c7a824315cd6c3df77d76de420f99a4265c3490a129b7bfe90
DEVICE_MAC     14:c1:9f:22:75:94
USB_IDENTITY   303a:1001, Espressif USB JTAG/serial debug unit
ESPTOOL_VERSION  5.3.1
FLASH_SIZE     16MB (manufacturer 0x68, device 0x4018)
READ_METHOD    NO_STUB_FULL (offset 0x000000, size 0x1000000, baud 115200)
READ_DURATION  1525.8 s (~25.4 min — the ROM-loader path has no stub-flasher speedup)
```

Cross-checked read-only, without modifying flash: the backup's own partition-table region
(offset `0x8000`, size `0x1000`) was parsed with `scripts/ota_partition_logic.py`'s existing pure
parser — 6 sane entries (`nvs`, `otadata`, `app0`, `app1`, `ffat`, `coredump`) matching the expected
`app3M_fat9M_16MB` scheme exactly. Separately, `scripts/verify_application_partition.py` was run
against the LIVE device (read-only) and reported `app0` at offset `0x010000`, size `3145728` —
an exact numeric match to the backup's own parsed `app0` entry. Full recovery manifest recorded at
`matdog_esp32s3_fullflash_2026-09-25_205220_nostub.manifest.txt` alongside the backup. The historical
`matdog_esp32s3_fullflash_2026-09-10.bin` was never touched.

The two failed stub-based attempts breaking at the identical byte offset both times is noted but not
explained — consistent with (not proof of) the intermittent USB-Serial-JTAG transport quirk this
repository's `flash_app_only.sh` already documents for post-write reads, now also observed on a
first-attempt pre-write read.

## `flash_app_only.sh` backup-gate fix (commit `ad08db6`)

The backup gate previously accepted exactly one full-flash backup, whose SHA256 was pinned in
source. A fresh session backup cannot be pinned ahead of time, so the gate needed to accept one
without weakening the historical default's protection or accepting *any* backup by path/size alone.

- New `scripts/backup_gate_logic.py` (pure, host-tested, mirrors `build_manifest.py`'s design
  exactly): the historical default's hash stays pinned and reviewed, unchanged. Any other backup
  path is a "custom" backup, accepted only together with an explicitly authorized expected SHA256 —
  `MATDOG_FLASH_BACKUP_SHA256` (stated directly) or a companion recovery manifest
  (`MATDOG_FLASH_BACKUP_MANIFEST`, defaulting to `<backup>.manifest.txt`) with a `BACKUP_SHA256=`
  line. 25 offline tests; the core no-hash-refusal path was mutation-tested (reverted, confirmed 3
  tests fail, restored).
- `scripts/static_audit.py` gained `check_backup_gate_provenance()`: the pinned historical hash's
  exact value, the absence of a permissive default, and the flash script's wiring are all
  audit-enforced. The PRE-EXISTING anti-weakening check that looked for the old literal bash
  `[ "$BACKUP_SIZE" -eq ... ]`/`[ "$BACKUP_SHA256" = ... ]` comparisons was updated to check the new
  mechanism's wiring instead (the comparison itself moved into the tested Python module) — both the
  new check and this update were mutation-tested (changing the pinned hash, and dropping the
  `--actual-size` wiring, were each confirmed to fail the audit, then reverted).
- Full offline matrix re-run clean: `run_host_tests.sh` (17 suites, 5804 checks), `static_audit.py`
  PASS (102 files), `git diff --check` clean. Committed and pushed as `ad08db6`; local HEAD verified
  equal to `origin/feat/controller-nextgen-integration-v1` before proceeding.

## Local Wi-Fi/OTA credentials

Both required local, gitignored files are now present (values never printed, never committed):

```text
WIFI_CREDENTIALS_PRESENT=YES   (chmod 600, gitignored, untracked — operator-supplied temporary
                                credentials for this validation campaign, to be changed later)
OTA_SECRET_PRESENT=YES         (chmod 600, gitignored, untracked — freshly generated via
                                `openssl rand -hex 32`, 32 bytes)
```

## Candidate V3 — exact provenance

Superseding the V2 candidate (`e3dd4e36e327...`, `APPLICATION_SHA256=1ac9694901e4...`), which had
neither real Wi-Fi credentials nor a real OTA secret compiled in. Built from the clean HEAD after the
backup-gate fix:

```text
SOURCE_HEAD          ad08db6d175e3a01ecd21186ec62870a62b14cfb
BUILD_ID             ad08db6d175e
SOURCE_STATE         CLEAN
HARDWARE_PROFILE     ROBOT_POWERED
OTA_INGEST_ENABLED   1   (explicit MATDOG_OTA_INGEST_VALIDATION=1 override; source default is 0)
WIFI_CREDENTIALS_PRESENT  YES
OTA_SECRET_PRESENT        YES
FQBN                 esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi
APPLICATION_SIZE      1027472
APPLICATION_SHA256    a067cbb5f89a1dbb645bf65accc8d43c8c6887590109bbb303fc2bcb355f606e
artifact path         05_Firmware/MATDOG_Controller/build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin
```

Reproducible byte-for-byte via `MATDOG_OTA_INGEST_VALIDATION=1 MATDOG_PROFILE=ROBOT_POWERED
scripts/build.sh` from this exact commit, with the same local credential files present. Neither
credential value appears anywhere in this document, the manifest, or any build log. Not flashed or
uploaded anywhere; the resting `build/` artifact was rebuilt back to the ordinary `USB_ONLY` default
afterward.

## Fail-closed re-verification (against this exact commit)

- `MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED` source default: `0` (`CalibrationManager.h:30`).
- `MATDOG_ACTIVE_HARDWARE_PROFILE` source default: `USB_ONLY` (`BuildConfig.h:30`).
- `MATDOG_OTA_INGEST_ENABLED` source default: `0` (`OtaManager.h:32`).
- `http_transport_.start()` never called from `Controller::begin()` — confirmed by direct grep
  (only `.begin(...)` and `.update(now_ms)` appear in `Controller.cpp`).
- No `plan()`/`commit()`/`execute()`/`abort()` call in `CommandRouter.cpp`/`ControllerService.h`/
  `Controller.cpp` — confirmed by direct grep, zero matches.
- The candidate's own `APPLICATION_SHA256` was independently re-verified against the actual built
  file (`sha256sum`), byte-for-byte identical to the manifest's recorded value.

## F0 status

The robot was powered off partway through this session (operator report, confirmed passively:
`lsusb`/`/dev/serial/by-id/` show no ESP32). `FINAL_FLASH_ELIGIBLE=DEFERRED_UNPOWERED` — the same
controlling classification this repository's F0 gate has used all session, regardless of every other
gate's status. No authorization question is asked while unpowered.

## Outcome

Candidate V3 (`ad08db6`, `APPLICATION_SHA256=a067cbb5f89a1dbb645bf65accc8d43c8c6887590109bbb303fc2bcb355f606e`)
is ready, offline-verified, and its recovery backup is in place. No flash performed or proposed, no
merge to `main`, no branch/worktree deletion. Flashing remains gated on the robot being powered again
and an explicit operator `YES` to the Section 6 authorization question.
