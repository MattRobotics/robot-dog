# F0 — Final flash readiness gate

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1` @ `56eb1ef27e4f67e27b3acbed5456214c69805944`

Evaluated after `I9 = PASS`. **No flash is attempted or proposed in this document.**

## Gate-by-gate evaluation

| Gate | Result | Basis |
|---|---|---|
| `SOFTWARE_FREEZE_GATE` | **PASS** | I9: 13 C++ suites / 5695 checks PASS; static audit PASS (85 files, 0 findings); Python suites 366/370 with the one known/justified exclusion; clean tree throughout |
| `ARTIFACT_PROVENANCE_GATE` | **PASS** | Exact `ROBOT_POWERED` provenance recorded in I9: `SOURCE_HEAD=cc0940b0f62f242f0ab66c09ea24f7cb8ed2aa08`, `BUILD_ID=cc0940b0f62f`, `SOURCE_STATE=CLEAN`, `APPLICATION_SIZE=978896`, `APPLICATION_SHA256=a292b2166d5381f1a8f75c494f79753e8aae4a23ee875c42325fe10ecb35203b` |
| `FAIL_CLOSED_GATE` | **PASS** | `hardware_motion_authorized=0`, `USB_ONLY` source default, OTA ingest `=0` — verified directly against source and audit-enforced; neither I4's `ActuatorRuntime` nor I5's `CalibrationExecutionEngine` is referenced by `Controller`/`CommandRouter`, and both are proven dead-code-eliminated from both compiled profiles |
| `POWER_ISOLATION_GATE` | **UNPROVEN** | No durable evidence ties today's (absent) connection to a proven controller-only power path — see Section 2 of the handoff; unchanged since I0 |
| `FLASH_RECOVERY_GATE` | **DEFERRED_UNPOWERED** | A fresh full-flash read-back is only eligible once `POWER_ISOLATION_GATE=PASS`; the device is not enumerated, so none was attempted. The 2026-09-10 historical backup was re-verified byte-for-byte in I0 and again in I9 (16,777,216 bytes, SHA256 `5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32`), but per Section 3 of the handoff that backup predates the currently-installed application (`6322563`) and is not treated as an exact backup of the present device state |

## `FINAL_FLASH_ELIGIBLE`

```text
FINAL_FLASH_ELIGIBLE = DEFERRED_UNPOWERED
```

Per the handoff's explicit rule: the ESP32 was not enumerated at any point in this session
(`lsusb`, `/dev/ttyACM*`, `/dev/serial/by-id/` all empty, checked at I0 and re-checked at I9). This
classification is controlling regardless of the three `PASS` gates above — three `PASS` and two
`UNPROVEN`/`DEFERRED` gates cannot combine to `FINAL_FLASH_ELIGIBLE=PASS`, and the unpowered state
alone is sufficient to resolve this gate without needing to weigh the others.

**No authorization question is asked.** Section 19 of the handoff (the explicit `AUTHORIZE FINAL
APPLICATION-ONLY FLASH: YES/NO` prompt) applies only when `FINAL_FLASH_ELIGIBLE=PASS`. It is not.

## The frozen artifact

Already built and verified in I9; not rebuilt here. To reproduce it byte-for-byte from a clean
checkout of this exact commit:

```bash
cd 05_Firmware/MATDOG_Controller
MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh
# then: bash scripts/build.sh   (restores the USB_ONLY default artifact)
```

Expected output: `build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin`, 978,896 bytes, SHA256
`a292b2166d5381f1a8f75c494f79753e8aae4a23ee875c42325fe10ecb35203b`, from source commit
`cc0940b0f62f242f0ab66c09ea24f7cb8ed2aa08` (`SOURCE_STATE=CLEAN`). This artifact has not been
flashed or uploaded anywhere; it exists only in this local, gitignored `build/` directory.

## Exact future flash procedure (for when the operator is physically present)

Recorded so nothing has to be re-derived later. **None of the following is authorized by this
document** — each step still requires the explicit-authorization gate it is written under.

1. **Prove power isolation.** With the main fuse still removed, connect the ESP32 through the
   already-validated external service port (GPIO19 D-, GPIO20 D+, GND — no host VBUS) or another
   path with equivalent durable evidence that it powers the controller only, not the servo rail or
   the protected robot power domain. This alone reclassifies `POWER_ISOLATION_GATE`; it is a
   hardware step requiring the operator's physical presence and explicit authorization for that
   session, not a software action.
2. **Re-evaluate `USB_STATE`.** Passive check only (`lsusb`, `/dev/serial/by-id`) — confirm
   enumeration before touching `esptool` or any active serial session.
3. **Take a fresh full-flash read-back**, only once step 1 is durably proven — never merely because
   the device enumerates. Save under a new timestamped filename, distinct from
   `matdog_esp32s3_fullflash_2026-09-10.bin` (never overwrite it). Record exact size, SHA256, and a
   recovery manifest (device identity, port, chip, flash size, backup path/SHA256, the
   currently-installed application's identity, exact restore command, toolchain/`esptool` version).
   This is what resolves `FLASH_RECOVERY_GATE` to `PASS`.
4. **Reproduce or archive the currently-installed application's identity** (source commit
   `63225631624b646d391ff335ca2e1377bd098d1a`, recorded application SHA beginning `e9283ced...`) —
   per Section 3 of the handoff, using the pinned toolchain/FQBN, without modifying historical
   source.
5. **Re-run F0** with `POWER_ISOLATION_GATE` and `FLASH_RECOVERY_GATE` now resolvable. If every gate
   is `PASS`, produce the Section 19 report (final HEAD, exact artifact SHA256/size, backup/recovery
   status, current USB/device identity, proven power topology, proof of servo/robot power-domain
   isolation, exact flash method, exact regions written/not written, expected reset/reboot side
   effect, recovery command, residual risks) and ask the single question: **"AUTHORIZE FINAL
   APPLICATION-ONLY FLASH: YES/NO"** — no other question.
6. **Only on an explicit `YES`**, perform an application-only flash via the already-reviewed
   `MATDOG_FLASH_PROFILE=ROBOT_POWERED scripts/flash_app_only.sh` path (never a full erase, never a
   bootloader/partition-table/NVS rewrite, never a servo EEPROM/DALY write, never live servo
   commands, no Wi-Fi/OTA qualification, no q0/preflight/motion). A reboot/reset inherent to the
   approved flash is permitted.
7. **After the flash:** record the flashing tool's result and the written application identity;
   allow passive USB re-enumeration to be observed if it happens naturally; send no active runtime
   commands for validation. Correct post-flash status:
   `FIRMWARE_INSTALLATION = COMPLETED`, `HARDWARE_VALIDATION = PENDING_OPERATOR_PRESENT`. All H0
   preflight, Wi-Fi qualification, OTA end-to-end, q0, contact, stand, gait and stabilization
   validation remain for that later physical campaign.

## Outcome

`F0 = DEFERRED_UNPOWERED`. No flash performed or proposed. The frozen artifact and the exact future
procedure are ready; nothing further is actionable in this session without the operator's physical
presence and explicit per-step authorization.
