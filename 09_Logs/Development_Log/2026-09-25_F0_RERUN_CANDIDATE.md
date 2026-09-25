# F0 (re-run) — Final flash readiness gate, against the integrated candidate

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1` @ `0bdc6269390db503da8d4711303d7e5e9b1a8f5b`

Re-evaluated after the new integrated freeze (superseding the earlier F0 re-run against the I9
freeze). **No flash is attempted or proposed in this document.**

## Gate-by-gate evaluation

| Gate | Result | Basis |
|---|---|---|
| `SOFTWARE_FREEZE_GATE` | **PASS** | [`2026-09-25_INTEGRATED_FREEZE_CANDIDATE.md`](2026-09-25_INTEGRATED_FREEZE_CANDIDATE.md): 14 C++ suites / 5726 checks PASS; static audit PASS (89 files, 0 findings); Python suites 366/370 with the one known/justified exclusion; clean tree throughout |
| `ARTIFACT_PROVENANCE_GATE` | **PASS** | Exact `ROBOT_POWERED` provenance recorded: `SOURCE_HEAD=c8906378df04468d886d6c1d064f67f74a16042b`, `BUILD_ID=c8906378df04`, `SOURCE_STATE=CLEAN`, `APPLICATION_SIZE=982432`, `APPLICATION_SHA256=94ae5c4a5152d914520db579d0282f0df5b540a56b90e9a772e67954e244b6b0` |
| `FAIL_CLOSED_GATE` | **PASS** | `hardware_motion_authorized=0`, `USB_ONLY` source default, OTA ingest `=0` — verified directly against source; `ActuatorRuntime`/`CalibrationExecutionEngine` now Controller-owned but wired with a `nullptr` backend and no geometry/limit/transform ever admitted, audit-enforced and mutation-verified this session |
| `POWER_ISOLATION_GATE` | **UNPROVEN** | Unchanged since I0 — no durable evidence ties today's (absent) connection to a proven controller-only power path |
| `FLASH_RECOVERY_GATE` | **DEFERRED_UNPOWERED** | The device is not enumerated, so no fresh read-back was attempted. The 2026-09-10 historical backup was re-verified byte-for-byte again this session (16,777,216 bytes, SHA256 `5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32`), but it predates the currently-installed application and is not treated as an exact backup of the present device state |

## `FINAL_FLASH_ELIGIBLE`

```text
FINAL_FLASH_ELIGIBLE = DEFERRED_UNPOWERED
```

The ESP32 was not enumerated at any point in this session (re-checked immediately before this
gate: `lsusb`, `/dev/ttyACM*`, `/dev/serial/by-id/` all empty). This classification is controlling
regardless of the three `PASS` gates above, per the handoff's explicit rule. **No authorization
question is asked** — Section 19 of the handoff applies only when `FINAL_FLASH_ELIGIBLE=PASS`.

## The candidate artifact

**MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE** — already built and verified in the
integrated freeze; not rebuilt here. Reproduce byte-for-byte from this exact commit:

```bash
cd 05_Firmware/MATDOG_Controller
MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh
# then: bash scripts/build.sh   (restores the USB_ONLY default artifact)
```

Expected: `build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin`, 982,432 bytes, SHA256
`94ae5c4a5152d914520db579d0282f0df5b540a56b90e9a772e67954e244b6b0`, from source commit
`c8906378df04468d886d6c1d064f67f74a16042b` (`SOURCE_STATE=CLEAN`). Not flashed or uploaded
anywhere; exists only in this local, gitignored `build/` directory.

## Exact future flash procedure

Unchanged from the earlier F0 (same seven steps: prove power isolation via the validated external
service port → re-check `USB_STATE` → fresh full-flash read-back only once isolation is proven →
archive the currently-installed application's identity → re-run F0 → if all gates PASS, produce the
Section 19 report and ask the single `AUTHORIZE FINAL APPLICATION-ONLY FLASH: YES/NO` question →
only on explicit `YES`, flash via `MATDOG_FLASH_PROFILE=ROBOT_POWERED scripts/flash_app_only.sh`) —
see [`2026-09-25_F0_FINAL_FLASH_READINESS.md`](2026-09-25_F0_FINAL_FLASH_READINESS.md) for the full
seven-step procedure, which now targets this candidate's exact SHA256 instead of the superseded I9
artifact.

## Outcome

`F0 = DEFERRED_UNPOWERED`. No flash performed or proposed, no merge to `main`, no branch/worktree
deletion. The MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE and the exact future
procedure are ready; nothing further is actionable in this session without the operator's physical
presence and explicit per-step authorization.
