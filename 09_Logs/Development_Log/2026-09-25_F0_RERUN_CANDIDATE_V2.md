# F0 (re-run V2) — Final flash readiness gate, against the I7/I8-hardened candidate

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1` @ `e3dd4e36e327c69d2a115c31a9831743f55f5359`

Re-evaluated after the new integrated freeze
([`2026-09-25_INTEGRATED_FREEZE_CANDIDATE_V2.md`](2026-09-25_INTEGRATED_FREEZE_CANDIDATE_V2.md),
superseding the earlier F0 re-run against the `c8906378` freeze,
[`2026-09-25_F0_RERUN_CANDIDATE.md`](2026-09-25_F0_RERUN_CANDIDATE.md)). **No flash is attempted or
proposed in this document.**

## Gate-by-gate evaluation

| Gate | Result | Basis |
|---|---|---|
| `SOFTWARE_FREEZE_GATE` | **PASS** | [`2026-09-25_INTEGRATED_FREEZE_CANDIDATE_V2.md`](2026-09-25_INTEGRATED_FREEZE_CANDIDATE_V2.md): 17 C++ suites / 5804 checks PASS; `test_build_manifest.py` 76/76 PASS; static audit PASS (100 files, 0 findings); Python suites 370/370 collected, 366/370 passed with the one known/justified exclusion; clean tree throughout |
| `ARTIFACT_PROVENANCE_GATE` | **PASS** | Exact **MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE** provenance recorded in the V2 freeze: `SOURCE_HEAD=e3dd4e36e327c69d2a115c31a9831743f55f5359`, `BUILD_ID=e3dd4e36e327`, `SOURCE_STATE=CLEAN`, `HARDWARE_PROFILE=ROBOT_POWERED`, `OTA_INGEST_ENABLED=1` (explicit override, independently authorized in the manifest, source default unchanged at `0`), `APPLICATION_SIZE=1027344`, `APPLICATION_SHA256=1ac9694901e464beb1f2e81089ff7f60f12beac69d5d4fb20ff10d3987e454f2` |
| `FAIL_CLOSED_GATE` | **PASS** | `hardware_motion_authorized=0`, `USB_ONLY`/`OTA_INGEST_ENABLED=0` both still the *source* defaults (verified directly against source), `HttpTransport::start()` never called from `Controller::begin()` (verified directly against source and audit-enforced, mutation-verified this session) — the candidate's `OTA_INGEST_ENABLED=1` is a build-time override compiled into this one artifact for the pending hardware-validation campaign, not a source-default change, and ingest stays structurally unreachable behind the MAINTENANCE-gated `@WEB SERVER START` command plus HMAC-SHA256 authentication regardless |
| `POWER_ISOLATION_GATE` | **UNPROVEN** | Unchanged since I0 — no durable evidence ties today's (absent) connection to a proven controller-only power path. **Correction (2026-09-25):** the validated external service connector (GPIO19/20, §15 of `MATDOG_POWER_STATES_AND_CHARGING.md`) is data-path-only (no VBUS) and cannot resolve this gate by itself — see the corrected procedure in [`2026-09-25_F0_FINAL_FLASH_READINESS.md`](2026-09-25_F0_FINAL_FLASH_READINESS.md) |
| `FLASH_RECOVERY_GATE` | **DEFERRED_UNPOWERED** | The device is not enumerated, so no fresh read-back was attempted. The 2026-09-10 historical backup was re-verified byte-for-byte again this session (16,777,216 bytes, SHA256 `5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32`), but it predates the currently-installed application and is not treated as an exact backup of the present device state |

## `FINAL_FLASH_ELIGIBLE`

```text
FINAL_FLASH_ELIGIBLE = DEFERRED_UNPOWERED
```

The ESP32 was not enumerated at any point in this session (re-checked immediately before this gate:
`lsusb`, `/dev/ttyACM*`, `/dev/serial/by-id/` all empty). This classification is controlling
regardless of the three `PASS` gates above, per the handoff's explicit rule. **No authorization
question is asked** — Section 19 of the handoff applies only when `FINAL_FLASH_ELIGIBLE=PASS`.

## The candidate artifact

**MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE** — already built and verified in the V2
integrated freeze; not rebuilt here. Reproduce byte-for-byte from this exact commit:

```bash
cd 05_Firmware/MATDOG_Controller
MATDOG_OTA_INGEST_VALIDATION=1 MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh
# then: bash scripts/build.sh   (restores the USB_ONLY default artifact)
```

Expected: `build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin`, 1,027,344 bytes, SHA256
`1ac9694901e464beb1f2e81089ff7f60f12beac69d5d4fb20ff10d3987e454f2`, from source commit
`e3dd4e36e327c69d2a115c31a9831743f55f5359` (`SOURCE_STATE=CLEAN`, `OTA_INGEST_ENABLED=1`). Not
flashed or uploaded anywhere; exists only in this local, gitignored `build/` directory (the resting
artifact was rebuilt back to the ordinary `USB_ONLY` default after recording this one's hash).

Unlike the V1 candidate (`c8906378`, `OTA_INGEST_ENABLED=0`), this one carries a real, reachable
(once authorized) OTA firmware-ingest writer, so the SAME flashed image the pending physical
campaign validates boot/IMU/BMS/LED/Wi-Fi/HostLink/Web against can *also* validate the OTA
end-to-end path — no second flash needed purely to turn ingest on.

## Exact future flash procedure

Same seven-step procedure as before, now targeting this candidate's exact SHA256 and carrying two
independent, explicit authorization inputs instead of one — see
[`2026-09-25_F0_FINAL_FLASH_READINESS.md`](2026-09-25_F0_FINAL_FLASH_READINESS.md) for the full
corrected procedure (step 1: the external service connector is data-path-only and cannot itself
prove power isolation — a genuinely separate, independently-proven power path is required). The
flash step itself now requires:

```bash
MATDOG_FLASH_PROFILE=ROBOT_POWERED MATDOG_FLASH_OTA_INGEST=1 scripts/flash_app_only.sh
```

Both `MATDOG_FLASH_PROFILE` and `MATDOG_FLASH_OTA_INGEST` are checked independently against the
build manifest — an operator who states only one of the two still has the flash refused
(`PROFILE_MISMATCH` or `OTA_INGEST_MISMATCH`, whichever is upstream), never silently defaulted.

## Outcome

`F0 = DEFERRED_UNPOWERED`. No flash performed or proposed, no merge to `main`, no branch/worktree
deletion. The MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE (now carrying a hardware-
validatable OTA ingest path) and the exact future procedure are ready; nothing further is actionable
in this session without the operator's physical presence and explicit per-step authorization.
