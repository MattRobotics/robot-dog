# Integrated software freeze V2 — MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1` @ `e3dd4e36e327c69d2a115c31a9831743f55f5359`

Supersedes [`2026-09-25_INTEGRATED_FREEZE_CANDIDATE.md`](2026-09-25_INTEGRATED_FREEZE_CANDIDATE.md)
(commit `c8906378`, F0 re-run recorded in
[`2026-09-25_F0_RERUN_CANDIDATE.md`](2026-09-25_F0_RERUN_CANDIDATE.md)). Per the operator's "FINAL
PRE-FLASH CONSOLIDATION" and "FINAL I7/I8 HARDENING" instructions: I7 (OTA network transport) and I8
(read-only Web status) are now genuinely implemented and offline-tested rather than deferred, and
six concrete hardening findings from operator review are addressed. **No flash is attempted or
proposed in this document.**

## What changed since the V1 freeze (`c8906378`)

Two commits, both already pushed:

- **`4048ce6`** — I7/I8 implementation. `src/network/HttpTransport.*` (an `esp_http_server`
  adapter — the CONTROL/AUTHORIZATION plane in front of the one existing
  `OtaManager`/`OtaPolicy`/`OtaEspBackend` writer, never a second writer, never ArduinoOTA);
  `src/update/OtaSession.*`/`src/update/Hmac256.*` (a pure, host-linkable HMAC-SHA256
  challenge/response authentication/session layer, RFC 4231-verified); `src/config/OtaCredentials.*`
  (the OTA shared secret, mirroring `WifiCredentials.h`'s exact discipline); new `@WEB SERVER
  START|STOP|STATUS` (MAINTENANCE-gated); new `check_http_transport_boundaries()` static-audit
  gate. `GET /status` serializes the full I8 field list (system/source identity, module health,
  Wi-Fi state, BMS cached telemetry, IMU summary, LED presentation, actuator/calibration readiness,
  OTA provenance) — strictly read-only. Full record:
  [`2026-09-25_I7_I8_NETWORK_TRANSPORT_IMPLEMENTATION.md`](2026-09-25_I7_I8_NETWORK_TRANSPORT_IMPLEMENTATION.md).
- **`e3dd4e3`** — I7/I8 hardening, six findings from operator review: (1) a new, independent
  `OTA_INGEST_ENABLED` build-manifest authorization axis (`MATDOG_OTA_INGEST_VALIDATION=1` /
  `MATDOG_FLASH_OTA_INGEST=1`), so the ONE hardware-validation candidate can carry a real OTA writer
  without weakening the `0` source default; (2) a mailbox stale-response fix (new `slot_free_`
  semaphore + a pure, host-tested `HttpMailbox` decision core) closing a real cross-request
  response-confusion bug; (3) a start/stop resource-lifecycle fix (`stop()` now releases every
  semaphore `start()` creates; previously leaked two handles per cycle); (4) an OTA
  challenge-lifecycle hardening (`issueChallenge()` now idempotent while a still-valid challenge is
  outstanding, closing a displacement-DoS availability gap); (5)/(6) the `/status` threat model and
  the OTA security model documented explicitly in `HttpTransport.h`. Full record:
  [`2026-09-25_I7_I8_HARDENING.md`](2026-09-25_I7_I8_HARDENING.md).

## Offline matrix results (re-run from this exact clean commit)

### C++ Controller host suites

**17 suites, 5804 checks, 0 failures** (+78 vs. the V1 freeze's 5726 — `test_hmac256` (11),
`test_ota_session` (40, up from the transport-implementation commit's 35 after the challenge-
lifecycle hardening added 5 more adversarial checks) and `test_http_mailbox` (27), all new):

| Suite | Checks |
|---|---:|
| Servo population / hardware profile | 313 |
| C018 profile / H0 preflight | 444 |
| DALY protocol / KEY probe | 422 |
| Wi-Fi runtime policy | 1141 |
| ActuatorAuthority | 751 |
| Safe Actuator write policy | 335 |
| Calibration bootstrap geometry | 388 |
| OTA-A (policy/boot-guard/sha256) | 561 |
| Calibration domain / LF V25 replay | 702 |
| CalibrationManager | 322 |
| Safe Actuator runtime adapter (I4) | 46 |
| Calibration Execution engine (I5) | 72 |
| HostLink readiness classifier (I6) | 31 |
| LED Status Manager (I2) | 198 |
| **HMAC-SHA256 (I7, new)** | **11** |
| **OTA transport session/auth (I7, new)** | **40** |
| **HTTP mailbox correlation (I7/I8, new)** | **27** |

### Build-manifest offline suite

`scripts/tests/test_build_manifest.py`: **76 tests, all passing** (+12 vs. V1's implicit baseline —
`TestOtaIngestAuthorization`, covering the new independent OTA-ingest authorization axis, including
that it is never inferred from `HARDWARE_PROFILE` and that a build failing both axes reports the
profile mismatch first). Run as part of `check_build_profile_provenance()`, below.

### Static safety audit

`python3 scripts/static_audit.py`: **PASS**, 100 source files, 0 findings. Includes, beyond the V1
baseline set: `check_http_transport_boundaries()` (new, I7/I8) — host-linkability of
`Hmac256`/`OtaSession`/`HttpMailbox`; the ESP-IDF HTTP server API confined to `HttpTransport.cpp`;
the server never started from `Controller::begin()`; `@WEB SERVER START` MAINTENANCE-gated inside
its own command branch; no self-reboot; the OTA secret's one-call-site discipline; matched
`xSemaphoreCreateBinary()`/`vSemaphoreDelete()` counts; `start()` calling `stop()` on every
partial-failure path — and the extended `check_build_profile_provenance()` (I7/I8 hardening) for the
new `OTA_INGEST_ENABLED` manifest axis. Every new check was manually mutation-tested this session
before being trusted:

- Injecting `http_transport_.start()` into `Controller::begin()` — caught.
- Injecting a second `httpd_start(` call site outside `HttpTransport.cpp` — caught.
- Deleting the `@WEB SERVER START` MAINTENANCE gate — caught only after the check itself was
  rewritten (its first draft used a fixed-width text-window heuristic that matched the *next*
  unrelated command's own gate instead of the deleted one; bounded to the branch itself, then
  re-verified).
- Removing one `vSemaphoreDelete()` call (simulating a leaked semaphore) — caught
  (`3 xSemaphoreCreateBinary() call(s) but 2 vSemaphoreDelete() call(s)`).
- Reverting `issueChallenge()` to unconditional rotation — caught by
  `test_repeated_challenge_calls_do_not_displace_a_still_valid_nonce`.
- A regex typo in the OTA-ingest equality-comparison check itself was caught by its own designed
  false-negative (it initially searched for a comparison pattern the actual code did not use),
  fixed, then re-verified to pass cleanly.

### Python kinematics/calibration suites

`python3 -m pytest calibration/tests kinematics/tests` (`06_Software/Matdog_Core/`): **370
collected, 366 passed, 4 failed** — identical to the V1 freeze, byte-for-byte the same failing test
names and root cause (the pre-existing, already-documented live-FK `calibration_status` YAML/loader
enum mismatch —
[`CALIBRATION_SOURCE_PRECEDENCE.md` §9 item 1](../../05_Firmware/MATDOG_Controller/CALIBRATION_SOURCE_PRECEDENCE.md#9-legacy-open-items-carried-forward-i1-2026-09-25)).
No Python source changed since the V1 freeze, so an identical result is the expected confirmation,
not a new finding.

### Builds

All three from this exact clean commit, `USB_ONLY` restored last as the resting build artifact:

| Profile | OTA ingest | Flash | RAM | Application SHA256 |
|---|---|---:|---:|---|
| `USB_ONLY` (source default) | `0` (source default) | 1,024,464 B (32%) | 56,668 B (17%) | `ffac738a4d60b70446b2801aa8882baf65443a382a6280f7b60c8db032c2d7c6` |
| `ROBOT_POWERED` | `0` (source default) | 1,025,024 B (32%) | 56,668 B (17%) | `51d0b184d33db50a878d460709868ef905d0bbb8fa9d49bf61249467c04fc449` |
| **`ROBOT_POWERED` + ingest override** | **`1` (explicit `MATDOG_OTA_INGEST_VALIDATION=1`)** | **1,027,344 B (32%)** | **56,668 B (17%)** | **`1ac9694901e464beb1f2e81089ff7f60f12beac69d5d4fb20ff10d3987e454f2`** |

The third row is the **MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE** — see below.

### Repository/remote identity

- `git diff --check`: clean.
- `git status`: clean working tree.
- Branch `feat/controller-nextgen-integration-v1` tracks
  `origin/feat/controller-nextgen-integration-v1`; local HEAD == remote HEAD ==
  `e3dd4e36e327c69d2a115c31a9831743f55f5359` (verified via `git fetch` immediately before this
  freeze).
- `main` unchanged at `2017277d8fb74a430f1f75593ace035dfa717d12`; H0 unchanged at `b95ea31`; all
  other frozen worktrees (`controller-calibration-bootstrap-v1`, `controller-calibration-manager-v1`,
  `controller-safe-actuator-layer-v1`, `controller-wifi-ota-v1`) unchanged.

### Flash/recovery evidence state

- Passive USB re-check: `lsusb`, `/dev/ttyACM*`, `/dev/serial/by-id/` all show no ESP32.
  `USB_STATE=ESP32_NOT_ENUMERATED`, unchanged all session.
- Historical full-flash backup re-verified byte-for-byte: 16,777,216 bytes, SHA256
  `5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32`.
  `POWER_ISOLATION_GATE=UNPROVEN`; `FLASH_RECOVERY_GATE=DEFERRED_UNPOWERED`.

## MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE — exact provenance

```text
SOURCE_HEAD         e3dd4e36e327c69d2a115c31a9831743f55f5359
BUILD_ID            e3dd4e36e327
SOURCE_STATE        CLEAN
HARDWARE_PROFILE    ROBOT_POWERED
OTA_INGEST_ENABLED  1   (explicit MATDOG_OTA_INGEST_VALIDATION=1 override — source default is 0)
FQBN                esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi
APPLICATION_SIZE    1027344
APPLICATION_SHA256  1ac9694901e464beb1f2e81089ff7f60f12beac69d5d4fb20ff10d3987e454f2
artifact path       05_Firmware/MATDOG_Controller/build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin
manifest fields     MATDOG_MANIFEST_VERSION=1, matdog_build_manifest.txt (regenerate via
                    MATDOG_OTA_INGEST_VALIDATION=1 MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh)
```

This is the artifact the operator's instruction names **the MATDOG NEXTGEN INTEGRATED HARDWARE
VALIDATION CANDIDATE**: the single build the later physical campaign is intended to validate,
superseding the V1 freeze's artifact (`c8906378`, which had `OTA_INGEST_ENABLED=0` and therefore
could never validate the OTA end-to-end path without a second flash). It has not been flashed or
uploaded anywhere; it exists only in this local, gitignored `build/` directory, and was not left as
the resting build artifact (`USB_ONLY` was rebuilt afterward — see below). Reproducible byte-for-byte
via `MATDOG_OTA_INGEST_VALIDATION=1 MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh` from this exact
commit; `flash_app_only.sh` will refuse to write it unless both `MATDOG_FLASH_PROFILE=ROBOT_POWERED`
**and** `MATDOG_FLASH_OTA_INGEST=1` are explicitly given, each independently authorized against the
manifest.

## Fail-closed proof (re-verified, now covering I7/I8's additions)

```text
MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED   #define ... 0   (CalibrationManager.h:30)
MATDOG_ACTIVE_HARDWARE_PROFILE (source default) USB_ONLY        (BuildConfig.h:30)
MATDOG_OTA_INGEST_ENABLED (source default)      0                (OtaManager.h:32)
```

- Every fail-closed property re-verified from the V1 freeze still holds unchanged: `ServoBus`'s
  only write is `safeOff()`; `ActuatorRuntime`/`CalibrationExecutionEngine` remain wired with a
  `nullptr` backend; no geometry/limit/transform is ever admitted from `Controller`; no command path
  calls `plan()`/`commit()`/`execute()`/`abort()` on any of the three (re-confirmed by direct grep
  against this exact commit: zero matches in `CommandRouter.cpp`/`ControllerService.h`/
  `Controller.cpp`).
- **New this freeze**: `http_transport_.start()` is never called from `Controller::begin()` —
  confirmed both by direct grep (only `.begin(...)` and `.update(now_ms)` appear in
  `Controller.cpp`) and by `check_http_transport_boundaries()`'s audit-enforced, mutation-verified
  check. The listening socket is reachable only via the MAINTENANCE-gated `@WEB SERVER START`
  command; the OTA ingest writer additionally requires `MATDOG_OTA_INGEST_ENABLED=1` at compile
  time — this candidate has it `1` (an explicit, loud, independently-authorized build override, not
  a change to the `0` source default) specifically so the pending hardware-validation campaign can
  exercise the real path, but ingest remains structurally unreachable until an operator with USB
  access issues `@WEB SERVER START` in MAINTENANCE mode **and** a caller then presents a valid
  HMAC-SHA256 signature over a fresh, server-issued challenge.
- Wi-Fi loss cannot destabilize Controller execution: `HttpTransport::update()` is a bounded,
  non-blocking per-tick poll (`xSemaphoreTake(request_ready_, 0)`), unchanged by this freeze's
  hardening, and every blocking wait on the httpd-task side carries the same 3000 ms bound it always
  did.

No reachable ordinary Torque ON, `GoalPosition`, or actuator motion exists anywhere in the frozen
tree. Global `SAFE_OFF` remains completely independent of every class introduced or wired this
session. `GET /status` remains strictly read-only — no actuator-write API of any kind, audit-enforced
by the pre-existing `check_no_network_to_servo_path` and `check_actuator_infrastructure_wired_fail_closed`
checks, both still holding with I7/I8's additions.

## Final classifications

Unchanged from the V1 freeze except where noted:

| Area | Classification |
|---|---|
| Controller V0.1 baseline | **PASS** |
| `ROBOT_POWERED` no-motion (G2/G3/G3.1) | **PASS** |
| DALY KEY / power baseline, external USB service port | **PASS** (data-path-only correction recorded in F0, see below) |
| Wi-Fi station runtime (W1) | **HARDWARE_TO_TEST** |
| OTA-A/OTA-B core | **HARDWARE_TO_TEST** |
| **OTA network transport + authentication (I7, new)** | **IMPLEMENTED_OFFLINE_TESTED** — compiled in, disabled at boot, ingest gated both by `MATDOG_OTA_INGEST_ENABLED` and by the MAINTENANCE `@WEB SERVER START` gate; **HARDWARE_TO_TEST** for the semaphore handoff under real concurrent load, Wi-Fi association timing, and end-to-end transfer |
| **Read-only Web status (I8, new)** | **IMPLEMENTED_OFFLINE_TESTED** — full field-list coverage, strictly read-only; **HARDWARE_TO_TEST** for the same reasons |
| ActuatorAuthority | **HARDWARE_TO_TEST** |
| Safe Actuator policy core + runtime adapter, Controller-wired | **HARDWARE_TO_TEST** — offline PASS, no production backend, fail-closed wiring audit-enforced |
| Calibration domain/Manager + Execution boundary, Controller-wired | **HARDWARE_TO_TEST** — offline PASS, `RESTORE` non-executing by construction |
| LED Status Manager (I2) | **HARDWARE_TO_TEST** |
| `@SYSTEM SOURCE_SIGNATURE` (I3) | **HARDWARE_TO_TEST** |
| HostLink telemetry layer + readiness classifier (I6) | **PASS** (offline) / **HARDWARE_TO_TEST** (the underlying Wi-Fi/OTA facts it reports) |
| Geometry Compiler V5 bundle | **PASS** (LF) / **HARDWARE_TO_TEST** (RF/RH/LH) |
| Formal recalibration (leg population) | **BLOCKED** |
| Live-FK / YAML `calibration_status` enum mismatch | **KNOWN_OPEN** |
| 8 unresolved conservative clearance bounds; RF/RH/LH hardware-oracle evidence | **KNOWN_OPEN** |
| HostLink *action* half (servo scan/census/preflight/read/safe_off, mode, DALY KEY, Wi-Fi, LED — unified command schema for a second transport) | **TO_DESIGN** |
| Persistence/promotion (`PROVISIONING` vs. separate transaction) | **TO_DESIGN** |
| Direction measurement (`MEASURED_CANDIDATE`/`ACCEPTED`) | **TO_IMPLEMENT** |
| Service/Provisioning/QC | **TO_DESIGN** |
| `SYSTEM_SELF_TEST`/`PROFILE_AUDIT`/consolidated servo health | **TO_IMPLEMENT** |
| Flash/recovery evidence (today's session) | **DEFERRED_UNPOWERED** |

## Outcome

Freeze **PASS**. This document and its exact provenance supersede the V1 freeze
(`2026-09-25_INTEGRATED_FREEZE_CANDIDATE.md`, commit `c8906378`) as the reference for the pending
physical validation campaign. No merge to `main`, no branch/worktree deletion, and no flash
performed — all explicitly deferred per the operator's instruction. Proceeding to a re-run of F0.
