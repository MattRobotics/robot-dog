# Integrated software freeze — MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1` @ `c8906378df04468d886d6c1d064f67f74a16042b`

Supersedes the earlier I9 freeze (same date, commit `56eb1ef`). Per the operator's objective-change
instruction: produce ONE maximally integrated, fail-closed `ROBOT_POWERED` firmware candidate,
containing every responsibly implementable software subsystem (I6 HostLink, I4/I5 Controller
wiring reconsidered), before the first new flash — so the later physical campaign validates this
single candidate rather than requiring repeated intermediate flashes.

## What changed since the I9 freeze

Five gates advanced between the two freezes, all already individually committed and pushed:

- **I6 (implementation)** — `ControllerService` (transport-neutral telemetry layer) and
  `ServiceReadiness` (pure `BLOCKED`/`TO_TEST`/`READY` classifier), `CommandRouter` refactored to
  route read-only commands through the former, new `@HOSTLINK READINESS`.
- **I7 (reconsidered)** — no code change; documented why the OTA ingest transport is not built
  this session (`CommandRouter`'s 96-byte line buffer forces a real second I/O mode, not a thin
  adapter).
- **I4/I5 Controller wiring** — `Controller` now owns real `SafeActuatorPolicy`/`ActuatorRuntime`/
  `CalibrationExecutionEngine` instances as fail-closed status infrastructure (`nullptr` backend,
  no geometry/limit/transform ever admitted, no command reaches `plan`/`commit`/`execute`/`abort`),
  new `@ACTUATOR STATUS`, new `check_actuator_infrastructure_wired_fail_closed()` audit gate
  (mutation-verified — caught and fixed a real false-negative in its first draft).
- **I8 (reconsidered)** — no code change; documented why an HTTP server is not built this session
  (same underlying reason as I7: no reviewed transport decision, and its safety property is
  untestable without live Wi-Fi).

## Offline matrix results (re-run from this exact clean commit)

### C++ Controller host suites

**14 suites, 5726 checks, 0 failures** (+31 vs. the I9 freeze — the new `ServiceReadiness` suite;
every other suite's count is unchanged, confirming no regression):

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
| **HostLink readiness classifier (I6, new)** | **31** |
| LED Status Manager (I2) | 198 |

### Static safety audit

`python3 scripts/static_audit.py`: **PASS**, 89 source files, 0 findings. Includes, beyond the I9
baseline set: `check_service_readiness_is_host_linkable()` (new, I6) and
`check_actuator_infrastructure_wired_fail_closed()` (new, I4/I5 wiring) — the latter was manually
mutation-tested twice this session (nullptr-backend removal, and a `->plan(` call injection into
`CommandRouter.cpp`) and correctly failed both times before being trusted; both mutations were
reverted and the tree re-verified clean.

### Python kinematics/calibration suites

`06_Software/Matdog_Core`: **370 collected, 366 passed, 4 failed** — identical result to the I9
freeze, byte-for-byte the same failing test names and root cause (the pre-existing, already-
documented live-FK `calibration_status` YAML/loader enum mismatch —
[`CALIBRATION_SOURCE_PRECEDENCE.md` §9 item 1](../../05_Firmware/MATDOG_Controller/CALIBRATION_SOURCE_PRECEDENCE.md#9-legacy-open-items-carried-forward-i1-2026-09-25)).
No Python source changed between the two freezes, so an identical result is the expected
confirmation, not a new finding.

### Builds

Both from this exact clean commit, default `USB_ONLY` restored last:

| Profile | Flash | RAM | Application SHA256 |
|---|---:|---:|---|
| `USB_ONLY` (source default) | 981,856 B (31%) | 53,356 B (16%) | `1cc4647e19dace68885a6a6857a8dd410288e6c76268033fe407a55a9c2c5241` |
| `ROBOT_POWERED` | 982,432 B (31%) | 53,356 B (16%) | `94ae5c4a5152d914520db579d0282f0df5b540a56b90e9a772e67954e244b6b0` |

### Repository/remote identity

- `git diff --check`: clean.
- `git status`: clean working tree.
- Branch `feat/controller-nextgen-integration-v1` tracks
  `origin/feat/controller-nextgen-integration-v1`; local HEAD == remote HEAD ==
  `c8906378df04468d886d6c1d064f67f74a16042b` (verified immediately before this freeze).
- `main` unchanged at `2017277`; H0 unchanged at `b95ea31`; all other frozen worktrees unchanged.

### Flash/recovery evidence state

- Passive USB re-check: `lsusb`, `/dev/ttyACM*`, `/dev/serial/by-id/` all show no ESP32.
  `USB_STATE=ESP32_NOT_ENUMERATED`, unchanged all session.
- Historical full-flash backup re-verified byte-for-byte: 16,777,216 bytes, SHA256
  `5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32`.
  `POWER_ISOLATION_GATE=UNPROVEN`; `FLASH_RECOVERY_GATE=DEFERRED_UNPOWERED`.

## MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE — exact provenance

```text
SOURCE_HEAD        c8906378df04468d886d6c1d064f67f74a16042b
BUILD_ID           c8906378df04
SOURCE_STATE       CLEAN
HARDWARE_PROFILE   ROBOT_POWERED
FQBN               esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi
APPLICATION_SIZE   982432
APPLICATION_SHA256 94ae5c4a5152d914520db579d0282f0df5b540a56b90e9a772e67954e244b6b0
artifact path      05_Firmware/MATDOG_Controller/build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin
```

This is the artifact the operator's instruction names **the MATDOG NEXTGEN INTEGRATED HARDWARE
VALIDATION CANDIDATE**: the single build the later physical campaign is intended to validate,
superseding the I9 freeze's artifact as the reference candidate. It has not been flashed or
uploaded anywhere; it exists only in this local, gitignored `build/` directory. Reproducible
byte-for-byte via `MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh` from this exact commit.

## Fail-closed proof (re-verified, now covering I4/I5/I6 additions)

```text
MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED   #define ... 0   (CalibrationManager.h:30)
MATDOG_ACTIVE_HARDWARE_PROFILE (source default) USB_ONLY        (BuildConfig.h:30)
MATDOG_OTA_INGEST_ENABLED (source default)      0                (OtaManager.h:32)
```

- `ServoBus`'s only write is `safeOff()` — unchanged all session.
- `ActuatorRuntime` (I4) is now owned by `Controller`, but wired with a `nullptr` backend: every
  `ACCEPT` it could ever reach resolves to `NO_BACKEND`. No production `ActuatorBackend` exists
  anywhere in this firmware.
- `CalibrationExecutionEngine` (I5) is now owned by `Controller`; `RESTORE`/`ABORT` remain
  categorically non-executing (never construct a command, never touch the policy) — authority loss
  still produces zero restore motion, by construction, unchanged from I5.
- No geometry is bound, no limit or transform is ever admitted, from `Controller` — audit-enforced
  (`check_actuator_infrastructure_wired_fail_closed`, mutation-verified this session).
- No command path (`CommandRouter`, `ControllerService`, or `Controller` itself) calls
  `plan()`/`commit()`/`execute()`/`abort()` on any of the three — audit-enforced with a regex that
  was specifically fixed this session after a manual mutation test proved the first version missed
  pointer-call syntax.
- `ControllerService`/`ServiceReadiness` (I6) add no write path: every accessor is read-only by
  construction (verified by inspection — each method body is a single forwarding return), and
  `ServiceReadiness` is a pure classifier with no module pointer at all.

No reachable ordinary Torque ON, `GoalPosition`, or actuator motion exists anywhere in the frozen
tree. Global `SAFE_OFF` remains completely independent of every class introduced or wired this
session.

## Final classifications

Unchanged from the I9 freeze except where noted:

| Area | Classification |
|---|---|
| Controller V0.1 baseline | **PASS** |
| `ROBOT_POWERED` no-motion (G2/G3/G3.1) | **PASS** |
| DALY KEY / power baseline, external USB service port | **PASS** |
| Wi-Fi station runtime (W1) | **HARDWARE_TO_TEST** |
| OTA-A/OTA-B core | **HARDWARE_TO_TEST** |
| ActuatorAuthority | **HARDWARE_TO_TEST** |
| Safe Actuator policy core + runtime adapter, now Controller-wired | **HARDWARE_TO_TEST** — offline PASS, no production backend, fail-closed wiring audit-enforced |
| Calibration domain/Manager + Execution boundary, now Controller-wired | **HARDWARE_TO_TEST** — offline PASS, `RESTORE` non-executing by construction |
| LED Status Manager (I2) | **HARDWARE_TO_TEST** |
| `@SYSTEM SOURCE_SIGNATURE` (I3) | **HARDWARE_TO_TEST** |
| **HostLink telemetry layer + readiness classifier (I6, new)** | **PASS** (offline) / **HARDWARE_TO_TEST** (the underlying Wi-Fi/OTA facts it reports) |
| Geometry Compiler V5 bundle | **PASS** (LF) / **HARDWARE_TO_TEST** (RF/RH/LH) |
| Formal recalibration (leg population) | **BLOCKED** |
| Live-FK / YAML `calibration_status` enum mismatch | **KNOWN_OPEN** |
| 8 unresolved conservative clearance bounds; RF/RH/LH hardware-oracle evidence | **KNOWN_OPEN** |
| HostLink *action* half (servo scan/census/preflight/read/safe_off, mode, DALY KEY, Wi-Fi, LED — unified command schema for a second transport) | **TO_DESIGN** |
| Wi-Fi/OTA authenticated transport; OTA ingest transport (needs a second Serial I/O mode) | **TO_DESIGN** |
| Read-only Web foundation (server half) | **BLOCKED** — same undecided-transport reason as OTA; data/schema half done via I6 |
| Persistence/promotion (`PROVISIONING` vs. separate transaction) | **TO_DESIGN** |
| Direction measurement (`MEASURED_CANDIDATE`/`ACCEPTED`) | **TO_IMPLEMENT** |
| Service/Provisioning/QC | **TO_DESIGN** |
| `SYSTEM_SELF_TEST`/`PROFILE_AUDIT`/consolidated servo health | **TO_IMPLEMENT** |
| Flash/recovery evidence (today's session) | **DEFERRED_UNPOWERED** |

## Outcome

Freeze **PASS**. This document and its exact provenance supersede the I9 freeze as the reference
for the pending physical validation campaign. No merge to `main`, no branch/worktree deletion, and
no flash performed — all explicitly deferred per the operator's instruction. Proceeding to a
re-run of F0.
