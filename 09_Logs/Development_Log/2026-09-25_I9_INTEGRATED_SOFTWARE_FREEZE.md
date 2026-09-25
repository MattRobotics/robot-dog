# I9 — Integrated software freeze

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

Closes gate **I9** (V3 handoff §17): the complete offline validation matrix, run from a clean
committed tree, ending in a clean pushed commit.

## Offline matrix results

### C++ Controller host suites

All 13 suites linked against the real shipped translation units (no mocks), run via
`scripts/tests/run_host_tests.sh`:

| Suite | Checks | Result |
|---|---:|---|
| Servo population / hardware profile | 313 | PASS |
| C018 profile / H0 preflight | 444 | PASS |
| DALY protocol / KEY probe | 422 | PASS |
| Wi-Fi runtime policy | 1141 | PASS |
| ActuatorAuthority | 751 | PASS |
| Safe Actuator write policy | 335 | PASS |
| Calibration bootstrap geometry | 388 | PASS |
| OTA-A (policy/boot-guard/sha256) | 561 | PASS |
| Calibration domain / LF V25 replay | 702 | PASS |
| CalibrationManager | 322 | PASS |
| Safe Actuator runtime adapter (I4) | 46 | PASS |
| Calibration Execution engine (I5) | 72 | PASS |
| LED Status Manager (I2) | 198 | PASS |
| **Total** | **5695** | **PASS, 0 failures** |

### Static safety audit (includes mutation guards, OTA partition tests, build-manifest tests)

`python3 scripts/static_audit.py`: **PASS**, 85 source files scanned, 0 findings. This single
command chains — and this run re-confirmed green — every boundary check accumulated across I0–I5,
including the DALY write prohibition, torque-enable/servo-ID-write prohibitions, USB CDC
non-blocking guarantees, `USB_ONLY` source-default and anti-back-power gates, the G2
transport-independence check, `ActuatorAuthority`/Safe Actuator/OTA boundary checks, the
`hardware_motion_authorized` source-default gate, the DALY audit mutation suite (52/52), the Safe
Actuator audit mutation suite, the OTA partition-logic suite (40/40), the build-manifest suite
(51/51), and this session's three new boundary checks (`check_led_status_boundaries`,
`check_actuator_runtime_boundaries`, `check_calibration_execution_engine_boundaries`).

### Python kinematics/calibration suites

`python3 -m pytest calibration/tests kinematics/tests` (`06_Software/Matdog_Core/`): **370
collected, 366 passed, 4 failed.**

**The 4 failures are a known, justified, pre-existing exclusion — not a regression introduced by
this integration.** All four are in `kinematics/tests/test_matdog_leg_fk_live.py`, all with the
identical root cause:

```text
RuntimeError: calibration_status inatteso: 'DIGITAL_ZERO_CALIBRATED_AND_VERIFIED';
              atteso 'VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION'
```

`kinematics/matdog_leg_fk_live.py`'s loader hard-asserts the YAML's `calibration_status` enum
equals `VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION`. `MATDOG_JOINT_CALIBRATION.yaml` deliberately
keeps the enum at `DIGITAL_ZERO_CALIBRATED_AND_VERIFIED` — the file's own header comment states this
is intentional: *"the enum value is deliberately UNCHANGED because six tools hard-assert it...
Changing the enum requires updating those six consumers in the same change."* This is the exact
legacy item identified in this session's `I1` audit and carried forward, with an explicit
not-yet-resolved status, into
[`CALIBRATION_SOURCE_PRECEDENCE.md` §9, item 1](../../05_Firmware/MATDOG_Controller/CALIBRATION_SOURCE_PRECEDENCE.md#9-legacy-open-items-carried-forward-i1-2026-09-25).
The V3 handoff's own §7 instruction is explicit: *"Do NOT fix this by blindly changing either
string."* Fixing it requires a coordinated change across all six consumers, which is out of scope
for this integration session and has not been attempted. No test was deleted, skipped, or modified
to hide this — it is reported here exactly as it ran.

No other exclusions were needed: the remaining 366 tests, including the computationally heavy
Geometry Compiler V5 mesh-kernel/contact-search/process-worker suites, all passed unmodified.

### Builds

Both from the same clean commit (`cc0940b0f62f242f0ab66c09ea24f7cb8ed2aa08`), default `USB_ONLY`
restored last:

| Profile | Flash | RAM | Application SHA256 |
|---|---:|---:|---|
| `USB_ONLY` (source default) | 978,336 B (31%) | 52,420 B (15%) | `585b8b3bac680798fd438be6eae1b0b4650cdecb7920ecf13716a178555b0561` |
| `ROBOT_POWERED` | 978,896 B (31%) | 52,420 B (15%) | `a292b2166d5381f1a8f75c494f79753e8aae4a23ee875c42325fe10ecb35203b` |

### Repository/remote identity

- `git diff --check`: clean, no whitespace errors.
- `git status`: clean working tree throughout this gate.
- Branch `feat/controller-nextgen-integration-v1` tracks `origin/feat/controller-nextgen-integration-v1`; local HEAD == remote HEAD == `cc0940b0f62f242f0ab66c09ea24f7cb8ed2aa08` (verified immediately before this freeze).
- `main` worktree unchanged at `2017277`; H0 worktree unchanged at `b95ea31`; all other frozen worktrees unchanged from their I0 state.

### Flash/recovery evidence state

- Passive USB re-check (read-only, no hardware modification): `lsusb`, `/dev/ttyACM*`,
  `/dev/serial/by-id/` all show no ESP32 present. `USB_STATE=ESP32_NOT_ENUMERATED`, unchanged since
  I0.
- Historical full-flash backup (`~/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-10.bin`)
  re-verified byte-for-byte: 16,777,216 bytes, SHA256
  `5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32` — unchanged.
  `POWER_ISOLATION_GATE=UNPROVEN` (moot while unpowered); `FLASH_RECOVERY_GATE=DEFERRED_UNPOWERED`.
  No fresh device read-back was attempted, consistent with Section 3 of the handoff.

## Final `ROBOT_POWERED` artifact provenance

```text
SOURCE_HEAD        cc0940b0f62f242f0ab66c09ea24f7cb8ed2aa08
BUILD_ID           cc0940b0f62f
SOURCE_STATE       CLEAN
HARDWARE_PROFILE   ROBOT_POWERED
FQBN               esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi
APPLICATION_SIZE   978896
APPLICATION_SHA256 a292b2166d5381f1a8f75c494f79753e8aae4a23ee875c42325fe10ecb35203b
artifact path      05_Firmware/MATDOG_Controller/build/esp32.esp32.esp32s3/MATDOG_Controller.ino.bin
```

This exact artifact was not flashed or uploaded anywhere; it exists only in the local (gitignored)
`build/` directory from this session's compile-only verification.

## Fail-closed proof

Verified directly against source (not merely cited from prior documentation) immediately before
this freeze, and re-confirmed by the static-audit PASS above, which enforces each as a build-failing
gate:

```text
MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED   #define ... 0   (CalibrationManager.h:30)
MATDOG_ACTIVE_HARDWARE_PROFILE (source default) USB_ONLY        (BuildConfig.h:30)
MATDOG_OTA_INGEST_ENABLED (source default)      0                (OtaManager.h:32)
```

No reachable ordinary Torque ON, `GoalPosition`, or actuator motion exists anywhere in the frozen
tree:

- `ServoBus`'s only write is `safeOff()` (`TorqueEnable = 0`) — unchanged all session.
- `check_torque_enable`/`check_servo_id_write` (unchanged, pre-existing) forbid any non-zero
  `EnableTorque` argument and any raw `writeByte`/`writeWord` call anywhere in the tree.
- The Safe Actuator runtime adapter introduced this session (I4, `ActuatorRuntime`) has **no
  production `ActuatorBackend`** anywhere in the firmware, is referenced by nothing in
  `Controller.cpp`/`CommandRouter.cpp`, and is proven dead-code-eliminated from both compiled
  profiles (byte-identical flash size with and without it) — enforced structurally by
  `check_actuator_runtime_boundaries()`.
- The Calibration Execution boundary introduced this session (I5, `CalibrationExecutionEngine`) is
  likewise unreferenced by `Controller`/`CommandRouter`, never computes a raw target from geometry,
  and is proven dead-code-eliminated identically — enforced structurally by
  `check_calibration_execution_engine_boundaries()`.
- Both new adapters therefore remain unable to make physical motion reachable in this frozen
  configuration, independent of whether their own internal decision logic would have refused a
  given command (it would — see the I4/I5 test suites) — the *stronger* property is that neither is
  wired into any reachable command path at all.

## Final classifications

| Area | Classification |
|---|---|
| Controller V0.1 baseline | **PASS** — validated `USB_ONLY`, frozen tag |
| `ROBOT_POWERED` no-motion (G2/G3/G3.1) | **PASS** — hardware-validated, no-motion scope |
| DALY KEY / power baseline | **PASS** — hardware-validated |
| External USB service port | **PASS** — hardware-validated |
| Wi-Fi station runtime (W1) | **HARDWARE_TO_TEST** — offline PASS, never associated with an AP |
| OTA-A/OTA-B core | **HARDWARE_TO_TEST** — offline PASS, never exercised on device |
| ActuatorAuthority | **HARDWARE_TO_TEST** — offline PASS |
| Safe Actuator policy core | **HARDWARE_TO_TEST** — offline PASS, no write path reachable |
| Safe Actuator runtime adapter (I4) | **HARDWARE_TO_TEST** — offline PASS, no production backend, dead-code-eliminated |
| LED Status Manager (I2) | **HARDWARE_TO_TEST** — offline PASS, visual/color review still pending |
| `@SYSTEM SOURCE_SIGNATURE` (I3) | **HARDWARE_TO_TEST** — offline-verifiable logic, output never printed on real hardware |
| Calibration domain / CalibrationManager | **HARDWARE_TO_TEST** — offline PASS, LF V25 replay matched |
| Calibration Execution boundary (I5) | **HARDWARE_TO_TEST** — offline PASS, no production backend, dead-code-eliminated |
| Geometry Compiler V5 bundle | **PASS** (LF hardware-validated) / **HARDWARE_TO_TEST** (RF/RH/LH) |
| Formal recalibration (leg population) | **BLOCKED** — last formal result 6/12, historical, not superseded |
| Live-FK / YAML `calibration_status` enum mismatch | **KNOWN_OPEN** — documented I1/I9, not fixed, 4 Python test failures explained by it |
| 8 unresolved conservative clearance bounds | **KNOWN_OPEN** — `CALIBRATION_BOOTSTRAP.md` §7, unchanged |
| RF/RH/LH hardware-oracle evidence | **KNOWN_OPEN** — no equivalent to LF V25 exists |
| HostLink semantic layer | **TO_DESIGN** — audited (I6), not implemented, correctly sequenced behind unmet prerequisites |
| Wi-Fi/OTA authenticated transport | **TO_DESIGN** — audited (I7), transport evaluated but not chosen |
| Read-only Web foundation | **BLOCKED** — I8 entry conditions unmet (I6 not stable, Wi-Fi hardware untested, no transport) |
| Persistence/promotion (`PROVISIONING` vs. separate transaction) | **TO_DESIGN** — explicitly not decided by I5, per instruction |
| Direction measurement (`MEASURED_CANDIDATE`/`ACCEPTED`) | **TO_IMPLEMENT** — no historical mechanism exists to recover it |
| Service/Provisioning/QC | **TO_DESIGN** — frozen bench oracles only, unchanged |
| `SYSTEM_SELF_TEST`/`PROFILE_AUDIT`/consolidated servo health | **TO_IMPLEMENT** — open design questions, not attempted (I3) |
| Flash/recovery evidence (today's session) | **DEFERRED_UNPOWERED** — device not enumerated; historical backup re-verified |

## Outcome

`I9 = PASS`. The offline matrix is green with one known, pre-existing, explicitly justified
exception (4 Python tests, one root cause, already tracked). No known failure was hidden by
deleting or modifying a test, and no historical truth was rewritten. Both hardware profiles compile
cleanly from this exact commit; the frozen firmware is fail-closed by construction and by
audit-enforcement, with neither of this session's two new adapters (I4, I5) reachable from any
production command path. Proceeding to `F0` — final flash readiness evaluation. No flash will be
attempted without explicit operator authorization (Sections 18–19 of the handoff).
