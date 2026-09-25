# I6 — HostLink semantic layer: implementation

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

Supersedes the earlier audit-only I6 gate (same date), per the operator's objective-change
instruction: implement the minimal transport-neutral semantic/service layer offline, exposing
BLOCKED/TO_TEST/READY for currently-unavailable functions instead of leaving them unreachable with
no explanation. That instruction did not conflict with the earlier finding — `DEVELOPMENT_GATES.md`
rule 6 gates *activation/validation claims*, not offline implementation of fail-closed software; no
gate status changed as part of this work.

## What shipped

**`src/core/ServiceReadiness.{h,cpp}`** — pure, host-linkable readiness classifier.
`ServiceCapability` (8 named capabilities: actuator torque/position, the three calibration
operations, Wi-Fi hardware association, OTA end-to-end, the read-only Web dashboard) ×
`ServiceReadinessInputs` (2 booleans: `hardware_motion_authorized`, plus separate Wi-Fi/OTA
hardware-validation flags) → `ServiceReadiness` (`READY`/`TO_TEST`/`BLOCKED`). `TO_TEST`, not a
separate "NOT_READY" word, deliberately: it is already the root README's status vocabulary.
31 offline checks.

**`src/core/ControllerService.h`** — the transport-neutral telemetry layer itself. Scope
deliberately bounded to passive status/telemetry reads (the "telemetry" half of "semantic
command/telemetry layer"): ~30 accessor methods, each forwarding a struct its owning module already
computed (`WifiStatus`, `OtaManagerStatus`, `CalibrationSessionStatus`, a `DalySample`, ...) — zero
duplicated logic, verified by the fact every method body is a one-line forward. Action/write
commands (servo scan/census/preflight/read/safe_off, mode changes, the DALY KEY write, Wi-Fi
enable/disable, LED test/off, system shutdown) are **not** routed through this layer: they carry
side effects and CommandRouter's existing per-command guards stay exactly where they are. Unifying
the *action* half of the semantic model is future work.

**`CommandRouter` refactored**, not duplicated: every read-only `print*` method (`@STATUS`,
`@IMU STATUS`, `@BMS STATUS`, the four `@BMS KEY *` status commands, `@LED STATUS`,
`@WIFI STATUS`, `@OTA STATUS`, `@AUTHORITY STATUS`, `@CALIBRATION STATUS`,
`@SYSTEM SOURCE_SIGNATURE`, and the three servo diagnostic result printers) now calls
`modules_.service->...()` instead of touching module pointers directly. Formatting (the `printf`
lines) stays in `CommandRouter` — that part is genuinely Serial-specific; the *data* a future
Web/Jetson client would need is now available from one class that does not carry
`servo::ServoBus*`/`power::DalyBms*` pointers at all.

**New `@HOSTLINK READINESS`** command: prints `capability=... readiness=...` for all eight
capabilities, computed on demand from `ControllerService::readiness()` — the first concrete
consumer of the readiness classifier.

## Boundary enforcement

- New `check_service_readiness_is_host_linkable()`: fails the build if `ServiceReadiness.{h,cpp}`
  ever gain an `<Arduino.h>` include, a `Serial.`/`millis(` call, or a reference to `ServoBus`,
  `CommandRouter` or `ControllerService` — module/transport wiring belongs in `ControllerService`,
  not the pure classifier.
- The pre-existing DALY-KEY-probe scope check (`check_daly_write`'s file-scope loop) initially
  failed against `ControllerService.h`, exactly as intended — it restricts which files may
  reference the DALY KEY probe to a reviewed allowlist, and `ControllerService.h` was not yet in
  it. Extended the allowlist to include it, with the same reasoning already applied to
  `CommandRouter.{h,cpp}`: it forwards the same already-computed snapshot, adding no second
  decision path. Re-ran the full DALY audit mutation suite (52/52) afterward to confirm the
  allowlist widening did not weaken any of the other 51 mutation-catching cases — it did not.

## Verification

- Offline suites: **14 host suites, 5726 checks** (previous baseline 13 / 5695, +31 from
  `ServiceReadiness`).
- Static audit: **PASS, 89 files** (+4: `ServiceReadiness.{h,cpp}`, `ControllerService.h`,
  `test_service_readiness.cpp`), 0 findings, including the DALY mutation suite (52/52) re-verified
  green after the allowlist change.
- `USB_ONLY`: PASS — 979,111 B flash (+775 B vs. the I9 baseline of 978,336 B), 52,484 B RAM
  (+64 B).
- `ROBOT_POWERED`: PASS — 979,655 B flash (+759 B vs. 978,896 B). Default `USB_ONLY` artifact
  restored afterward.

`ControllerService`/`CommandRouter` are Arduino-dependent glue (like `Controller`/`CommandRouter`
already were) and are not host-unit-tested directly; correctness of the refactor was verified by
(a) the Arduino build compiling clean on the first attempt for both profiles, (b) every accessor
being a literal one-line forward with no logic to diverge, and (c) `ServiceReadiness`, the one
piece of genuinely new decision logic, being fully pure and host-tested.

## Outcome

`I6 = PASS` (implementation, superseding the earlier audit-only classification). No actuator write
path was touched; `ActuatorAuthority`/`SafeActuatorPolicy` are unreferenced by this layer.
Proceeding to `I7`.
