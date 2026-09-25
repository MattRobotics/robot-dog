# I4 — Safe Actuator runtime boundary

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

Closes gate **I4** (V3 handoff §12): "Implement the runtime adapter/interfaces required to
connect the already-tested Safe Actuator policy toward `ServoBus`. Offline only. Use fake/test
backends. The existence of the adapter must NOT make ordinary physical motion reachable yet."

## Audit before implementation

`src/actuator/ActuatorWritePolicy.h` (335 offline checks, not modified this gate) is the decision
core; its own architecture comment names the runtime adapter as `[TO_IMPLEMENT]` between itself and
`ServoBus`. Reading `src/servo/ServoBus.h` first: it exposes exactly one write, `safeOff()`
(`TorqueEnable = 0`), and no `EnableTorque(id, 1)` / `GoalPosition` write primitive exists anywhere
in the firmware. That is a hard boundary, not an oversight to work around — Torque ON and
`GoalPosition` are both on this session's explicit forbidden list. A "runtime adapter to `ServoBus`"
therefore cannot mean a real backend that writes through `ServoBus` today: there is nothing on the
other end to call. It has to mean the abstract interface and the bridge logic, offline-verified
against a fake backend, with no production implementation — exactly what "Offline only. Use
fake/test backends" says.

Two further findings shaped the scope:

1. `calibration::JointIdentity` (the type `ActuatorCommand.joint` carries) has no `bus_id` field by
   design — see `ActuatorWritePolicy.h`'s own comment: "the same discipline that keeps `bus_id` out
   of `JointIdentity`." Resolving a joint identity to a bus id is the canonical allocation's job
   (`servo/ServoPopulation.h`), so the runtime adapter does not attempt that resolution itself —
   duplicating it here would create a second place identity→bus-id could silently disagree.
2. `ActuatorCommand` carries three different target representations depending on operation:
   `target_tick` (raw, `POSITION_COMMAND` only), `delta_ticks` (signed excursion, `DIRECTION_VERIFY`
   only), and `target_urad` (URDF-frame angle, the two geometry-authorised moves). Converting the
   latter two into a raw tick needs an accepted q0/direction transform applied by a real execution
   engine — the "Calibration Execution Engine" the policy's own diagram marks `[NOT IMPLEMENTED]`.
   No such conversion exists anywhere in the current codebase (`JointTransform` stores a captured
   `q0_tick`, it does not expose a angle→tick conversion). Guessing that conversion here would be
   exactly the kind of unreviewed architectural decision the NextGen handoff says to stop for.
   **Resolution:** the adapter handles `TORQUE_ENABLE` and `POSITION_COMMAND` completely (both
   already carry a ready-to-write raw value); the three geometry-authorised operations
   (`CALIBRATION_CONTACT_PROBE`/`DIRECTION_VERIFY`/`CALIBRATION_AUXILIARY_MOVE`) always resolve to
   an explicit `NO_RAW_TARGET` result rather than an invented conversion. This is deferred to I5,
   not silently guessed.

## What shipped

`src/actuator/ActuatorRuntime.{h,cpp}` — new files, `ActuatorWritePolicy.h` untouched.

- `BackendCallKind backendCallFor(ActuatorOperation)` — pure, total function mapping every one of
  the six `ActuatorOperation` values to `NONE` / `ENABLE_TORQUE` / `WRITE_GOAL_POSITION`. Exposed
  separately from `execute()` specifically so it is exhaustively host-testable without first
  reconstructing a full policy `ACCEPT` for the three geometry-authorised operations, whose real
  `ACCEPT` path needs a compiled geometry profile, a live bootstrap context and an endpoint plan —
  exactly what `test_actuator_write_policy.cpp` already proves at length.
- `ActuatorBackend` — abstract interface, two methods (`enableTorque(bus_id)`,
  `writeGoalPosition(bus_id, target_tick)`), unsigned tick domain throughout, no signed-wrap
  expressible.
- `ActuatorRuntime` — the bridge. `execute(transaction, bus_id, decision_out)` calls
  `policy->commit(transaction)` exactly once; only if that returns `ACCEPT` does it make exactly one
  backend call, chosen by `backendCallFor()`. Any other decision reaches the backend **zero times** —
  proven directly by a host test that asserts the fake backend's call counters stay at zero after a
  rejected commit.

## Safety-boundary enforcement

New `check_actuator_runtime_boundaries()` in `scripts/static_audit.py`:

- fails the build if `ActuatorRuntime.h`/`.cpp` include `<Arduino.h>`, reference `Serial.`,
  `millis(` or `ServoBus` — keeping the adapter host-linkable, the same contract as
  `ActuatorWritePolicy` itself;
- fails the build if the identifier `ActuatorRuntime` appears in any file outside `src/actuator/`
  or the offline test suite — so nothing in `Controller.cpp`/`CommandRouter.cpp` can construct or
  own one without the audit catching it immediately, structurally enforcing "the existence of the
  adapter must not make ordinary physical motion reachable."

Confirmed independently by the build itself: `USB_ONLY` and `ROBOT_POWERED` compile to the exact
same byte counts as the I3 baseline (977,747 B stayed 978,195 B `USB_ONLY`, 978,751 B
`ROBOT_POWERED` — unchanged from before `ActuatorRuntime` existed), because the linker dead-code-
eliminates the entire adapter when nothing references it. The adapter is compiled and offline-
tested but contributes zero bytes to the actual flashed firmware.

## Offline test suite

New `scripts/tests/test_actuator_runtime.cpp`, linking the real `ActuatorRuntime`, the real
`SafeActuatorPolicy` and the real `ActuatorAuthorityArbiter` against a `FakeActuatorBackend` (call-
counting, independently steerable success/failure per method) — the same contract as the OTA
suite's fake `OtaBackend`. **46 checks, 0 failures.** Covers: `backendCallFor()` for all six
operations; a rejected commit never reaching the backend; `TORQUE_ENABLE` and `POSITION_COMMAND`
`ACCEPT` writing through with the exact bus id and tick (proving no rescaling/off-by-one on the
unsigned path); `ACCEPT` with no backend installed failing closed (`NO_BACKEND`, not a crash); a
backend-reported failure surfacing as `BACKEND_REJECTED` rather than being swallowed; and
`toString()` for every value including a corrupted-enum fallback.

Updated offline baseline: **12 host suites, 5623 checks** (previous baseline 11 / 5577, +46 from
this gate). Static audit: **PASS, 82 files** (+3: the two adapter files plus the new test suite).

### Builds

- `USB_ONLY`: PASS — 978,195 B flash (unchanged vs. the I3 baseline), 52,420 B RAM (unchanged).
- `ROBOT_POWERED`: PASS — 978,751 B flash (unchanged). Default `USB_ONLY` artifact restored
  afterward.

## Outcome

`I4 = PASS`. Preserved unmodified: authority lease/generation, the owner/operation matrix,
plan/commit semantics, stale/replay/provenance refusal, geometry binding, the unsigned
`GoalPosition` domain, and `SAFE_OFF`'s structural independence — none of `ActuatorWritePolicy.*`
was touched. No direct network→servo path was created or implied. The adapter adds no reachable
write path: no production backend exists, nothing wires the class into `Controller`, and the audit
now enforces that structurally rather than by convention. Proceeding to `I5` — Calibration
Execution Architecture.
