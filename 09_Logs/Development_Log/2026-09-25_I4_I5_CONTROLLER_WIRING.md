# I4/I5 — Controller wiring: fail-closed infrastructure integration

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

Per the operator's objective-change instruction: wire `ActuatorRuntime` and `CalibrationExecutionEngine`
into `Controller` as status/lifecycle infrastructure, without making Torque ON, `GoalPosition`,
contact motion or calibration motion reachable.

## What changed

`Controller` now owns real instances of `actuator::SafeActuatorPolicy`, `actuator::ActuatorRuntime`
and `calibration::CalibrationExecutionEngine` — previously none of the three existed inside
`Controller` at all; they were exercised only by their own offline test suites. `Controller::begin()`
wires them:

```cpp
actuator_policy_.begin(&authority_);
actuator_runtime_.begin(&actuator_policy_, /*backend=*/nullptr);
calibration_execution_.begin(&actuator_policy_, &actuator_runtime_);
```

Three deliberate choices make this safe:

1. **The runtime adapter's backend is `nullptr`.** Every `ActuatorRuntime::execute()` call that
   somehow reached an `ACCEPT` decision would still resolve to `ExecuteResult::NO_BACKEND` — there
   is nothing to write to, independent of policy state, geometry, or authority.
2. **No geometry is bound, no limit or transform is admitted, no live bootstrap context is set.**
   `actuator_policy_` starts and stays in exactly the state its own offline suite already proves is
   maximally restrictive: an empty limit table (`REJECT_NO_ACCEPTED_LIMITS`), no geometry
   (`REJECT_NO_GEOMETRY_PROFILE`), no transform (`REJECT_NO_ACCEPTED_TRANSFORM`).
3. **No command path calls `plan()`/`commit()`/`execute()`/`abort()`.** `ControllerService` exposes
   only read-only status (`epoch()`, `hasOutstandingTransaction()`, `counters()`, `lastDecision()`,
   `limits().size()`, `transforms().size()`, a geometry-bound boolean) — new `@ACTUATOR STATUS`
   presents it. No new write-capable command was added anywhere.

`ActuatorRuntime` and `CalibrationExecutionEngine` themselves expose no status of their own — both
are effectively stateless between calls neither of which `Controller` ever makes — so
`ControllerService` reads only from `SafeActuatorPolicy`, the layer both sit on top of.

## Boundary enforcement, and a real bug it caught

Both I4/I5 boundary checks (`check_actuator_runtime_boundaries`,
`check_calibration_execution_engine_boundaries`) were extended to allow `Controller.h`/`Controller.cpp`
as the one reviewed consumer, by exact filename — not by directory — so `CommandRouter.cpp` and
`ControllerService.h`, which also live in `src/core/`, remain excluded and cannot reference either
class by name.

New `check_actuator_infrastructure_wired_fail_closed()` enforces the three choices above
structurally:

- `actuator_runtime_.begin(...)` must contain the literal `nullptr`.
- `Controller.cpp` must never contain `bindGeometry(`, `.admit(`, or `setBootstrapContext(`.
- `CommandRouter.cpp`, `ControllerService.h` and `Controller.cpp` must never call
  `plan()`/`commit()`/`execute()`/`abort()`.

**The first version of the third rule was wrong and would not have caught a real violation.** It
checked for the literal substrings `".plan("`, `".commit("`, etc. — but `modules_.actuator_policy`
is a pointer, so every real call site uses `->plan(`, not `.plan(`. A manual mutation check
(temporarily injecting `modules_.actuator_policy->plan(...)` into `CommandRouter.cpp`) confirmed
`static_audit.py` still reported `PASS` with the dot-only check — a false negative, caught before
being trusted. Fixed to a regex matching both call syntaxes
(`(?:\.|->)\s*(plan|commit|execute|abort)\s*\(`), re-verified to catch the same injected call, then
re-verified clean after reverting. The equivalent `nullptr`-presence check was also manually
mutated-and-reverted to confirm it fires.

## Verification

- Offline suites: **14 host suites, 5726 checks** (unchanged — this gate is Arduino-only glue with
  no new pure logic; correctness rests on the static-audit boundary checks above plus the already-
  exhaustive `SafeActuatorPolicy`/`ActuatorRuntime`/`CalibrationExecutionEngine` suites).
- Static audit: **PASS, 89 files**, 0 findings, including both manually-verified mutation checks.
- `USB_ONLY`: PASS — 981,727 B flash (+2,616 B vs. the I6 baseline of 979,111 B — this code is no
  longer dead-code-eliminated, since `Controller` now genuinely calls into it), 53,356 B RAM
  (+872 B).
- `ROBOT_POWERED`: PASS — 982,299 B flash (+2,644 B vs. 979,655 B). Default `USB_ONLY` artifact
  restored afterward.

## Outcome

`I4`/`I5` Controller wiring complete. `hardware_motion_authorized` stays `0`; no accepted current
`JointTransform` means no raw target is ever computable; the runtime backend is `nullptr` so no
write is ever reachable even if every upstream check were somehow bypassed; `RESTORE` remains
categorically non-executing in `CalibrationExecutionEngine` (unchanged from I5, so authority loss
still produces zero restore motion); global `SAFE_OFF` remains completely independent of all three
new instances (`ServoBus::safeOff()` has no reference to any of them). Proceeding to `I8`.
