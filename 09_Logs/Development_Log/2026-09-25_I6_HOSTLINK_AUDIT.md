# I6 — HostLink semantic layer: audit

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

Closes gate **I6** (V3 handoff §14: "Audit need for a transport-neutral command/telemetry semantic
layer before web/OTA transport. Avoid binding domain logic to Serial or HTTP."). This gate is
**audit-only** — no code changes. The reasoning for that scoping decision is recorded first, since
it departs from what a literal reading of "I6 — HostLink / service semantics" might suggest.

## Why this gate is scoped as audit-only

`DEVELOPMENT_GATES.md` states its own binding rule 6: *"A gate's technical prerequisites do not
override `ROADMAP.md` sequencing... A stage may only begin when the roadmap sequence has reached
it. Reordering requires an explicit, reviewed change to the roadmap itself — never a local reading
of one gate's entry list."*

The HostLink gate's own `ENTRY` field states: *"all preceding `ROADMAP.md` stages through formal
recalibration completed... HostLink follows Service/Provisioning/QC and Full Leg Calibration in the
canonical sequence and must not be pulled forward ahead of them."*

None of those prerequisite stages are complete: Service/Provisioning is `TO_DESIGN`, QC is
`TO_DESIGN`, and formal recalibration is `BLOCKED` on `CALIBRATION_RESET_PENDING_FULL_RECALIBRATION`
(last formal population result 6/12, historical). Implementing and declaring the HostLink gate
`PASS` today would therefore directly violate the repository's own sequencing contract — no
"explicit, reviewed change to the roadmap" authorizing that reorder exists.

The V3 handoff's own I6 instruction, read literally, only asks to **audit** whether a semantic layer
is needed — it does not say to implement or pass the gate. That framing does not conflict with rule
6; declaring the gate "generically implemented" would have. This gate is scoped to the audit only,
consistent with both sources rather than picking one over the other.

The HostLink gate's stated `PASS CRITERIA` — *"a second transport can consume the same semantic
state without reimplementation or a duplicate hardware transaction"* — is inherently unverifiable
today regardless of sequencing: there is exactly one transport (USB CDC), and building a second one
(Wi-Fi-hosted HTTP/WebSocket) is itself gated behind Wi-Fi hardware validation (`ROADMAP.md` stage
10, `HARDWARE TO_TEST`), which this software-only phase does not authorize.

## Audit: how transport-coupled is `CommandRouter` today?

`src/core/CommandRouter.cpp`: 792 lines, 25 `@COMMAND` branches in `handleLine()`.

**Already transport-independent** (18 call sites): every read-only status command formats an
already-computed structured snapshot rather than querying hardware or computing anything itself —
`modules_.wifi->status()` (`WifiStatus`), `modules_.ota->status()` (`OtaManagerStatus`),
`modules_.daly->sample()`/`keyConfigSnapshot()`/`keyWriteStatus()`, `modules_.authority->current()`/
`counters()`, `modules_.calibration->status()` (`CalibrationSessionStatus`), `modules_.led_status->state()`,
`modules_.servo_census->result()`, `modules_.servo_preflight->result()`, and every module's
`availability()` (`AvailabilityStatus`). This is precisely the G2-established pattern
(`check_g2_state_is_transport_independent` in `scripts/static_audit.py`, already enforced) —
"transport-independent state, transport-specific presentation." A second transport consuming these
same structs would not need to touch the modules that produce them at all.

**Still transport-coupled**: the 792-line `handleLine()` itself. Command *dispatch* (matching an
incoming string to an action) and *presentation* (formatting a result as `KEY=value` lines via
`Serial.printf`) are both embedded directly in `CommandRouter`, which is explicitly documented as
"a USB CDC adapter" (`ROADMAP.md` stage 9) rather than a transport-neutral service layer over one.
A second transport (HTTP/WebSocket) would today have to reimplement this entire dispatch table
against its own request format, and reformat every response from scratch — exactly the duplication
V3 §14 and the HostLink gate's `FORBIDDEN` clause ("duplicate command semantics per transport") warn
against.

**Action commands** (not pure presentation) call a module method directly after parsing:
`@MODE RUN|MAINTENANCE` → `operating_mode->setMode()` + `authority->onOperatingModeChanged()`;
`@WIFI ON|OFF` → `wifi->setEnabled()`; `@LED OFF|TEST` → `led->off()`/`startTest()`;
`@BMS KEY READ|SET DISCHARGE CONFIRM` → the guarded DALY KEY calls; `@SYSTEM SHUTDOWN` →
`power_state->requestShutdown()`. Each already goes through exactly one owning module — `CommandRouter`
holds no domain logic of its own here, only the text-to-call mapping. This matches "no transport
callback may directly become actuator authority": none of these paths touches `ActuatorAuthority`,
`SafeActuatorPolicy` or `ActuatorRuntime` at all, and static audit already keeps it that way
(`check_no_network_to_servo_path`, `check_actuator_runtime_boundaries`).

## What a future HostLink layer would need to add (design sketch, not implemented)

Not built in this gate — recorded so the eventual design does not start from nothing:

- A **command ID / schema** independent of the literal `@STRING` — e.g. an enum or small integer
  per action, with USB CDC's text parser becoming one encoding of it rather than the only one.
- A **response struct per command family**, mirroring the structs that already exist
  (`WifiStatus`, `OtaManagerStatus`, `CalibrationSessionStatus`, ...) — most of the hard part
  (structured, transport-independent state) is already done; what is missing is a uniform way to
  *address* and *serialize* it that is not "a `Serial.printf` format string hand-written per
  command."
- An explicit **session/identity model** if a second transport is concurrent with USB CDC (today
  there is exactly one client; `ARCHITECTURE.md`'s Web UI contract already anticipates "bounded
  clients" for this reason).
- Nothing about this sketch implies which transport comes first, nor does it commit to a specific
  serialization (text, binary, JSON) — that remains `TO_DESIGN`, consistent with
  `DEVELOPMENT_GATES.md`'s own "A formal Controller Service Layer and schema are `TO_DESIGN`."

## Outcome

`I6 (audit) = PASS`. No code changed. `HostLink semantic layer` remains `PARTIAL` /
`TO_DESIGN` in `DEVELOPMENT_GATES.md` and `ROADMAP.md` — this audit does not change either
document's status field, since doing so would itself be the "local reading of one gate's entry
list" rule 6 forbids. Proceeding to `I7` — Wi-Fi / OTA transport-security completion.
