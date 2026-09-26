# I8 — Read-only Web foundation: deferred

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

Gate **I8** (V3 handoff §16) is not attempted this session. No code or documentation status
changed — this note records why, so the gate is not silently skipped without explanation.

## Why

V3 §16 states I8 applies **"only after I6 semantics are stable."** I6 (this session, previous gate)
was explicitly scoped as audit-only: `HostLink semantic layer` remains `PARTIAL` / `TO_DESIGN` in
`DEVELOPMENT_GATES.md`, not stable. I8's own entry condition is therefore unmet by the plain text of
the handoff.

Independently, `DEVELOPMENT_GATES.md`'s `UI-1` gate (the read-only dashboard) states its entry as
**"Wi-Fi runtime PASS"** — the Wi-Fi station runtime is `IMPLEMENTED`/`OFFLINE TESTED` but explicitly
`NOT HARDWARE TESTED` (no MATDOG build has ever associated with an access point). This session's
scope is software-only with the main fuse removed; Wi-Fi hardware validation is out of scope and
listed among today's forbidden actions.

A read-only Web foundation also cannot exist without a web transport, and I7 (previous gate)
deliberately left that transport choice unimplemented: the Controller README's own evaluation table
frames `esp_http_server`/`esp_https_server`/`WebServer` as options still awaiting a decision, not a
settled architecture. Building I8 would require either picking that transport unilaterally — exactly
the unreviewed architectural commitment I7 declined to make — or serving assets over a transport that
does not exist yet.

Three independent reasons (I6 not stable, Wi-Fi hardware entry unmet, no chosen web transport) point
the same direction, so this gate is deferred rather than attempted with a workaround.

## What is NOT deferred

Nothing about UI-0 (`ARCHITECTURE.md`'s "Web architecture contract: frontend/backend boundary, typed
semantic command model, telemetry snapshot schema... designable offline now") was found to be
missing new content beyond what `ARCHITECTURE.md` and `DEVELOPMENT_GATES.md` already record for it —
no gap was found there worth a separate change.

## Outcome

`I8 = DEFERRED`, not `PASS`, not `FAIL`. `ROADMAP.md`/`DEVELOPMENT_GATES.md`'s existing `BLOCKED`
status for `UI-1` is accurate and unchanged. Proceeding to `I9` — integrated software freeze.
