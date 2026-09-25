# I8 — Read-only Web foundation: reconsidered under the objective change

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

The operator's objective-change instruction: *"If HostLink semantics are sufficiently stable after
I6, implement the read-only Web foundation offline... It may compile into the candidate while
remaining disabled/not qualified until Wi-Fi hardware validation."* I6 is now substantially
implemented (`ControllerService`, `ServiceReadiness`), so this reopens the question the earlier
audit-only I8 gate deferred. Reconsidered here; conclusion: no new server code is added, for the
same architectural reason I7 declined to choose an OTA transport.

## What "read-only Web foundation" can mean without a server

`ARCHITECTURE.md`'s UI-0 gate — *"Web architecture contract: frontend/backend boundary, typed
semantic command model, telemetry snapshot schema, session identity, bounded clients, static-asset
storage plan"* — is explicitly `designable offline now`, and is split into two genuinely different
pieces:

1. **The data/schema layer**: a typed, transport-neutral model of what a client would read. This
   is exactly what I6 built. `ControllerService` already is "frontend/backend boundary, typed
   semantic command model, telemetry snapshot schema" for every read-only surface in the
   Controller — a future HTTP handler would call the same accessors `CommandRouter` now calls,
   serializing to JSON instead of `printf` lines. No additional offline work closes this half
   further without inventing an actual serialization format nobody has reviewed.
2. **The server itself**: an actual `WebServer`/`esp_http_server`/`esp_https_server` instance,
   listening on a port, parsing HTTP, serving responses. This is where "compile into the candidate
   while remaining disabled" would apply — mirroring `MATDOG_OTA_INGEST_ENABLED`.

## Why the server half is not built this gate

Every option in the Controller README's own transport evaluation table (`WebServer`,
`esp_http_server`, `esp_https_server`) requires `<WiFi.h>` and, for two of the three, adds a second
FreeRTOS task whose interaction with `Controller::update()`'s non-blocking guarantee has never been
measured. The gate's own requirement — *"Keep memory/latency effects bounded and testable"* — is not
testable without actually receiving an HTTP request, which needs live Wi-Fi association; `@WIFI
STATUS`'s `last_us`/`max_us` fields already exist for exactly this measurement and are empty today
because no MATDOG build has ever associated with an access point (`ROADMAP.md` stage 10). Building
an HTTP server now would add a compiled-in-by-default-off dependency whose central safety property
— bounded memory/latency — could not be verified until the same hardware session that would also
resolve the transport choice I7 already deferred for the same reason (an undecided authenticated-
transport question, here compounded by an undecided *server library* question on top of it).
Unlike `MATDOG_OTA_INGEST_ENABLED`, which gates an already-fully-specified, already-tested state
machine (`OtaPolicy`) behind one flag, a Web server would be new, unreviewed surface area with no
existing offline-tested decision core underneath it to gate — there is no `WebPolicy.*` this session
built the way `OtaPolicy.*` already existed before OTA-A's transport question came up.

## What this gate confirms instead

- `ControllerService` (I6) already satisfies the "typed semantic command model, telemetry snapshot
  schema" half of UI-0 for every currently-implemented read-only surface.
- `ServiceReadiness`'s `WEB_READ_ONLY_DASHBOARD` capability (I6) already reports `BLOCKED` until
  Wi-Fi is hardware-validated, then `TO_TEST` — the exact "disabled/not qualified until Wi-Fi
  hardware validation" state the objective asks for, expressed as data rather than as inert server
  code. `@HOSTLINK READINESS` already surfaces it.
- No browser/network callback can own hardware or actuator authority: there is no browser, no
  network callback, and no code path from any future one to `ActuatorAuthority`/
  `SafeActuatorPolicy` — `check_no_network_to_servo_path` (pre-existing) and
  `check_actuator_infrastructure_wired_fail_closed` (I4/I5, this session) both still hold with zero
  Web-related surface added.

## Outcome

`I8` remains without new server code, now for an architectural reason specific to this gate rather
than only the earlier sequencing-based deferral: building the transport half responsibly requires
the same hardware session that would resolve I7's transport question, and gating an unreviewed HTTP
stack behind a compile flag is a materially different risk than gating an already-reviewed OTA
state machine behind one. The data/schema half is done, via I6. Proceeding to the full software
freeze.
