# I7 — Wi-Fi / OTA transport + security: audit

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

Closes gate **I7** (V3 handoff §15). This gate is **audit-only** — no code changes. W1 Wi-Fi and
OTA-A/OTA-B already exist and are not replaced, per the explicit instruction.

## V3's eleven sub-items, checked against what already exists

| Item | Status | Evidence |
|---|---|---|
| authorization | **DONE** | OTA-B: `OtaAuthorityGate` backed by the real `ActuatorAuthorityArbiter`; an exclusivity inhibit, atomically acquired (`requestInhibit()`), closing the TOCTOU window a plain `authority == NONE` check would leave open across a multi-second update |
| update session semantics | **DONE** | `OtaPolicy`'s state machine: `IDLE → TARGET_RESOLVED → RECEIVING → IMAGE_SEALED → IDENTITY_VERIFIED → BOOT_TARGET_SET`, `FAILED` terminal, boot target reachable from exactly one state |
| manifest/build identity | **DONE** | `build::kBuildId` + `scripts/build_manifest.py` (`SOURCE_COMMIT`, `APPLICATION_SHA256`, `APPLICATION_SIZE`) — the V3 §16.1 identity chain, not `esp_app_desc_t` |
| size/hash validation | **DONE** | streaming `Sha256`, declared size bounded against the target partition before any byte is written, our own SHA-256 checked against the declared one at `finishStream()` |
| pending-reboot behavior | **DONE** | `OtaBootGuard`: first-boot confirmation requires `Controller::begin()` complete, `CommandRouter` bound, uptime ≥ 15 s / ≥ 2000 ticks, no PANIC/WDT/BROWNOUT reset |
| recovery invariant | **DONE** | wired USB recovery remains the permanent path (`flash_app_only.sh` is never the OTA writer); full-flash backup re-verified byte-for-byte this session (I0 audit) |
| bounded memory | **DONE** | no image is ever buffered in RAM — bytes are hashed and written per chunk; `OTA_WITH_SEQUENTIAL_WRITES` erases incrementally per sector |
| concurrency | **DONE** | the OTA-B inhibit is the concurrency answer: single-threaded `Controller::update()` plus an atomic acquire-and-hold means no actuator owner can appear mid-update |
| status reporting | **DONE** | `@OTA STATUS`, `OtaManagerStatus` (policy state, fault, running/target partition, boot state, self-check fault, `open_us`/`write_us`/`end_us` once measured) |
| **authenticated transport** | **NOT DONE, deliberately** | see below |
| **failure/retry** | **NOT SEPARABLE from transport** | see below |

Nine of eleven items were already complete before this gate; nothing needed to change for them.

## Why transport/authentication and failure/retry are not implemented here

The Controller `README.md` already contains a five-option transport evaluation table (`ArduinoOTA`
rejected, `WebServer`, `esp_http_server`, `esp_https_server`, raw TCP framing, USB CDC ingest) with
an explicit recommendation: exercise ingest over USB CDC first (zero network exposure), then raw TCP
framing with a pre-shared key, then `esp_https_server` once a certificate story exists. That
recommendation is explicitly labelled **"not yet implemented"** and `DEVELOPMENT_GATES.md` states
**"Transport and authentication are both TO_IMPLEMENT."**

Implementing even the recommended first step — USB CDC ingest — means writing the first reachable
firmware-write code path anywhere in this codebase, gated behind `MATDOG_OTA_INGEST_ENABLED=0` by
default but real code nonetheless: chunk parsing from `Serial`, wiring `writeChunk()` into
`CommandRouter`, and the framing/error-handling around it. Getting that wrong has a uniquely severe
failure mode among everything advanced in this integration session — a flawed ingest path risks
bricking the device in a way that depends on the same partition/bootloader state the wired-recovery
invariant assumes stays untouched. The existing evaluation table frames the choice as a
**recommendation awaiting review**, not a decided task; treating "advance as far as possible
offline" as authorization to build it now would be exactly the kind of unreviewed architectural
commitment the NextGen handoff says to stop for rather than make unilaterally.

`failure/retry` semantics are meaningless in isolation: there is no transport to retry against, so
"retry" is a property of whichever transport gets built, not something separable from that decision.

`ArduinoOTA` remains rejected (unchanged, already audited: `check_ota_boundaries` in
`scripts/static_audit.py` still enforces "no duplicate OTA writer"). `FIRMWARE_UPDATE` inhibit is
still held through a successful commit until reboot (unchanged, `OtaAuthorityGate`'s existing
behaviour, re-verified by reading — not modifying — `OtaManager.cpp`/`OtaAuthorityGate.cpp` this
gate). Real ingest stays disabled by default (unchanged, audit-enforced).

## Outcome

`I7 (audit) = PASS`. No code changed; no static-audit or host-suite change was needed since nothing
in `src/` was touched. `OTA`/`Wi-Fi runtime` remain `PARTIAL` / `HARDWARE TO_TEST` in
`DEVELOPMENT_GATES.md` and `ROADMAP.md`, unchanged — both already correctly state everything this
audit confirms. Transport and authentication remain `TO_IMPLEMENT`, deliberately, pending a
reviewed transport decision. Proceeding to `I8` — read-only Web foundation.
