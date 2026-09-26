# I7/I8 — Network transport implementation (supersedes the earlier reconsiderations)

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

Supersedes [`2026-09-25_I7_RECONSIDERED.md`](2026-09-25_I7_RECONSIDERED.md),
[`2026-09-25_I7_WIFI_OTA_AUDIT.md`](2026-09-25_I7_WIFI_OTA_AUDIT.md),
[`2026-09-25_I8_RECONSIDERED.md`](2026-09-25_I8_RECONSIDERED.md) and
[`2026-09-25_I8_WEB_FOUNDATION_DEFERRED.md`](2026-09-25_I8_WEB_FOUNDATION_DEFERRED.md). Those four
notes correctly declined to build a transport under their own reasoning; the operator's "FINAL
PRE-FLASH CONSOLIDATION" instruction (same date) supplied the missing piece each one was blocked on,
and this gate implements the result.

## Section A — the correction that unblocked I7

The earlier `I7_RECONSIDERED.md` treated `CommandRouter::kLineBufSize = 96` (a 96-byte USB CDC line
buffer) as a reason the *entire OTA transport architecture* needed a new, unreviewed binary I/O mode
inside `CommandRouter`/`Controller`. That conflated two different things:

- streaming firmware bytes through the **line-oriented USB CDC command parser** — genuinely blocked
  by the 96-byte buffer, and still blocked; nothing here changes that path;
- streaming firmware bytes through **any transport at all** — not blocked by it, because
  `esp_http_server` request bodies never touch `CommandRouter::handleLine()`. `httpd_req_recv()`
  reads directly off the HTTP connection into a caller-owned buffer
  (`network/HttpTransport.cpp`, `kHttpChunkBufferBytes = 1024`), on the httpd task, entirely outside
  the USB CDC line parser.

Once the CONTROL/AUTHORIZATION plane (HTTP request headers, the challenge/response handshake) and
the BINARY DATA plane (the request body, chunked into `OtaManager::writeChunk()` calls) are
recognized as never sharing the 96-byte buffer, the blocker dissolves. Firmware bytes still flow
through exactly one path: `HttpTransport -> OtaSession (auth only) -> OtaManager -> OtaPolicy ->
OtaEspBackend` — the same single writer `check_ota_boundaries()` already enforces, never a second
one. `network callback -> flash primitive` directly is not used anywhere; every OTA callsite in
`HttpTransport.cpp` calls `OtaManager`'s five substitution-point methods
(`prepare`/`openStream`/`writeChunk`/`finishStream`/`commitBootTarget`, plus `abort`), never
`esp_ota_*` directly — `check_ota_boundaries()`'s existing "ESP-IDF OTA API confined to
`OtaEspBackend.cpp`" rule already covers `HttpTransport.cpp` too, since it is not exempted from it.

## Toolchain audit (before writing any code)

Confirmed already installed, zero new dependencies added:

- `esp_http_server.h` / `esp_https_server.h` — `~/.arduino15/packages/esp32/tools/esp32s3-libs/
  3.3.11/include/esp_http_server/include/` and `esp_https_server/include/`. Signatures confirmed:
  `httpd_start`, `httpd_uri_t`, `httpd_register_uri_handler`, `httpd_req_recv`,
  `httpd_req_get_hdr_value_len`, `httpd_req_get_hdr_value_str`, `httpd_resp_send`,
  `httpd_resp_send_chunk`, `httpd_resp_send_err`.
- mbedTLS crypto primitives (`sha256.h`, `gcm.h`, `hmac_drbg.h`, `x509.h`, `x509_crt.h`, `ecdsa.h`,
  `esp_hmac_pbkdf2.h`) under the same toolchain tree — available, but not used; see below.
- FreeRTOS queue API (`freertos/queue.h`, `xQueueGenericSend`/`xQueueReceive`) — available as an
  alternative to the binary-semaphore mailbox actually used; documented here as the road not taken.

## Why HMAC-SHA256 over a pre-shared secret, not full TLS

Both are available in the installed stack. HMAC was chosen:

- **Certificate provisioning.** `esp_https_server` needs a server certificate/key provisioned onto
  the device and a trust decision made by every client — an unreviewed PKI question with no existing
  answer in this repository, versus a pre-shared secret that fits the exact pattern
  `WifiCredentials.local.h` already established and `static_audit.py` already enforces
  (gitignored local file, empty-fallback, exactly-one-use-site).
- **Unmeasured resource cost.** TLS handshake CPU/heap/latency on an ESP32-S3 under
  `Controller::update()`'s non-blocking budget has never been measured on this hardware; HMAC-SHA256
  reuses the already-reviewed, already-tested `Sha256` streaming implementation
  (`src/update/Sha256.h`), adding one small, host-testable primitive (`src/update/Hmac256.*`)
  instead of a new TLS stack integration.
- **Matches the existing trust model.** MATDOG's other network secret (the Wi-Fi passphrase) is
  already a pre-shared value known only to the operator; the OTA secret follows the same shape
  deliberately, as a *separate* secret (`src/config/OtaCredentials.h`) — knowing the Wi-Fi password
  only gets an attacker onto the LAN, not the ability to push firmware.

If a future session's hardware measurements show HMAC-over-plain-HTTP is insufficient (e.g. a
requirement for transport confidentiality, not just integrity/authentication), `esp_https_server` is
confirmed available and the `OtaSession` authentication layer underneath it does not change — only
the transport wrapping it would.

## The authentication/session design (`src/update/OtaSession.h/.cpp`)

A bounded state machine between the network transport and `OtaManager`, covering every state/
failure path the operator's instruction listed:

| Requirement | Where it lives |
|---|---|
| Authentication | `OtaSession::authenticate()` — HMAC-SHA256 over a fixed-width signed payload |
| Authorization | `hasSecret()`: no configured secret means every request fails closed (`REJECTED_NO_SECRET`) — the same shape `WifiPolicy` uses for an absent SSID |
| Metadata acceptance | `authenticate()`'s `OtaImageMetadata` parameter — the exact struct passed on to `OtaManager::prepare()` on success, so what was authenticated is what gets written |
| Content length | `metadata.image_size`, part of the signed payload — tampering with it invalidates the signature |
| Application SHA256 | `metadata.sha256`, part of the signed payload |
| Build identity | `metadata.build_id`, part of the signed payload |
| Session timeout | `OtaSessionConfig::activity_timeout_ms` (default 30000 ms) + `timedOut()`, reset by `noteActivity()` on every chunk |
| Duplicate/concurrent request rejection | `issueChallenge()` invalidates any previous outstanding nonce — never more than one live challenge; `HttpTransport`'s single-slot mailbox serializes all requests onto one Controller-thread pass regardless |
| Abort | `HttpRequestKind::OTA_ABORT` -> `OtaManager::abort()` + `OtaSession::reset()`, dispatched on every read failure/rejected chunk in `HttpTransport::handleOtaUpdate()` |
| Failed-transfer cleanup | Same abort path; a failed `writeChunk()` triggers it automatically |
| Successful pending-reboot state | `OTA_FINISH` -> `OtaManager::finishStream() && commitBootTarget()`, then `OtaSession::reset()`; `HttpTransport` deliberately never calls `esp_restart()` — a committed image waits for a separate, explicit reboot step |
| Retry policy | `OtaPolicy::reset()` (pre-existing, OTA-A) already returns the state machine to `IDLE` after any abort/failure; nothing in this layer adds a cooldown or attempt limit beyond the per-nonce single-use rule |

Replay protection uses a single-use, server-issued random nonce rather than a timestamp (the device
has no clock sync): `issueChallenge()` hands out one nonce with a bounded lifetime
(`challenge_ttl_ms`, default 30000 ms); `authenticate()` consumes it exactly once, on the first
presentation of the *matching* nonce, regardless of whether that attempt then succeeds or fails on
expiry or signature. A presented nonce that does not match the outstanding one consumes nothing.

The signed payload is fixed-width by construction — `nonce[16] || schema_version(BE32) ||
image_size(BE32) || sha256[32] || build_id[kOtaBuildIdBytes]`, never length-prefixed or
delimiter-separated — so no two distinct `(nonce, metadata)` pairs can ever serialize to the same
bytes, closing a classic class of HMAC framing-ambiguity bugs by construction rather than by review.

### Two bugs found and fixed by the suite before being trusted

1. **Nonce-consumption availability bug.** The original `authenticate()` unconditionally cleared
   `challenge_outstanding_` regardless of whether the presented nonce matched, so a single garbage/
   stale-nonce request could burn the real outstanding challenge as a side effect — a denial-of-
   service against the legitimate client. Caught by
   `test_new_challenge_invalidates_the_previous_one` in `scripts/tests/test_ota_session.cpp`. Fixed
   by only clearing `challenge_outstanding_` once `memcmp(nonce, nonce_, kOtaNonceBytes) == 0`.
2. **`build_id` tail nondeterminism.** `buildOtaSignedPayload()` originally copied the raw
   `build_id` array into the signed payload verbatim; two metadata structs with the same intended
   build id but different uninitialized bytes past the NUL terminator (e.g. one filled via
   `strncpy()` without zeroing first) would sign to different payloads for the same logical
   identity. Fixed by explicitly zero-padding past `strnlen(build_id, ...)`. Verified by
   `test_signed_payload_ignores_garbage_past_the_build_id_nul`.

Verified against RFC 4231's official HMAC-SHA256 test vectors (`test_hmac256.cpp`, 11 checks) and a
35-check adversarial suite covering every `OtaAuthResult` path plus timeout/reset/payload-determinism
(`test_ota_session.cpp`) — both link the REAL implementation, not a copy, and both are now wired into
`scripts/tests/run_host_tests.sh` and `scripts/static_audit.py`'s `check_host_tests()`.

## The transport adapter (`src/network/HttpTransport.h/.cpp`)

Three endpoints on one `esp_http_server` instance: `GET /status` (I8, read-only), `GET
/ota/challenge` (I7, issues a nonce), `POST /ota/update` (I7, the ingest path, headers `X-Ota-Nonce`/
`X-Ota-Signature`/`X-Ota-Sha256`/`X-Ota-Build-Id` plus the streamed body). `POST /ota/update` checks
`OtaManager::ingestEnabled()` first and returns `403` immediately if `MATDOG_OTA_INGEST_ENABLED=0`
(unchanged default) — starting the Web server alone can never make ingest reachable.

### The concurrency problem and its resolution

`esp_http_server` always runs registered handlers on its own FreeRTOS task, never on the caller of
`httpd_start()`. Every other MATDOG module is deliberately single-threaded
(`core/ActuatorAuthority.h`, `update/OtaPolicy.h`) — `ControllerService`/`OtaSession`/`OtaManager`
must only ever be touched from the one Controller thread. `HttpTransport` resolves this with a
single-slot, strictly-alternating mailbox: two FreeRTOS binary semaphores
(`request_ready_`/`response_ready_`), one pending request/response struct, no separate mutex needed
because the alternation itself guarantees only one side ever touches them at a time. `dispatch()`
(called from the httpd task) gives `request_ready_`, then blocks on `response_ready_` with a bounded
3000 ms timeout; `HttpTransport::update()` (called from `Controller::update()`, non-blocking, drains
at most one request per tick) does the reverse. A timeout fails the HTTP request closed (`503`)
rather than blocking either task indefinitely.

**What this does not prove:** whether the semaphore handoff is genuinely deadlock-free and correctly
timed under real concurrent HTTP load cannot be exercised by a host test — there is no FreeRTOS
scheduler on the host. This is `HARDWARE_TO_TEST`, the same classification this repository already
gives Wi-Fi association timing and OTA flash-write costs; it is not claimed as offline-validated.

### Server lifecycle

`start()`/`stop()` are never called from `Controller::begin()` — confirmed both by direct reading and
by a `static_audit.py` check (`check_http_transport_boundaries`) that fails the build if
`Controller::begin()` ever calls `http_transport_.start()`. The server is reachable only from
`@WEB SERVER START|STOP|STATUS`, MAINTENANCE-gated in `CommandRouter.cpp` the same way the DALY KEY
write and the servo scan/census/preflight commands are — physical/USB access is the trust boundary
for *opening the socket at all*; the socket itself grants no actuator authority regardless of state.

## I8 — the read-only `/status` payload

`GET /status` serializes exactly the operator's explicit field list, every value read through
`ControllerService` accessors (never a direct hardware read):

- **system/source identity** — `build_id`, `system_health`, `power_state`, `mode`, `authority`
- **module health** — `imu_availability`, `bms_availability`, `servo_availability`,
  `led_availability` (the same init/detected/expected/result classification `@STATUS` uses)
- **Wi-Fi state** — `wifi_state`, `wifi_connected`, `wifi_fault`
- **BMS cached telemetry** — `bms_comm`, `bms_age_ms`, `bms_valid`, `bms_pack_v`, `bms_soc_percent`
- **IMU/status summary** — `imu_stream`, `imu_rv_count`
- **LED presentation state** — `led_presentation`
- **actuator readiness/fail-closed state** — `actuator_policy_epoch`, `actuator_geometry_bound`,
  `actuator_readiness_torque_enable`, `actuator_readiness_position_command` (the last two via
  `ServiceReadiness`/`ServiceCapability`, the same classifier `@HOSTLINK READINESS` uses)
- **calibration readiness/state** — `calibration_session_state`,
  `calibration_hardware_motion_authorized`, `calibration_restore_required`
- **OTA status/provenance** — `ota_state`, `ota_running_build_id`, `ota_ingest_compiled`,
  `ota_rollback_possible`

`kHttpStatusTextBytes = 2048` (bumped from an initial 1024 once the full field list was drafted) —
a fixed, bounded buffer, never a growing one; `snprintf()`'s own truncation-length return is used to
compute `text_len`, so a future field addition that overflows it truncates safely rather than
overrunning.

`HttpTransport` owns no hardware, reads no servo/BMS/IMU bus directly, and calls no actuator
policy/runtime method — `check_no_network_to_servo_path` (pre-existing) and
`check_actuator_infrastructure_wired_fail_closed` (I4/I5) both still hold with this file added, and
`check_http_transport_boundaries` (new, this gate) adds the transport-specific invariants. Hardware
Wi-Fi timing/heap/RF effects of actually serving a request remain `HARDWARE_TO_TEST`, not claimed
here as offline-validated — only the code paths that do not require a live radio (the format string
itself, its accessors, the session state machine) are.

## New static-audit coverage (`check_http_transport_boundaries`, `scripts/static_audit.py`)

Mirrors the existing `check_ota_boundaries`/`check_wifi_runtime_boundaries` pattern:

1. `Hmac256.{h,cpp}`/`OtaSession.{h,cpp}` stay free of `<Arduino.h>`, `<esp_http_server.h>`,
   `<WiFi.h>` and `Serial.` — host-linkable, the same contract as `WifiPolicy`/`OtaPolicy`.
2. The ESP-IDF HTTP server API (`httpd_start`, `httpd_register_uri_handler`, `httpd_req_recv`, ...)
   is confined to `network/HttpTransport.cpp` — one auditable translation unit, the same rule
   `check_ota_boundaries` already gives the ESP-IDF OTA API.
3. `Controller::begin()` never calls `http_transport_.start()`.
4. `@WEB SERVER START` checks `OperatingMode::MAINTENANCE` inside its own command branch.
5. `HttpTransport.cpp` never calls `esp_restart()`.
6. `kOtaSecret` is referenced only at the one `http_transport_.begin()` call site in
   `Controller.cpp`; `OtaCredentials.local.h` stays gitignored, templated, and untracked — the exact
   discipline `check_wifi_runtime_boundaries` already gives the Wi-Fi passphrase.

Every new check was manually mutation-tested before being trusted (this session's established
discipline): injecting `http_transport_.start()` into `Controller::begin()`, injecting a second
`httpd_start(` call site, and deleting the `@WEB SERVER START` MAINTENANCE gate were each confirmed
to fail the audit, then reverted. The MAINTENANCE-gate check's first draft used a fixed 600-character
lookahead window and was itself caught by its own mutation test picking up the *next* unrelated
command's gate instead of the deleted one; it was rewritten to bound the search to the `@WEB SERVER`
branch itself (up to the next `} else if (upper` boundary) before being trusted.

## Outcome

`I7 = IMPLEMENTED_OFFLINE_TESTED`: the transport, authentication/session layer, and single-writer
wiring are all compiled into the candidate and offline-tested; `MATDOG_OTA_INGEST_ENABLED` remains
`0` by default (unchanged), so no reachable firmware writer exists in the frozen candidate without a
separate, reviewed compile-time decision. `I8 = IMPLEMENTED_OFFLINE_TESTED`: the read-only `/status`
endpoint is compiled in, disabled at boot, MAINTENANCE-gated to start. Both remain `HARDWARE_TO_TEST`
for the properties that require a live radio and a real HTTP client — Wi-Fi association timing, the
semaphore handoff under real concurrent load, and end-to-end OTA transfer timing — none of which are
claimed as validated here. Proceeding to the new integrated freeze.
