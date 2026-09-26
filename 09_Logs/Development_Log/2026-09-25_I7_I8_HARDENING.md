# I7/I8 — network transport hardening before the one-flash candidate

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

Follow-up to [`2026-09-25_I7_I8_NETWORK_TRANSPORT_IMPLEMENTATION.md`](2026-09-25_I7_I8_NETWORK_TRANSPORT_IMPLEMENTATION.md)
(commit `4048ce6`, pushed). The operator reviewed that implementation and raised six concrete
findings before the true one-flash hardware-validation candidate could be produced. Each is
addressed below; nothing here reopens I0–I6 or the completed Safe Actuator/Calibration work.

## 1. OTA-ingest validation override, so ONE flash can validate everything

**Finding:** `MATDOG_OTA_INGEST_ENABLED` staying `0` by default is correct and must not change, but
a candidate built that way can never hardware-validate the OTA end-to-end path without a second
flash later.

**Fix:** A second, orthogonal, build-time override alongside `MATDOG_PROFILE`:

```bash
MATDOG_OTA_INGEST_VALIDATION=1 MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh
```

- `scripts/build.sh` gains the override (default: unset/`0`, unchanged behavior), echoed loudly in
  the build banner (`ota_ingest : ENABLED (OVERRIDE — hardware-validation candidate only)`) and
  passed as `-DMATDOG_OTA_INGEST_ENABLED=1` — the same compiler-flag mechanism the source `#ifndef`
  guard in `OtaManager.h` already supported, now with an explicit, named build-script entry point
  instead of an ad hoc `-D`.
- `scripts/build_manifest.py` gains `OTA_INGEST_ENABLED` as a new required manifest field and a
  second, fully independent authorization axis (`KNOWN_OTA_INGEST_VALUES`,
  `OTA_INGEST_VALUES_REQUIRING_EXPLICIT_AUTHORIZATION`, `Refusal.OTA_INGEST_MISMATCH`) — the exact
  same shape `HARDWARE_PROFILE`/`PROFILE_MISMATCH` already has, verified never inferred from the
  profile (a `ROBOT_POWERED` build with ingest still `0` is the ordinary, expected case).
- `scripts/flash_app_only.sh` gains `MATDOG_FLASH_OTA_INGEST` (default `0`) and prints
  `OTA_INGEST_ENABLED` in the same loud pre-write banner `HARDWARE_PROFILE` already gets, plus a
  `FLASHED_OTA_INGEST_ENABLED=` line in the post-flash summary.
- 12 new tests in `scripts/tests/test_build_manifest.py` (`TestOtaIngestAuthorization`), including
  an explicit check that `OTA_INGEST_ENABLED` is never inferred from `HARDWARE_PROFILE` and that a
  build failing both axes reports the profile mismatch first (deterministic ordering).
- `scripts/static_audit.py`'s `check_build_profile_provenance()` extended with the same anti-
  weakening checks the profile axis already has: no permissive default on `requested_ota_ingest`,
  the equality comparison must actually exist in code (not just be recorded), `--ota-ingest`/
  `--requested-ota-ingest` must actually be wired end to end.

Verified end to end: `MATDOG_OTA_INGEST_VALIDATION=1 MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh`
produces a manifest with `OTA_INGEST_ENABLED=1`; the ordinary default build still produces `0`.

## 2. Mailbox stale-response correlation

**Finding:** if `dispatch()` times out waiting for a response, the Controller thread might still
process the abandoned request later and signal `response_ready_` — with nobody left waiting for it.
A subsequent, unrelated `dispatch()` call could then have its own `Take(response_ready_)` satisfied
by that stale signal instead of its own request ever being processed.

**Fix:** a third semaphore, `slot_free_`, plus a small pure decision core, `HttpMailbox`
(`src/network/HttpMailbox.h`, header-only, host-linkable, 27-check adversarial suite in
`scripts/tests/test_http_mailbox.cpp`):

- `slot_free_` must be claimed (bounded `Take`) by `dispatch()` before it may write into
  `pending_request_`/`pending_response_`, and is only ever released by the Controller thread, once
  it has *fully* finished with those structs — this is what eliminates the underlying data race: a
  new dispatch cannot overwrite state a still-running (even abandoned) Controller-thread pass might
  still be touching, because it cannot even acquire the slot until that pass has finished.
- `HttpMailbox` answers the remaining question — "is anyone still listening?" — with two calls:
  `beginDispatch()` (set on the httpd-task side right after claiming the slot),
  `abandon()` (set on timeout), `awaitingResponse()`/`delivered()` (read/reset on the Controller-
  thread side, immediately before deciding whether to `Give(response_ready_)`). A generation counter
  was considered and found unnecessary: `slot_free_` already guarantees at most one dispatch is ever
  in flight, so a boolean is sufficient and correct.
- Mutation-verified: temporarily reverting to the un-hardened `update()`/`dispatch()` pair (no
  `slot_free_`, unconditional `Give(response_ready_)`) is not itself testable on the host (it is a
  concurrency bug, not a pure-logic one), but `HttpMailbox`'s own suite directly exercises the
  decision the fix depends on, including the exact scenario from the finding
  (`test_a_fresh_dispatch_after_an_abandoned_one_awaits_correctly`).

## 3. Start/stop resource lifecycle

**Finding:** `xSemaphoreCreateBinary()` in `start()`, no matching delete in `stop()` — a repeated
`START -> STOP -> START -> STOP` cycle would leak two semaphore handles per cycle, and a partial
`start()` failure (one `CreateBinary()` succeeding, the next failing, or `httpd_start()` itself
failing) leaked whatever had already been allocated.

**Fix:** `stop()` now unconditionally (and individually-guarded, since `vSemaphoreDelete()` on a
null handle is unsafe) deletes `request_ready_`/`response_ready_`/`slot_free_` and resets `mailbox_`
to a fresh `HttpMailbox{}`; it is idempotent and safe to call with nothing allocated. `start()` now
calls `stop()` defensively before allocating anything (so it can never `CreateBinary()` over a live
handle) and again on every failure path (three-way null check after `CreateBinary()`, and again if
`httpd_start()` itself fails) — a bounded cycle can no longer leak, and a partial failure releases
exactly what it had already claimed.

`scripts/static_audit.py`'s `check_http_transport_boundaries()` gained two structural checks:
`xSemaphoreCreateBinary()`/`vSemaphoreDelete()` call counts must match exactly, and `start()` must
call `stop()` at least twice (the defensive upfront call plus at least one failure-path call).
Mutation-verified: deleting one `vSemaphoreDelete()` call was confirmed to fail the audit
(`3 xSemaphoreCreateBinary() call(s) but 2 vSemaphoreDelete() call(s)`), then reverted.

**Residual, explicitly documented risk (`HARDWARE_TO_TEST`):** `stop()` racing an httpd worker task
genuinely mid-`dispatch()` (i.e. `@WEB SERVER STOP` arriving while a real HTTP client's request is
in flight) has not been exercised against real concurrent load — `httpd_stop()` is expected to tear
down the worker task and its in-flight connections first, but that has not been hardware-validated.
`stop()` itself is only ever called from the Controller thread, so there is no Controller-thread-vs-
itself race; the residual risk is specifically the httpd task losing its semaphore handles out from
under it during shutdown.

## 4. OTA challenge lifecycle hardening

**Finding:** `issueChallenge()` unconditionally replaced the outstanding nonce on every call. An
unauthenticated client hitting `GET /ota/challenge` repeatedly could therefore invalidate a
legitimate client's still-valid, in-flight nonce before that client's signed `POST /ota/update`
ever arrived — an availability weakness, not an integrity bypass (the earlier, already-fixed
nonce-consumption bug this session found meant a *mismatched* nonce never burned the real challenge;
this is the adjacent case of a *matching-context* re-issue doing so instead).

**Fix:** `issueChallenge()` (`src/update/OtaSession.cpp`) is now idempotent while a still-valid
challenge is outstanding — it returns the SAME nonce again rather than rotating, until that
challenge either expires (bounded by `challenge_ttl_ms`, unchanged default 30 s) or is consumed by
a successful-or-failed `authenticate()` call. This is safe because a nonce is not bound to a caller
identity, only single-use-consumed by whichever request first presents a valid signature over it —
sharing visibility of the same in-flight nonce among multiple honest holders of the shared secret is
not a new capability for any of them. Three tests replace the one this superseded
(`test_repeated_challenge_calls_do_not_displace_a_still_valid_nonce`,
`test_challenge_rotates_once_the_previous_one_expires`,
`test_challenge_rotates_immediately_after_being_consumed`), and the fix was mutation-verified:
reverting to unconditional rotation was confirmed to fail
`test_repeated_challenge_calls_do_not_displace_a_still_valid_nonce`, then reverted.

An active, already-authenticated session (`session_active_`) was independently confirmed to already
be structurally immune to a challenge call under both old and new code — `issueChallenge()` never
touches `session_active_`/`last_activity_ms_`, only the pre-authentication `challenge_outstanding_`/
`nonce_` state — so requirement "an active session cannot be displaced by a challenge call" needed
no code change, only this explicit confirmation.

## 5/6. `/status` threat model and OTA security model — documented explicitly

Both fully written out in `src/network/HttpTransport.h`'s class comment (not duplicated here to
avoid the two copies drifting):

- **`/status`**: left unauthenticated, deliberately. Reaching it at all already requires
  `@WEB SERVER START` over USB CDC in MAINTENANCE mode (the same physical-access trust boundary the
  DALY KEY write uses); its content carries no secret; and authenticating it would either reuse the
  OTA HMAC secret (raising read access to write-access trust level, backwards from the required
  ordering) or invent a second credential system for a page with nothing to protect. Chosen as the
  leaner, justified design over both alternatives the operator offered.
- **OTA security model**: HMAC-SHA256 over plain HTTP. Explicitly documented what it DOES protect
  (writer authentication, metadata authenticity, image integrity via the signed-then-streamed-and-
  verified SHA256, challenge/replay resistance) and does NOT (confidentiality of firmware bytes or
  `/status` content; availability against an attacker who can simply drop traffic). No TLS guarantee
  is implied anywhere. Firmware confidentiality was evaluated and found not to be a current MATDOG
  requirement — this is engineering firmware for a private robot on a private LAN — so no TLS/
  certificate lifecycle was added; `esp_https_server` remains confirmed-available and adoptable
  later without changing `OtaSession`'s authentication layer underneath, per the original I7
  transport evaluation.

## Offline verification after this round

- `scripts/tests/run_host_tests.sh`: now 17 suites (`test_hmac256`, `test_ota_session` — 40 checks,
  up from 35 — and `test_http_mailbox`, new), all passing.
- `scripts/tests/test_build_manifest.py`: 76 tests (12 new), all passing.
- `python3 scripts/static_audit.py`: PASS, with the new `check_http_transport_boundaries` structural
  checks and the extended `check_build_profile_provenance` checks, both mutation-verified.
- Both `USB_ONLY` and `ROBOT_POWERED` builds, plus the new `MATDOG_OTA_INGEST_VALIDATION=1` override
  combined with `ROBOT_POWERED`, all compile clean.

## Outcome

All six findings addressed with code, tests and documentation; nothing deferred. Proceeding to the
final complete offline validation matrix and the ONE `MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION
CANDIDATE`, built `ROBOT_POWERED` with `MATDOG_OTA_INGEST_VALIDATION=1`, superseding `4048ce6`.
