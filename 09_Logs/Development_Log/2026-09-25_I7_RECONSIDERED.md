# I7 — Wi-Fi/OTA: reconsidered under the objective change, scope unchanged

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

The operator's objective-change instruction asks to "implement all software-only transport/
security/session/provenance work that can be completed responsibly offline," explicitly allowing
real ingest to stay disabled by default. This reopens the question the earlier I7 audit (same date)
deferred. Reconsidered here; conclusion is the same — no transport is implemented this gate — for a
more specific reason than before.

## Why building USB CDC ingest now would not change the frozen candidate

The frozen candidate ships `MATDOG_OTA_INGEST_ENABLED=0` regardless of whether the ingest transport
exists in source, because the operator's own instruction requires it: *"Real firmware ingest may
remain disabled by default if that is required for the frozen candidate."* Building the transport
now would not make any additional behavior *reachable* in the artifact this session freezes — the
gate stays compiled out either way. Its only value would be "software work advanced for later,"
which is worth doing only if it can be done at genuinely low risk.

## The concrete blocker found this pass

`CommandRouter::kLineBufSize = 96` bytes (`CommandRouter.h`). The entire command surface is built
on one assumption: a command is a short text line, read a few dozen bytes at a time between
`Controller::update()` ticks, with no other read mode. Landing a real firmware image (hundreds of
KB) through this path needs either:

- thousands of tiny (~40-byte) round-trip text commands — impractical, and does not exercise
  anything resembling the eventual real transport's chunk size or timing; or
- a genuine second I/O mode inside `CommandRouter`/`Controller` — a binary-ingest state that
  suspends line parsing and reads length-prefixed bytes directly — which is real, new I/O
  architecture, not a thin adapter over the already-audited `OtaManager` API.

The second option is the only one worth building, and it is exactly the kind of architectural
addition (a second read-mode for the one shared Serial stream, with its own framing and
back-pressure) that was not reviewed in this session's scope and is not obviously "small". Getting
its interaction with the G3.1 non-blocking USB CDC guarantee wrong — the same guarantee
`static_audit.py` fails the build over — has a worse failure mode than almost anything else
advanced this session. Building it under this session's time budget, for zero change in the frozen
candidate's reachable behavior, is not "responsible" by the operator's own qualifier.

## What was re-confirmed instead

- `OtaPolicy::reset()` already provides the retry primitive available without a transport: a
  failed or aborted attempt returns the state machine to `IDLE` (refused only after a boot-target
  commit, correctly — the pending switch is a real fact that must stay reported), and a fresh
  `prepare()` is unconditionally possible again. This is the "failure/retry" semantics that exist
  independent of any transport; already implemented, already in the 561-check OTA-A suite.
- No provenance, session, or security gap was found beyond what the original I7 audit already
  listed as complete (authorization, session semantics, manifest identity, hash validation,
  pending-reboot, recovery invariant, bounded memory, concurrency, status reporting).

## Outcome

`I7` scope is unchanged from the earlier audit: nine of eleven V3 sub-items were already complete;
authenticated transport remains `TO_IMPLEMENT`, now for a documented architectural reason (a second
Serial I/O mode, not yet designed) rather than only "recommendation not yet chosen." Proceeding to
`I8`.
