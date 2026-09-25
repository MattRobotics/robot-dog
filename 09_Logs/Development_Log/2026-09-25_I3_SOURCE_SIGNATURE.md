# I3 — Diagnostics/Maintenance: SOURCE_SIGNATURE

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

Closes the `SOURCE_SIGNATURE` item of gate **I3** (V3 handoff §11). Audited first: G4's four
documented gaps are `SYSTEM_SELF_TEST`, `SOURCE_SIGNATURE`, `PROFILE_AUDIT`, and a consolidated
servo health summary. Of these, `SOURCE_SIGNATURE` was the only one with all its inputs already
computed elsewhere in the firmware and no open design question — `build::kBuildId` existed only in
the once-per-boot banner, with no way to query it after boot without a reset. The other three
(`SYSTEM_SELF_TEST`, `PROFILE_AUDIT`, consolidated servo health) each require a design decision
(what "self-test" aggregates beyond what `@STATUS` already reports, what a "profile audit" checks
against `MATDOG_C018_V1` that `@SERVO PREFLIGHT` does not already cover) that is intentionally left
open rather than guessed under gate pressure — restated as explicit `TO_IMPLEMENT` in
`DEVELOPMENT_GATES.md`/`ROADMAP.md` G4 rather than built speculatively.

## What shipped

`@SYSTEM SOURCE_SIGNATURE` — a new read-only command, `MAINTENANCE`-agnostic like `@STATUS` (no
mode gate needed: it issues no bus transaction and blocks nothing). Reports:

- `build::kBuildId`, `kFirmwareName`, `kFirmwareVersion`, `kTestProfile`, `kBoardName` — all
  existing compile-time constants, previously only printed once in the boot banner;
- the OTA manager's `running_build_id`, `running_image_state` and `reset_reason` — already computed
  by `update::OtaManager` for its own first-boot self-check, now also surfaced on demand;
- the ESP-IDF running partition (label/address/size) via `esp_ota_get_running_partition()`, the
  same call already used in `Controller::printBootBanner()`.

No new hardware read, no new bus traffic, no new persistent state. Deliberately does not surface
`esp_app_desc_t` — per the V3 handoff §16.1 / `update/OtaPolicy.h`, that Arduino-core metadata
describes `arduino-lib-builder`, not MATDOG, and is not authoritative identity.

## Verification

- Static audit: PASS, 79 files, 0 findings (no new check needed — this command touches no
  forbidden surface any existing boundary check governs).
- Host suites: all 11 unchanged (this is presentation-only glue in `CommandRouter.cpp`, the same
  category as the existing `printWifiStatus()`/`printOtaStatus()`, neither of which has a dedicated
  host suite either — the underlying data they present is what is host-tested).
- `USB_ONLY`: PASS — 978,195 B flash (+448 B vs. the I2 baseline of 977,747 B), 52,420 B RAM
  (unchanged).
- `ROBOT_POWERED`: PASS — 978,751 B flash (+452 B vs. 978,299 B). Default `USB_ONLY` artifact
  restored afterward.

## Outcome

`I3 (SOURCE_SIGNATURE) = PASS`. `SYSTEM_SELF_TEST`, `PROFILE_AUDIT` and the consolidated servo
health summary remain explicitly `TO_IMPLEMENT` — not attempted this gate, to avoid guessing their
design under autonomous-workflow pressure. Proceeding to `I4` — Safe Actuator runtime boundary.
