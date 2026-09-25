# I2 — LED Status Manager

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

This closes gate **I2** of the MATDOG NextGen software-only integration workflow (V3 handoff,
2026-09-25, §10). Purpose: a higher-level, non-blocking status/indicator manager above the
existing `LedRing` driver, with a single presentation owner, deterministic priority arbitration,
and no invented battery/charger thresholds.

## Architecture table

Reviewed before implementation, per the V3 handoff's explicit instruction not to freeze colors or
precedence blindly.

| STATE | SOURCE | TRIGGER | CLEAR | PRIORITY | EFFECT | STALE DATA | CONFLICT | HARDWARE_TO_TEST |
|---|---|---|---|---|---|---|---|---|
| `FAULT` | `SystemState::systemHealth()` | health == `FAULT` (any module reports `ModuleHealth::FAULT`) | health leaves `FAULT` | 1 (highest) | solid red, full ceiling brightness | N/A — recomputed synchronously every tick from module health, no cached snapshot | wins over every other state unconditionally | yes — never exercised on real hardware |
| `FIRMWARE_UPDATE_IN_PROGRESS` | `ActuatorAuthorityArbiter` | `inhibited() && inhibitReason() == FIRMWARE_UPDATE` (OTA-B exclusivity hold, held for the whole update including after a successful commit until reboot) | inhibit released | 2 | breathing blue (triangle wave, 2 s period, ceiling = `LedRing::kMaxBrightness`) | N/A — authority state is synchronous, no polling staleness | beats calibration/degraded/wifi/booting/ready; loses only to `FAULT` | yes — no OTA hardware session has ever run |
| `CALIBRATION_IN_PROGRESS` | `CalibrationManager::sessionLive()` | session state `PREFLIGHT` or `ACTIVE` (live or `HISTORICAL_REPLAY`) | session ends (`COMPLETED`/`ABORTED`/`FAILED`/`NO_SESSION`) | 3 | breathing violet | N/A — synchronous | beats degraded/wifi/booting/ready; loses to fault/update | yes — no live calibration session has ever run; replay-only is host-tested |
| `DEGRADED` | `SystemState::systemHealth()` | health == `DEGRADED` (a non-critical module reports `DEGRADED`/`OFFLINE`) | health leaves `DEGRADED` | 4 | solid amber, half ceiling | N/A — synchronous | beats wifi/booting/ready; loses to fault/update/calibration | yes |
| `WIFI_CONNECTING` | `WifiManager::status()` | state in `{RADIO_STARTING, CONNECTING}` | state leaves both (reaches `CONNECTED`, `BACKOFF` or `INACTIVE` — falls through to booting/ready, no separate "connected" LED language is defined yet) | 5 | breathing cyan | N/A — `WifiPolicy` recomputes its snapshot every tick; no LED-side caching | beats booting/ready; loses to fault/update/calibration/degraded | yes — Wi-Fi has never associated with a real access point |
| `BOOTING` | `SystemState::systemHealth()` | health == `BOOTING` (a module is still `NOT_INITIALIZED`) | health leaves `BOOTING` | 6 | solid dim white | N/A — synchronous | beats only `READY` | yes |
| `READY` | `SystemState::systemHealth()` | default/fallback — health == `READY`, or an unclaimed value (`MAINTENANCE`, never actually produced today — see `SystemState.cpp`) | superseded by any higher-priority trigger | 7 (lowest) | solid dim green | N/A — synchronous | loses to everything above | yes |

Deliberately **not implemented** in this gate: `CHARGING`, `CHARGE_COMPLETE` (`FULL`), `LOW_BATTERY`,
`CRITICAL_BATTERY`, and a standalone `MAINTENANCE_SERVICE` state.

- **Battery/charging** — `MATDOG_POWER_STATES_AND_CHARGING.md` §14 is explicit that FULL must never
  be inferred from SOC alone and that no reviewed empirical threshold exists yet for full/taper
  detection, low-battery or critical-battery cutoffs. `DalySample` also carries a real staleness
  dimension (`sampled_at_ms`) that every state above deliberately does not have, since every other
  input here is recomputed synchronously each Controller tick. Adding a battery/charging tier
  requires its own reviewed threshold policy first; inventing one here would violate the V3 handoff
  contract directly. The `LedPresentationState` enum and priority list are structured so adding
  these states later is an additive change, not a rewrite — see `LedStatusPolicy.h`'s header
  comment.
- **`MAINTENANCE_SERVICE`** — the candidate family from the V3 handoff §9 envisions an occasional,
  visible operator activity analogous to calibration. No such session exists in current code:
  `ActuatorAuthority::DIAGNOSTICS/QC/PROVISIONING` are real owners but nothing acquires them yet (no
  write-capable command exists — ROADMAP.md stage 6 is `FUTURE`), and `OperatingMode::MAINTENANCE`
  is the permanent default in V0.1/NextGen (no `RUN` transition exists), so keying an LED state off
  it would fire 100% of the time and communicate nothing. Deferred to whichever of I3 (Diagnostics/
  Maintenance) or a later Service/Provisioning/QC gate first introduces a real, occasional session
  concept for these owners.

## Implementation

Reused exactly, unchanged: `src/status/LedRing.*` (WS2812B driver, USB_ONLY anti-back-power
guarantee, `@LED TEST` diagnostic chase). No new LED library, no GPIO47 changes.

New files, following the existing pure-policy/thin-manager split used throughout this codebase
(`network::WifiPolicy`/`WifiManager`, `update::OtaPolicy`/`OtaManager`):

- `src/status/LedStatusPolicy.{h,cpp}` — pure decision core. `<stdint.h>` plus
  `core/SystemState.h` only, no `<Arduino.h>`, no `LedRing` dependency — host-linkable, the same
  contract as `WifiPolicy`/`OtaPolicy`. `selectLedState()` implements the priority table above;
  `ledEffectFor()` computes RGB + a triangle-wave brightness envelope, pure functions of
  `(state, now_ms, max_brightness)` — no delay, no blocking wait, matching the non-blocking
  requirement.
- `src/status/LedStatusManager.{h,cpp}` — the single periodic owner. Every `Controller::update()`
  tick it builds one `LedEffect` from the policy and calls `LedRing::setSolid()` exactly once. It
  steps aside whenever `LedRing::testRunning()` is true, so the existing `@LED TEST` diagnostic chase
  is untouched and remains the one tightly-scoped manual override path. `@LED OFF` still blanks the
  ring for one tick via `LedRing::off()`; the manager's next tick (~20 ms later) resumes the current
  status — a manual diagnostic blip, not a persistent override, because presentation has exactly one
  owner.
- `Controller` gathers the snapshot (`system_state_.systemHealth()`, the authority inhibit reason,
  `calibration_.sessionLive()`, the Wi-Fi state) after every other module has updated for the tick,
  and calls `led_status_.update(now_ms, inputs)` last.
- `@LED STATUS` now also reports `presentation=<state>` — read-only, the manager's last decision,
  never a second source of truth.

### Static audit

New `check_led_status_boundaries()` in `scripts/static_audit.py`:

- fails the build if `LedStatusPolicy.h`/`.cpp` include `<Arduino.h>`, reference `Serial.`,
  `millis(` or `LedRing` — keeping the decision core host-linkable;
- fails the build if any `.cpp` other than `LedStatusManager.cpp` (or `LedRing.cpp` itself, for its
  own method body) calls `setSolid(` — enforcing the single-presentation-owner contract structurally,
  the same style as the existing `check_led_anti_back_power` / `check_actuator_authority` boundary
  checks.

### Offline test suite

New `scripts/tests/test_led_status_policy.cpp`, wired into `run_host_tests.sh` and
`static_audit.py::check_host_tests()` exactly like the other nine suites (links the real
`LedStatusPolicy.cpp`, no mock). **198 checks, 0 failures.** Covers: every solo trigger, full
pairwise priority-conflict resolution, the unclaimed-`MAINTENANCE`-health fail-closed case, solid
states ignoring `now_ms`, exact colors per state, the triangle wave's period/peak/floor/monotonic
shape, the degenerate-ceiling case (`max_brightness <= floor` must not underflow a `uint8_t`),
`toString()` for every state plus a corrupted-value fallback, and the stateful `LedStatusPolicy`
wrapper `Controller`/`LedStatusManager` actually use.

Updated offline baseline: **11 host suites, 5577 checks** (previous baseline 10 suites / 5379
checks, +198 from this gate). Static audit: **PASS, 79 files** (+5 from this gate: the four new LED
status files plus the new test suite), 0 findings.

### Builds

- `USB_ONLY`: PASS — 977,747 B flash (+756 B vs. the I1 baseline of 976,991 B), 52,420 B RAM
  (+16 B vs. 52,404 B).
- `ROBOT_POWERED`: PASS — 978,299 B flash (+828 B vs. 977,471 B).

After the `ROBOT_POWERED` compile-check the default `USB_ONLY` artifact was rebuilt and restored,
as every prior gate in this repository requires.

## Outcome

`I2 = PASS`. LED presentation has exactly one periodic owner, is fully offline-tested, adds no
actuator-authority implication, invents no battery/charger threshold, and preserves the `USB_ONLY`
anti-back-power guarantee unchanged (verified by the unmodified `check_led_anti_back_power` audit
and by `LedStatusManager` never touching `LedRing::begin()`/`pixels_`/GPIO47 directly). Proceeding
to `I3` — Diagnostics/Maintenance software.
