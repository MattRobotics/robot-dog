# MATDOG Controller — Changelog

## Unreleased — MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE — 2026-09-25

Documentation only; **no code change**. Full integrated software freeze superseding the earlier I9
freeze, run from this exact clean commit after I6 (HostLink), I7 (reconsidered), I4/I5 (Controller
wiring) and I8 (reconsidered):

- C++ host suites: **14 suites, 5726 checks, 0 failures**.
- Static audit: **PASS, 89 files, 0 findings**.
- Python suites: **370 collected, 366 passed, 4 known/justified failures** (identical result to
  I9 — same pre-existing root cause, no Python source changed).
- `USB_ONLY` 981,856 B / `ROBOT_POWERED` 982,432 B (SHA256
  `94ae5c4a5152d914520db579d0282f0df5b540a56b90e9a772e67954e244b6b0`), both clean from commit
  `c8906378df04468d886d6c1d064f67f74a16042b`.
- Fail-closed re-verified, now covering the I4/I5/I6 additions: `ActuatorRuntime`/
  `CalibrationExecutionEngine` are Controller-owned but wired with a `nullptr` backend and no
  geometry/limit/transform ever admitted; no command path reaches `plan`/`commit`/`execute`/
  `abort` (audit-enforced, mutation-verified).

This artifact is named **MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE** per the
operator's instruction — the single build the later physical campaign will validate. Full record:
[`09_Logs/Development_Log/2026-09-25_INTEGRATED_FREEZE_CANDIDATE.md`](../../09_Logs/Development_Log/2026-09-25_INTEGRATED_FREEZE_CANDIDATE.md).

## Unreleased — I8 reconsidered, no server code added — 2026-09-25

Documentation only; **no code change**. Reconsidered the read-only Web foundation now that I6 is
substantially implemented. Split UI-0 into a data/schema half (already done — `ControllerService`
IS the typed semantic/telemetry layer a future HTTP handler would serve) and a server half
(`WebServer`/`esp_http_server`/`esp_https_server`, all requiring `<WiFi.h>` and, for two of the
three, an additional FreeRTOS task). Declined to add the server half: its central safety property
("bounded memory/latency") is not testable without live Wi-Fi association, and unlike
`MATDOG_OTA_INGEST_ENABLED` (which gates an already-reviewed state machine), there is no existing
offline-tested decision core to gate an HTTP stack behind — it would be new, unreviewed surface
area. `ServiceReadiness`'s `WEB_READ_ONLY_DASHBOARD` capability already reports the
"disabled/not qualified until Wi-Fi hardware validation" state as data. Full reasoning:
[`09_Logs/Development_Log/2026-09-25_I8_RECONSIDERED.md`](../../09_Logs/Development_Log/2026-09-25_I8_RECONSIDERED.md).

## Unreleased — I4/I5 Controller wiring — 2026-09-25

**Implemented, compiled and offline-tested. NOT flashed. NOT hardware-tested. Fail-closed by
construction and by audit.**

- **`Controller` now owns real `SafeActuatorPolicy`/`ActuatorRuntime`/`CalibrationExecutionEngine`
  instances** as status/lifecycle infrastructure — previously none of the three were wired into
  `Controller` at all. `actuator_runtime_` is given `nullptr` as its backend (every `ACCEPT`
  resolves to `NO_BACKEND` regardless of anything else); no geometry/limit/transform is ever
  admitted from `Controller`; no command path calls `plan()`/`commit()`/`execute()`/`abort()`.
- **New `@ACTUATOR STATUS`** (read-only): policy epoch, outstanding-transaction flag, last
  decision, counters, limits/transforms-admitted counts, geometry-bound flag.
- **New static-audit check** `check_actuator_infrastructure_wired_fail_closed()` enforces the
  three fail-closed choices above structurally. **A manual mutation check caught a real bug**: the
  first version of the "no plan/commit/execute/abort call" rule matched only `.method(` and missed
  every actual call site, which uses `->method(` (`modules_.actuator_policy` is a pointer) — fixed
  to a regex matching both syntaxes, re-verified via the same injected-then-reverted mutation.
  The I4/I5 boundary checks were extended to allow `Controller.h`/`Controller.cpp` by exact
  filename (not by directory), so `CommandRouter.cpp`/`ControllerService.h` remain excluded.
- **Cost:** `USB_ONLY` flash 979,111 B -> 981,727 B (+2,616 B — no longer dead-code-eliminated,
  since `Controller` now genuinely calls into it), RAM +872 B; `ROBOT_POWERED` 979,655 B ->
  982,299 B (+2,644 B). Full record:
  [`09_Logs/Development_Log/2026-09-25_I4_I5_CONTROLLER_WIRING.md`](../../09_Logs/Development_Log/2026-09-25_I4_I5_CONTROLLER_WIRING.md).

## Unreleased — I7 reconsidered, scope unchanged — 2026-09-25

Documentation only; **no code change**. Reconsidered building the USB CDC OTA ingest transport
under the objective-change instruction (real ingest may stay disabled by default in the frozen
candidate either way). Found a concrete blocker: `CommandRouter::kLineBufSize = 96` bytes means a
usable ingest transport needs a genuine second Serial I/O mode (suspending line parsing for
length-prefixed binary reads), not a thin adapter over the existing `OtaManager` API — real,
unreviewed I/O architecture whose interaction with the G3.1 non-blocking USB CDC guarantee was not
assessed this session. Building it would not change the frozen candidate's reachable behavior
(ingest stays compiled out either way) while carrying real risk. Re-confirmed `OtaPolicy::reset()`
already provides the transport-independent retry primitive. Full reasoning:
[`09_Logs/Development_Log/2026-09-25_I7_RECONSIDERED.md`](../../09_Logs/Development_Log/2026-09-25_I7_RECONSIDERED.md).

## Unreleased — I6 HostLink implementation (candidate objective change) — 2026-09-25

**Implemented, compiled and offline-tested. NOT flashed. NOT hardware-tested.** Supersedes the
earlier audit-only I6 gate, per the operator's instruction to produce one maximally integrated
hardware-validation candidate: sequencing gates activation/validation, not offline preparation of
fail-closed software.

- **New `src/core/ServiceReadiness.{h,cpp}`** — pure readiness classifier. 8 named capabilities x
  hardware-validation flags -> `READY`/`TO_TEST`/`BLOCKED`. 31 offline checks.
- **New `src/core/ControllerService.h`** — transport-neutral telemetry layer, scope bounded to
  passive status reads. ~30 accessors, each a one-line forward of a struct its module already
  computed — zero duplicated logic. Action/write commands stay `CommandRouter`-direct.
- **`CommandRouter` refactored** (not duplicated): every read-only `print*` method now routes
  through `ControllerService`. New `@HOSTLINK READINESS` command surfaces the classifier.
- **New static-audit check** `check_service_readiness_is_host_linkable()`. The pre-existing
  DALY-KEY-probe scope check was extended to allow `ControllerService.h` as a reviewed consumer
  (same reasoning already applied to `CommandRouter`); the full DALY mutation suite (52/52) was
  re-verified green afterward.
- **Cost:** `USB_ONLY` flash 978,336 B -> 979,111 B (+775 B), RAM +64 B; `ROBOT_POWERED`
  978,896 B -> 979,655 B (+759 B). Offline baseline now 14 host suites / 5726 checks. Full record:
  [`09_Logs/Development_Log/2026-09-25_I6_HOSTLINK_IMPLEMENTATION.md`](../../09_Logs/Development_Log/2026-09-25_I6_HOSTLINK_IMPLEMENTATION.md).

## Unreleased — F0 final flash readiness — 2026-09-25

Documentation only; **no flash attempted or proposed**.

- `SOFTWARE_FREEZE_GATE`, `ARTIFACT_PROVENANCE_GATE` and `FAIL_CLOSED_GATE` all **PASS**.
  `POWER_ISOLATION_GATE=UNPROVEN` and `FLASH_RECOVERY_GATE=DEFERRED_UNPOWERED` — unchanged since
  I0, the ESP32 was never enumerated this session (`USB_STATE=ESP32_NOT_ENUMERATED`).
- `FINAL_FLASH_ELIGIBLE=DEFERRED_UNPOWERED` per the handoff's explicit rule. No authorization
  question was asked (Section 19 applies only when `FINAL_FLASH_ELIGIBLE=PASS`).
- Recorded the exact future flash procedure (prove power isolation → fresh full-flash read-back
  under proven power → re-run F0 → Section-19 report → explicit YES/NO → application-only flash via
  `flash_app_only.sh`) so nothing has to be re-derived once the operator is physically present.
  Full record:
  [`09_Logs/Development_Log/2026-09-25_F0_FINAL_FLASH_READINESS.md`](../../09_Logs/Development_Log/2026-09-25_F0_FINAL_FLASH_READINESS.md).

## Unreleased — I9 integrated software freeze — 2026-09-25

Documentation only; **no code change**. Full offline validation matrix run from a clean tree:

- C++ Controller host suites: **13 suites, 5695 checks, 0 failures**.
- Static safety audit (mutation guards, OTA partition tests, build-manifest tests included):
  **PASS, 85 files, 0 findings**.
- Python kinematics/calibration suites (`06_Software/Matdog_Core`): **370 collected, 366 passed, 4
  known/justified failures** — all four are the pre-existing, already-documented live-FK
  `calibration_status` YAML/loader enum mismatch (I1, `CALIBRATION_SOURCE_PRECEDENCE.md` §9 item
  1), not a regression and explicitly not fixed per the V3 handoff's own instruction.
- Both hardware profiles compiled clean from commit `cc0940b0f62f242f0ab66c09ea24f7cb8ed2aa08`:
  `USB_ONLY` 978,336 B / `ROBOT_POWERED` 978,896 B (SHA256
  `a292b2166d5381f1a8f75c494f79753e8aae4a23ee875c42325fe10ecb35203b`). Default `USB_ONLY` artifact
  restored last.
- Fail-closed re-verified directly against source: `hardware_motion_authorized=0`,
  `USB_ONLY` source default, OTA ingest `=0` — all audit-enforced. Neither of this session's two
  new adapters (I4 `ActuatorRuntime`, I5 `CalibrationExecutionEngine`) is referenced by
  `Controller`/`CommandRouter`; both are proven dead-code-eliminated from both compiled profiles.
- Full record, final artifact provenance and classification matrix:
  [`09_Logs/Development_Log/2026-09-25_I9_INTEGRATED_SOFTWARE_FREEZE.md`](../../09_Logs/Development_Log/2026-09-25_I9_INTEGRATED_SOFTWARE_FREEZE.md).

## Unreleased — I8 deferred — 2026-09-25

No code or status change. I8 (read-only Web foundation) requires I6 semantics "stable" (I6 was
audit-only, `HostLink` remains `TO_DESIGN`), Wi-Fi runtime `PASS` (hardware-untested, out of scope
this session) and a chosen web transport (deliberately left undecided in I7). Deferred rather than
attempted with a workaround. Full reasoning:
[`09_Logs/Development_Log/2026-09-25_I8_WEB_FOUNDATION_DEFERRED.md`](../../09_Logs/Development_Log/2026-09-25_I8_WEB_FOUNDATION_DEFERRED.md).

## Unreleased — I7 Wi-Fi/OTA audit — 2026-09-25

Documentation only; **no code change**.

- Checked V3 handoff's eleven Wi-Fi/OTA sub-items against the existing implementation: nine
  (authorization, session semantics, manifest identity, hash validation, pending-reboot,
  recovery invariant, bounded memory, concurrency, status reporting) were already complete.
  Authenticated transport remains deliberately unimplemented — the Controller README already
  evaluates five transport options with a "not yet implemented" recommendation, and building even
  the recommended first step (USB CDC ingest) means writing the first reachable firmware-write
  code path in this codebase, which is a decision-grade commitment, not routine offline
  advancement. `failure/retry` semantics are inseparable from that undecided transport. Full
  reasoning:
  [`09_Logs/Development_Log/2026-09-25_I7_WIFI_OTA_AUDIT.md`](../../09_Logs/Development_Log/2026-09-25_I7_WIFI_OTA_AUDIT.md).

## Unreleased — I6 HostLink audit — 2026-09-25

Documentation only; **no code change**.

- Audited `CommandRouter`'s transport coupling: most read-only state is already
  transport-independent (structured snapshots per module, the G2 pattern); dispatch and
  presentation for all 25 `@COMMAND`s remain embedded in the 792-line USB CDC adapter itself.
  Recorded a design sketch for a future semantic layer, not implemented.
- Scoped this gate to audit-only rather than implementation: `DEVELOPMENT_GATES.md`'s own rule 6
  forbids pulling the HostLink gate forward of its unmet prerequisites (Service/Provisioning/QC,
  formal recalibration), and its `PASS CRITERIA` (a second transport consuming the same semantics)
  is unverifiable without a second transport, which is itself gated behind Wi-Fi hardware
  validation this phase does not authorize. Full reasoning:
  [`09_Logs/Development_Log/2026-09-25_I6_HOSTLINK_AUDIT.md`](../../09_Logs/Development_Log/2026-09-25_I6_HOSTLINK_AUDIT.md).

## Unreleased — I5 Calibration Execution Architecture — 2026-09-25

**Implemented, compiled and offline-tested. NOT flashed. NOT hardware-tested. No production
backend exists. Persistence/promotion remains TO_DESIGN.**

- **Corrected stale documentation**: `DEVELOPMENT_GATES.md` previously said the calibration
  execution engine meant "the 18 recovered phases" — conflating the LF V25 historical oracle with
  the production architecture, which V3 handoff §15.11 forbids. Fixed.
- **New `src/calibration/CalibrationExecutionEngine.{h,cpp}`** — a generic, intent-based
  Calibration Execution boundary (`CalibrationIntent`: `CONTACT_PROBE`/`AUXILIARY_MOVE`/
  `DIRECTION_VERIFY`/`RESTORE`/`ABORT`), routing the three executable intents through the
  unmodified `SafeActuatorPolicy`/`ActuatorRuntime`. `RESTORE` and `ABORT` are categorically
  non-executing — neither ever reaches a backend call, by construction, which is what makes
  "authority loss → zero restore motion" true without a special-cased guard. Owns no session
  state, no persistence, no geometry profile, and never references `CalibrationPhase` — the LF V25
  18-phase sequence stays a historical oracle, exercised only by
  `test_calibration_domain.cpp`'s existing replay.
- **New static-audit check** `check_calibration_execution_engine_boundaries()`: fails the build if
  the engine stops being host-linkable, references the 18-phase sequence, names a torque-removal
  primitive, or is referenced anywhere outside `src/calibration/`/the offline suite.
  `check_actuator_runtime_boundaries()` (I4) was extended to allow this new legitimate consumer.
  Confirmed independently: both hardware profiles compile to byte-identical flash sizes with and
  without the engine present.
- **New offline suite** `test_calibration_execution_engine.cpp`: 72 checks, 0 failures, covering
  all ten operator-specified adversarial cases (authority loss → no restore motion, stale
  generation, diagnostic-endpoint refusal, wrong geometry provenance, replay-origin refusal, no
  accepted transform → no raw target, CALIBRATION-only eligibility, MOTION exclusion, abort/
  restore/SAFE_OFF distinctness, unknown-intent fail-closed). Offline baseline is now 13 host
  suites / 5695 checks (previously 12 / 5623). Full record:
  [`09_Logs/Development_Log/2026-09-25_I5_CALIBRATION_EXECUTION_ENGINE.md`](../../09_Logs/Development_Log/2026-09-25_I5_CALIBRATION_EXECUTION_ENGINE.md).
- **Cost:** `USB_ONLY` and `ROBOT_POWERED` flash/RAM unchanged (978,195 B / 978,751 B).

## Unreleased — I4 Safe Actuator runtime boundary — 2026-09-25

**Implemented, compiled and offline-tested. NOT flashed. NOT hardware-tested. No production
backend exists.**

- **New `src/actuator/ActuatorRuntime.{h,cpp}`** — the runtime adapter `SAFE_ACTUATOR_LAYER.md`
  §7 marks `TO_IMPLEMENT`: an abstract `ActuatorBackend` interface (`enableTorque`/
  `writeGoalPosition`, unsigned tick domain) and `ActuatorRuntime`, which calls
  `SafeActuatorPolicy::commit()` once and issues at most one backend call, only on `ACCEPT`.
  `ActuatorWritePolicy.*` itself is untouched.
- **`ServoBus` still exposes exactly one write, `safeOff()`** — no `EnableTorque(id, 1)`/
  `GoalPosition` primitive exists, so no production `ActuatorBackend` was created. The adapter is
  exercised only offline against a fake backend.
- **The three geometry-authorised operations** (`CALIBRATION_CONTACT_PROBE`/`DIRECTION_VERIFY`/
  `CALIBRATION_AUXILIARY_MOVE`) have no raw-tick target yet — that conversion needs an accepted
  q0/direction transform from a real execution engine (deferred to I5) — and always resolve to
  `ExecuteResult::NO_RAW_TARGET`, never a guessed conversion.
- **New static-audit check** `check_actuator_runtime_boundaries()`: fails the build if the adapter
  stops being host-linkable, or if `ActuatorRuntime` is referenced anywhere outside
  `src/actuator/`/the offline suite. Confirmed independently: both hardware profiles compile to
  byte-identical flash sizes with and without the adapter present — the linker dead-code-eliminates
  it entirely since nothing references it.
- **New offline suite** `test_actuator_runtime.cpp`: 46 checks, 0 failures, including an explicit
  proof that a rejected commit never reaches the backend. Offline baseline is now 12 host suites /
  5623 checks (previously 11 / 5577). Full record:
  [`09_Logs/Development_Log/2026-09-25_I4_ACTUATOR_RUNTIME.md`](../../09_Logs/Development_Log/2026-09-25_I4_ACTUATOR_RUNTIME.md).
- **Cost:** `USB_ONLY` and `ROBOT_POWERED` flash/RAM unchanged (978,195 B / 978,751 B) — the
  adapter contributes zero bytes to either compiled image.

## Unreleased — I3 SOURCE_SIGNATURE — 2026-09-25

- **New `@SYSTEM SOURCE_SIGNATURE`** read-only command: reports `build::kBuildId`, firmware name/
  version, active profile/board, the OTA manager's running-image build id/state/reset reason, and
  the ESP-IDF running partition. All facts were already computed elsewhere (boot banner, OTA
  first-boot self-check) — no new hardware read, no new bus traffic. Closes the `SOURCE_SIGNATURE`
  item of G4's documented gap list. `SYSTEM_SELF_TEST`, `PROFILE_AUDIT` and a consolidated servo
  health summary remain open design questions, deliberately not guessed — see
  [`09_Logs/Development_Log/2026-09-25_I3_SOURCE_SIGNATURE.md`](../../09_Logs/Development_Log/2026-09-25_I3_SOURCE_SIGNATURE.md).
- **Cost:** `USB_ONLY` flash 977,747 B -> 978,195 B (+448 B); `ROBOT_POWERED` 978,299 B -> 978,751 B
  (+452 B). RAM unchanged.

## Unreleased — I2 LED Status Manager — 2026-09-25

**Implemented, compiled and offline-tested. NOT flashed. NOT hardware-tested.**

- **New `src/status/LedStatusPolicy.{h,cpp}`** — pure LED presentation decision core, host-linkable
  like `network::WifiPolicy`/`update::OtaPolicy` (no `<Arduino.h>`, no `LedRing` dependency).
  Deterministic priority: `FAULT` > `FIRMWARE_UPDATE_IN_PROGRESS` > `CALIBRATION_IN_PROGRESS` >
  `DEGRADED` > `WIFI_CONNECTING` > `BOOTING` > `READY`. Solid effects for fault/degraded/booting/
  ready; a pure integer triangle-wave breathing effect for update/calibration/wifi-connecting.
- **New `src/status/LedStatusManager.{h,cpp}`** — the single periodic owner of `LedRing`
  presentation, wired into `Controller::update()` last, after every input it reads has refreshed
  for the tick. Steps aside while the existing `@LED TEST` diagnostic chase is running; reuses
  `LedRing` completely unchanged, including its `USB_ONLY` anti-back-power guarantee.
- **Battery/charging states deliberately NOT implemented**: no reviewed SOC/taper threshold policy
  exists yet (`MATDOG_POWER_STATES_AND_CHARGING.md` §14). Full architecture table and rationale in
  [`09_Logs/Development_Log/2026-09-25_I2_LED_STATUS_MANAGER.md`](../../09_Logs/Development_Log/2026-09-25_I2_LED_STATUS_MANAGER.md).
- **New static-audit check** `check_led_status_boundaries()`: fails the build if the decision core
  stops being host-linkable, or if any translation unit other than `LedStatusManager.cpp` calls
  `LedRing::setSolid()`.
- **New offline suite** `test_led_status_policy.cpp`: 198 checks, 0 failures. Offline baseline is
  now 11 host suites / 5577 checks (previously 10 / 5379).
- **`@LED STATUS`** now also reports `presentation=<state>`, read-only.
- **Cost:** `USB_ONLY` flash 976,991 B -> 977,747 B (+756 B), RAM 52,404 B -> 52,420 B (+16 B);
  `ROBOT_POWERED` flash 977,471 B -> 978,299 B (+828 B).

## Unreleased — NextGen software integration I0/I1 — 2026-09-25

Documentation and repository-topology only; **no firmware change**.

- **I0:** created the single active integration branch/worktree
  `feat/controller-nextgen-integration-v1` from exact H0 HEAD `b95ea31641609fbc29c5d67dd1deb776d59c2504`
  (`feat/h0-current-leg-preflight-v1`), no content commit, pushed with upstream set. `main`,
  `feat/h0-current-leg-preflight-v1` and every other frozen branch/worktree left untouched.
- **I1:** repository-truth reconciliation. Fixed the root `README.md` snapshot banner (stale at
  2026-09-15, now 2026-09-25). Added the H0 `@SERVO PREFLIGHT` capability, previously undocumented
  in `ROADMAP.md` and `DEVELOPMENT_GATES.md`, to both. Carried the six legacy open items from the
  historical `REPOSITORY_VERIFICATION_INDEX.md` (2026-08-11) snapshot forward into
  `CALIBRATION_SOURCE_PRECEDENCE.md` §9 as explicit, unresolved, owned items. Full record:
  [`09_Logs/Development_Log/2026-09-25_I1_REPOSITORY_TRUTH_RECONCILIATION.md`](../../09_Logs/Development_Log/2026-09-25_I1_REPOSITORY_TRUTH_RECONCILIATION.md).

## Unreleased — post-rewire power validation & external USB service port closeout — 2026-09-24

Documentation and evidence only; **no firmware change**. No firmware commit since `6322563`
(2026-09-19) touched source (`4604e36`/`efba2dc`/`19fe837` are test-only or docs-only), so the
robot exercised in this session ran the already-installed powered firmware from the earlier live
DALY work. Full evidence:
[`09_Logs/Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md`](../../09_Logs/Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md).

- **Hardware `B-`/`P-` bypass corrected and verified:** TECNOIOT `VIN-` now returns to DALY `P-`
  instead of raw battery `B-`. Post-rewire power gate A–E passed live: with no charger and no USB
  present, physical KEY OFF now removes the entire protected robot domain (servo rail, TECNOIOT
  output and PAD+→PAD- all measured 0 V), and the powered no-motion regression (BNO085/DALY/servo
  census/`SAFE_OFF`) still holds.
- **Manual charging common-port behaviour discovered and documented:** a charger connected across
  `B+`/`P-` backfeeds that bus independent of KEY/Discharge-MOS state — KEY OFF while a charger is
  connected does **not** de-energize the robot. The previous "manual charging with KEY OFF → robot
  domain OFF" description was never live-verified and is now corrected; see
  `04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md` § 8 for the renamed `MANUAL_CHARGE_KEY_OFF`
  state and its accurate semantics.
- **External USB service/programming port (GPIO19 D-, GPIO20 D+, GND, no host VBUS) validated:**
  native enumeration, bidirectional CDC, and the `esptool` reset/flash-identification path all
  confirmed through the external connector alone, with correct re-enumeration after reset. The port
  cannot power the ESP32 on its own.
- Remaining open: BMS KEY-configuration persistence across a true DALY power cycle (**TO_TEST**);
  autonomous dock/contact hardware, reverse-polarity protection, unattended charge
  acceptance/termination, future Jetson charging (**FUTURE**); charging LED-ring progress
  indication (**FUTURE / TO_DESIGN, NOT IMPLEMENTED** — no firmware for it exists).

## Unreleased — calibration foundation + LF V25 oracle recovery — 2026-09-21

**Implemented, compiled and offline-tested. NOT flashed. NOT hardware-tested.**
**No write path was added**: the firmware's only actuator write is still `EnableTorque(id, 0)`
inside `safeOff()`. The installed robot's calibration remains
`CALIBRATION_RESET_PENDING_FULL_RECALIBRATION` and hardware motion remains **BLOCKED**.

- **C0 evidence audit recorded** in `CALIBRATION_SOURCE_PRECEDENCE.md`: source precedence,
  current calibration truth, what LF V25 actually proved, the three evidence vocabularies, four
  discrepancies, the Generic V25 component assessment and the EEPROM boundary.
- **New `src/calibration/`** — a pure, host-linkable domain model recovered from the archive
  (no Arduino runtime, no ServoBus, no Wi-Fi, no OTA) and a `CalibrationManager` session
  foundation over the real `ActuatorAuthority`, creating no second lock.
- **Confirmed from the archive, not assumed:** 24 contact profiles (4x3x2, and the archive's own
  test asserts it), 58 sequence steps, six LF contacts, 18 execution phases.
- **Three findings encoded as types.** q0 cannot default to the raw servo centre - 2048 is three
  different quantities and LF V25's measured q0 was 2067/2040/2074. There is no "direction
  witness" in the archive; `direction` is a static spec constant, and the witness that exists is
  the CONTACT witness. Evidence is keyed by physical unit, never bus id, because unit M11 is
  NECK_PITCH today while bus id 11 still means "LF lower".
- **The 24-tick witness band is historical and LF-only**, kept in the fixture rather than
  promoted to a universal domain constant - the evidence file forbids mirroring LF onto RF/RH/LH.
- **"H1" avoided in new code.** The repository uses it for both the Full-Leg population gate and
  the Controller's own boot test; new code says leg population gate. It evaluates evidence and
  never scans - there is one bus discovery path and a second census is forbidden.
- **LF V25 replayed offline and MATCHED**, including the one documented failure. Every replayed
  record carries `HISTORICAL_REPLAY`, which `mayPromote()` refuses.
- **Surface:** `@CALIBRATION STATUS`, read-only, leading with the stale/blocked verdict, plus a
  `calibration` line on the boot banner. No START/RUN/MOVE command was added.
- **Audit gains twelve calibration guards**, all twelve mutation-verified.
- **Cost:** flash 970,127 B -> 973,463 B (+3,336 B, 30% of the 3 MB slot); static RAM 51,740 B ->
  51,812 B (+72 B).

## Unreleased — ActuatorAuthority + OTA-B authorization — 2026-09-21

**Implemented, compiled and offline-tested. NOT flashed. NOT hardware-tested.**
**No new write path was added** — the firmware's only actuator write is still
`EnableTorque(id, 0)` inside `safeOff()`, nothing can acquire an owner yet, and there is
deliberately no command that does.

- **New `src/core/ActuatorAuthority.*`** — the single central arbiter of actuator write
  authority. Pure and host-linkable: no Arduino runtime, no `ServoBus`, no `Serial`. Exactly one
  instance exists, owned by `Controller`, reset to `NONE` at boot; no component caches its value.
  Both are audit-enforced.
- **Orthogonal to `OperatingMode`, and enforced as such.** `MAINTENANCE` hosts the service owners
  but not `MOTION`; `RUN` hosts `MOTION` and nothing else. A mode change that strands an owner
  clears it instead of leaving a suspended authority — a no-op today, added so the first real
  owner does not have to remember it.
- **`SAFE_OFF` is outside arbitration, structurally.** `ServoBus` has no reference to the arbiter
  and the arbiter has none to `ServoBus`, so `safeOff()` cannot consult an authority even if a
  later edit wanted it to. The audit fails the build if `ServoBus` names one, or if the
  `@SERVO SAFE_OFF` branch gains an authority or mode condition.
- **Read-only diagnostics deliberately take no lock.** `@SERVO SCAN`/`CENSUS`/`READ` are
  `Ping`/`readByte`/`readWord` and are `MAINTENANCE`-gated because they **block**. The arbiter
  prevents write conflicts; it does not serialize reads.
- **Two semantics chosen deliberately.** A same-owner re-request returns `ALREADY_OWNED` and
  issues **no** second lease — a second valid lease would let two holders each believe they own it
  and either release it out from under the other. And leases carry a generation, which catches the
  case owner-matching cannot: the *same* owner across two sessions, where a late callback from the
  first would otherwise clear the second.
- **OTA-B authorization implemented; the OTA-A placeholder removed, not kept.** `OtaStageAGate`
  and `PERMITTED_OTA_A_NO_AUTHORITY_MODEL_YET` are gone and the audit fails the build if either
  name reappears. `src/update/OtaAuthorityGate.*` is backed by the real arbiter.
- **OTA is not an actuator owner** and no OTA entry was added to the enum — audit-enforced. It
  takes an **exclusivity inhibit** instead, because it requires that nobody is using the actuators
  rather than competing for them.
- **The TOCTOU analysis, and what it did NOT require.** `if (authority == NONE) { start OTA }` is
  genuinely insufficient — not because of threading (MATDOG's Controller is single-threaded, so a
  check-then-act inside one call is already atomic) but because of **duration**: an update spans
  thousands of loop passes. The fix is a hold, not a query: `requestInhibit()` performs the check
  and takes the hold in one arbiter call. **No `SystemActivity` layer was needed and none was
  built.** The hold is released on every failure, abort and reset, and kept after a successful
  commit until the reboot.
- **Surface:** `@AUTHORITY STATUS` (read-only), `authority=` on the existing `@STATUS` SYSTEM line
  (~24 B, keeping the worst single-pass burst at ~2782 B against the 3072 B TX ring),
  `actuator_authority` on the boot banner, and `@MODE MAINTENANCE|RUN` now telling the arbiter.
- **`core/OperatingMode` became host-linkable** (`<stdint.h>` instead of `<Arduino.h>`); it only
  ever needed `uint8_t`, and the arbiter consults it while staying testable off the device.
- **Offline suites:** 751 new checks for the arbiter, including the exhaustive 20-pair conflict
  matrix with each challenger tried in its own legal mode, corrupted-enum fail-closed, stale-lease
  refusal, force-clear under every reason, and the full inhibit lifecycle. The OTA suite grew
  468 → 561 and now links the **real** gate and the **real** arbiter.
- **Audit gains eleven guards**, all eleven mutation-verified.
- **Cost:** flash 967,915 B → 970,127 B (+2,212 B, 30% of the 3 MB slot); static RAM 51,676 B →
  51,740 B (+64 B).

## Unreleased — OTA-A update core — 2026-09-21

**Implemented, compiled and offline-tested. NOT flashed. NOT hardware-tested** — no device has
received an OTA image, no otadata has been written, no rollback has been observed. OTA-A is the
update **core only**: no transport, no authentication, and byte ingest compiled out by default.

- **New `src/update/` module**, split the way `network/` already is: `OtaPolicy.*` (pure update
  state machine), `OtaBootGuard.*` (first-boot rollback lifecycle), `Sha256.*` (image identity) —
  all Arduino-free and ESP-IDF-free — behind an `OtaBackend` interface implemented by
  `OtaEspBackend.*`, the only translation unit in the firmware that calls `esp_ota_*`.
- **The inactive-slot rule is structural, not documentary.** `target != running`,
  `subtype ∈ ota_0..ota_15` and `image_size ≤ target.size` are explicit refusals rather than
  inferences from a backend error; `OtaEspBackend::setBootPartition()` re-reads
  `esp_ota_get_running_partition()` and refuses independently; and `commitBootTarget()` is the one
  method that moves the boot target, with one call site, reachable from exactly one state.
  `flash_app_only.sh` was audited for reusable logic and deliberately **not** adopted as the OTA
  writer — it writes the **active** slot over USB and is a different guarantee.
- **Five kinds of verification kept distinct** — transport integrity, image validity
  (`esp_ota_end`), cryptographic hash identity (our SHA-256), firmware/build identity
  (`build::kBuildId`), bootloader validity (otadata). Only the third distinguishes "a valid image"
  from "the expected image", and the host suite proves it with a mismatched image of identical
  length.
- **`esp_app_desc_t` rejected as firmware identity**, on evidence: parsing the real binary shows
  `version='ee57070'`, `project_name='arduino-lib-builder'`, `date='Jul 20 2026'` — the core's
  identity, not MATDOG's. The existing scheme (`build::kBuildId` + the manifest's
  `APPLICATION_SHA256`/`APPLICATION_SIZE`) is reused; no second version scheme was invented.
- **First-boot confirmation is earned, not granted.** With `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`
  in the real build, never confirming is the safe default — the bootloader aborts a
  `PENDING_VERIFY` image on the next boot by itself. `esp_ota_mark_app_valid_cancel_rollback()` is
  called only after Controller init completed, CommandRouter is bound, identity is readable, the
  boot did not follow PANIC/WDT/BROWNOUT, and the image survived ≥ 15 s **and** ≥ 2000 loop ticks.
  The audit fails the build if `Controller::begin()` ever confirms an image.
- **Refusal does not reboot.** Declining to confirm is already sufficient; rebooting a robot is not
  OTA-A's decision. Explicit operator rollback is **OTA-B**.
- **OTA-B boundary without a mini-authority.** `OtaAuthorizationGate` fails closed with no gate
  installed; OTA-A permission is a named object (`OtaStageAGate`) whose verdict is literally
  `PERMITTED_OTA_A_NO_AUTHORITY_MODEL_YET`.
- **Security stated as a compile-time fact.** `MATDOG_OTA_INGEST_ENABLED` defaults to `0` and the
  audit fails the build if the source default changes — same shape as the `USB_ONLY` profile gate —
  so an unauthenticated firmware writer cannot reach a production image.
- **Erase strategy chosen for loop responsiveness.** `esp_ota_begin()` uses
  `OTA_WITH_SEQUENTIAL_WRITES`: an explicit size erases ~1 MB up front, seconds of blocking. The
  size bound stays an explicit policy check instead of being delegated to the erase argument.
- **Offline suite:** 468 checks against the real state machine with a fake backend — every target,
  metadata, stream and verification failure; the ordering property that the boot target never moves
  from any state but `IDENTITY_VERIFIED`; replay, idempotent abort and state-machine reset; and the
  full rollback lifecycle. SHA-256 checked against FIPS 180-4 vectors, at eight chunk sizes, and
  byte-for-byte against `sha256sum` on the real 959 KB binary.
- **Audit gains thirteen OTA guards**, each mutation-verified. One initially escaped because the
  mutation hit the comment documenting the erase strategy rather than the call; re-run against the
  real call site, it fires.
- **Cost:** flash 959,043 B → 967,915 B (+8,872 B, 30% of the 3 MB slot); static RAM 50,868 B →
  51,676 B (+808 B). No image is ever held in RAM.

## Unreleased — W1 Wi-Fi station runtime — 2026-09-21

**Implemented, compiled and offline-tested. NOT flashed. NOT hardware-tested** — no MATDOG build
has associated with an access point. Wi-Fi is a network **link** here and nothing else: no server,
no endpoint, no remote command, no update path.

- **New `src/network/` module**, split the way the project already splits `power/` and `servo/`:
  `WifiPolicy.*` is a pure, `<Arduino.h>`-free and `<WiFi.h>`-free lifecycle state machine driven
  purely by `(now_ms, link_up)`; `WifiManager.*` is the sole owner of the radio and the only
  translation unit that includes `<WiFi.h>`. `WifiStatus` is a plain copyable snapshot, so
  `CommandRouter` formats it without querying the radio and a future Web adapter renders the same
  struct without a second hardware path.
- **Two non-obvious decisions, both forced by the platform.** The first state is `INACTIVE`, not
  `DISABLED`, because `<esp32-hal-gpio.h>` `#define`s `DISABLED`; the host suite now passes
  `-DDISABLED=0x00` so the clash is caught off-device, as the DALY suite already did.
  `RADIO_STARTING` exists because `WiFi.begin()` reaches
  `waitStatusBits(ESP_NETIF_STARTED_BIT, 1000)` in `esp32:esp32 3.3.11` — a blocking wait of up to
  **one second**. Starting the driver and the association on separate ticks makes that wait
  unreachable.
- **Bounded runtime is measured, not asserted.** `@WIFI STATUS` reports `last_us`/`max_us` for
  `WifiManager::update()`. Those numbers do not exist yet; they need the hardware test.
- **Retry policy owned by MATDOG.** The core's auto-reconnect is turned off, so the doubling
  ladder (2 s → 60 s, reset on success, restarted from the bottom after a link that was up drops)
  is the single description of what actually happens. `persistent(false)` keeps the passphrase out
  of NVS and out of a reconnect-loop flash-wear path.
- **Credentials outside Git.** The repository had no convention; this adds the smallest one:
  `-D` build flags, else a gitignored `src/config/WifiCredentials.local.h`, else **empty** — and
  empty is supported, building and booting normally with the radio never started
  (`state=INACTIVE fault=NO_CREDENTIALS`). `config::kWifiPassword` is named in exactly one place.
- **Audit gains eleven Wi-Fi guards**, each verified by breaking it on purpose: host-linkability
  and `Serial`-freedom of the policy layer; no `waitForConnectResult`, blocking `WiFi.disconnect()`,
  blocking scan, `WiFi.SSID()` poll, `delay()` or `while` loop in the Wi-Fi tick; never
  `setMode(` from a network unit; one and only one passphrase reference; no secret-shaped field in
  the snapshot; the `.gitignore` rule present as an **exact active line** (the first version of
  that check passed on a comment, which the mutation test found); and the local header never
  tracked.
- **Offline suite:** `test_wifi_policy.cpp` links the real state machine — credential gate,
  two-phase start, one transition per tick, connect deadline, backoff ladder and ceiling,
  reset-on-success, link loss, operator enable/disable, fail-closed action failures, the invariant
  that a connect is never issued while connected, `millis()` wraparound, IPv4 formatting and its
  bounds.
- **Cost, same FQBN and profile, against frozen `19fe837`:** flash 392,468 B → 959,051 B
  (12% → 30% of the 3 MB slot); static RAM 28,536 B → 50,868 B (8% → 15%). The ~40–50 KB the
  driver allocates at first `WiFi.mode()` is heap and is not in those figures.
- **Wi-Fi contributes nothing to `SystemState` health.** A missing access point is not a robot
  health fact, and the G3/G3.1-validated meaning of `SYSTEM health=` must not change because a
  router rebooted. Whether it ever should is **TO_DESIGN**.

## Unreleased — power/KEY architecture closeout — 2026-09-20

Documentation and evidence only; **no firmware change** (the shipped Controller already satisfies
every frozen policy — audited 2026-09-20).

- **Live KEY write recorded:** `0x0120 := 0x005A` was sent once on 2026-09-19, acknowledged
  (`51 06 01 20 00 5A 05 97`) and read back as `0x005A`, with no BMS restart needed. Persistence
  across a BMS power cycle remains TO_TEST.
- **New canonical owner:** `04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md` — power domains
  (`B-` to the DALY only, every load on `B+`/`P-`), KEY = discharge MOS only, Charge MOS normally
  ON, the full power-state table, daily use, storage, service isolation, manual charging and the
  future docking/charging and Jetson behaviour, each with its validation status.
- **New blocker recorded:** the physical KEY test was inconclusive because of a hardware `B-`/`P-`
  bypass (TECNOIOT `VIN-` on raw `B-`). KEY OFF is not a trusted power-off until that rewire and
  its validation; the fused disconnect remains the trusted isolation.
- Charging hardware is a separate OPEN gate — no charger/dock evidence exists.
- The one-time commissioning write is **retained** as tightly gated re-commissioning
  functionality: with the register now `0x005A` it answers `ALREADY_CONFIGURED` and transmits
  nothing, so it can only act on a replaced or factory-reset BMS.
- Host suite gains a post-commissioning regression: the current `0x005A` state can never produce a
  write, on arrival or at the pre-transmit re-check.

## Unreleased — DALY KEY write: live-mode pre-transmit re-check — 2026-09-19

Review fix; **not flashed at commit time; still no DALY write sent.**

- The FC06 KEY write can now leave the UART only if the operating mode is **still MAINTENANCE at
  the final pre-transmit check**. `DalyBms::update(now_ms, mode)` receives
  `operating_mode_.mode()` from the Controller every loop; the pure
  `dalyKeyWritePreTransmitCheck()` uses it instead of a literal `true`. A switch to RUN after the
  command was accepted cancels the write with zero bytes sent (`reason=NOT_IN_MAINTENANCE_MODE`).
- Host regression test added (DALY suite 404 checks); audit rule + 5 new mutation cases (52/52).

## Unreleased — DALY KEY discharge configuration (guarded write) — 2026-09-19

**Not flashed; never sent to hardware; live validation TO_TEST.**

- **One semantic DALY write:** `@BMS KEY SET DISCHARGE CONFIRM` (MAINTENANCE only, no argument,
  once per boot) sends FC06 `81 06 01 20 00 5A 16 07` — KEY logic `0x0120 := 0x005A` (DISCHARGE) —
  the exact frame DALY BMSTool V1.14.79 builds (static IL analysis: address `0x81`, big-endian
  register/value, CRC-16/MODBUS). The acknowledgement must be the exact echo
  `51 06 01 20 00 5A 05 97`; an FC03 read-back always follows and only `0x005A` read back is
  `VERIFIED` (`0x0055` → `PENDING_RESTART`, anything else → `MISMATCH`).
- **Fail-closed preconditions**, checked on arrival and again just before transmitting: MAINTENANCE,
  no earlier write this boot, idle bus, fresh `0xD2` telemetry, no alarms, a successful KEY read
  ≤ 30 s old showing exactly `0x0055` with MOS control `1`/`1`. `0x005A` already →
  `ALREADY_CONFIGURED`, zero TX. `@BMS KEY WRITE STATUS`: cached, zero TX.
- `DalyBusScheduler` serves one operator transaction (KEY read or write) at a time; telemetry
  resumes after success, failure or timeout. Nothing persists on the ESP32.
- Static audit: `DALY_THE_ONE_WRITE` is the only admitted write; FC10, other registers (MOS control
  included), values, addresses, a second write, caller-supplied targets and persistence fail. 47/47
  mutation cases; DALY host suite 365 checks. `DalyProtocol.h` no longer calls the live-verified
  read map unvalidated.
- Boot banner now reads `daly_write : KEY_LOGIC_DISCHARGE_ONLY (operator command; no MOS/power-cut
  write)`.
- Unchanged: `requestDischargeOff()` (no-op), `@SYSTEM SHUTDOWN` → `POWER_CUT_FAILED`. No restart,
  no rollback command (rollback `0x0055` documented only).

## Unreleased — DALY KEY live read-only validation — 2026-09-19

Documentation only; no firmware change. Records the live validation of `a57fcdd`
(ROBOT_POWERED, application-only flash, byte-identical clean rebuild, SHA-256 `7c0d5d35…c6ba`).

- **DALY 0x81 read personality — PASS:** one `@BMS KEY READ` returned a CRC-valid 245-byte reply.
- **Current KEY logic: DISABLED (`0x0055`)** — explains why the physical KEY did not switch the
  discharge MOS in G3. Charge/discharge MOS control `1`/`1`; sleep time 360 → 3600 s (the manual's
  default).
- **Telemetry resumption — PASS:** `0xD2` `comm=OK` after the probe; `runtime_resets` 0 throughout.
- Candidate `0x0120 = 0x005A` **not written, not validated**; DALY configuration write still
  **BLOCKED**; the physical KEY is still **not** a validated safety barrier.
- No DALY write, KEY toggle, MOS command, servo command or motion.

## Unreleased — DALY KEY read-only probe — 2026-09-19

**Not flashed; live validation TO_TEST.** No DALY write, KEY toggle or power-state change.

- **Research:** public DALY documents publish no K-series KEY register. DALY's official BMSTool
  V1.14.79 (static inspection, never run) reveals a second Modbus personality (`0x81` → reply
  `0x51`) with KEY logic at `0x0120`, charge/discharge MOS control at `0x0121`/`0x0122` and sleep
  time at `0x0115` — not yet live-validated on MATDOG's unit.
- **`@BMS KEY READ`** (MAINTENANCE only): one FC03 read `81 03 01 00 00 78 5B D4`, reply validated
  (245 bytes, `51 03 F0`, CRC), result reported asynchronously. **`@BMS KEY STATUS`**: cached
  snapshot, zero bus traffic. Neither takes an argument.
- New Arduino-free `power/DalyProtocol`: the two whitelisted read frames, CRC, validation, the
  `0xD2` telemetry decoder (moved unchanged), the KEY decoder and `DalyBusScheduler` — one
  transaction owner, so telemetry and the KEY read never overlap; a KEY read defers at most one
  telemetry poll and polling resumes on its own.
- `requestDischargeOff()` unchanged (no-op, `false`); `@SYSTEM SHUTDOWN` still resolves to
  `POWER_CUT_FAILED`. Candidate `0x0120 = 0x005A` is **not** implemented.
- Static audit: DALY may transmit only the two whitelisted FC03 frames through one
  `bms_uart_.write()`; FC06/FC10, any other frame, `bms_uart_` use or UART2 route, and any
  argument-taking `@BMS` command fail. New mutation suite (24/24) and DALY host suite
  (214 checks). G3.1 ring floor 2560 → 3072 bytes (largest loop pass now 2588 bytes).

## Unreleased — G3 / G3.1 live closure — 2026-09-18

Documentation only; no firmware change. Records the live validation of `e2fc605`.

- **G3 formal PASS:** a second read-only `@SERVO CENSUS` matched the first exactly — census stable
  across repeats. The complete first-boot banner now shows `startup_motion` / `startup_torque` /
  `startup_servo_scan : DISABLED` directly.
- **G3.1 PASS:** with the host port closed 62 s after a host had opened and closed it, BNO085 RV
  ran at 50.10 Hz (0.68 Hz before the fix); 50.13 Hz open; 50.11 Hz with `@BMS STREAM` enabled;
  `runtime_resets` 0; every command reply complete once the backlog had drained.
- No commanded servo motion occurred and no robot motion was observed during validation.
- DALY `KEY` recorded as OPEN (not a validated shutdown barrier); GPIO19/20 stay native USB D−/D+.
- Next: DALY KEY investigation, then G4 — Diagnostics / Maintenance.

## Unreleased — G3.1 CDC-independent Controller loop — 2026-09-18

Fix for a regression found **live after the G3 powered session**. **Not flashed; live re-test TO_TEST.**
No servo, BMS, LED, KEY or power state was changed to produce it.

**Finding.** Zero-TX passive A/B on one boot, USB cable attached throughout: with the host's
CDC port closed, BNO085 rotation-vector processing fell to **0.68 Hz**; with it open,
**50.07 Hz** (configured 50 Hz). `runtime_resets` stayed 0. Autonomous Controller execution
depended on a host keeping the port open.

**Root cause (installed `esp32:esp32 3.3.11` `cores/esp32/HWCDC.cpp`).** HWCDC latches
`connected = true` on the first host transfer and clears it only on a USB bus reset or SOF
loss; closing the host tty is neither. In that state `HWCDC::write()` waits up to
`tx_timeout_ms` (100 ms) × 20 attempts ≈ 2 s per call on a full 256-byte TX ring.
`operator bool()` reports the same latched flag, so `if (Serial)` cannot guard it. The IMU
services one SH2 event per loop pass, so the stalls were lost acquisition.

- `src/core/Controller.cpp`: `Controller::begin()` calls `Serial.setTxBufferSize(3072)` and
  `Serial.setTxTimeoutMs(0)` before `Serial.begin()`. With timeout 0 every wait in
  `HWCDC::write()` becomes an immediate drop, so USB CDC transmit cannot stall the
  Controller. The ring holds the largest single loop-pass burst (2395 bytes): replies are
  expected complete while a host is reading and draining. Right after reopening a port that
  was closed for a long time, a stale backlog may still occupy the ring and a reply may
  short-write instead of block — accepted for this diagnostic surface, **not** a HostLink
  guarantee. `#error` guards pin hwcdc + CDC-on-boot and core 3.3.11.
- `src/config/BuildConfig.h`: `kUsbTxTimeoutMs = 0`, `kUsbTxRingBytes = 3072`.
- `scripts/static_audit.py`: `check_usb_cdc_tx_never_blocks` — timeout exactly 0, ring
  ≥ 2560, both set before `Serial.begin()` and any output, exactly one call each, guards
  present, no `Serial.flush()` / debug-output routing. 11/11 regression mutations caught.
- Docs: G3 powered no-motion evidence PASS, formal census-repeat criterion outstanding; G3.1
  gate added (`FAIL → FIX UNDER VALIDATION`); stage 4 onward and all motion `BLOCKED` until
  G3.1 PASS.

## Unreleased — pre-G3 provenance closure — 2026-09-16

Final pre-G3 closure amendment. **No hardware was flashed, no rail energized, G3 not
executed.** The compiled-in default profile remains `USB_ONLY`.

**Finding A — the recorded build FQBN went unverified.**
The build manifest has recorded `FQBN` since it was introduced, but `build_manifest.py
verify` never compared it against the FQBN `flash_app_only.sh` pins. The right source
compiled with the wrong toolchain configuration is still the wrong artifact: the FQBN
carries the partition scheme, flash size/mode, PSRAM mode, USB/CDC mode and CPU
frequency, and a partition-scheme change silently relocates the application partition.

- `scripts/build_manifest.py`: `verify_manifest()` now takes a required `expected_fqbn`
  keyword (no default) and refuses on exact inequality with the new stable reason
  `FQBN_MISMATCH`, whose detail names both the manifest value and the expected value.
  The CLI gains a required `--expected-fqbn` and reports `VERIFIED_FQBN` on success.
- `scripts/flash_app_only.sh`: passes its own pinned `"$FQBN"` into the verifier and
  surfaces the verified value in the pre-write report. The FQBN itself is unchanged.
- A successful verification now positively proves source commit + clean build state +
  clean current tree + application filename + exact binary size + exact binary SHA256 +
  hardware profile + FQBN all belong to the artifact being authorized.
- No existing flash protection was weakened: backup size/digest, device MAC,
  partition/otadata verification, rollback/anti-rollback, static audit, single
  application-partition write and post-write `verify-flash` are all retained.
- `scripts/tests/test_build_manifest.py`: 36 → **51 tests**, adding canonical-FQBN PASS,
  differing-FQBN `FQBN_MISMATCH`, per-option drift (partition scheme, flash size, PSRAM,
  CPU frequency, flash mode, CDC mode), exact-not-substring comparison, empty/missing
  FQBN falling to `MANIFEST_INCOMPLETE`, refusal ordering, and both profiles passing
  under their own authorization with the canonical FQBN.
- `scripts/static_audit.py`: new tripwires for a deleted FQBN comparison, a permissive
  `expected_fqbn` default, a defaulted `--expected-fqbn`, a removed `FQBN_MISMATCH`
  reason, and `flash_app_only.sh` no longer passing its pinned `"$FQBN"`. All six
  mutation-tested.

**Finding B — Development Gates could be read as reordering the roadmap.**
The HostLink gate's ENTRY read "Diagnostics/Maintenance foundation present", which could
be taken as authorizing HostLink immediately after Diagnostics, ahead of Service/
Provisioning/QC, calibration integration and formal recalibration.

- [`DEVELOPMENT_GATES.md`](DEVELOPMENT_GATES.md): HostLink ENTRY now requires all
  preceding roadmap stages through formal recalibration, and a new global rule 6 states
  that a gate's technical prerequisites never override
  [`ROADMAP.md`](../../01_Docs/02_Architecture/ROADMAP.md) sequencing — reordering
  requires an explicit reviewed roadmap change. The roadmap sequence itself is unchanged
  and no HostLink implementation was started.

**Finding C** — `ARCHITECTURE.md` header date corrected to 2026-09-16, the date of the
approved Embedded Web UI decision it now contains.

## Unreleased — G2 pre-G3 hardening amendment — 2026-09-16

Resolves two issues raised by an independent review of commit `34afbc7`. Both are
pre-G3 corrections to the G2 software scope. **No hardware was flashed, no rail was
energized, and G3 was not executed.** The compiled-in default profile remains `USB_ONLY`.

**Finding 1 — the build profile was not bound to the flashed artifact.**
`build.sh` can emit a `USB_ONLY` or a `ROBOT_POWERED` image from the same commit to the
same path, and the build id is identical for both — so the pre-existing flash gate could
not tell them apart. A stale image of the wrong profile could have been written under the
wrong assumption. (Confirmed concrete: the two profiles really do produce different
binaries — 387312 vs 387776 bytes, different digests.)

- `scripts/build_manifest.py`: new. Writes and verifies a build manifest binding source
  commit, build id, clean/dirty source state, hardware profile, FQBN, and the binary's
  filename/size/SHA256. Pure logic plus a thin CLI; no device I/O.
- `build.sh`: emits `build/esp32.esp32.esp32s3/matdog_build_manifest.txt` after every
  successful compile, and removes any stale manifest if no binary was produced. The
  manifest lives in the gitignored build directory and is never committed.
- `flash_app_only.sh`: fail-closed manifest gate before any device write. Requires
  manifest present/parseable/known-version, commit == `HEAD`, tree clean at build time
  *and* now, binary present with matching size **and** SHA256, recognized profile, and
  manifest profile == requested profile. Profile is named via `MATDOG_FLASH_PROFILE`
  (default `USB_ONLY`), so a `ROBOT_POWERED` image is refused unless explicitly asked
  for. The verified profile is printed in a banner immediately before the write.
- No pre-existing protection was weakened: backup size/digest, device MAC, verified
  application partition, rollback/anti-rollback, static audit, single-partition write and
  post-write `verify-flash` are all retained and now individually audited.

**Finding 2 — `SystemHealth::READY` was unreachable under `ROBOT_POWERED`.**
`classify()` converted `DetectedState::UNKNOWN` into a verdict, treating "nothing has
established anything" the same as "we asked and it did not answer".

- `core/Availability.cpp`: `UNKNOWN` is now handled separately from `NO_RESPONSE`/
  `UNPOWERED`. `UNKNOWN + OPTIONAL -> PASS`; `UNKNOWN + REQUIRED -> UNKNOWN` (system
  reports `BOOTING`, a visible gap, not a false alarm). Observed-absence escalation is
  unchanged: `NO_RESPONSE + REQUIRED -> FAULT`, `NO_RESPONSE + OPTIONAL -> DEGRADED`.
- This fixed two defects with one rule: the LED ring (non-probeable, so permanently
  `UNKNOWN` when powered) no longer degrades the system, and the servo bus (`REQUIRED`
  but deliberately never probed at boot) no longer reports `FAULT` before anything has
  been asked of it. The second case was found while evaluating the first across both
  profiles, as the review instructed.
- No physical detection is faked: the LED still reports `detected=UNKNOWN`, and the
  static audit forbids `detectedStateForLedRail()` from ever returning `ONLINE`.
- `USB_ONLY` classification is bit-for-bit unchanged — verified by a test that reproduces
  the hardware-validated Session 2 / H3 table exactly.
- `core/SystemState`: `beginBoot()` takes `now_ms` instead of calling `millis()`, making
  the translation unit Arduino-free so the offline tests link the real aggregation.

Tests and tooling:

- `scripts/tests/test_build_manifest.py`: new, 36 offline tests covering every refusal
  reason by exact code, the full authorization matrix, and write→verify round trips.
- `scripts/tests/test_servo_population.cpp`: +4 cases / +39 checks for the Availability
  truth table, the unchanged `USB_ONLY` boot table, `READY` reachability under
  `ROBOT_POWERED`, and a real optional-module failure still degrading.
- `static_audit.py`: two new check groups for profile provenance and the `UNKNOWN`
  distinction, plus anti-weakening assertions anchored to the actual comparisons in
  `flash_app_only.sh` rather than to token presence. All 15 new tripwires mutation-tested.

## Unreleased — G2 ROBOT_POWERED configuration support — 2026-09-16

Software preparation for the powered robot. **No powered hardware validation was
performed** — that is gate G3 and is authorized separately. The compiled-in default
hardware profile deliberately remains `USB_ONLY`.

Release identity is intentionally unchanged (`kFirmwareVersion` stays `0.1.0`): the
`0.2.x` number is decided at the release gate, not by a development branch name.
Development builds are distinguished by the git-SHA build id.

- `config`: new `HardwareProfile.h` — one enum (`USB_ONLY` / `ROBOT_POWERED`), one
  `expectationsFor()` mapping table. `BuildConfig.h` now *derives*
  `kServoPowerAvailable`/`kBatteryAvailable`/`kLedRailPowered`/`kTestProfile` from the
  selected profile instead of storing four independently editable facts that could
  contradict each other. Switching profiles is a one-symbol change.
- `core`: `Availability` gains `expectedStateForServoBus`/`Battery`/`LedRail` and
  `detectedStateForLedRail` — the profile → `ExpectedState` rule stated once instead of
  inlined per module. `classify()` itself is unchanged: the same `NO_RESPONSE` becomes
  `PASS` under `USB_ONLY` and `FAULT` under `ROBOT_POWERED`, which is what the V0.1
  model was designed for. `Availability.h`/`SystemState.h` now include `<stdint.h>`
  rather than `<Arduino.h>` so the offline host tests link the shipped logic.
- `servo`: new `ServoPopulation` — canonical 17 / expected-now 13 / absent-by-design 4
  (52-55) kept explicitly distinct, with pure per-ID and whole-census classification
  (`PRESENT_EXPECTED` / `MISSING_EXPECTED` / `ABSENT_BY_DESIGN` /
  `ABSENT_BY_DESIGN_PRESENT` / `UNEXPECTED_ID` / `NOT_PROBED`, verdict `PASS` /
  `PROFILE_MISMATCH` / `RANGE_INCOMPLETE` / `NOT_RUN`). Fail-closed: a partial or
  truncated scan can never report `PASS`. A healthy powered census is 13 present + 4
  absent by design — "17 = PASS" is never encoded anywhere.
- `servo`: new `ServoCensus` — Controller-owned service that drives the existing
  `ServoBus` scan state machine and holds the structured `CensusResult`. No second bus
  owner, no duplicate UART, no `Serial`, never auto-started.
- `core`: `@SERVO CENSUS` added to the USB command surface (MAINTENANCE-gated, like
  `@SERVO SCAN`). `CommandRouter` only formats the stored result — no G2 domain logic
  lives inside Serial parsing or printing, so a future Web UI / HostLink adapter can
  consume the same `CensusResult` without re-scanning the bus.
- Boot banner reports the profile with its rail facts, the declared servo population,
  and `startup_servo_scan : DISABLED` alongside the existing motion/torque lines.
- `scripts`: static audit extended with six G2 checks (profile authority + the
  `USB_ONLY`-default G3 gate, population model + YAML provenance cross-check, transport
  independence, no startup bus traffic, network→servo tripwire, host test suite). All
  pre-existing checks retained; every new tripwire was mutation-tested.
- `scripts/tests`: new offline C++ suite (`test_servo_population.cpp` +
  `run_host_tests.sh`) covering the required census classification cases and both
  profiles' expected-hardware semantics.

Explicitly NOT in this gate: Wi-Fi, HTTP/WebSocket/REST, Web UI, OTA transport, command
lease/deadman, teleoperation, IK/gait/pose, Safe Actuator, the full ActuatorAuthority
framework, provisioning/QC/source-signature/calibration integration, DALY writes, servo
EEPROM/ID writes, and any motion.

## 0.1.0 — 2026-09-15

First unified operational ESP32-S3 runtime. Integration milestone: brings ST3215
ServoBus, BNO085 IMU, DALY BMS telemetry and a new LED ring module into one modular,
deployable firmware image with a shared USB diagnostic surface, cooperative
non-blocking scheduling, module health aggregation and a power-state machine
baseline.

- `core`: `Controller`, `SystemState`, `PowerState`, `CommandRouter`.
- `servo`: `ServoBus` — read-only ST3215 transport adapted from the frozen Bench QC
  V6.1 source; `@SERVO SAFE_OFF` is the only write, torque-off only.
- `imu`: `Bno085Imu` — adapted from `matdog_bno085_dcd_phase_c3`; preserves the
  viewer-required `RV`/`MAG`/`GYR`/`COUNTS`/`SAVE_GATE` text protocol exactly; no
  DCD-save path exists in this firmware at all.
- `power`: `DalyBms` — adapted from `matdog_daly_rs485_probe_v2`; non-blocking poll
  state machine; `requestDischargeOff()` is a fail-closed placeholder pending K-Series
  write-protocol verification.
- `status`: `LedRing` — new module (no prior MATDOG source existed), built on
  Adafruit NeoPixel 1.15.5; boots OFF, brightness capped well under hardware maximum.
- Scope explicitly excludes gait, operational IK, closed-loop stabilization, ROS 2,
  Wi-Fi/OTA, autonomous behaviour, automatic calibration and any DALY/servo/BNO085
  write beyond the ones listed above.

See `SOURCE_PROVENANCE.md` for exact source hashes and `VALIDATION.md` for what has
and has not been exercised on real hardware.

## 0.1.0 — Session 2 hardening — 2026-09-15

Corrective/procedural hardening on top of the same 0.1.0 scope above — no new
firmware functionality, no motion/gait/IK/DALY-write/KEY work. Flashed and
hardware-validated as commit `04dfa52d1b5ab37ac4099792bb8c874c5ee4842a`
(`FIRMWARE_SOURCE_COMMIT`; see `VALIDATION.md` Session 2 for why this document's
own commit is necessarily later and must not be confused with it).

- `core`: adds `Availability` (`InitializationState`/`DetectedState`/
  `ExpectedState` → `Classification`), fixing `@STATUS` reporting a module as
  `OK` merely because its driver initialized rather than because the hardware
  was actually detected. `Controller::update()` now drives `ServoBus::update()`
  every tick.
- `status/LedRing`: no longer initializes the NeoPixel transport or transmits
  any WS2812 frame while `build::kLedRailPowered` is false (current `USB_ONLY`
  profile) — GPIO47 is left in a defined `INPUT` state instead. `@LED TEST`
  now refuses explicitly rather than driving an unpowered rail.
- `servo/ServoBus`: `@SERVO SCAN` is now a non-blocking one-`Ping()`-per-tick
  state machine instead of a synchronous loop that could monopolize `loop()`
  for seconds against an unresponsive ID range.
- `scripts/`: adds `flash_app_only.sh` (writes only the verified active
  application/OTA partition, never bootloader/partition-table/boot_app0) and
  `verify_application_partition.py` (reads the device's own partition table +
  otadata rather than assuming an offset). Corrects `upload.sh`'s header
  comment, which had incorrectly described a full Arduino upload as
  application-only.
- `scripts/static_audit.py`: four new regression checks for the above.

See `VALIDATION.md` Session 2 for the full read-only flash audit, the
application-only flash provenance record, and H1–H6 hardware results.

## 0.1.0 — Session 2.1 final pre-merge hardening — 2026-09-15

Two findings from an independent review of Session 2, fixed before merge —
no new functionality. Flashed and hardware-validated as commit
`07592b9d7f284eb6a24d18d69b57351281676e14` (`FIRMWARE_SOURCE_COMMIT`; see
`VALIDATION.md` Session 2.1).

- **Terminology correction**: Session 2's `@SERVO SCAN` was described as
  "non-blocking". `SCServo::Ping()` is still a synchronous call with its own
  bounded per-call timeout, so the accurate description is "incremental
  scan with bounded per-ID blocking". Corrected everywhere it was live
  documentation; Session 2's own record is preserved with a correction
  note, not rewritten.
- `core`: adds `OperatingMode` (`MAINTENANCE`/`RUN`) — a deliberately tiny
  boundary, not a state machine. `@SERVO SCAN`/`@SERVO READ` now refuse
  outside `MAINTENANCE`; `@SERVO SAFE_OFF` stays reachable in every mode
  (it can only remove torque). Default is `MAINTENANCE` (no motion loop
  exists yet); the future motion controller must flip the default to `RUN`.
- `servo/ServoBus`: `kPingTimeoutMs` reduces SCServo's 100ms default
  `IOTimeOut` to 20ms, justified by real hardware measurement (NEW01
  characterization campaign, ~600us measured round trip) rather than an
  assumed value — a public library field, not a vendored-source edit.
  `ScanResult` now reports measured `elapsed_ms`/`max_ping_us` instead of
  asserting scan cost. Measured this session: 45-ID scan in 1218ms
  (was ~4.5s), max single probe 20.815ms; BNO085 RV rate ~48.8Hz baseline
  vs ~47.3Hz during a scan.
- `scripts/ota_partition_logic.py` (new): replaces Session 2's OTA
  slot-selection logic, which compared raw `ota_seq` numbers and mislabeled
  the struct's `ota_state` field as `crc`, never validating the real CRC.
  Rewritten against the exact installed ESP-IDF v5.5.5 source
  (`bootloader_common_loader.c`/`bootloader_utility.c`), fails closed
  (`OtaAmbiguous`) on every state the real bootloader does not
  deterministically resolve. `scripts/tests/test_ota_partition_logic.py`:
  10/10 offline tests pass, including the exact regression case (a
  CRC-invalid entry with a higher raw sequence number than the valid one).
- `scripts/static_audit.py`: three new checks — MAINTENANCE-mode gating on
  servo scan/read (and that SAFE_OFF never gains one), the OTA parser's
  fail-closed primitives, and runs the OTA parser's own offline test suite
  as part of the audit.

See `VALIDATION.md` Session 2.1 for the full measurement evidence, the ESP-IDF
source citations, and H1/H2/H3/H4/H6 hardware re-validation.

## 0.1.0 — Session 2.2 final merge gate — 2026-09-15

Four findings from a final review of Session 2.1, fixed before merge — no
new functionality. Flashed and hardware-validated as commit
`cb53c63206b0ccad68055ecc993a0a9e03f5b545` (`FIRMWARE_SOURCE_COMMIT`; see
`VALIDATION.md` Session 2.2).

- `scripts/ota_partition_logic.py` (Finding A): OTA subtype recognition
  corrected from a `subtype >= 0x10` threshold to the real ESP-IDF bitmask
  `(subtype & 0xF0) == PART_SUBTYPE_OTA_FLAG` — the old check wrongly
  matched `PART_SUBTYPE_TEST`(0x20) and `PART_SUBTYPE_TEE_0/1`(0x30/0x31)
  as if they were OTA app slots. Also refuses on duplicate OTA slot
  indices instead of silently picking one.
- `scripts/ota_partition_logic.py` (Finding B): the real, installed build
  has `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` (confirmed by reading the
  actual sdkconfig, not assumed). `resolve_application_partition()` now
  takes required `rollback_enabled`/`anti_rollback_enabled` parameters
  (no default) and refuses when anti-rollback is enabled at all, or when
  rollback is enabled and the selected otadata entry's state is
  `NEW`/`PENDING_VERIFY` (states the bootloader can autonomously rewrite
  on the next boot). `verify_application_partition.py` now requires
  `--sdkconfig`; `flash_app_only.sh` passes it and prints both flags
  before writing. The real device's state (`ota_state=UNDEFINED`) is
  stable regardless, so flashing proceeded correctly this session.
- `servo/ServoBus` (Finding C): `kPingTimeoutMs` (20ms) is no longer set
  globally in `begin()` — it was silently becoming a standing override for
  every future SCServo call. `ScopedPingTimeout` (RAII) now brackets each
  individual bus transaction and restores the library's own 100ms default
  afterward, on every exit path.
- `servo/ServoBus` (Finding D): `safeOff()` returned `bool` based on
  `EnableTorque()`'s own return value, which is `SCS::Ack()` — `0` on any
  failure, not `-1` like `Ping()`/`readByte()`, so `result >= 0` was always
  true and could never observe a failure. Confirmed live last session:
  `@SERVO SAFE_OFF 11` reported `OK` with the servo bus completely
  unpowered. `safeOff()` now returns `SafeOffResult`
  (`VERIFIED_OFF`/`UNVERIFIED_NO_RESPONSE`/`VERIFY_FAILED`), classified
  strictly from an independent `TorqueEnable` readback taken after the
  write, never from the write's own ACK. Re-tested live this session:
  `UNVERIFIED_NO_RESPONSE`, correctly.
- `core/OperatingMode.h`, `core/PowerState.h`: cross-reference comments
  clarifying `OperatingMode::RUN` and `PowerState::RUN` are orthogonal
  concepts that share a name coincidentally. `OperatingMode`'s approved
  MAINTENANCE/RUN boundary itself is unchanged.
- `scripts/static_audit.py`: five new regression checks for the above;
  27/27 OTA parser offline tests (17 new this session).

See `VALIDATION.md` Session 2.2 for the full measurement evidence, the
ESP-IDF rollback source citations, and H1/H3/H4/H5/H6 hardware
re-validation including live SAFE_OFF and OperatingMode transcripts.

## 0.1.0 — Session 2.3 final consistency fix — 2026-09-15

Three consistency findings from a final review of Session 2.2, fixed before
merge — no new functionality. Flashed and hardware-validated as commit
`5b371da5482f9b0bd2df1c37ed361250ea54ae8f` (`FIRMWARE_SOURCE_COMMIT`; see
`VALIDATION.md` Session 2.3).

- `servo/ServoBus` (Finding 1): Session 2.2's fix still applied the 20ms
  diagnostic timeout to *every* transaction, including `safeOff()` and
  `readRuntimeState()` — so the documented "operational timeout = 100ms,
  diagnostic timeout = 20ms" split was not actually true in code.
  `ScopedPingTimeout` is renamed `ScopedIOTimeout` and now takes an explicit
  timeout argument at every call site; `kDiagnosticTimeoutMs` (20ms, still
  `ping()`/`readModel()`/the scan's per-ID probe) and `kOperationalTimeoutMs`
  (100ms, new: `safeOff()`/`readRuntimeState()` — the safety de-escalation
  path and the primitive a future motion controller will reuse) are now two
  separately named constants.
- `scripts/ota_partition_logic.py` (Finding 2): `parse_sdkconfig_ota_flags()`
  treated a Kconfig symbol completely absent from the sdkconfig text the
  same as one explicitly disabled (both → `False`) — not fail-closed.
  `parse_sdkconfig_flag()` now returns a three-way `SdkconfigFlag`
  (`ENABLED`/`DISABLED`/`UNKNOWN`); `resolve_application_partition()`
  REFUSEs on `UNKNOWN` for either `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` or
  `CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK`, exactly like it does on `ENABLED`.
  The real build's config (`ROLLBACK_ENABLE=y`, `ANTI_ROLLBACK` not set)
  still resolves correctly — re-verified against the actual device.
- `scripts/ota_partition_logic.py` (Finding 3): `ota_app_partitions()` now
  requires OTA slot indices to be exactly `{0, ..., N-1}`; a sparse set
  (`{1}`, `{0,2}`, `{0,1,3}`, ...) raises `OtaAmbiguous` instead of
  resolving. MATDOG has no use for a sparse OTA layout in V0.1; this avoids
  ambiguity between raw OTA subtype numbers and the bootloader's own
  `app_count`/modulo slot selection ahead of any future OTA subsystem.
- `scripts/tests/test_ota_partition_logic.py`: removed
  `test_flags_absent_entirely_defaults_to_disabled`, which encoded the
  non-fail-closed policy Finding 2 corrects; added coverage for
  symbol-absent → `UNKNOWN`, `UNKNOWN` → REFUSE (both symbols,
  independently and together), and the four contiguity scenarios above.
  27 → 40 tests, all PASS.
- `scripts/static_audit.py`: fixed references broken by the Finding 1
  rename and the Finding 2/3 API changes; added a check that
  `safeOff()`/`readRuntimeState()` use `kOperationalTimeoutMs` and never
  `kDiagnosticTimeoutMs`, and extended the OTA fail-closed check to require
  `SdkconfigFlag.UNKNOWN` handling and the slot-contiguity check.

See `VALIDATION.md` Session 2.3 for the full measurement evidence
(including live `@SERVO SAFE_OFF` timing with the servo bus unpowered) and
H1/H2/H3/H4/H5/H6 hardware re-validation.
