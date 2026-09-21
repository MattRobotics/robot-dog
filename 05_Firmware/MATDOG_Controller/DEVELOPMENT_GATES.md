# MATDOG Controller — Development Gates

**Canonical owner of the technical pass/fail authorization criteria for each Controller
development gate.** Last updated 2026-09-18.

This file answers *what must be true before this stage may begin, what it may and may not do, and
what proves it passed*. It is not a narrative roadmap and not an evidence log:

| For | Read |
|---|---|
| Where the project stands and why stages are ordered | [`ROADMAP.md`](../../01_Docs/02_Architecture/ROADMAP.md) |
| Permanent system contracts and forbidden architectures | [`ARCHITECTURE.md`](../../01_Docs/02_Architecture/ARCHITECTURE.md) |
| What was actually validated, with dated evidence | [`VALIDATION.md`](VALIDATION.md) |
| Current snapshot and immediate milestone | [Root `README.md`](../../README.md) |

## Rules that apply to every gate

1. A gate's existence in this file is **not** authorization to execute it. Hardware gates require
   explicit operator authorization for that specific session.
2. **IMPLEMENTED ≠ VALIDATED.** A gate may only be marked PASS on the evidence its PASS CRITERIA
   name. Passing a precursor, or a standalone bench tool, never transfers.
3. A failed prerequisite is never bypassed to finish a session. Stop, preserve evidence, report.
4. `SAFE_OFF` must remain reachable in every mode, at every gate, forever.
5. No gate may weaken a protection established by an earlier gate.
6. **A gate's technical prerequisites do not override
   [`ROADMAP.md`](../../01_Docs/02_Architecture/ROADMAP.md) sequencing.** Satisfying a gate's ENTRY
   conditions early means the gate is technically *ready*, not that it is *authorized*. A stage may
   only begin when the roadmap sequence has reached it. Reordering requires an explicit, reviewed
   change to the roadmap itself — never a local reading of one gate's entry list.

**Status values:** `PASS` · `CURRENT` · `TO_TEST` · `TO_DESIGN` · `BLOCKED` · `PARTIAL`.

---

## G0 — Post-cleanup entry audit

- **PURPOSE** — establish the exact post-cleanup source of truth before any firmware edit.
- **ENTRY** — repository-cleanup PR merged to `main`.
- **ALLOWED** — read-only inspection of Git state, source, tests, tags, branches, worktrees.
- **FORBIDDEN** — any file modification; branch creation.
- **PASS CRITERIA** — written report; no ambiguity about the development base.
- **NEXT** — G1.
- **STATUS** — **PASS** (not rerun since; accepted base `1a8c5bc3ced4f49ea36902526f232b2785a7ab70`).

## G1 — V0.1 regression freeze

- **PURPOSE** — prove post-cleanup `main` still reproduces the V0.1 software baseline.
- **ENTRY** — G0 PASS.
- **ALLOWED** — compile, offline tests, static audit, viewer regression.
- **FORBIDDEN** — feature work; hardware action.
- **PASS CRITERIA** — compile PASS, static audit PASS, OTA tests PASS, viewer PASS, clean tree.
- **NEXT** — G2.
- **STATUS** — **PASS** (not rerun since).

## G2 — ROBOT_POWERED configuration support

- **PURPOSE** — prepare the firmware for the real current robot, in software only, without motion.
- **ENTRY** — G0 + G1 PASS.
- **ALLOWED** — hardware-profile authority; canonical-17 / expected-now-13 / absent-by-design-4
  population model; structured census classification; profile-derived module expectations;
  transport-independent state; offline tests; static-audit extension.
- **FORBIDDEN** — Torque ON; `GoalPosition`; servo EEPROM/ID/`CalibrationOfs`/factory-reset/
  broadcast writes; DALY writes; BNO085 DCD writes; automatic scan/census at boot; energizing any
  external rail; flashing; Web/Wi-Fi implementation; ActuatorAuthority framework; motion.
- **PASS CRITERIA** — census/profile host tests PASS; build-manifest provenance tests PASS; static
  audit PASS; OTA tests PASS; `USB_ONLY` compile PASS; `ROBOT_POWERED` compile-only PASS; viewer
  PASS; `USB_ONLY` semantics unchanged; source default remains `USB_ONLY`.
- **NEXT** — G3, and only with explicit powered-session authorization.
- **STATUS** — **PASS** (software/offline scope only). ROBOT_POWERED remains **IMPLEMENTED**, not
  **VALIDATED**.
- **EVIDENCE** — [`VALIDATION.md` § G2](VALIDATION.md) and § G2 pre-G3 hardening amendment.

## G3 — ROBOT_POWERED live validation, no motion

- **PURPOSE** — first powered session; prove the powered robot's read-only behaviour and safe-state
  semantics.
- **ENTRY** — G2 PASS; robot mechanically supported so no load-bearing motion is possible;
  accessible disconnect/`KEY`; fuse and polarity verified; full-flash backup verified; image built
  with `MATDOG_PROFILE=ROBOT_POWERED` and flashed only via the manifest-verified
  `MATDOG_FLASH_PROFILE=ROBOT_POWERED scripts/flash_app_only.sh`; **explicit operator
  authorization for that session**.
- **ALLOWED** — protected-domain boot; DALY read-only; conservative LED test; ST3215 read-only
  census and register reads; `SAFE_OFF` with readback; concurrent soak.
- **FORBIDDEN** — Torque ON; `GoalPosition`; any motion; servo EEPROM/ID writes; DALY writes; BNO085
  DCD writes; partition rewrite; full erase.
- **PASS CRITERIA** — clean boot (`health=BOOTING` before any probe, by design); no resets; DALY
  live and plausible; LED validated without disturbing bus timing; census exactly `PASS`
  (13 present-expected, 4 absent-by-design, 0 missing, 0 unexpected) and stable across repeats;
  `VERIFIED_OFF` on all 13; `READY` during soak; **no motion observed at any point**.
- **NEXT** — G3.1 (PASS), then the DALY KEY investigation, then G4 — Diagnostics / Maintenance.
- **STATUS** — **PASS (formal).** Powered no-motion evidence live 2026-09-17; a second census on
  2026-09-18 matched the first exactly, satisfying "census … stable across repeats" (criterion
  unchanged). A Controller-loop regression found in between is tracked as G3.1, now PASS; it did
  not invalidate the servo/DALY/LED evidence, each item of which was measured per transaction.
- **EVIDENCE** — [`VALIDATION.md` § G3](VALIDATION.md); procedure in
  [`G3_ROBOT_POWERED_VALIDATION_PLAN.md`](G3_ROBOT_POWERED_VALIDATION_PLAN.md).

## G3.1 — CDC-independent Controller loop

- **PURPOSE** — prove Controller execution does not depend on a USB CDC host keeping the
  port open. Regression found live after G3: with the cable attached and the host port
  closed, BNO085 RV processing fell to 0.68 Hz (50.07 Hz with the port open, same boot).
- **INVARIANT** — no USB CDC transmit condition may block `Controller::update()`.
- **DELIVERY** — best effort. Command replies are expected complete under normal
  connected/draining conditions with adequate TX-ring free space; after a long closed-port
  interval a stale backlog may occupy the ring at reopen, and a reply emitted before it
  drains may short-write rather than block. Accepted for the diagnostic USB surface; **not**
  part of any HostLink contract.
- **ENTRY** — G3 powered no-motion evidence PASS (the outstanding census repeat is run
  inside this session); patched image built from a **clean commit** with
  `MATDOG_PROFILE=ROBOT_POWERED` and flashed only via
  `MATDOG_FLASH_PROFILE=ROBOT_POWERED scripts/flash_app_only.sh`; explicit operator
  authorization for that session.
- **ALLOWED** — application-only flash of the patched image; zero-TX passive serial
  observation; the read-only replies already used in G3 (`@STATUS`, `@MODE STATUS`,
  `@IMU STATUS`, `@BMS STATUS`, `@LED STATUS`, `@HELP`); `@BMS STREAM ON|OFF` for the stream
  case; **one** read-only `@SERVO CENSUS`, which also closes the outstanding G3 census repeat.
- **FORBIDDEN** — everything G3 forbids; any other servo command; LED TEST; mode change.
- **PASS CRITERIA** — zero-TX A/B on one boot: RV rate **≥ 45 Hz with the port closed for
  ≥ 60 s** (including after a host has opened and closed it) and 45–55 Hz with it open;
  the same with `@BMS STREAM ON` enabled before closing; `runtime_resets` unchanged; no
  reset/brownout/panic; every line syntactically valid on reopen; the viewer receives
  telemetry with zero commands; command replies complete when issued with the host reading
  and the reconnect backlog drained.
- **NEXT** — DALY KEY investigation, then G4 — Diagnostics / Maintenance (roadmap stage 4).
- **STATUS** — **PASS** (live, 2026-09-18). Fix: native HWCDC TX timeout 0 + 3 KB TX ring
  (`e2fc605`). BNO085 RV 50.10 Hz with the port closed 62 s after a host had opened and closed
  it (0.68 Hz before the fix), 50.13 Hz open, 50.11 Hz with `@BMS STREAM` enabled;
  `runtime_resets` 0; every command reply complete once the backlog had drained.
- **EVIDENCE** — [`VALIDATION.md` § G3.1](VALIDATION.md).

## Open hardware item — DALY KEY (before G4)

- **FINDING** — during G3 both positions of the physical KEY switch produced identical
  DALY-reported state (`discharge_mos=ON` in both).
- **RESEARCH** — **COMPLETE** (read-only, 2026-09-19). Public DALY K-series documentation does
  not publish a KEY configuration register. Static inspection (not execution) of DALY's official
  BMSTool V1.14.79 found a second Modbus personality — request address `0x81`, reply `0x51` —
  whose parameter block holds KEY logic at `0x0120` (`0x55` DISABLED, `0xA5`
  DISCHARGE_AND_SLEEP, `0x5A` DISCHARGE, `0xAA` CHARGE_AND_DISCHARGE, `0xA6`
  CHARGE_DISCHARGE_AND_SLEEP), charge/discharge MOS control at `0x0121`/`0x0122` and sleep time at
  `0x0115` (shown as raw × 10 s). It uses a different register map from the live-validated `0xD2`
  telemetry personality. Both the read and the write side were live-verified the same day (below).
- **READ PROBE** — **LIVE VERIFIED READ-ONLY** (2026-09-19, firmware `a57fcdd`). `@BMS KEY READ`
  (MAINTENANCE only) sends the single FC03 frame `81 03 01 00 00 78 5B D4` once; `@BMS KEY STATUS`
  prints the cached result with zero bus traffic. One live transaction: `result=OK`, 245 bytes,
  `key_logic_raw=0x0055 key_logic=DISABLED`, `charge_mos_control=1`, `discharge_mos_control=1`,
  `sleep_time_raw=360 sleep_time_s=3600` (matches the manual's 3600 s default). `0xD2` telemetry
  resumed (`comm=OK`), `runtime_resets` 0 before and after —
  [`VALIDATION.md` § DALY KEY live read-only validation](VALIDATION.md).
- **RESULT** — `0x81` parameter personality **live verified (read-only)**; KEY logic register
  `0x0120` **live verified (read-only)**. At read-probe time, **pre-commissioning**, the register
  read **DISABLED (`0x0055`)** — strong evidence for why the physical KEY did not control the
  discharge MOS in G3. That value is historical: the **current last live-verified configuration is
  `0x005A` DISCHARGE** (single commissioning write, same day — see below).
- **WRITE / CONFIGURATION** — **LIVE VERIFIED** (2026-09-19, firmware `6322563`). Exactly one
  semantic write exists: `@BMS KEY SET DISCHARGE CONFIRM` (MAINTENANCE only) sends FC06
  `81 06 01 20 00 5A 16 07` — KEY logic `0x0120 := 0x005A` (DISCHARGE: KEY OFF → discharge MOS
  OFF, charge MOS kept) — reconstructed from BMSTool V1.14.79's own write path, then always reads
  the register back. It refuses (zero TX) unless: MAINTENANCE (re-checked immediately before
  transmitting); no write yet this boot; bus idle; `0xD2` telemetry OK ≤ 5 s old; no alarms; a
  successful KEY read ≤ 30 s old showing exactly `0x0055` with charge/discharge MOS control
  `1`/`1`; `0x005A` already → `ALREADY_CONFIGURED`. The static audit admits only this write (FC10,
  other registers — MOS control `0x0121`/`0x0122` included — other values and any caller-supplied
  target stay forbidden). `requestDischargeOff()` remains a fail-closed stub that transmits
  nothing; `@SYSTEM SHUTDOWN` still resolves to `POWER_CUT_FAILED`.
- **LIVE WRITE RESULT** — sent **once**: `ACK result=OK rx_bytes=8` (exact echo
  `51 06 01 20 00 5A 05 97`) and `readback=VERIFIED key_logic_raw=0x005A`, with no BMS restart
  needed. Telemetry resumed, MOS stayed ON/ON with the KEY ON, `runtime_resets` 0 —
  [`VALIDATION.md` § DALY KEY single live configuration write](VALIDATION.md). Persistence across
  a BMS power cycle is **TO_TEST**. The write is retained as tightly gated re-commissioning
  functionality and is now inert on this unit (`ALREADY_CONFIGURED`, zero TX).
- **PHYSICAL KEY TEST** — **BLOCKED / INCONCLUSIVE.** Operator-reported: with KEY OFF the DALY
  showed the discharge MOS OFF while the robot load rail stayed powered. Root cause is a hardware
  `B-`/`P-` bypass — the TECNOIOT step-down input return sits on raw battery `B-`, and being a
  non-isolated buck it bridges `B-` to `P-` through the ESP32/Seeed/servo grounds.
- **REQUIRED CORRECTION (operator)** — move `TECNOIOT VIN-` from `B-` to `P-`, then run the
  dead-circuit / KEY ON / KEY OFF (USB disconnected) / KEY ON / powered no-motion procedure in
  [`04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md`](../../04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md)
  § 11. Until it passes, KEY OFF must not be trusted to remove the robot rails.
- **CHARGING** — separate **OPEN** gate: no charger/dock hardware evidence exists. Manual charging
  with KEY OFF is an architectural target only.
- **STATUS** — **OPEN.** The BMS-side KEY configuration is verified; the KEY as a power-off is
  blocked on the hardware bypass above.
- **INTERIM RULE** — the fused disconnect is the trusted physical isolation method.

## G4 — Diagnostics / Maintenance

- **PURPOSE** — permanent read-only maintenance capability over the single shared `ServoBus`.
- **ENTRY** — G3 formal PASS (including the census repeat); **G3.1 PASS**. Both satisfied
  2026-09-18. The DALY KEY investigation is closed on the BMS side (read and write both
  live-verified); the roadmap puts the hardware `B-`/`P-` rewire and its post-rewire power
  validation first.
- **ALLOWED** — `SYSTEM_SELF_TEST`, consolidated servo health, source-signature read, profile audit;
  extension of the existing census/read/`SAFE_OFF` surface.
- **FORBIDDEN** — any new persistent write path; any transport→register access; motion.
- **PASS CRITERIA** — all operations read-only; no new write path; classification correct for
  expected/absent/missing/unexpected; reachable over USB CDC.
- **NEXT** — Authority model.
- **STATUS** — **PARTIAL.** Already implemented: `@STATUS` availability, `@SERVO SCAN`,
  `@SERVO READ`, `@SERVO CENSUS`, `@SERVO SAFE_OFF`, `@IMU`/`@BMS`/`@LED` status. Not implemented:
  `SYSTEM_SELF_TEST`, `SOURCE_SIGNATURE`, `PROFILE_AUDIT`, consolidated health summary.

## Authority model — OperatingMode / ActuatorAuthority

- **PURPOSE** — central arbitration so at most one subsystem may own actuator-write authority.
- **ENTRY** — Diagnostics/Maintenance foundation present.
- **ALLOWED** — explicit mode set (`MAINTENANCE`/`CALIBRATION`/`SERVICE`/`RUN`) and actuator
  authority (`NONE`/`DIAGNOSTICS`/`CALIBRATION`/`QC`/`PROVISIONING`/`MOTION`).
- **FORBIDDEN** — two simultaneous actuator owners; implicit authority survival across reset or
  host disconnect; any gate that makes `SAFE_OFF` unreachable.
- **PASS CRITERIA** — failure-injection tests for illegal cross-mode operations; reset clears write
  authority; host disconnect leaves no armed write transaction.
- **NEXT** — Service / Provisioning / QC.
- **STATUS** — **PARTIAL.** `OperatingMode{MAINTENANCE, RUN}` exists and gates blocking servo
  diagnostics. Full `ActuatorAuthority` is **TO_DESIGN**.

## Service / Provisioning

- **PURPOSE** — tightly gated servo replacement and canonical provisioning without reflashing a
  specialist tool.
- **ENTRY** — Authority model PASS.
- **ALLOWED** — `SERVICE` mode only: source snapshot, model/profile gate, explicit register
  allowlist, ordered persistent writes, readback, cold-persistence verification.
- **FORBIDDEN** — generic host arbitrary EEPROM/register write; broadcast write; `CalibrationOfs`;
  factory reset; blind ID-change retry; provisioning from an unknown source signature.
- **PASS CRITERIA** — a successful provisioning marks the joint `JOINT_CALIBRATION_REQUIRED`, never
  motion-ready.
- **NEXT** — QC.
- **STATUS** — **TO_DESIGN.** Provisioner V6 remains a **FROZEN** oracle, not runtime source.

## QC

- **PURPOSE** — servo health characterization, in situ and on the bench.
- **ENTRY** — Service foundation PASS.
- **ALLOWED** — `QC_IN_SITU`: conservative telemetry/stability/profile/fault checks.
  `QC_FULL_BENCH`: service-only, isolated-servo context.
- **FORBIDDEN** — full bench QC motion reachable from normal `RUN`; dormant EEPROM normalization
  copied in by accident.
- **PASS CRITERIA** — normal `RUN` cannot trigger bench QC motion; in-situ checks add no write path.
- **NEXT** — Calibration integration.
- **STATUS** — **TO_DESIGN.** Bench QC V6.1 remains a **FROZEN** oracle.

## Calibration — Full Leg Calibration integration and formal H1+

- **PURPOSE** — make calibration a permanent Controller capability and recalibrate the installed
  robot.
- **ENTRY** — G3 formal PASS (including the census repeat); G3.1 PASS; Authority model PASS;
  powered bus health proven.
- **ALLOWED** — H1 census semantics, q0 evidence capture, direction witnesses, characterization,
  then staged calibration motion **each with its own session authorization**.
- **FORBIDDEN** — merging `matdog/full-leg-calibrator-v1` wholesale; duplicating `ServoBus`/UART/
  scheduler/`SAFE_OFF`; auto-promoting measurements; assuming `q0 = 2048`; any H2+ motion before a
  formal H1 PASS.
- **PASS CRITERIA** — formal H1 satisfied for all 12 leg servos (ID 51 is not in Full-Leg H1); the
  evidence lifecycle `MEASURED → CANDIDATE → ACCEPTED → PROMOTED` is preserved.
- **NEXT** — motion may be considered, behind the Safe Actuator Layer.
- **STATUS** — **BLOCKED** by `CALIBRATION_RESET_PENDING_FULL_RECALIBRATION`. Last formal H1 was
  6/12 and is **not** superseded by a Controller census.

## HostLink semantic layer

- **PURPOSE** — one semantic command/service contract independent of transport.
- **ENTRY** — all preceding [`ROADMAP.md`](../../01_Docs/02_Architecture/ROADMAP.md) stages through
  **formal recalibration** completed, plus the Diagnostics/Maintenance foundation required by the
  semantic service layer. The Diagnostics/Maintenance foundation alone is a technical prerequisite,
  **not** an authorization: HostLink follows Service/Provisioning/QC and Full Leg Calibration in
  the canonical sequence and must not be pulled forward ahead of them (see rule 6 above).
- **ALLOWED** — transport adapters over one Controller service implementation; structured state and
  telemetry snapshots.
- **NOTE (G3.1)** — the USB CDC diagnostic surface's best-effort delivery (timeout 0, short writes
  possible after a stale backlog) is not inherited as a HostLink guarantee. HostLink defines its
  own framing, acknowledgement and reliability semantics.
- **FORBIDDEN** — duplicate command semantics per transport; transport owning hardware; any
  business logic reachable only inside a parser or printer.
- **PASS CRITERIA** — a second transport can consume the same semantic state without
  reimplementation or a duplicate hardware transaction.
- **NEXT** — Wi-Fi runtime.
- **STATUS** — **PARTIAL.** `CommandRouter` is a USB CDC adapter; G2 made the census layer
  transport-independent (no `Serial`, no `<Arduino.h>`, structured `CensusResult`), which is the
  precondition. A formal Controller Service Layer and schema are **TO_DESIGN**.

## Wi-Fi runtime

- **PURPOSE** — bounded network connectivity for the dashboard and OTA.
- **ENTRY** — HostLink semantic contract defined.
- **ALLOWED** — connection management, bounded client count, bounded queues, non-blocking I/O.
- **FORBIDDEN** — network callbacks calling servo primitives; network traffic starving
  BNO085/DALY/ServoBus/motion; Wi-Fi loss destabilizing low-level control.
- **PASS CRITERIA** — reconnect cycles stable; no heap leak; no scheduler starvation; no bus timing
  degradation.
- **NEXT** — UI-0/UI-1 and OTA.
- **STATUS** — **PARTIAL (W1).** A station-mode runtime is **IMPLEMENTED**, **COMPILED** and
  **OFFLINE TESTED**; it is **NOT HARDWARE TESTED** — no MATDOG build has associated with an
  access point yet, so every PASS CRITERION above remains **TO_TEST**.
  - Implemented: `src/network/WifiPolicy.*` (pure, host-linkable lifecycle state machine) and
    `src/network/WifiManager.*` (sole owner of the radio, sole includer of `<WiFi.h>`),
    `@WIFI STATUS|ON|OFF`, one `WIFI` line in `@STATUS`, credentials resolved outside Git.
  - Offline evidence: `scripts/tests/test_wifi_policy.cpp` links the real state machine
    (credential gate, two-phase radio start, connect deadline, backoff ladder and ceiling, link
    loss, enable/disable, fail-closed action failures, `millis()` wraparound, IPv4 formatting).
  - Bounded-runtime evidence is **measured, not asserted**: `@WIFI STATUS` reports `last_us` and
    `max_us` for `WifiManager::update()`. Those numbers do not exist yet — they require the
    hardware test.
  - Build cost, same FQBN and profile, against frozen `19fe837`: flash 392,468 B → 959,051 B
    (12% → 30% of the 3 MB slot); static RAM 28,536 B → 50,868 B (8% → 15%). The ~40–50 KB the
    Wi-Fi driver allocates at first `WiFi.mode()` is heap and is **not** in those figures;
    `@STATUS` reports `heap_free`/`heap_min_free` to observe it on device.
  - Deliberately absent: any server, endpoint, remote command or update path. Wi-Fi is a link.
  - Deliberately absent: any contribution to `SystemState` health aggregation — a missing access
    point is not a robot health fact. Whether it should ever contribute is **TO_DESIGN**.
  - Still **TO_TEST** on hardware: association, DHCP, RSSI/IP reporting, reconnect after AP loss,
    heap stability over reconnect cycles, and the effect (if any) on BNO085/DALY/ServoBus timing.

## OTA

- **PURPOSE** — make wireless update the normal path while wired recovery remains mandatory.
- **ENTRY** — Wi-Fi runtime PASS; partition verifier PASS; full-flash recovery verified; USB
  recovery proven; authority model exists.
- **ALLOWED** — upload to the inactive slot, validation, reboot, version confirmation, rollback
  handling.
- **FORBIDDEN** — OTA during motion, calibration motion or an active service write transaction;
  weakening partition/rollback checks; removing USB recovery.
- **PASS CRITERIA** — update + reboot + identity confirmation + rollback behaviour all demonstrated;
  refused in unsafe states.
- **NEXT** — UI-9.
- **STATUS** — **PARTIAL (OTA-A core complete, not hardware tested).** The ENTRY condition above is
  **not met**: Wi-Fi runtime is implemented but not hardware-tested, so OTA cannot be gate-passed.
  - **IMPLEMENTED / COMPILED / OFFLINE TESTED** — the update core. `src/update/OtaPolicy.*` (pure
    state machine), `OtaBootGuard.*` (first-boot rollback lifecycle), `Sha256.*` (image identity),
    `OtaEspBackend.*` (the only unit calling `esp_ota_*`), `OtaManager.*`, `@OTA STATUS`.
    Host-side partition-selection logic remains **IMPLEMENTED** and offline-tested (40/40).
  - **Inactive-slot rule enforced structurally** — `target != running`, `subtype ∈ ota_0..ota_15`
    and `image_size ≤ target.size` are explicit refusals; the backend re-checks the running
    partition independently; `commitBootTarget()` has one call site and is reachable from exactly
    one state, `IDENTITY_VERIFIED`. `flash_app_only.sh` is **not** reused as the OTA writer.
  - **First-boot validation** — confirmation is earned: Controller init complete, CommandRouter
    bound, identity readable, no PANIC/WDT/BROWNOUT reset, uptime ≥ 15 s and ≥ 2000 loop ticks.
    The audit fails the build if `Controller::begin()` ever confirms an image. Criteria are
    software-only and **provisional until ActuatorAuthority exists**.
  - **Offline evidence** — 468 checks against the real state machine with a fake backend: every
    target/metadata/stream/verification failure, the ordering property that the boot target never
    moves outside `IDENTITY_VERIFIED`, replay/idempotence, and the whole rollback lifecycle.
    SHA-256 checked against FIPS 180-4 vectors and against `sha256sum` on the real binary.
  - **TO_IMPLEMENT** — transport and authentication. OTA-A ships neither; ingest is compiled out
    (`MATDOG_OTA_INGEST_ENABLED` defaults to `0`, audit-enforced), so no production image contains
    a reachable firmware writer. Transport options are evaluated in the Controller README.
  - **TO_IMPLEMENT / OTA-B** — `OtaAuthorizationGate` backed by the real `ActuatorAuthority`, and
    an explicit authorized operator rollback. OTA-A fails closed with no gate installed.
  - **HARDWARE TO_TEST** — everything: no device has received an OTA image, no otadata has been
    written, no rollback has been observed, and the measured erase/write blocking costs
    (`@OTA STATUS` `open_us`/`write_us`/`end_us`) do not exist yet.
  - Cost: flash 959,043 B → 967,915 B (+8,872 B, 30% of the 3 MB slot); static RAM 50,868 B →
    51,676 B (+808 B).

## Safe Actuator Layer

- **PURPOSE** — the single legal path from joint-level commands to `ServoBus` writes.
- **ENTRY** — Authority model PASS; valid calibration available.
- **ALLOWED** — enforcement of authority, mode, calibration validity and transform, joint limits,
  unsigned `0..4095` domain, goal validity, telemetry/fault guards, watchdog/deadman, safe stop.
- **FORBIDDEN** — any other code path writing a servo target; a gait engine owning `ServoBus`.
- **PASS CRITERIA** — no direct-to-`ServoBus` write path exists outside this layer; static audit
  enforces it.
- **NEXT** — first motion.
- **STATUS** — **TO_DESIGN.** No motion primitive exists in the firmware today.

## First motion

- **PURPOSE** — first commanded joint movement, bounded and suspended.
- **ENTRY** — G3 formal PASS (including the census repeat); G3.1 PASS; formal calibration
  valid; Safe Actuator Layer PASS; robot suspended.
- **ALLOWED** — one bounded calibrated joint, then controlled multi-joint pose, then suspended
  behaviour, then fault injection.
- **FORBIDDEN** — load-bearing stand; unbounded travel/velocity; motion from stale calibration.
- **PASS CRITERIA** — commanded motion matches expectation within bounds; safe-stop and fault
  injection behave as designed.
- **NEXT** — UI-4, then poses and IK.
- **STATUS** — **BLOCKED** (authority model, calibration, Safe Actuator).

## IK · Gait · Stabilization

- **IK** — operational inverse kinematics on the Controller. Entry: controlled poses PASS.
  Forbidden: converting browser XYZ directly to raw servo values. **STATUS — FUTURE.**
- **Gait** — progression stand → walk → trot, each within a validated dynamic envelope, always
  through the Safe Actuator Layer. **STATUS — FUTURE.**
- **Stabilization** — BNO085 closed loop. Entry: stable gait baseline, deterministic actuator
  timing, verified IMU frame. Corrections flow through the same authority layer; the browser never
  computes a correction. **STATUS — FUTURE.**

---

## Embedded Web UI gates

Architecture contract: [`ARCHITECTURE.md`](../../01_Docs/02_Architecture/ARCHITECTURE.md#embedded-matdog-web-ui--control--service-dashboard).
Sequencing: [`ROADMAP.md`](../../01_Docs/02_Architecture/ROADMAP.md#embedded-web-ui-placement-in-the-sequence).

**Global UI rules, applicable to every UI gate:**

1. STOP / `SAFE_OFF` visible on every write-capable page.
2. Enable/disable state comes from authoritative Controller state, never frontend assumption.
3. Browser reconnect never restores actuator ownership automatically.
4. Continuous commands always expire (firmware-side lease/deadman).
5. Write-capable service operations carry transaction IDs.
6. No raw EEPROM/register access, ever.
7. Frontend convenience never weakens Controller safety.
8. Wi-Fi loss cannot destabilize low-level control.
9. Network traffic cannot starve BNO085/DALY/ServoBus/motion.
10. UI limits derive from MATDOG canonical data, never from XGO/DOGZILLA reference material.

| Gate | Purpose | Entry | Pass criteria | Status |
|---|---|---|---|---|
| **UI-0** | Web architecture contract: frontend/backend boundary, typed semantic command model, telemetry snapshot schema, session identity, bounded clients, static-asset storage plan | none — designable offline now | proves `Web → Controller Services → Authority/Safety → hardware` with **no** direct web→`ServoBus` path | **TO_DESIGN** (may start) |
| **UI-1** | Read-only dashboard: overview, BNO085 3D, BMS, servo census/health, mode/authority/fault | Wi-Fi runtime PASS | reconnect cycles stable; no heap leak; no scheduler starvation; no bus timing degradation; **no actuator command from web** | **BLOCKED** (Wi-Fi) |
| **UI-2** | Maintenance controls: self-test, census, health, profile audit, source signature, `SAFE_OFF`, logs | Diagnostics/Maintenance PASS + UI-1 | no EEPROM write, no motor motion | **BLOCKED** |
| **UI-3** | Calibration / service workflow UI | Authority model PASS + backend workflows | firmware owns the transaction state machine; refresh/reconnect never silently resumes a dangerous transaction | **BLOCKED** |
| **UI-4** | Joint test: bounded single-joint command | Safe Actuator PASS + first motion PASS | mandatory command lease/watchdog; no raw slider; no EEPROM | **BLOCKED** |
| **UI-5** | Body pose / single-leg XYZ | operational IK + reachability gates | all commands semantic/model-based | **BLOCKED** |
| **UI-6** | Manual teleoperation joystick (`vx`, `vy`, `yaw_rate`) | safe multi-joint motion + manual-motion authority + proven safe stop | mandatory firmware-side lease; Wi-Fi loss / browser close / phone sleep / stale sequence expire the command and trigger safe stop | **BLOCKED** |
| **UI-7** | Gait selection / preset actions | validated gaits and actions only | operator controls stay simple; engineering parameters bounded and separate | **BLOCKED** |
| **UI-8** | Stabilization ON/OFF and status | closed-loop stabilization hardware-validated | browser requests a mode; never computes actuator corrections | **BLOCKED** |
| **UI-9** | OTA UI and future extensions | OTA safety policy validated | rejects OTA in unsafe modes; USB recovery remains mandatory | **BLOCKED** |
