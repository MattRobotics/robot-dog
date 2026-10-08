# MATDOG — Custom Quadruped Robot

MATDOG is Matt Robotics' custom quadruped platform: a 17-DOF mechanical design with an
articulated head, an ESP32-S3 real-time controller, and a future Jetson-based high-level stack.
This repository is the single active engineering repository for the robot.

> **⚠ FLASH SAFETY NOTICE — 2026-10-08, updated for PR-3. DO NOT FLASH MAIN: do not use `main`
> to update the current robot.**
>
> - `main` carries Calibration Persistence V1, the flash layout `MATDOG_16M_2x5M_NVS_V1` (PR-1),
>   a selective part of the dev.1 line (PR-2) and, since PR-3, the dev.2/dev.3 source delta. Its
>   firmware identity is `0.2.0-dev.3`. `scripts/build.sh` uses `PartitionScheme=custom` and
>   `scripts/upload.sh` refuses every full-image upload.
> - The firmware sources on `main` correspond to the dev.3 candidate that runs on the robot
>   (provenance `b3fd945`), apart from the deferred material. A build of `main` is **not** that
>   installed image: it has a different source commit and build ID, and binary identity is not
>   asserted.
> - dev.3 is **not an accepted release**: the Hardware Validation of 2026-10-07 has execution
>   COMPLETE and acceptance **BLOCKED**. No release is approved.
> - These integrations are **not** a flash authorization and **not** a hardware acceptance. No
>   `main` build has been accepted on hardware.
> - `MOTION_AUTHORIZED=0`, `RESTORE=NOT_IMPLEMENTED`. Nothing here authorizes a flash, a migration
>   or any motion.
>
> Details and sources: [`FLASH_LAYOUT_SAFETY_NOTICE.md`](05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md).
> The quoted snapshot below is the historical state of `main` on 2026-10-01 and has not been
> updated. Elsewhere in this file only the statements on Persistence V1, Wi-Fi/OTA, the motion
> library, the boot census, the thermal verdict and the build identity were updated; a full status
> synchronization is a separate change.

### What is on `main` (after PR-3)

| Layer | Content |
|---|---|
| Code integrated | Persistence V1 and layout V1; post-abort recovery; guarded release stages with DALY supervision; startup RF recovery; Wi-Fi/OTA V3 (TLS, OTA ingest compiled out by default); charging presentation priority; **dev.2 read-only boot servo census**; **dev.3 thermal verdict from direct reads only** and the host runner's one-shot post-abort recovery; identity `0.2.0-dev.3`; pure motion library G1–G5-A, **unwired** (no path to the servo bus) |
| Offline tests | static audit and C++/Python host suites, USB_ONLY and ROBOT_POWERED builds: see [`VALIDATION.md`](05_Firmware/MATDOG_Controller/VALIDATION.md), section PR-3 |
| Hardware results (historical, of the dev.2/dev.3 candidate, not of a `main` build) | 2026-10-06: boot census 13/13; fresh Q0 and calibration 24/24 under dev.3; SAVE/ACK and LOAD across a reset (no real power cycle); the automatic post-abort recovery did not run. Records: [dev.2 audit](09_Logs/Validation_Reports/MATDOG_V0_2_DEV2_BOOT_SELFTEST_DELTA_AUDIT_2026-10-06.md), [dev.3 audit](09_Logs/Validation_Reports/MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md). The 2026-10-07 report is not in the repository |
| Hardware acceptance / release acceptance | none: dev.3 execution COMPLETE, acceptance BLOCKED; no `main` build accepted; no release approved |
| Deferred | motion execution suites, fresh oracles and the G35/G4/G4.1/G5-A evidence; 38 third-party XGO extracts and XGO-derived data are **not reachable from `main`**; they remain present on separate public historical branches ([ADR-004](09_Logs/Architecture_Decisions/ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md)) |

> **Historical snapshot — 2026-10-01 (PR #35 merged)**
>
> - **Installed population:** 13 ST3215 (12 leg joints plus neck ID 51); 17 canonical slots,
>   with IDs 52–55 intentionally absent.
> - **Frozen release:** `matdog-controller-v0.1.0`; the separately hardware-validated development
>   firmware is `dfcecb670d05`, `ROBOT_POWERED`.
> - **TRUE Full Calibration HARDWARE-VALIDATED, 2026-10-01:** LF/RF/RH/LH each passed 6/6,
>   all **24/24 contacts** accepted, fine-pass repeatability <= 4 ticks, SAFE_OFF 13/13.
> - **Calibration is RAM-only.** Evidence is exported and backed up; boot-time restoration is
>   not implemented. Operational envelopes are not approved and stand/gait are not authorized.
> - The gait engine remained offline on 2026-10-01. Since PR-2 the pure motion library is on
>   `main`, unwired; its execution suites are deferred.
>
> [2026-10-01 full hardware report](09_Logs/Validation_Reports/Full_Calibration_24_Contact_Hardware_2026-10-01/README.md).

| Track | State |
|---|---|
| `ROBOT_POWERED` no-motion baseline | **VALIDATED** |
| TRUE Full Calibration software and hardware | **24/24 HARDWARE-VALIDATED — 2026-10-01** |
| Calibration Persistence V1 | **IMPLEMENTED on `main` (PR-1, 2026-10-08)** — offline/host-tested; RESTORE not implemented; no hardware acceptance |
| Operational envelopes / JointLimits | **BLOCKED** — not approved |
| Telemetry integrity, LOWER MAX margins, q0 refinement | **OPEN follow-ups** |
| Gait engine | **IN PROGRESS** — pure library on `main` (PR-2), unwired; execution suites and evidence DEFERRED |
| Stand / gait hardware | **BLOCKED** |

The step-by-step route, and what blocks what, is in [`ROADMAP.md`](01_Docs/02_Architecture/ROADMAP.md);
the calibration state in detail is under *Calibration on `main`* below.

## Status vocabulary

Every current-facing document uses these meanings:

| Label | Meaning |
|---|---|
| **VALIDATED** | Implemented and exercised within a stated real-hardware scope. |
| **IMPLEMENTED** | Present in code or hardware, but not yet fully exercised in the relevant hardware scope. |
| **DECIDED** | An approved architecture or policy; implementation may be partial or absent. |
| **TO_TEST** | Implemented or assembled enough for a defined validation step that has not yet passed. |
| **TO_DESIGN** | Direction is known, but the detailed design is not frozen. |
| **FROZEN** | Immutable release, tool, or evidence baseline. |
| **SUPERSEDED** | Replaced as current truth; retained only for traceability. |
| **HISTORICAL** | Accurate evidence of an earlier state, never current authorization. |

## Project status

### VALIDATED

- The 17 allocated ST3215 units completed bench QC/provisioning and hold the frozen
  `MATDOG_C018_V1` persistent profile. This proves the bench campaign, not present installation.
- MATDOG Controller V0.1 boot, native USB CDC, live BNO085 acquisition, viewer-compatible output,
  expected-offline classification, and concurrent soak passed on the real ESP32-S3 under the
  `USB_ONLY` profile.
- Direct ESP32-S3 ownership of the ST3215 bus was demonstrated by the frozen bench tools.
- `ROBOT_POWERED` no-motion operation on the real robot (G3 formal PASS and G3.1 PASS,
  2026-09-18): the hardware profile, the servo population model (canonical 17 / expected-now 13 /
  absent-by-design 52-55) and structured census, live read-only DALY, live LED, two identical
  censuses, `SAFE_OFF` `VERIFIED_OFF` and `torque=0` on all 13, and a Controller loop independent
  of USB CDC host presence. That validation applies to the firmware of that date. The dedicated bounded-calibration
  path subsequently passed its own 24-contact hardware gate on 2026-10-01; operational motion
  has not been validated.
- The post-rewire power gate (2026-09-24): the historical `B-`/`P-` bypass is corrected, and
  physical KEY OFF now measures 0 V on every protected rail with no charger and no USB present.
- LED Status Manager V2 (2026-09-26, source `88062e1`) and the read-only q0 capture on the real
  robot (CR2-C, 2026-09-27, build `315d4ade6ff0`): 12/12 joints, nine samples per joint, Torque
  OFF, maximum spread 0 ticks. It proves the capture path, not calibration.
- The external USB service/programming port (2026-09-24): GPIO19/GPIO20/GND, no host VBUS —
  enumeration, bidirectional CDC and the `esptool` reset/flash-identification handshake all
  confirmed with the onboard USB-C disconnected.
- Manual charging, common-port electrical behaviour (2026-09-22/23 + 2026-09-24): a connected
  charger backfeeds the `B+`/`P-` load bus directly, independent of KEY/Discharge-MOS state. Full
  charging/dock qualification remains **FUTURE**.

- Bounded four-leg calibration passed on hardware on 2026-10-01 (firmware `dfcecb6`):
  24/24 contacts and SAFE_OFF 13/13. This does not validate operational motion.

### IMPLEMENTED

- One modular Controller image contains ServoBus diagnostics, BNO085 acquisition, read-only DALY
  telemetry, LED-ring control, USB diagnostics, health aggregation, and a power-state baseline.
- Wi-Fi/OTA core is implemented and offline-tested; the relevant hardware gates remain open.
- `ActuatorAuthority`, the Safe Actuator policy and the four-leg Full Calibration engine are
  integrated. Only the bounded calibration write path has been hardware-validated; service/QC
  writes and operational stand/gait have not been validated.
- Calibration Persistence V1 (record, A/B store in the dedicated `matdog_nvs` partition,
  SAVE/ACK/RECONCILE, boot LOAD) and the flash layout `MATDOG_16M_2x5M_NVS_V1` are on `main`
  since PR-1 (2026-10-08), offline/host-tested. LOAD never restores a calibration into the
  motion path (`RESTORE` not implemented). Approved operational envelopes are not implemented.

### DECIDED

- MATDOG will use one permanent Controller firmware, extended by reviewed modules rather than
  replaced by separate operational sketches.
- The ESP32-S3 is the sole operational ST3215 bus owner. The high-level host sends semantic intent,
  never raw bus traffic in normal operation.
- Normal future updates will use Wi-Fi/OTA. Native USB CDC/USB-C remains the permanent wired
  service and recovery path.
- GPIO19/GPIO20 mean native USB D-/D+; their historical Jetson-UART use is **SUPERSEDED**.
- The ESP32-S3 is powered from the DALY-protected B+/P− domain through the 5 V step-down. The
  bistable logo pushbutton connects directly to DALY `KEY`; no ESP32 KEY GPIO is required. Its
  function is now a **VALIDATED** power-off (2026-09-24, no charger/USB present) — the fused
  disconnect remains the trusted isolation whenever a charger may be connected, because a
  connected charger backfeeds the load bus independent of KEY state (see *Where we are* below).

### TO_TEST

- Telemetry integrity following 23 isolated temperature outliers and 39 diagnostic held-speed
  transients in the successful hardware run (no confirmed thermal fault or held-role failure).
- LOWER MAX shortfall and reduced repeat-calibration scout margins, particularly RF (+6 ticks)
  and LH (+13 ticks); separate reviewed resolution, no unilateral corridor change.
- BMS KEY configuration persistence across a true DALY power cycle.
- Charging/dock qualification beyond the attended manual session.

### TO_DESIGN

- The host command/telemetry protocol carried over USB CDC.
- Calibration restore across reboot (`RESTORE`), the operational stand/gait workspace with its
  joint limits and margins, and the integrated maintenance, service, Servo QC and provisioning
  workflows.
- Stand and gait motion on hardware, stabilization, ROS 2/MoveIt 2, and complete Jetson
  integration. The pure motion library is on `main` since PR-2 but unwired and not operational;
  its execution suites and evidence are deferred.

### FROZEN, SUPERSEDED, and HISTORICAL

- **FROZEN:** the official Controller tag and the three ST3215 bench tools: Bench QC V6.1, Source
  Signature Survey V1, and Provisioner V6.
- **SUPERSEDED:** mandatory NormaCore Station bus ownership, GPIO19/20 Jetson UART, and Generic V25
  as the current development milestone.
- **HISTORICAL:** dated calibration reports, Geometry/Phase 2A records, old handoffs, and previous
  Station-mediated evidence. They remain preserved and do not describe today's robot.

## Physical hardware today

The allocation registry and the installed population answer different questions:

| Fact | Current value | Canonical source |
|---|---:|---|
| Allocated unit/joint/bus-ID slots | **17** | [`MATDOG_SERVO_ALLOCATION.yaml`](06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml) |
| Physically installed servos | **13** | This current project snapshot |
| Installed population | 12 leg servos + `NECK_ROTATION` ID 51 | This current project snapshot |
| Allocated but intentionally absent | 52 `NECK_PITCH`, 53 `HEAD_ROTATION`, 54 `HEAD_PITCH`, 55 `JAW` | Allocation registry + this snapshot |

Do not reduce the 17-unit registry to match the current 13-unit build. Allocation is a design/ID
contract; installation is physical state.

```text
ASUS Ubuntu development host / future Jetson Orin Nano Super
    -> native USB 2.0 Full-Speed / USB CDC
MATDOG Controller on ESP32-S3
    -> UART: GPIO17 TX, GPIO18 RX, shared GND
Seeed Bus Servo Driver
    -> Feetech serial bus, 1 Mbps
13 physically installed ST3215 servos
    = 12 legs + NECK_ROTATION ID51
```

All 17 allocated units were bench-provisioned with `PositionOffset = 0`, a servo-level
fact distinct from joint calibration. The current installation passed TRUE Full Calibration
24/24 on 2026-10-01. The 12 fresh q0 values, contacts and diagnostics were exported and
archived. They are RAM-only in the Controller: after a power cycle no automatic restore exists,
no motion is thereby authorized, and no servo EEPROM, ID, `PositionOffset` or NVS write occurred.

## Official firmware baseline

| Item | Value |
|---|---|
| Frozen release | `matdog-controller-v0.1.0` |
| Tagged repository commit | `c54862f38a9cbd5e46d6b1770a6d109cc99b5c02` |
| V0.1 USB_ONLY hardware-validated source | `5b371da5482f9b0bd2df1c37ed361250ea54ae8f` |
| Latest hardware-validated development firmware (scope: 2026-10-01 Full Calibration) | `dfcecb670d0565d2db1a8152b6cd7ad230bdb87d` (`ROBOT_POWERED`) |
| Installed candidate (2026-10-06/07) | `0.2.0-dev.3`, provenance `b3fd945bdaf37d192d97b605ac0f59b67f1dba45`; execution COMPLETE, acceptance **BLOCKED**; not an accepted release |
| Full Calibration merge commit on main | `1fd0f5afc3cc737d1ac82183b4ce204dcd01c402` (PR #35; documentation included) |
| Hardware report | [hardware-validation report](09_Logs/Validation_Reports/Full_Calibration_24_Contact_Hardware_2026-10-01/README.md) |

The frozen release, the hardware-flashed source and the later Git merge commit are distinct
identities. The merge commit itself was not flashed.

## Where we are, and the next gate

```text
COMPLETE   Controller V0.1 baseline              VALIDATED (USB_ONLY hardware)
COMPLETE   G2 ROBOT_POWERED software             PASS / FROZEN
COMPLETE   G3 ROBOT_POWERED no-motion            PASS (formal, 2026-09-18)
COMPLETE   G3.1 CDC-independent Controller loop  PASS (2026-09-18)
COMPLETE   DALY KEY research                     COMPLETE (read-only, 2026-09-19)
COMPLETE   DALY 0x81 read                        LIVE VERIFIED (read-only, 2026-09-19)
COMPLETE   DALY KEY write (0x0120 := 0x005A)     LIVE VERIFIED (2026-09-19; ACK + read-back)
                                                 current KEY logic = DISCHARGE (0x005A)
COMPLETE   B-/P- hardware bypass correction      VERIFIED (2026-09-24; TECNOIOT VIN- now on P-)
COMPLETE   post-rewire power gate A-E            PASS (2026-09-24; dead-circuit + KEY OFF/ON +
                                                 powered no-motion regression, live)
COMPLETE   external USB service/programming port VALIDATED (2026-09-24; GPIO19/20, no host VBUS —
                                                 enumeration, CDC, esptool reset/flash-ID)

OPEN       manual charging common-port behaviour VERIFIED electrically (2026-09-22/23 + 09-24):
                                                 a connected charger backfeeds B+/P- regardless
                                                 of KEY state
OPEN       autonomous dock/charging              OPEN — no dock hardware evidence beyond one
                                                 attended manual charging session

COMPLETE   read-only q0 capture              VALIDATED (CR2-C, 2026-09-27)
SUPERSEDED UPPER-only 8-contact milestone      Historical scope error
COMPLETE   TRUE 24-contact calibration         HARDWARE PASS (2026-10-01, PR #35)
CURRENT    Calibration Persistence V1          IMPLEMENTED on main (PR-1); RESTORE not
                                                 implemented; no hardware acceptance
OPEN       Telemetry, LOWER MAX, zero review   Before hardware locomotion
THEN       Operational envelopes / JointLimits BLOCKED pending approval
THEN       Stand / gait hardware               BLOCKED
```

Power domains, KEY semantics, every power state and the charging gates are owned by
[`04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md`](04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md).

`ROBOT_POWERED` is **VALIDATED for no-motion operation**: DALY live read-only, LED live, 13/13
expected servos present with 4 absent by design in two identical censuses, `SAFE_OFF`
`VERIFIED_OFF` and `torque=0` on all 13. The Controller loop is independent of USB CDC host
presence: BNO085 acquisition runs at 50.1 Hz with the port closed (G3.1). No commanded servo motion occurred and no robot motion was observed during validation.

### Calibration on `main`

- **Validated scope:** LF V25-derived full-leg state machine, generalized to four legs and 24
  physical contacts (HIP/UPPER/LOWER, MIN/MAX), `HARDWARE_CONTACT_CALIBRATED` 6/6 per leg.
- **Hardware session:** 2026-10-01, firmware `dfcecb6`, `ROBOT_POWERED`, 24/24 witnesses
  accepted on the first attempt and SAFE_OFF 13/13 verified. [hardware-validation report](09_Logs/Validation_Reports/Full_Calibration_24_Contact_Hardware_2026-10-01/README.md).
- **Authority:** session-scoped permits, promoted fresh q0 and the Safe Actuator layer govern
  all bounded calibration writes. No operational actuator authority is implied.
- **Persistence:** in the 2026-10-01 session the promoted calibration was RAM-only and the
  exported data is not an implemented restore. Since PR-1 `main` contains Calibration
  Persistence V1 (SAVE/ACK/LOAD); `RESTORE` is still not implemented.
- **Operational envelope:** not approved (`parameters_approved=0`, `envelope_accepted=0`);
  no stand or gait hardware motion is authorized.
- **Persistence status on `main` (PR-1):**
  - Calibration Persistence V1 **IMPLEMENTED**: SAVE/ACK/RECONCILE and boot LOAD,
    offline/host-tested.
  - `RESTORE` **NOT_IMPLEMENTED**.
  - **No hardware acceptance** for any `main` build.
  - A real power cycle of the persistence path is **not validated**.
- **Follow-ups:** persistence hardware acceptance (including a real power cycle) and
  `RESTORE`; telemetry integrity; LOWER MAX scout margin;
  reviewed mechanical-zero refinement; operational workspace and limits.

### Open hardware notes

- **DALY `KEY` — RESOLVED as a power-off, 2026-09-24.** Research (2026-09-19): no public DALY
  document publishes a K-series KEY register; DALY's own BMSTool V1.14.79 (static inspection)
  points to KEY logic at `0x0120` on a second Modbus personality (`0x81`). Live-verified read-only
  the same day with `@BMS KEY READ`: the unit's KEY logic was **DISABLED (`0x0055`)**
  pre-commissioning, consistent with the G3 finding, where toggling the physical KEY produced no
  observed change in `discharge_mos`. The single guarded write (`@BMS KEY SET DISCHARGE CONFIRM`,
  `0x0120 := 0x005A`) was sent **once, live, on 2026-09-19** and read back as `0x005A`
  (DISCHARGE). The follow-on physical KEY test was inconclusive at the time: a hardware `B-`/`P-`
  bypass (TECNOIOT `VIN-` on raw `B-`) kept the load rail powered with the discharge MOS open. That
  rewire is now **complete**, and the post-rewire power gate A–E **passed live on 2026-09-24**:
  with no charger and no USB present, KEY OFF measures 0 V on every protected rail. A connected
  charger still backfeeds the `B+`/`P-` bus independent of KEY state, so the fused disconnect
  remains the trusted physical isolation method whenever a charger may be present.
- **GPIO19/GPIO20 — VALIDATED 2026-09-24.** GPIO19 = native USB D−, GPIO20 = native USB D+. The
  external 19/20/GND connector (host VBUS intentionally not wired) was validated as a
  service/programming port: native enumeration, bidirectional CDC and the `esptool`
  reset/flash-identification handshake all confirmed with the onboard USB-C disconnected. It does
  not power the ESP32 on its own.

Criteria are owned by the Controller
[`DEVELOPMENT_GATES.md`](05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md); evidence by the
Controller [`VALIDATION.md`](05_Firmware/MATDOG_Controller/VALIDATION.md).

### Everything after that

The full development sequence, its hard dependencies and what is blocked by what are owned by
[`ROADMAP.md`](01_Docs/02_Architecture/ROADMAP.md). Per-gate entry conditions and pass/fail
criteria are owned by
[`DEVELOPMENT_GATES.md`](05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md). They are
deliberately not duplicated here.

## Target architecture

The permanent MATDOG Controller is the integration point for future Diagnostics, Maintenance,
Service, Servo QC, Provisioning, Full Leg Calibration, Wi-Fi/OTA, and host transport. Calibration
motion is implemented behind the Safe Actuator layer; stand, IK, gait, and stabilization come
later, behind explicit safety and calibration gates.

**DECIDED (2026-09-16):** MATDOG will also host a permanent **Embedded Web UI / Control & Service
Dashboard** served by the ESP32-S3 and reached from a phone, tablet, the ASUS or a future Jetson.
It is a staged, safety-gated target and none of it exists in firmware yet. Every browser command
must travel `Browser -> CommandRouter -> Controller services -> authority -> Safe Actuator ->
ServoBus`; a direct browser-to-`ServoBus` path is permanently forbidden. Contract in
[`ARCHITECTURE.md`](01_Docs/02_Architecture/ARCHITECTURE.md#embedded-matdog-web-ui--control--service-dashboard).

The Full Leg Calibrator V1 tree is a preserved oracle/evidence source, held by the annotated tag
`archive/2026-08-29/full-leg-calibrator-v1-h0` -> `15f3fb8f378e6cadf6bc479bfcaca2947741c9fd` (the
former branch `matdog/full-leg-calibrator-v1` was archived 2026-09-18 and no longer exists). Its
useful calibration engine, safety, and evidence patterns may later be migrated selectively into
the Controller; it is not the final runtime architecture and must not be merged wholesale.

The frozen ST3215 tools remain immutable qualification evidence even after equivalent service
capabilities are integrated into the Controller.

## Canonical documentation

| Question | Canonical owner |
|---|---|
| What exists and what happens next? | This `README.md` |
| What are the architecture contracts and target direction? | [`ARCHITECTURE.md`](01_Docs/02_Architecture/ARCHITECTURE.md) |
| What is the development sequence, and what blocks what? | [`ROADMAP.md`](01_Docs/02_Architecture/ROADMAP.md) |
| What must be true before a stage may begin? | [`DEVELOPMENT_GATES.md`](05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md) |
| How are power, wiring, and connectors implemented? | [`04_Electronics/README.md`](04_Electronics/README.md) |
| What firmware exists? | [`05_Firmware/README.md`](05_Firmware/README.md) |
| May `main` be flashed on the current robot? | [`FLASH_LAYOUT_SAFETY_NOTICE.md`](05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md) — **no** |
| What is the Controller baseline and service model? | [Controller README](05_Firmware/MATDOG_Controller/README.md) |
| What Controller behavior was actually validated? | [Controller validation](05_Firmware/MATDOG_Controller/VALIDATION.md) |
| Which 17 units/joints/IDs are allocated? | [`MATDOG_SERVO_ALLOCATION.yaml`](06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml) |
| Is calibration commandable? | [`MATDOG_JOINT_CALIBRATION.yaml`](06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml) |
| Where is frozen ST3215 evidence? | [`ST3215_EVIDENCE_INDEX.md`](09_Logs/ST3215_EVIDENCE_INDEX.md) |
| Where is superseded/historical material? | [`09_Logs/Historical/`](09_Logs/Historical/README.md) |
| Where is Geometry Compiler verification evidence? | [`REPOSITORY_VERIFICATION_INDEX.md`](REPOSITORY_VERIFICATION_INDEX.md) (**HISTORICAL/REFERENCE**, not project status) |

## Repository layout

```text
01_Docs/        architecture and stable technical references
02_BOM/         components, suppliers, and costs
03_CAD/         CAD, URDF, meshes, and mechanical exports
04_Electronics/ wiring, power, and servo-mapping references
05_Firmware/    permanent Controller plus frozen bench tools
06_Software/    calibration, geometry, kinematics, and host-side software
07_Media/       images, renders, and videos
08_Tests/       repeatable validation procedures
09_Logs/        evidence, decisions, reports, and historical archives
```

## Safety boundary

- No motion from stale or un-restored calibration. The 2026-10-01 contact validation
  does not authorize stand, gait or load-bearing operation. Any new hardware calibration
  or motion session requires separate operator authorization.
- No servo EEPROM/ID change or hardware reflash is authorized by this documentation. In
  particular, never flash a `main` build onto the current robot (see the flash safety notice
  at the top of this file).
- `PositionOffset = 0` is the persistent baseline; mechanical mounting errors are corrected
  mechanically, not hidden in EEPROM.
- Frozen tools and historical evidence are never edited to make them appear current.
- `UNRESOLVED != PASS`, `DIAGNOSTIC != EXECUTABLE`, and geometric contact is not motion
  authorization.

Built and documented by Matt Robotics.
