# MATDOG — Custom Quadruped Robot

MATDOG is Matt Robotics' custom quadruped platform: a 17-DOF mechanical design with an
articulated head, an ESP32-S3 real-time controller, and a future Jetson-based high-level stack.
This repository is the single active engineering repository for the robot.

> **Current snapshot — 2026-09-29**
>
> - **13 servos are physically installed** (all 12 leg servos plus `NECK_ROTATION`, ID 51);
>   **17 remain canonically allocated** — IDs 52–55 are intentionally absent today.
> - **MATDOG Controller V0.1 is the official firmware baseline**, and `ROBOT_POWERED` no-motion
>   operation is **VALIDATED** (G3/G3.1, 2026-09-17/18).
> - **TRUE Full Calibration = 4 legs × 3 joints × MIN/MAX = 24 contacts.** The 24-contact
>   implementation (the LF V25 full-leg state machine for all four legs) is offline-validated on
>   its PR branch and has never run on the robot. The UPPER-only "Full Calibration" on `main`
>   (2 contacts per leg) was a scope error and is superseded. The hardware session is the
>   immediate gate.
> - Gait exists only as an offline engine on a separate, unmerged feature branch. No stand or
>   gait hardware motion is authorized.
> - Joint calibration is not accepted. No stale calibration authorizes motion.

| Track | State |
|---|---|
| `ROBOT_POWERED` no-motion baseline | **VALIDATED** |
| Full 4-leg calibration software (LF / RF / RH / LH) | **IMPLEMENTED** — host, audit, mutation and build gates PASS |
| Hardware calibration, LF → RF → RH → LH | **TO_TEST** — the immediate gate |
| Calibration Persistence V1, then accepted stand/gait workspace and joint limits | **NEXT** |
| Gait engine | **IN PROGRESS** — separate feature branch, not merged |

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
  of USB CDC host presence. That validation covers the firmware of that date; calibration
  and motion software merged since is **IMPLEMENTED**, not hardware validated.
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

### IMPLEMENTED

- One modular Controller image contains ServoBus diagnostics, BNO085 acquisition, read-only DALY
  telemetry, LED-ring control, USB diagnostics, health aggregation, and a power-state baseline.
- Wi-Fi/OTA core, `ActuatorAuthority`, the Safe Actuator policy core and the calibration
  domain/manager (**OFFLINE TESTED**; the calibration path is not yet exercised on hardware).
- Four-leg Full Calibration on `main`: fresh read-only q0 capture and current-q0 promotion, the
  Full Leg sequence for LF, RF, RH and LH, contact-evidence lifecycle and deterministic evidence
  export — see *Calibration on `main`* below. **Scope superseded 2026-09-30:** `main` measures
  UPPER only (8 contacts); TRUE Full Calibration is 24 contacts
  ([development log](09_Logs/Development_Log/2026-09-30_TRUE_24_CONTACT_FULL_CALIBRATION.md)).

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

- **Full 4-leg calibration on the robot (LF → RF → RH → LH)** — the immediate gate, runbook:
  [`FULL_CALIBRATION_4LEG_HARDWARE_RUNBOOK.md`](05_Firmware/MATDOG_Controller/FULL_CALIBRATION_4LEG_HARDWARE_RUNBOOK.md).
- BMS KEY-configuration persistence across a true DALY power cycle (see *Where we are* below).
- Charging qualification beyond the one attended manual session (autonomous dock/contact hardware,
  reverse-polarity protection, unattended charge acceptance/termination).

### TO_DESIGN

- The host command/telemetry protocol carried over USB CDC.
- Calibration Persistence V1 (across reboot), the operational stand/gait workspace with its
  joint limits and margins, and the integrated maintenance, service, Servo QC and provisioning
  workflows.
- Stand and gait motion on hardware, stabilization, ROS 2/MoveIt 2, and complete Jetson
  integration. A gait engine exists offline on `feat/gait-engine-offline-v1`; it is not merged
  and not operational.

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

All 17 allocated units were bench-provisioned with `PositionOffset = 0`, but that servo-level fact
is not joint calibration. The active calibration state remains
`CALIBRATION_RESET_PENDING_FULL_RECALIBRATION`; the preserved joint values describe an earlier
installation and cannot authorize motion. A live calibration session keeps its results in RAM
only: nothing is persisted across a reboot yet and no servo EEPROM value is ever written.

## Official firmware baseline

| Item | Value |
|---|---|
| Release | `matdog-controller-v0.1.0` (**FROZEN**) |
| Tagged repository commit | `c54862f38a9cbd5e46d6b1770a6d109cc99b5c02` |
| Hardware-validated firmware source | `5b371da5482f9b0bd2df1c37ed361250ea54ae8f` |
| Validated hardware profile | `USB_ONLY` |
| Build last flashed on the robot | `315d4ade6ff0` (2026-09-27, `ROBOT_POWERED`, read-only q0 capture); `main` is **not** flashed |
| Detailed evidence | [`05_Firmware/MATDOG_Controller/VALIDATION.md`](05_Firmware/MATDOG_Controller/VALIDATION.md) |

The tag identifies the official merged repository release. The earlier source SHA identifies the
exact firmware exercised in the final hardware session; subsequent differences in the tagged
Controller tree are documentation only. Neither identifier proves `ROBOT_POWERED` validation.

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

COMPLETE   read-only q0 capture on the robot     VALIDATED (CR2-C, 2026-09-27; 12/12, spread 0)
SUPERSEDED UPPER-only four-leg calibration on main  2026-09-29 - 8 contacts, NOT Full Calibration
COMPLETE   TRUE 24-contact Full Calibration sw   IMPLEMENTED / OFFLINE-VALIDATED (2026-09-30, PR branch)
NEXT       Full 4-leg calibration on hardware    TO_TEST — LF -> RF -> RH -> LH, 6/6 each = 24/24
THEN       Calibration Persistence V1            NEXT AFTER hardware Full Calibration PASS
THEN       Operational envelopes / JointLimits   BLOCKED until calibration is hardware accepted
THEN       Stand / gait hardware                 BLOCKED
```

Power domains, KEY semantics, every power state and the charging gates are owned by
[`04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md`](04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md).

`ROBOT_POWERED` is **VALIDATED for no-motion operation**: DALY live read-only, LED live, 13/13
expected servos present with 4 absent by design in two identical censuses, `SAFE_OFF`
`VERIFIED_OFF` and `torque=0` on all 13. The Controller loop is independent of USB CDC host
presence: BNO085 acquisition runs at 50.1 Hz with the port closed (G3.1). No commanded servo motion occurred and no robot motion was observed during validation.

### Calibration on `main`

- **Chain.** Every actuator write must pass the Safe Actuator layer: `ActuatorAuthority`, a fresh
  motion permit, a live calibration session and Geometry V5 provenance. A promoted q0 alone
  authorizes nothing.
- **q0.** `@CALIBRATION Q0 CAPTURE` reads 12/12 servos read-only with Torque OFF.
  `@CALIBRATION Q0 PROMOTE` then promotes **that same fresh capture** into the RAM transform
  table and replaces the previous q0. The frozen CR2-C values remain a comparison reference and
  are never promoted.
- **Full Leg Calibration** is generalized to LF, RF, RH and LH: UPPER MIN contact, an optional
  auxiliary park, UPPER MAX contact, `SAFE_OFF`, then the evidence lifecycle. **(Superseded
  2026-09-30: that is 2 of a leg's 6 contacts. TRUE Full Calibration runs all six — UPPER, LOWER,
  HIP × MIN/MAX — per leg, 24 in all.)** Parking: LF parks
  `LH_UPPER` (bus 42), RF parks `RH_UPPER` (bus 32), RH and LH need none. Contact evidence is
  exported deterministically.
- **Offline gates PASS:** host suite, static audit, Safe Actuator/DALY/LED mutation suites,
  `USB_ONLY` and `ROBOT_POWERED` builds. **Hardware: the four-leg calibration has not been run.**
- **Acceptance levels.** `HARDWARE_CONTACT_CALIBRATED` is the next hardware acceptance level.
  `FINAL_OPERATIONAL_ENVELOPE_ACCEPTED` is **not** yet available: no stand/gait workspace and
  margins are approved, so 0/12 final `JointLimits` are intentionally admitted — a boundary, not a
  failure.
- **Persistence.** Results live in RAM and are lost on reboot; nothing writes NVS, servo EEPROM,
  `PositionOffset` or servo IDs. **Calibration Persistence V1** is the gate right after the
  hardware Full Calibration PASS (see the roadmap).

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

- No motion from stale calibration; no stand, gait, or load-bearing attempt before recalibration.
  The Full Calibration on hardware is a separately authorized, attended session.
- No servo EEPROM/ID change or hardware reflash is authorized by this documentation.
- `PositionOffset = 0` is the persistent baseline; mechanical mounting errors are corrected
  mechanically, not hidden in EEPROM.
- Frozen tools and historical evidence are never edited to make them appear current.
- `UNRESOLVED != PASS`, `DIAGNOSTIC != EXECUTABLE`, and geometric contact is not motion
  authorization.

Built and documented by Matt Robotics.
