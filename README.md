# MATDOG — Custom Quadruped Robot

MATDOG is Matt Robotics' custom quadruped platform: a 17-DOF mechanical design with an
articulated head, an ESP32-S3 real-time controller, and a future Jetson-based high-level stack.
This repository is the single active engineering repository for the robot.

> **Current snapshot — 2026-09-15**
>
> - **13 servos are physically installed:** all 12 leg servos plus `NECK_ROTATION` (ID 51).
> - **17 servos remain canonically allocated.** IDs 52–55 are intentionally absent today.
> - **MATDOG Controller V0.1 is the official firmware baseline.** Its validated scope is the
>   `USB_ONLY` bench profile, not the powered robot.
> - The immediate milestone is the no-motion `ROBOT_POWERED` validation sequence.
> - Joint calibration remains reset. No stale calibration authorizes motion.

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
  of USB CDC host presence. No motion or calibration capability exists yet.

### IMPLEMENTED

- One modular Controller image contains ServoBus diagnostics, BNO085 acquisition, read-only DALY
  telemetry, LED-ring control, USB diagnostics, health aggregation, and a power-state baseline.

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
  actual function is **OPEN**: in G3 the KEY switch produced no observed DALY state change, so it
  is not a validated shutdown or safety barrier — the fused disconnect is.

### TO_TEST

- The DALY `KEY` function (see *Where we are* below).
- The external four-pin connector (GPIO19, GPIO20, GND, 5 V not connected) as a future USB
  service port — not a UART — pending electrical and signal-integrity validation.

### TO_DESIGN

- The host command/telemetry protocol carried over USB CDC.
- Integrated maintenance, service, Servo QC, provisioning, and full-leg calibration workflows.
- Motion control, operational IK, gait, stabilization, ROS 2/MoveIt 2, and complete Jetson
  integration.

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
installation and cannot authorize motion.

## Official firmware baseline

| Item | Value |
|---|---|
| Release | `matdog-controller-v0.1.0` (**FROZEN**) |
| Tagged repository commit | `c54862f38a9cbd5e46d6b1770a6d109cc99b5c02` |
| Hardware-validated firmware source | `5b371da5482f9b0bd2df1c37ed361250ea54ae8f` |
| Validated hardware profile | `USB_ONLY` |
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
                                                 current KEY logic = DISABLED (0x0055)
COMPLETE   Narrow KEY write (0x0120 := 0x005A)   OFFLINE VALIDATED (never sent to hardware)

NEXT       DALY KEY write live validation        PENDING (needs its own authorization)
PENDING    physical KEY behaviour with 0x005A    PENDING
OPEN       physical KEY safety barrier           NOT VALIDATED (fused disconnect = trusted isolation)
THEN       G4 Diagnostics / Maintenance          NOT STARTED
```

`ROBOT_POWERED` is **VALIDATED for no-motion operation**: DALY live read-only, LED live, 13/13
expected servos present with 4 absent by design in two identical censuses, `SAFE_OFF`
`VERIFIED_OFF` and `torque=0` on all 13. The Controller loop is independent of USB CDC host
presence: BNO085 acquisition runs at 50.1 Hz with the port closed (G3.1). No commanded servo motion occurred and no robot motion was observed during validation.

### Open hardware notes

- **DALY `KEY` — OPEN.** Toggling the physical KEY switch produced no observed change in any
  DALY-reported state (`discharge_mos=ON` in both positions). KEY is **not** a validated shutdown
  or safety barrier. Research (2026-09-19): no public DALY document publishes a K-series KEY
  register; DALY's own BMSTool V1.14.79 (static inspection) points to KEY logic at `0x0120` on a
  second Modbus personality (`0x81`). Live-verified read-only the same day with `@BMS KEY READ`:
  the unit's KEY logic is **DISABLED (`0x0055`)**, consistent with the G3 finding. A single
  guarded write (`@BMS KEY SET DISCHARGE CONFIRM`, `0x0120 := 0x005A` with read-back) is
  implemented and offline-validated but has never been sent; its live validation is pending. The
  fused disconnect remains the trusted physical isolation method.
- **GPIO19/GPIO20 — frozen.** GPIO19 = native USB D−, GPIO20 = native USB D+. The external
  19/20/GND connector remains a future USB service-port candidate — **not** a UART — pending
  electrical and signal-integrity validation.

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
Service, Servo QC, Provisioning, Full Leg Calibration, Wi-Fi/OTA, and host transport. Motion, IK,
gait, and stabilization come later, behind explicit safety and calibration gates.

**DECIDED (2026-09-16):** MATDOG will also host a permanent **Embedded Web UI / Control & Service
Dashboard** served by the ESP32-S3 and reached from a phone, tablet, the ASUS or a future Jetson.
It is a staged, safety-gated target and none of it exists in firmware yet. Every browser command
must travel `Browser -> CommandRouter -> Controller services -> authority -> Safe Actuator ->
ServoBus`; a direct browser-to-`ServoBus` path is permanently forbidden. Contract in
[`ARCHITECTURE.md`](01_Docs/02_Architecture/ARCHITECTURE.md#embedded-matdog-web-ui--control--service-dashboard).

The branch `matdog/full-leg-calibrator-v1` is a preserved oracle/evidence branch. Its useful
calibration engine, safety, and evidence patterns may later be migrated selectively into the
Controller; the branch is not the final runtime architecture and must not be merged wholesale.

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
- No servo EEPROM/ID change or hardware reflash is authorized by this documentation.
- `PositionOffset = 0` is the persistent baseline; mechanical mounting errors are corrected
  mechanically, not hidden in EEPROM.
- Frozen tools and historical evidence are never edited to make them appear current.
- `UNRESOLVED != PASS`, `DIAGNOSTIC != EXECUTABLE`, and geometric contact is not motion
  authorization.

Built and documented by Matt Robotics.
