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

### IMPLEMENTED

- One modular Controller image contains ServoBus diagnostics, BNO085 acquisition, read-only DALY
  telemetry, LED-ring control, USB diagnostics, health aggregation, and a power-state baseline.
- `@SERVO SAFE_OFF` performs an independent torque-state readback, but a real powered-servo
  `VERIFIED_OFF` result is still **TO_TEST**.
- DALY, LED, and servo modules coexist in firmware; their powered-robot behavior is not yet
  validated.
- The `ROBOT_POWERED` hardware profile, servo population model (canonical 17 / expected-now 13 /
  absent-by-design 52-55) and structured census classification exist and are offline-tested on
  branch `feat/controller-robot-powered-v02`. The powered configuration itself remains **TO_TEST**.

### DECIDED

- MATDOG will use one permanent Controller firmware, extended by reviewed modules rather than
  replaced by separate operational sketches.
- The ESP32-S3 is the sole operational ST3215 bus owner. The high-level host sends semantic intent,
  never raw bus traffic in normal operation.
- Normal future updates will use Wi-Fi/OTA. Native USB CDC/USB-C remains the permanent wired
  service and recovery path.
- GPIO19/GPIO20 mean native USB D-/D+; their historical Jetson-UART use is **SUPERSEDED**.
- The ESP32-S3 is powered from the DALY-protected B+/P− domain through the 5 V step-down. The
  bistable logo pushbutton connects directly to DALY `KEY`; no ESP32 KEY GPIO is required.

### TO_TEST

- The exact `ROBOT_POWERED` no-motion gate described below.
- The external four-pin connector (GPIO19, GPIO20, GND, 5 V not connected) as a USB-data/service
  connection.
- Full powered-system coexistence and the 13 installed servos as a population.

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
COMPLETE   G2 ROBOT_POWERED preparation          PASS (software/offline only, 2026-09-16)

NEXT GATE  G3 ROBOT_POWERED no-motion validation TO_TEST — never executed
```

`ROBOT_POWERED` **software** support exists on branch `feat/controller-robot-powered-v02`:
one hardware-profile authority, the canonical-17 / expected-now-13 / absent-by-design-4 servo
population model, structured census classification, and a fail-closed build-manifest gate that
proves which profile a binary was built for before it can be flashed. It is **IMPLEMENTED**, not
**VALIDATED** — the powered robot has never been energized.

### Immediate milestone: TO_TEST, no motion

```text
MATDOG Controller
-> ROBOT_POWERED Hardware Validation
-> no motion
-> DALY live read-only
-> LED live
-> 13 expected servos read-only (13 present + 4 absent by design = PASS)
-> SAFE_OFF real readback
-> concurrent soak
```

This sequence has not been performed. It must not include robot motion, servo EEPROM writes, ID
changes, calibration writes, or hardware reflashing as an incidental documentation step. The
procedure is designed in
[`G3_ROBOT_POWERED_VALIDATION_PLAN.md`](05_Firmware/MATDOG_Controller/G3_ROBOT_POWERED_VALIDATION_PLAN.md);
detailed validation ownership lives in the Controller
[`VALIDATION.md`](05_Firmware/MATDOG_Controller/VALIDATION.md).

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
