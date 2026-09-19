# MATDOG Architecture

**Canonical architecture decisions as of 2026-09-18.**

This document owns system contracts and target direction. It does not own the changing physical
population or next milestone; those live in the [root project snapshot](../../README.md). The
development sequence and its dependencies are owned by [`ROADMAP.md`](ROADMAP.md), the technical
pass/fail authorization criteria by
[`DEVELOPMENT_GATES.md`](../../05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md).
Implementation wiring lives in [electronics](../../04_Electronics/README.md), and exact firmware
evidence lives in [Controller validation](../../05_Firmware/MATDOG_Controller/VALIDATION.md).

## Status notation

| Label | Architectural use |
|---|---|
| **VALIDATED** | Implemented and exercised on real hardware within the linked scope. |
| **IMPLEMENTED** | Present, but not fully exercised in the relevant hardware configuration. |
| **DECIDED** | Approved contract or direction; implementation may be partial or absent. |
| **TO_TEST** | The implementation or connection still needs its defined validation gate. |
| **TO_DESIGN** | The detailed contract is not frozen. |
| **FROZEN** | Immutable release, tool, or evidence source. |

The complete repository vocabulary, including **SUPERSEDED** and **HISTORICAL**, is defined by the
[root README](../../README.md#status-vocabulary).

## System architecture

```text
HIGH-LEVEL HOST
  ASUS Ubuntu workstation                         current development host
  Jetson Orin Nano Super                          DECIDED future onboard host
  ROS 2 / MoveIt 2, AI, vision, voice, planning   TO_DESIGN / not integrated
          |
          | native USB 2.0 Full-Speed / USB CDC
          | GPIO19 = D-, GPIO20 = D+
          v
PERMANENT MATDOG CONTROLLER — ESP32-S3
  Controller V0.1 platform                        official baseline
  core / USB diagnostics / BNO085                 VALIDATED in USB_ONLY scope
  ServoBus / DALY / LED / power-state baseline    VALIDATED no-motion in ROBOT_POWERED (G3/G3.1)
  maintenance / service / calibration modules     TO_DESIGN
  motion / IK / gait / stabilization              TO_DESIGN, later
          |
          | UART: GPIO17 TX -> driver RX
          |       GPIO18 RX <- driver TX
          |       shared GND, 1 Mbps
          v
SEEED BUS SERVO DRIVER
          |
          v
FEETECH ST-3215-C018 BUS
  17 canonical allocation slots
  13 servos physically installed today
```

The Controller is a permanent runtime, not a disposable bring-up sketch. New operational
capabilities are integrated behind reviewed module and safety boundaries.

## Responsibility split

| High-level host | ESP32-S3 MATDOG Controller |
|---|---|
| ROS 2 / MoveIt 2 integration | sole ST3215 bus ownership |
| AI, vision, voice, and planning | joint-to-actuator conversion |
| UI and semantic behavior | deterministic real-time execution |
| high-level motion intent | IMU acquisition and future stabilization |
| logging and fleet/user workflows | battery telemetry, health, watchdog, and safety |
|  | future diagnostics, maintenance, service, QC, provisioning, and calibration |

The high-level host sends semantic intent. In normal operation it does not send raw encoder targets
or own the ST3215 protocol. The application-level host command/telemetry protocol is **TO_DESIGN**.

## Permanent Controller direction

One MATDOG Controller firmware will integrate, in reviewed stages:

1. Diagnostics
2. Maintenance
3. Service
4. Servo QC
5. Provisioning
6. Full Leg Calibration
7. Wi-Fi / OTA
8. host transport
9. Embedded Web UI / Control & Service Dashboard (see the dedicated section below)
10. later Motion / IK / Gait / Stabilization

Controller V0.1 is the platform baseline, not a motion controller. It already implements the
module boundaries and read-only/safety surfaces documented in the
[Controller README](../../05_Firmware/MATDOG_Controller/README.md). A listed target capability is
not validated merely because its precursor or standalone tool exists.

## Servo allocation versus installation

These are separate facts with separate owners:

- The canonical allocation is **17** unit/joint/bus-ID slots, owned by
  [`MATDOG_SERVO_ALLOCATION.yaml`](../../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml).
- The current installed population is owned by the [root README](../../README.md#physical-hardware-today):
  12 leg servos plus `NECK_ROTATION` ID 51, for **13 installed**.
- IDs 52 `NECK_PITCH`, 53 `HEAD_ROTATION`, 54 `HEAD_PITCH`, and 55 `JAW` remain allocated
  but are intentionally absent today.

Physical absence never authorizes deletion or reassignment of a canonical allocation entry.

## Servo-bus and calibration contracts

1. The ESP32-S3 is the sole operational owner of the ST3215 serial bus.
2. Exactly one bus owner may exist at a time.
3. `GoalPosition` uses the unsigned `0..4095` domain; signed wrap is forbidden.
4. `PositionOffset = 0` is the persistent baseline for all 17 allocated units.
5. Mechanical mounting error is corrected mechanically, never hidden by rewriting
   `PositionOffset`.
6. `CalibrationOfs`, one-key-middle, factory reset, and broadcast writes are forbidden.
7. The C018 model word is read at register `0x03` (expected `777`), never `0x00`.
8. Allocation, physical installation, servo persistent state, and joint calibration remain
   separate layers.
9. Only current-installation calibration can authorize motion.

The machine-readable calibration state remains
`CALIBRATION_RESET_PENDING_FULL_RECALIBRATION` with hardware motion unauthorized in
[`MATDOG_JOINT_CALIBRATION.yaml`](../../06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml).
Dated calibration results remain evidence of earlier installations.

## Host transport, service, and recovery

### Native USB

- **DECIDED:** native USB 2.0 Full-Speed / USB CDC is the wired host/service transport.
- **VALIDATED:** Controller V0.1 communicates through the ESP32-S3's native onboard USB connection
  in the `USB_ONLY` validation profile.
- **DECIDED:** GPIO19 = USB D- and GPIO20 = USB D+.
- **SUPERSEDED:** the historical use of GPIO19/GPIO20 as a Jetson UART.
- The higher-level command/telemetry protocol over CDC remains **TO_DESIGN**.

An existing external four-pin connector carries GPIO19, GPIO20, ESP32 GND, and an unconnected 5 V
position. It is a USB-data/service predisposition, **TO_TEST** on the physical connector. The
onboard native-USB validation does not validate that external wiring.

### Update policy

- **DECIDED:** Wi-Fi/OTA is the normal future firmware-update path.
- **DECIDED:** native USB CDC/USB-C remains available for wired service and recovery.
- OTA must never remove or make wired recovery dependent on a working application image.
- Controller V0.1's application-partition USB flashing procedure is not a Wi-Fi/OTA
  implementation.

Detailed service/update semantics belong to the
[Controller README](../../05_Firmware/MATDOG_Controller/README.md).

## Power and wake contract

```text
battery B+
   -> DALY-protected power domain
   -> 5 V step-down
   -> ESP32-S3 / logic

system return = DALY P-
never use B- as the protected-load return

bistable pushbutton under the MATDOG logo
   -> DALY KEY directly
   -> no ESP32 GPIO
```

The KEY path itself is **OPEN**: in G3 the physical KEY switch produced no observed DALY state
change, so it is not yet a validated shutdown or safety barrier. The DALY's KEY configuration is
read-only observable through `@BMS KEY READ` (`0x81` register map from DALY BMSTool V1.14.79,
live-verified read-only 2026-09-19: KEY logic **DISABLED**, `0x0055`). The only DALY write is the
guarded `@BMS KEY SET DISCHARGE CONFIRM` (`0x0120 := 0x005A`, with read-back), offline-validated
and not yet sent to the BMS.

The Controller cannot be the primary wake source because it is powered downstream of the DALY
protected domain. The hardware button is the primary ON/OFF/wake interface. Electronics owns the
detailed implementation record and validation state.

## Validation boundary

### VALIDATED

- Direct ESP32-S3 operation and ownership of the ST3215 bus in the frozen bench campaigns.
- Bench QC and provisioning of all 17 allocated units.
- Controller V0.1 boot, USB CDC, live BNO085 acquisition, viewer-compatible output,
  expected-offline classification, and soak under `USB_ONLY`.

### VALIDATED no-motion in ROBOT_POWERED (G3 / G3.1, 2026-09-18)

- Read-only DALY decode/polling.
- LED-ring module and anti-back-power behavior.
- Servo scan/read diagnostics.
- Independent readback classification after `@SERVO SAFE_OFF`.
- Multi-module health and power-state baseline.
- **The `ROBOT_POWERED` hardware profile itself** (2026-09-16, gate G2): one profile authority with
  profile-derived module expectations, the canonical-17 / expected-now-13 / absent-by-design-4
  servo population model, structured census classification, and the build-manifest flash-profile
  provenance gate. Validated no-motion on the powered robot, together with a Controller loop that
  does not depend on USB CDC host presence (G3.1). Evidence in
  [`VALIDATION.md`](../../05_Firmware/MATDOG_Controller/VALIDATION.md).

### OPEN

- DALY `KEY` function — not a validated shutdown or safety barrier; the fused disconnect is the
  trusted isolation method until the KEY investigation. Next steps are owned by the
  [root snapshot](../../README.md#where-we-are-and-the-next-gate).

## Embedded MATDOG Web UI / Control & Service Dashboard

**DECIDED — permanent architecture. Implementation is staged and safety-gated; none of it exists
in the firmware today.**

MATDOG shall host a permanent responsive Web UI served by the ESP32-S3 and rendered by the browser
of a phone, tablet, the ASUS workstation or a future Jetson. This supersedes the narrower idea of
using Wi-Fi only for OTA and telemetry. It is MATDOG's low-level human interface for the robot
lifecycle, not merely an IMU debug page.

The ESP32-S3 serves static assets, exposes bounded telemetry, and accepts semantic commands through
the same Controller service layer used by USB/host transports. The browser performs layout,
interaction, WebGL rendering and interpolation between telemetry updates. **No 3D computation
belongs on the ESP32-S3.**

Progressive functional target:

```text
Overview / system health        Calibration            Body Pose
BNO085 3D viewer                Service / Provisioning Single Leg XYZ
Battery / DALY                  QC                     Manual Teleoperation
Servo census / telemetry        OTA                    Gait Selection
Diagnostics / Maintenance       Joint Test             Preset Actions
                                                       Stabilization
```

### Mandatory command path

Every command origin — USB CDC, Web, or a future host — converges on one safety and ownership
model. There is exactly one semantic Controller implementation behind all of them.

```text
Browser
  -> Web/API transport
  -> CommandRouter
  -> Controller Service Layer
  -> OperatingMode / ActuatorAuthority
  -> functional subsystem / Safe Actuator
  -> ServoBus
```

### Forbidden architectures

These are permanent prohibitions, enforced by review and by
[`static_audit.py`](../../05_Firmware/MATDOG_Controller/scripts/static_audit.py), which already
fails the build if a translation unit references both network transport symbols and servo
primitives:

```text
Browser -> ServoBus                              FORBIDDEN
Browser -> raw GoalPosition                      FORBIDDEN
Browser -> arbitrary servo register/EEPROM write FORBIDDEN
network callback -> direct actuator primitive    FORBIDDEN
transport parser -> duplicate hardware path      FORBIDDEN
```

The permanent rule is `network callback != servo command authority`. Network tasks may parse,
authenticate, queue high-level commands and expose telemetry. They may not call servo-write
primitives.

### Telemetry snapshot model

The Web UI must never query physical hardware in response to a browser refresh, REST request or
WebSocket message. Acquisition rate and browser render rate are independent:

```text
hardware acquisition -> subsystem state -> Controller-owned telemetry snapshot
                     -> transport adapters -> USB / Web UI / future host
```

G2 established the first concrete instance of this: `ServoCensus` holds a structured, copyable
`CensusResult` with no `Serial` dependency, and `CommandRouter` only formats it. A future Web
adapter renders the same result without re-scanning the bus.

### Continuous command lease / deadman

A browser joystick must never create an indefinite sticky velocity command. Continuous commands
require a **firmware-side** lease/deadman — not JavaScript:

```text
control held      -> periodic semantic command + sequence/timestamp
                  -> Controller accepts while the lease is fresh

Wi-Fi drop / browser close / phone sleep / tab suspended /
stale sequence / expired heartbeat / session loss
                  -> command authority expires
                  -> Controller executes the defined safe-stop policy
```

Timeout values are not frozen here; they require real timing measurements first.

### Resource isolation and authorization

Realtime control has priority over the web subsystem: non-blocking networking, bounded client
count, bounded queues, no unbounded allocation in hot paths, compressed static assets sized against
the real partition layout. Network failure must not starve `ServoBus`, BNO085, DALY or motion.

Once the dashboard can command the robot, browser access is a control surface. It requires session
identity, explicit service/calibration authorization, command-origin tracking, transaction IDs,
rejection of stale/replayed continuous commands, and read-only versus write-capable separation.
**Network presence alone never grants provisioning, calibration or motion authority.**

### Reference research boundary

XGO Lite and Yahboom DOGZILLA-Lite are used as **control-surface and operator-contract references
only**. MATDOG must never import their joint/servo limits, body geometry, translation ranges, gait
tuning constants or joint-zero definitions. MATDOG uses its own URDF, calibration, hardware profile,
actuator limits and safety envelope.

```text
COPY THE CONTROL-SURFACE CONCEPTS
DO NOT COPY ROBOT-SPECIFIC NUMBERS OR HIDDEN MOTION ASSUMPTIONS
```

### Relationship to the existing engineering viewer

The host-side BNO085 full-body viewer under `06_Software/Matdog_Core/viewer/` is an engineering
tool and is **not** the embedded dashboard. Both are kept: the ESP32 dashboard for status, mobile
access, maintenance, calibration, service and OTA; the ASUS/Jetson tools for high-rate logging,
plots, replay, ROS 2, RViz/MoveIt and camera/AI visualization. Low-level maintenance and
calibration must never depend on Jetson availability.

### Staging

UI gate sequence and per-gate criteria live in
[`DEVELOPMENT_GATES.md`](../../05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md#embedded-web-ui-gates).
Software-only UI scaffolding may be prepared early, but **write-capable controls remain disabled
until the corresponding firmware gate passes**, and the UI derives its enable/disable state from
authoritative Controller state.

## Frozen tools and calibration oracle

The following ST3215 tools are **FROZEN** immutable evidence:

- Bench QC V6.1
- Source Signature Survey V1
- Provisioner V6

They remain qualification instruments, not editable Controller modules. Equivalent future
maintenance/service features must be integrated without rewriting their source or historical
evidence.

The branch `matdog/full-leg-calibrator-v1` is preserved as an oracle/evidence branch. It is not
the final runtime architecture. Selected calibration-engine, safety, and evidence logic may be
migrated later into the permanent Controller; the branch must not be rebased, deleted, or merged
wholesale for that purpose.

## Repository responsibilities

| Location | Responsibility |
|---|---|
| `MattRobotics/robot-dog` | sole active MATDOG engineering repository |
| `README.md` | current project/physical snapshot and immediate milestone |
| `01_Docs/02_Architecture/ARCHITECTURE.md` | architecture contracts and target direction |
| `01_Docs/02_Architecture/ROADMAP.md` | development sequence, dependencies, and current position |
| `05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md` | per-gate entry conditions and pass/fail criteria |
| `04_Electronics/README.md` | power, wiring, connectors, and their validation state |
| `05_Firmware/MATDOG_Controller/` | permanent Controller firmware, design, and validation |
| `05_Firmware/ST3215_Bench_Tools/` | frozen qualification tools |
| `06_Software/Matdog_Core/` | robot-specific configuration, calibration, geometry, kinematics, and host tools |
| `09_Logs/` | evidence, decisions, chronological records, and historical archives |
| `MattRobotics/norma-core` | upstream/reference repository only; no active MATDOG ownership |

## Superseded architecture

The Station-mediated runtime, Waveshare-as-production-driver path, GPIO19/20 Jetson UART, and
Generic V25 as the immediate project milestone are **SUPERSEDED**. Their evidence remains
**HISTORICAL** under [`09_Logs/Historical/`](../../09_Logs/Historical/README.md) and dated logs.

[`CURRENT_STATE.md`](CURRENT_STATE.md) is retained only as a compatibility tombstone. It is not a
third source of current truth.

## Development order

The development sequence, its hard dependencies and the current position are owned by
[`ROADMAP.md`](ROADMAP.md). The per-gate entry conditions, allowed/forbidden operations and pass
criteria are owned by
[`DEVELOPMENT_GATES.md`](../../05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md).

They are deliberately not restated here: this document owns *contracts*, not *sequence*. No
document may authorize a hardware stage — only an explicit operator authorization for that
specific session can.
