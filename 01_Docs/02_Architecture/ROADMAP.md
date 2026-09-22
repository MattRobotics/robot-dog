# MATDOG Roadmap

**Canonical owner of the MATDOG development sequence, its dependencies, and where the project
currently stands.** Last updated 2026-09-18.

This file answers *where are we, what is next, and why a stage cannot be skipped*. It does not
duplicate other owners:

| For | Read |
|---|---|
| Current snapshot and immediate milestone | [Root `README.md`](../../README.md) |
| Permanent system contracts and boundaries | [`ARCHITECTURE.md`](ARCHITECTURE.md) |
| Technical pass/fail authorization criteria per gate | [`DEVELOPMENT_GATES.md`](../../05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md) |
| Actual Controller validation evidence | [`VALIDATION.md`](../../05_Firmware/MATDOG_Controller/VALIDATION.md) |

## How to read this file

Two orthogonal axes are used deliberately, and conflating them is the most common way a robotics
roadmap becomes dangerous:

```text
IMPLEMENTED  !=  VALIDATED
```

**Stage status** — where the project is in the sequence:

| Label | Meaning |
|---|---|
| **COMPLETE** | Finished and accepted. Its acceptance evidence exists in the repository. |
| **CURRENT** | The stage actually being worked or awaiting its authorization right now. |
| **NEXT** | The immediately following stage. Its prerequisites are met or nearly met. |
| **FUTURE** | Planned and sequenced, not started. |
| **BLOCKED** | Cannot start until a named blocker is cleared, regardless of effort available. |
| **PARTIAL** | Some of the stage's capability exists; the rest does not. Always enumerated. |

**Capability status** — the repository-wide vocabulary defined in the
[root README](../../README.md#status-vocabulary): **VALIDATED**, **IMPLEMENTED**, **DECIDED**,
**TO_TEST**, **TO_DESIGN**, **FROZEN**, **SUPERSEDED**, **HISTORICAL**.

A stage may be COMPLETE in software while its capability is only IMPLEMENTED, never VALIDATED,
because validation requires hardware evidence that does not exist yet. Say so explicitly rather
than rounding up.

---

## Where we are right now

```text
COMPLETE   Controller V0.1 baseline               VALIDATED (USB_ONLY hardware)
COMPLETE   G0 post-cleanup entry audit            PASS
COMPLETE   G1 V0.1 regression freeze              PASS
COMPLETE   V2 architecture delta audit            PASS
COMPLETE   G2 ROBOT_POWERED software              PASS / FROZEN
COMPLETE   G3 ROBOT_POWERED no-motion             PASS (formal, 2026-09-18)
COMPLETE   G3.1 CDC-independent Controller loop   PASS (2026-09-18)
COMPLETE   DALY KEY research                      COMPLETE (read-only, 2026-09-19)
COMPLETE   DALY 0x81 read                         LIVE VERIFIED (read-only, 2026-09-19)
COMPLETE   DALY KEY write (0x0120 := 0x005A)      LIVE VERIFIED (2026-09-19; ACK + read-back)
                                                  current KEY logic = DISCHARGE (0x005A)

BLOCKED    physical KEY OFF removes robot rails   BLOCKED by a hardware B-/P- bypass:
                                                  TECNOIOT VIN- must move from B- to P-
NEXT       post-rewire power validation           TO_TEST (operator rewire, then dead-circuit +
                                                  KEY OFF/ON + powered no-motion regression)
OPEN       charging hardware (manual + dock)      OPEN — no charger/dock evidence exists
OPEN       physical KEY safety barrier            NOT VALIDATED (fused disconnect = trusted isolation)
THEN       G4 Diagnostics / Maintenance           NOT STARTED
```

Power domains, KEY/Charge-MOS semantics, every power state and the charging gates are owned by
[`04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md`](../../04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md).

`ROBOT_POWERED` is **VALIDATED for no-motion operation** (G3, formal): DALY live read-only, LED
live, 13/13 expected servos with 4 absent by design in two identical censuses, `VERIFIED_OFF` and
`torque=0` on all 13. G3.1 closed a regression found after the first powered session — the
Controller loop starved while no USB CDC host held the port open (0.68 Hz); with the native HWCDC
fix it runs at 50.1 Hz with the port closed. No motion or calibration capability exists yet.

G2 through G3.1 were developed on branch `feat/controller-robot-powered-v02` and merged to `main`
by the G3/G3.1 closeout pull request.

### Open hardware notes

- **DALY `KEY` — OPEN.** Toggling the physical KEY switch produced no observed change in any
  DALY-reported state (`discharge_mos=ON` in both positions). KEY is **not** a validated shutdown
  or safety barrier. Research (2026-09-19): public DALY documents publish no K-series KEY
  register; DALY's own BMSTool V1.14.79 (static inspection) points to KEY logic at `0x0120` on a
  second Modbus personality (`0x81`). **Live-verified read-only the same day** with `@BMS KEY
  READ`: the `0x81` personality answers and the unit's KEY logic is **DISABLED (`0x0055`)** —
  consistent with the G3 finding. The single guarded write — `@BMS KEY SET DISCHARGE CONFIRM`,
  `0x0120 := 0x005A` (DISCHARGE) — was sent **once, live, on 2026-09-19**: acknowledged and read
  back as `0x005A`, so the BMS now switches the discharge MOS from the KEY. The follow-on physical
  KEY test was **inconclusive**: a hardware `B-`/`P-` bypass (TECNOIOT `VIN-` on raw `B-`) kept the
  load rail powered with the discharge MOS open. Until that rewire and its validation, KEY OFF must
  not be trusted to remove the rails, and the fused disconnect remains the trusted physical
  isolation method — see
  [`04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md`](../../04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md).
- **GPIO19/GPIO20 — frozen.** GPIO19 = native USB D−, GPIO20 = native USB D+. The external
  19/20/GND connector remains a future USB service-port candidate — **not** a UART — pending
  electrical and signal-integrity validation.

---

## Dependency sequence

Each row's *Blocks* column states what it gates. Arrows are hard dependencies, not preferences.

### Phase 1 — Controller platform and powered baseline

| # | Stage | Status | Reality in this repository |
|---|---|---|---|
| 1 | **MATDOG Controller baseline** | **COMPLETE** | V0.1 unified runtime, **FROZEN** at tag `matdog-controller-v0.1.0`. **VALIDATED** on real hardware in the `USB_ONLY` profile only. |
| 2 | **ROBOT_POWERED preparation** | **COMPLETE** | `HardwareProfile` authority (`USB_ONLY` / `ROBOT_POWERED`), profile-derived module expectations, canonical-17 / expected-13 / absent-by-design-4 servo population model, structured census classification, build-manifest flash provenance. Software **FROZEN**; hardware-validated by stage 3. |
| 3 | **ROBOT_POWERED no-motion validation** | **COMPLETE** | **G3 PASS (formal)**, live 2026-09-17 / 2026-09-18, two identical censuses. **G3.1 CDC-independent Controller loop PASS**, 2026-09-18. Evidence in [`VALIDATION.md`](../../05_Firmware/MATDOG_Controller/VALIDATION.md). Open hardware item: DALY `KEY` (see above). |

### Phase 2 — Maintenance, authority, service

| # | Stage | Status | Reality in this repository |
|---|---|---|---|
| 4 | **G4 — Diagnostics / Maintenance** | **PARTIAL — NOT STARTED** (DALY KEY read and write both live-verified; the open item is the hardware `B-`/`P-` bypass, not the Controller) | Already exist: `@STATUS` module availability, `@SERVO SCAN`, `@SERVO READ`, `@SERVO CENSUS` (structured population classification), `@SERVO SAFE_OFF` with independent readback, `@IMU`/`@BMS`/`@LED` status. Do **not** exist: `SYSTEM_SELF_TEST`, `SOURCE_SIGNATURE`, `PROFILE_AUDIT`, consolidated servo health summary. |
| 5 | **OperatingMode / ActuatorAuthority** | **IMPLEMENTED / OFFLINE TESTED** | `src/core/ActuatorAuthority.*` is the single central arbiter (`NONE`/`DIAGNOSTICS`/`CALIBRATION`/`QC`/`PROVISIONING`/`MOTION`, at most one owner at a time), pure and host-linkable, one instance owned by `Controller`, `NONE` at boot, leases carrying a generation against stale releases. Orthogonal to `OperatingMode` and enforced as such. `SAFE_OFF` is outside arbitration structurally. Read-only servo diagnostics deliberately take no lock. **No new write path was added** — the only actuator write is still `EnableTorque(id, 0)`, and nothing can acquire an owner yet. 751 offline checks. **HARDWARE TO_TEST.** |
| 6 | **Service / Provisioning / QC** | **FUTURE** | Frozen bench tools (Bench QC V6.1, Source Signature Survey V1, Provisioner V6) remain **FROZEN** oracles; nothing is integrated into the Controller. Blocked by stage 5. |
| 7 | **Full Leg Calibration integration** | **PARTIAL — offline foundation IMPLEMENTED / OFFLINE TESTED** | `src/calibration/` holds a pure host-linkable domain model recovered from the LF V25 archive plus a `CalibrationManager` over the real `ActuatorAuthority` (branch `feat/controller-calibration-manager-v1`). LF V25 replayed offline: 58 steps, 6 LF contacts of 24, matched. **No write path was added** and hardware motion is compile-time blocked. The execution engine and the Safe Actuator Layer beneath it are **TO_IMPLEMENT**. Audit and source precedence: [`CALIBRATION_SOURCE_PRECEDENCE.md`](../../05_Firmware/MATDOG_Controller/CALIBRATION_SOURCE_PRECEDENCE.md). Note: `matdog/full-leg-calibrator-v1` no longer exists as a branch; it was archived 2026-09-18 under the annotated tag `archive/2026-08-29/full-leg-calibrator-v1-h0` -> `15f3fb8f378e6cadf6bc479bfcaca2947741c9fd` (local and on `origin`), with its worktree-only hardware evidence in `~/MATDOG/archive/full-leg-calibrator-v1/`. No oracle material was lost. |
| 8 | **Formal recalibration of the installed robot** | **BLOCKED** | Blocker: `CALIBRATION_RESET_PENDING_FULL_RECALIBRATION`, `hardware_motion_authorized: false`. Last formal Full-Leg H1 result was 6/12 and has not been superseded. A Controller census finding 13 servos proves bus visibility, **not** calibration H1. The offline foundation (stage 7) models this gate and enforces it: historical population evidence, however complete, can never produce a current PASS. Blocks all motion. |

### Phase 3 — Host transport, network, Web UI, OTA

| # | Stage | Status | Reality in this repository |
|---|---|---|---|
| 9 | **HostLink semantic layer** | **PARTIAL** | `CommandRouter` exists as a USB CDC adapter. G2 introduced transport-independent Controller state (`ServoPopulation`/`ServoCensus` produce structured results with no `Serial` dependency), which is the precondition for a second transport. A formal Controller Service Layer and transport-neutral command/telemetry schema are **TO_DESIGN**. |
| 10 | **Wi-Fi runtime** | **PARTIAL** | Station-mode runtime **IMPLEMENTED / COMPILED / OFFLINE TESTED**, **NOT HARDWARE TESTED** (W1, branch `feat/controller-wifi-ota-v1`). `src/network/WifiPolicy.*` is a pure host-linkable lifecycle state machine; `src/network/WifiManager.*` is the sole owner of the radio and the only unit including `<WiFi.h>`. Surface: `@WIFI STATUS\|ON\|OFF` plus one `WIFI` line in `@STATUS`. Credentials resolve outside Git and an absent SSID is a supported state in which the radio never starts. No HTTP, WebSocket or server code exists — Wi-Fi is a link, nothing more. Build cost vs frozen `19fe837`: flash 12% → 30%, static RAM 8% → 15%. |
| 11 | **Embedded Web Dashboard — read-only** | **FUTURE** | **DECIDED** architecture (see [ARCHITECTURE.md](ARCHITECTURE.md#embedded-matdog-web-ui--control--service-dashboard)). Gate UI-0 (architecture contract) may be designed offline; UI-1 needs stage 10. |
| 12 | **IMU 3D / BMS / servo-health UI** | **FUTURE** | Part of UI-1. The existing host-side BNO085 viewer (`06_Software/.../matdog_bno085_fullbody_viewer`, 59/59 tests) is an engineering tool and is **not** the embedded dashboard; it remains preserved separately. |
| 13 | **OTA** | **PARTIAL** | **OTA-A update core IMPLEMENTED / COMPILED / OFFLINE TESTED, NOT HARDWARE TESTED** (branch `feat/controller-wifi-ota-v1`). `src/update/` holds a pure host-linkable state machine, a first-boot rollback guard, streaming SHA-256 image identity, and the single translation unit allowed to call `esp_ota_*`. Writes the inactive slot only, enforced by explicit refusals plus an independent re-check in the backend; the boot target is reachable from exactly one validated state. Host-side partition-selection logic remains **IMPLEMENTED** (40/40). **Transport and authentication do NOT exist** and byte ingest is compiled out by default. Stage 10 is not hardware-tested, so the OTA gate cannot pass. **OTA-B authorization is now IMPLEMENTED / OFFLINE TESTED**: OTA takes an exclusivity inhibit on the stage-5 arbiter (never an actuator ownership) for the whole update, closing the TOCTOU window a plain `authority == NONE` check leaves open. |

### Phase 4 — Motion

| # | Stage | Status | Reality in this repository |
|---|---|---|---|
| 14 | **Safe Actuator Layer** | **FUTURE / TO_DESIGN** | The only legal path from joint-level commands to `ServoBus` writes. Does not exist. No motion primitive exists in the firmware at all today. |
| 15 | **First bounded joint motion** | **BLOCKED** | Blockers: stages 3, 5, 8, 14. |
| 16 | **Joint Test UI (UI-4)** | **BLOCKED** | Blockers: stages 11, 14, 15. |
| 17 | **Controlled poses** | **BLOCKED** | Blocker: stage 15. |
| 18 | **Operational IK** | **FUTURE** | Geometry/kinematics assets exist in `06_Software/Matdog_Core/`; no operational IK runs on the Controller. |
| 19 | **Body Pose / Single Leg UI (UI-5)** | **BLOCKED** | Blockers: stages 11, 18. |
| 20 | **Gait engine** | **FUTURE** | Blocked by stages 17 and 18. |
| 21 | **Manual teleoperation / gait UI (UI-6, UI-7)** | **BLOCKED** | Blockers: stages 11, 20, plus a firmware-side command lease/deadman (see [ARCHITECTURE.md](ARCHITECTURE.md#continuous-command-lease--deadman)). |
| 22 | **Preset actions** | **FUTURE** | Only MATDOG-validated actions may ever be exposed. |
| 23 | **BNO085 closed-loop stabilization** | **FUTURE** | The IMU acquisition path is **VALIDATED**; the control loop does not exist. |
| 24 | **Stabilization UI (UI-8)** | **BLOCKED** | Blocker: stage 23. |
| 25 | **Jetson / ROS 2 / MoveIt / AI integration** | **FUTURE** | High-level host responsibility. Must never become an ESP32-S3 responsibility. |

---

## Why stages cannot be reordered

These are the dependencies that actually matter. Each has caused, or would cause, a real hazard:

1. **Nothing powered before G3.** The `ROBOT_POWERED` profile changes what the firmware *expects*,
   not what the hardware *does*. Until a powered session proves the DALY, LED rail and ST3215 bus
   behave as designed, no powered assumption is evidence.
2. **No write-capable service before an authority model.** Provisioning, QC and calibration can all
   command the same servo. Without one arbitrated owner, two subsystems can drive one actuator.
3. **No motion before recalibration.** Every joint zero, direction and limit was invalidated by the
   2026-08-27 reassembly. Motion from stale calibration is motion toward an unknown position.
4. **No motion before the Safe Actuator Layer.** It is the single place joint limits, calibration
   validity, raw-domain enforcement and the deadman can be enforced. Bypassing it once means the
   bypass exists forever.
5. **No Web write-controls before their backend gate.** A browser control whose firmware gate does
   not yet exist is a direct path from a network callback to an actuator — the explicitly forbidden
   architecture.
6. **No gait before IK, no IK before poses, no poses before one bounded joint.** Each step bounds
   the failure envelope of the next.

---

## Embedded Web UI placement in the sequence

The dashboard is a **permanent architectural target**, not an add-on, and it is sequenced against
firmware capability rather than UI effort. Full contract in
[ARCHITECTURE.md](ARCHITECTURE.md#embedded-matdog-web-ui--control--service-dashboard); per-gate
criteria in [DEVELOPMENT_GATES.md](../../05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md).

```text
UI-0  architecture contract          may be designed now (offline)
UI-1  read-only dashboard + IMU 3D   needs Wi-Fi runtime (stage 10)
UI-2  maintenance controls           needs Diagnostics/Maintenance (stage 4)
UI-3  calibration / service UI       needs authority model (stage 5) + workflows (6, 7)
UI-4  joint test                     needs Safe Actuator (14) + first motion (15)
UI-5  body pose / single leg         needs operational IK (18)
UI-6  manual teleop                  needs gait (20) + command lease/deadman
UI-7  gait / preset actions          needs validated gaits (20, 22)
UI-8  stabilization                  needs closed-loop stabilization (23)
UI-9  OTA / future extensions        needs OTA runtime (13)
```

UI scaffolding may be prepared earlier. **Write-capable controls stay disabled until the
corresponding firmware gate passes**, and the UI derives its enable/disable state from
authoritative Controller state, never from frontend assumptions.

---

## Current blockers

| Blocker | Blocks | Cleared by |
|---|---|---|
| **Hardware `B-`/`P-` bypass**: TECNOIOT `VIN-` returns to raw battery `B-`, so the load rail stayed powered with the discharge MOS open | KEY OFF as a power-off; every OFF / MANUAL_CHARGE_OFF / SERVICE_ISOLATED state; the physical KEY test | operator rewire (`TECNOIOT VIN-` → `P-`) then the dead-circuit + KEY OFF/ON + powered no-motion validation in [`MATDOG_POWER_STATES_AND_CHARGING.md`](../../04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md) (KEY logic itself is live-verified `0x005A`; fused disconnect is the trusted isolation meanwhile) |
| Charging hardware (charger, dock, contacts, return path) never tested | manual charging with KEY OFF; autonomous docking/charging | a separate charging-hardware gate once charger/dock hardware exists |
| `CALIBRATION_RESET_PENDING_FULL_RECALIBRATION` | all motion (15, 17, 20, 23) | stages 7 + 8 |
| No `ActuatorAuthority` model | write-capable service, QC, calibration, motion | stage 5 |
| No Safe Actuator Layer | all motion | stage 14 |
| No Wi-Fi runtime | all Web UI and OTA runtime | stage 10 |

---

## Version line

`matdog-controller-v0.1.0` is **FROZEN** and immutable. The embedded firmware version string is
still `0.1.0`; the development branch name containing `v02` is a branch name, not a release
identity. The `0.2.x` number is decided at a release gate, not by this roadmap, and no
`matdog-controller-v0.2.0` tag exists or is authorized.
