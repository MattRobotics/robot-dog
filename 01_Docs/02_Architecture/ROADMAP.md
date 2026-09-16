# MATDOG Roadmap

**Canonical owner of the MATDOG development sequence, its dependencies, and where the project
currently stands.** Last updated 2026-09-16.

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
COMPLETE   G2 ROBOT_POWERED preparation           PASS (software/offline only)

CURRENT    G3 ROBOT_POWERED no-motion validation  TO_TEST — awaiting hardware authorization
```

The `ROBOT_POWERED` **software** support exists, is offline-tested and compiles cleanly for both
profiles. The `ROBOT_POWERED` **hardware configuration has never been powered or exercised**. No
document may describe it as VALIDATED until G3 produces evidence.

G2 currently lives on branch `feat/controller-robot-powered-v02` and is not merged to `main`.

---

## Dependency sequence

Each row's *Blocks* column states what it gates. Arrows are hard dependencies, not preferences.

### Phase 1 — Controller platform and powered baseline

| # | Stage | Status | Reality in this repository |
|---|---|---|---|
| 1 | **MATDOG Controller baseline** | **COMPLETE** | V0.1 unified runtime, **FROZEN** at tag `matdog-controller-v0.1.0`. **VALIDATED** on real hardware in the `USB_ONLY` profile only. |
| 2 | **ROBOT_POWERED preparation** | **COMPLETE** | `HardwareProfile` authority (`USB_ONLY` / `ROBOT_POWERED`), profile-derived module expectations, canonical-17 / expected-13 / absent-by-design-4 servo population model, structured census classification, build-manifest flash provenance. **IMPLEMENTED + offline-tested; not hardware-validated.** |
| 3 | **ROBOT_POWERED no-motion validation** | **CURRENT** | **TO_TEST.** Procedure designed in [`G3_ROBOT_POWERED_VALIDATION_PLAN.md`](../../05_Firmware/MATDOG_Controller/G3_ROBOT_POWERED_VALIDATION_PLAN.md). Requires explicit operator authorization to energize the robot. Blocks everything powered below. |

### Phase 2 — Maintenance, authority, service

| # | Stage | Status | Reality in this repository |
|---|---|---|---|
| 4 | **Diagnostics / Maintenance** | **PARTIAL → NEXT** | Already exist: `@STATUS` module availability, `@SERVO SCAN`, `@SERVO READ`, `@SERVO CENSUS` (structured population classification), `@SERVO SAFE_OFF` with independent readback, `@IMU`/`@BMS`/`@LED` status. Do **not** exist: `SYSTEM_SELF_TEST`, `SOURCE_SIGNATURE`, `PROFILE_AUDIT`, consolidated servo health summary. |
| 5 | **OperatingMode / ActuatorAuthority** | **PARTIAL** | `OperatingMode{MAINTENANCE, RUN}` exists and gates blocking servo diagnostics — deliberately minimal. The full `ActuatorAuthority` model (`NONE`/`DIAGNOSTICS`/`CALIBRATION`/`QC`/`PROVISIONING`/`MOTION`, one owner at a time) is **TO_DESIGN**. Required before any write-capable service or motion. |
| 6 | **Service / Provisioning / QC** | **FUTURE** | Frozen bench tools (Bench QC V6.1, Source Signature Survey V1, Provisioner V6) remain **FROZEN** oracles; nothing is integrated into the Controller. Blocked by stage 5. |
| 7 | **Full Leg Calibration integration** | **FUTURE** | Branch `matdog/full-leg-calibrator-v1` preserved as an oracle. Not merged, not ported. Blocked by stages 3 and 5. |
| 8 | **Formal recalibration of the installed robot** | **BLOCKED** | Blocker: `CALIBRATION_RESET_PENDING_FULL_RECALIBRATION`. Last formal Full-Leg H1 result was 6/12 and has not been superseded. A Controller census finding 13 servos proves bus visibility, **not** calibration H1. Blocks all motion. |

### Phase 3 — Host transport, network, Web UI, OTA

| # | Stage | Status | Reality in this repository |
|---|---|---|---|
| 9 | **HostLink semantic layer** | **PARTIAL** | `CommandRouter` exists as a USB CDC adapter. G2 introduced transport-independent Controller state (`ServoPopulation`/`ServoCensus` produce structured results with no `Serial` dependency), which is the precondition for a second transport. A formal Controller Service Layer and transport-neutral command/telemetry schema are **TO_DESIGN**. |
| 10 | **Wi-Fi runtime** | **FUTURE** | Not implemented. No Wi-Fi, HTTP, WebSocket or network code exists in the Controller. |
| 11 | **Embedded Web Dashboard — read-only** | **FUTURE** | **DECIDED** architecture (see [ARCHITECTURE.md](ARCHITECTURE.md#embedded-matdog-web-ui--control--service-dashboard)). Gate UI-0 (architecture contract) may be designed offline; UI-1 needs stage 10. |
| 12 | **IMU 3D / BMS / servo-health UI** | **FUTURE** | Part of UI-1. The existing host-side BNO085 viewer (`06_Software/.../matdog_bno085_fullbody_viewer`, 59/59 tests) is an engineering tool and is **not** the embedded dashboard; it remains preserved separately. |
| 13 | **OTA** | **PARTIAL** | OTA **partition-selection logic** is **IMPLEMENTED** and offline-tested (40/40), and the application-only flash workflow uses it. OTA **transport/runtime** (upload, reboot, rollback orchestration) does **not** exist. Blocked by stage 10. |

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
| Powered hardware never exercised | stages 3 onward | G3 session, explicit operator authorization |
| `CALIBRATION_RESET_PENDING_FULL_RECALIBRATION` | all motion (15, 17, 20, 23) | stages 7 + 8 |
| No `ActuatorAuthority` model | write-capable service, QC, calibration, motion | stage 5 |
| No Safe Actuator Layer | all motion | stage 14 |
| No Wi-Fi runtime | all Web UI and OTA runtime | stage 10 |
| G2 not merged to `main` | nothing technically, but `main` does not yet contain ROBOT_POWERED support | reviewed merge of `feat/controller-robot-powered-v02` |

---

## Version line

`matdog-controller-v0.1.0` is **FROZEN** and immutable. The embedded firmware version string is
still `0.1.0`; the development branch name containing `v02` is a branch name, not a release
identity. The `0.2.x` number is decided at a release gate, not by this roadmap, and no
`matdog-controller-v0.2.0` tag exists or is authorized.
