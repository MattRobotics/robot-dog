# Validation reports — index

Each entry is evidence for its recorded date and scope only. A PASS here does not carry over to a
later firmware, a later mechanical state or a later power architecture, and no entry authorizes
motion (`MOTION_AUTHORIZED=0`). Recorded PASS/FAIL results are not edited by this index.
Current state: [root README](../../README.md). Chronology: [`HISTORY_INDEX.md`](../HISTORY_INDEX.md).

Legend — **Era**: *Station* = Station-mediated, 12-servo phase (historical); *Rebuild* = after the
2026-08-27 rebuild (17 servos, ESP32-S3 Controller). **Kind**: HW = hardware evidence, OFFLINE =
software/model only.

## Model, kinematics and early hardware (June–July 2026)

| Report | Date | Kind | Era | Scope as recorded |
|---|---|---|---|---|
| [ST3215 Bus Validation](2026-06-17_ST3215_Bus_Validation.md) | 2026-06-17 | HW | Station, SUPERSEDED banner | Station-mediated ST3215 bus validation |
| [URDF REV00 Kinematic Validation](2026-06-30_URDF_REV00_Kinematic_Validation.md) | 2026-06-30 | OFFLINE | model | URDF topology, frames, axes, limits (see [ADR-003](../Architecture_Decisions/ADR-003_URDF_REV00_Kinematic_Baseline.md)) |
| [Visual-Zero and Static Actuation Validation](2026-07-06_Visual_Zero_and_Static_Actuation_Validation.md) | 2026-07-06 | HW | Station, SUPERSEDED banner | Visual-zero calibration and static actuation, 12-servo installation |
| [C4-A offline safe-stand candidate](2026-07-08_175245_C4A_offline_safe_stand_candidate.json) | 2026-07-08 | OFFLINE | model | Offline candidate; `command_eligible=false`. Path is globbed by kinematics tools: do not rename |
| C4 offline policy set — [collision/contact](C4_collision_contact_policy/), [rest-to-stand trajectory](C4_rest_to_stand_trajectory/), [timing envelope](C4_trajectory_timing_envelope/), [static stability](C4_static_stability_support_polygon/), [hardware safe-mode preflight](C4_hardware_safe_mode_preflight/) | 2026-07-08 | OFFLINE | model | C4-B…C4-F offline artifacts |
| [Geometry Compiler artifacts](Geometry_Compiler/README.md) | 2026-08-07 – 2026-08-11 | OFFLINE | model | v1–v5 geometry runs with per-run status and SHA-256 (index inside) |

## ST3215 bench phase (August 2026)

| Report | Date | Kind | Scope as recorded |
|---|---|---|---|
| [ST3215 Bench QC](ST3215_Bench_QC_2026-08-24/README.md) | 2026-08-24/25 | HW (bench) | 26/26 runs COMPLETE, protocol V6.1 |
| [ST3215 Provisioning](ST3215_Provisioning_2026-08-27/README.md) | 2026-08-27 | HW (bench) | 17 units provisioned, 17/17 PASS. Does not state that 17 servos are installed today |

Navigation for the whole ST3215 evidence set: [`ST3215_EVIDENCE_INDEX.md`](../ST3215_EVIDENCE_INDEX.md).

## Controller on the rebuilt robot (September–October 2026)

| Report | Date | Kind | Scope as recorded |
|---|---|---|---|
| [CR2-C current-installation q0 capture](Calibration_Q0_CR2C_2026-09-27/README.md) | 2026-09-27 | HW (read-only) | Read-only q0 capture, "CR2-C PASS"; does not accept or promote calibration |
| [Full Calibration sequence geometry](Full_Calibration_Sequence_Geometry_2026-09-30/README.md) | 2026-09-30 | OFFLINE | Nominal CAD mesh non-intersection, 24-contact sequence; no hardware |
| [TRUE Full Calibration 24/24](Full_Calibration_24_Contact_Hardware_2026-10-01/README.md) | 2026-10-01 | HW | 24/24 contacts, `all_contact_calibrated=1`, no operational envelope approved. Its statement "RAM-only" is correct for that session and firmware; for the later persistence work see the dev.3 audit below and [`OPEN_ITEMS.md`](../OPEN_ITEMS.md) |
| [Wi-Fi / provisioning / OTA V3 offline](2026-10-04_WIFI_OTA_SHELLY_V3_OFFLINE.md) | 2026-10-04 | OFFLINE | Offline qualification only. It names a local worktree and a branch that no longer exist; read it as historical |
| [dev.2 boot self-test delta audit](MATDOG_V0_2_DEV2_BOOT_SELFTEST_DELTA_AUDIT_2026-10-06.md) | 2026-10-06 | HW + audit | Boot census 13/13; calibration stopped at LF under dev.2 (block-read temperature artifact) |
| [dev.3 thermal delta audit](MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md) | 2026-10-06 | HW + audit | 24/24 under dev.3; SAVE/ACK and LOAD across a reset observed; automatic recovery not triggered; no real power cycle |

## Not in this directory

- **Hardware Validation 2026-10-07 (dev.3):** execution COMPLETE, acceptance BLOCKED, as stated by
  the project owner. The full report is **not in the repository** and is not reconstructed here.
  See [`EXTERNAL_ARCHIVES.md`](../EXTERNAL_ARCHIVES.md).
- **Firmware offline test results** (PR-1, PR-2, PR-3 gates) are recorded in
  [`05_Firmware/MATDOG_Controller/VALIDATION.md`](../../05_Firmware/MATDOG_Controller/VALIDATION.md).
- **Controller V0.1, G2, G3 sessions** (2026-09-15, 2026-09-18) are recorded in the same
  `VALIDATION.md`, not as separate reports.
- **Motion G1–G5-A evidence** (G4, G4.1, G5-A, G35 pose audit) is deferred by
  [ADR-004](../Architecture_Decisions/ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md)
  and is not on `main`.
