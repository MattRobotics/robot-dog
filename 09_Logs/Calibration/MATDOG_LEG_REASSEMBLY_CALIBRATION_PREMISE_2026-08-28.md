# MATDOG leg reassembly — calibration premise

**Date:** 2026-08-28
**Status:** CURRENT
**Scope:** the 12 leg servos, now physically installed
**Supersedes nothing.** This document *adds* physical evidence recorded after the
2026-08-27 repository consolidation. No historical log or frozen evidence file was
rewritten or deleted.

Read together with:

- [Calibration reset 2026-08-27](MATDOG_CALIBRATION_RESET_2026-08-27.md) — the authoritative
  statement that all calibration is invalid and must be redone;
- [MATDOG_C018_V1 profile](../../01_Docs/02_Architecture/MATDOG_ST3215_C018_V1_PROFILE.md);
- [Servo allocation](../../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml).

---

## 1. Physical state

The **12 leg servos are physically installed**. Each joint was mounted with the servo
held at physical RAW ≈ 2048, so that mechanical calibration pose and servo electrical
centre coincide as closely as the mechanics allow.

The **5 head/jaw servos (ids 51–55) are NOT installed** — the head has not been built.
Their absence is expected and must never block a legs-only calibration session.

## 2. The current mechanics match the existing URDF

The rebuilt leg mechanics are the correct physical realization of the existing
CAD/URDF.

> **No URDF change is required, and none was made.**
> The canonical URDF `03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf`
> is unchanged by this work.

Geometry Compiler V5 outputs remain valid: geometry describes the *design*, and the
design did not change. Calibration describes the *build*, and the build is new.

## 3. The HIP_MAX endstop carrier was corrected

A previous upper-back component carried the HIP_MAX mechanical endstops. It could not be
fixed precisely to `base_link` and **may have sat too high** during the historical LF V25
hardware test. That would have shifted the physical HIP_MAX contact position relative to
the URDF.

That component has been **redesigned, remade and mounted correctly**.

Consequence: the historical HIP_MAX contact position is not a valid expectation for the
current robot. This is the strictest case, but the same reasoning applies to every joint,
because every servo was removed and remounted.

## 4. LF V25 is declassified: algorithmic oracle, not numeric oracle

The [LF V25 hardware oracle](../Historical/NormaCore_MATDOG_Archive/LF_V25_Hardware_Oracle/)
remains **extremely valuable**, and its archive is preserved untouched.

### Still authoritative — as algorithm and method

- leg session state-machine design;
- bounded contact detection with multi-channel evidence;
- persistence over consecutive samples rather than single-sample triggers;
- retreat-and-re-approach for repeatability;
- torque-off discipline and fail-closed exits;
- affine q0 derivation *concepts*;
- URDF cross-check *concepts*;
- transactional / fail-closed thinking;
- session logging and provenance discipline.

### No longer authoritative — as numbers for this installation

| Artifact | Status for the current build |
|---|---|
| LF V25 RAW MIN/MAX values | **HISTORICAL — not an acceptance band** |
| LF V25 endpoint angles as expected measurements | **HISTORICAL** |
| Old LF q0 values | **HISTORICAL** |
| Old `PositionOffset` values | **HISTORICAL — all offsets are now 0** |
| Old per-servo runtime torque settings | **HISTORICAL** |
| Old HIP_MAX contact position | **HISTORICAL — carrier has been corrected** |
| Any threshold justified only by "it passed LF V25" | **NOT VALID for current hardware** |

### Additional reason: the fleet was not in a homogeneous runtime torque state

The pre-recenter backup
[`C5_R_digital_recenter/2026-07-10_132236Z_st3215_pre_recenter_backup.json`](C5_R_digital_recenter/2026-07-10_132236Z_st3215_pre_recenter_backup.json)
records the runtime `torque_limit` of all 12 leg servos as they were at that time:

| Runtime `torque_limit` | Bus ids (as of 2026-07-10) | Count |
|---|---|---|
| **500** | **13, 23** | 2 |
| 1000 | 11, 12, 21, 22, 31, 32, 33, 41, 42, 43 | 10 |

This matches `TORQUE_LIMIT: u16 = 500` in the LF V25 calibrator
(`auto_calibrate/matdog.rs`), which wrote that runtime limit to the servos it drove.

Two consequences:

1. Historical measurements were **not** taken under a uniform runtime torque regime
   across the fleet. Any comparison that treats old per-joint numbers as mutually
   consistent is unsound.
2. `TorqueLimit` is a RAM/runtime register, not part of the persistent
   `MATDOG_C018_V1` profile. The provisioning campaign observed 1000 on all units after a
   true cold cycle, so the 500 did not persist. It is evidence about *how those numbers
   were produced*, not about the current servos.

> ⚠️ **Bus ids were recoded during provisioning.** The ids in the table above are the
> *2026-07-10* ids. They do **not** map to the same physical units as today's ids. See
> the `bus_id_warning` in `MATDOG_JOINT_CALIBRATION.yaml`.

## 5. q0 philosophy for this build

The assembly intentionally tries to make mechanical calibration pose and servo electrical
centre coincide. But the horn/pulley spline is finite-toothed: a joint can sit as much as
roughly **±5°** from the ideal CAD pose — about **±57 ticks** at 4096 ticks/rev — even
though the servo was held at RAW ≈ 2048 during mounting.

Therefore:

- ❌ **Do not assert `q0_raw = 2048`.** A measured residual is a legitimate, expected outcome.
- ❌ **Do not "fix" the residual by writing `PositionOffset`.** Compensating a mechanical
  mounting error in EEPROM is the specific thing the policy exists to prevent.
- ✅ **Measure it.** Software q0 lives in MATDOG configuration, not in servo EEPROM.

`PositionOffset` remains **0** on all 17 units and is never written by the calibrator.

### Two distinct q0 concepts, never conflated

| Concept | What it is | How it is obtained |
|---|---|---|
| `manual_pose_q0_candidate` | encoder RAW while the user holds the nominal CAD pose | read-only, torque OFF, multi-sample with dispersion |
| `derived_q0_final` | q0 solved from measured contacts + current URDF geometry | affine derivation, cross-checked |

The user sets the manual pose with a **small square / reference tool**. That pose is
useful evidence with **finite uncertainty** — it is not an infinitely precise metrology
fixture. The two estimates are compared, never averaged. Disagreement beyond an
evidence-based tolerance is a structured FAIL.

## 6. Everything is re-measured from scratch

All endpoint and q0 calibration for the current build starts from zero:

- no import of any pre-2026-08-27 q0;
- no reuse of LF V25 endpoint values as acceptance bands;
- joint **direction** is re-measured, not inherited — the stale directions in
  `MATDOG_JOINT_CALIBRATION.yaml` describe an installation that no longer exists.

Hardware is the measurement. The URDF is the geometry reference. **Neither may be
silently altered to force the other to agree.** A conflict is recorded, diagnosed and
failed closed.

## 7. Tooling

This premise is implemented by
[MATDOG Full Leg Calibrator V1](../../05_Firmware/Full_Leg_Calibrator_V1/README.md),
an ESP32-S3-native, Station-free, EEPROM-write-free calibration engine covering all 12
leg joints with one generic state machine and progressive H0–H7 hardware gates.

Its offline validation and H0 no-servo hardware evidence:
[Full_Leg_Calibrator_V1 validation reports](../Validation_Reports/Full_Leg_Calibrator_V1/README.md).
