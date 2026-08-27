# ⚠️ CALIBRATION RESET — 2026-08-27

# ALL MATDOG CALIBRATION MUST BE REDONE FROM ZERO

**Status:** `CALIBRATION_RESET_PENDING_FULL_RECALIBRATION`
**Effective:** 2026-08-27
**Trigger:** complete physical reassembly of all 17 servos

---

## What is happening

The robot is being **physically reassembled**:

1. the **12 leg servos** are being remounted;
2. then the **5 head servos** (neck rotation, neck pitch, head rotation, head pitch, jaw).

Every servo has been removed, provisioned on the bench to `PositionOffset = 0`, and is being
remounted into its mechanical linkage.

## Consequence

> **The entire robot calibration must be considered invalid and redone from zero.**

Nothing about joint zero, joint direction, or joint limits survives a remount. The servo output
splines have been physically separated from their horns and links.

---

## What is no longer active truth

| Artifact | New status |
|---|---|
| Digital zero / q0 values captured before 2026-08-27 | **HISTORICAL — not active truth** |
| `09_Logs/Calibration/Digital_Zero/` snapshots (2026-08-14) | **HISTORICAL EVIDENCE — superseded** |
| `09_Logs/Calibration/C5_R_digital_recenter/` (2026-07-10) | **HISTORICAL EVIDENCE — superseded** |
| LF V25 hardware calibration results | **ORACLE / HISTORY — not proof of the new installation** |
| `MATDOG_JOINT_CALIBRATION.yaml` joint entries | **STALE — pending full recalibration** |
| Frozen LF `PositionOffset` values (M11 127, M12 851, M13 −486) | **HISTORICAL — all offsets are now 0** |

These records are **preserved deliberately**. They are historical evidence of what was measured
then. They are **not** a description of the robot now, and nothing may import them as current
state.

### LF V25 specifically

LF V25 remains a valuable **oracle** — it is the only leg ever calibrated against real mechanical
endstops, and its measurements remain the reference for what a correct result looks like. But the
LF servos have been removed and remounted. V25's numbers describe the *previous* installation.

### Geometry Compiler is a different thing

Geometry Compiler / CAD geometry is **distinct** from the calibration of the physical assembly.
The URDF, meshes and derived geometric endpoints are unaffected by reassembly — they describe the
design, not the build. Geometry remains valid; **calibration does not**.

---

## The new calibration philosophy

### Baseline: every servo has `PositionOffset = 0`

All 17 units were provisioned to `PositionOffset = 0` with physical RAW centre 2048 ±1.
See the [provisioning campaign](../Validation_Reports/ST3215_Provisioning_2026-08-27/README.md).

**This must remain the baseline.**

### During assembly

The mechanical intent is to hold each servo at physical RAW ≈ **2048** while mounting horns,
brackets and links directly in the calibration/q0 mechanical pose — so that mechanical zero and
electrical centre coincide as closely as the mechanics allow.

### Permanently forbidden

- ❌ **Do not** use `CalibrationOfs`.
- ❌ **Do not** use one-key-middle / one-key centering.
- ❌ **Do not** rewrite `PositionOffset` to compensate for mounting error.
- ❌ **Do not** automatically import any old digital q0.

> Compensating a mechanical mounting error by rewriting EEPROM `PositionOffset` is the specific
> thing this policy exists to prevent. If the mechanics are wrong, fix the mechanics.

### What q0 correction is *not*

The mechanical intent is to make the q0 correction zero or minimal — but **the final value must be
measured, not assumed**. Do **not** impose `q0_correction = 0` as a requirement or an assertion.
A small measured residual is an expected, legitimate outcome.

The forbidden action is compensating the mechanics by writing EEPROM `PositionOffset`. Recording a
measured software q0 offset is normal calibration and is **not** forbidden.

---

## Four layers that must stay separate

Conflating these is how calibration goes wrong. Keep them distinct in code, in data and in
documentation:

| Layer | What it is | Where it lives |
|---|---|---|
| 1. Physical RAW centre | servo electrical centre, 2048 | servo EEPROM, `PositionOffset = 0` |
| 2. Mechanics | how horns, brackets and links are physically mounted | the assembly itself |
| 3. Software digital-zero mapping | encoder ticks → joint zero | calibration config |
| 4. Joint calibration | measured limits, directions, ranges | calibration config |

Layer 1 is **done and frozen**. Layers 2–4 are **pending**.

---

## Required sequence after assembly

1. **Full calibration from scratch** — no reuse of prior calibration data.
2. **Re-capture / redefine digital q0** on the new installation, measured not imported.
3. **Verify mapping and directions** for all 17 joints — including the 5 new head joints, which
   have never been calibrated at all.
4. **Keep the four layers separate** as above.

### Gate before any motion

> **No hardware motion may be commanded from stale calibration.**
> **Full recalibration must complete before any stand, gait or load-bearing attempt.**

---

## Repository state — what was changed and what was not

### Preserved untouched

`09_Logs/Calibration/Digital_Zero/` and `09_Logs/Calibration/C5_R_digital_recenter/` are
**preserved in full**, with their `.sha256` companions. Nothing was deleted or edited. They are
marked historical by [`Digital_Zero/README.md`](Digital_Zero/README.md) and by this document.

### `MATDOG_JOINT_CALIBRATION.yaml` — deliberately additive

A `calibration_reset` block was **added** to
[`06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml`](../../06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml).

The existing enum values were **not** changed:

```yaml
robot.calibration_status: DIGITAL_ZERO_CALIBRATED_AND_VERIFIED   # unchanged
digital_zero_calibration.status: PASS_FINAL_EEPROM_READBACK      # unchanged
```

**Why they were left alone.** Six tools hard-assert those exact strings and refuse to run on any
other value:

```text
06_Software/Matdog_Core/calibration/matdog_visual_zero_pose_probe.py:132
06_Software/Matdog_Core/calibration/matdog_capture_visual_zero.py:205
06_Software/Matdog_Core/calibration/matdog_apply_visual_zero.py:134
06_Software/Matdog_Core/calibration/matdog_calibration_validate.py:104
06_Software/Matdog_Core/calibration/matdog_live_joint_monitor.py:105
06_Software/Matdog_Core/kinematics/matdog_leg_fk_live.py:101
```

Changing the enum would silently alter the runtime behaviour of all six — a code change disguised
as a documentation edit, and outside the scope of an archival task. Inventing a new enum value
without updating its consumers is exactly what the freeze policy forbids.

> ### ⚠️ Open item — deliberate enum transition required
>
> The stale-but-asserted enum is a **known, documented gap**. Before recalibration begins, a
> follow-up change should introduce a proper reset state (e.g.
> `CALIBRATION_RESET_PENDING_FULL_RECALIBRATION`) **together with** updates to all six consumers,
> so the tools fail closed on stale calibration instead of silently accepting it.
>
> Until that lands, treat every one of those six tools as **unsafe to run against hardware**
> regardless of what the YAML says.

---

## Related

- [ST3215 provisioning campaign 17/17](../Validation_Reports/ST3215_Provisioning_2026-08-27/README.md)
- [MATDOG_C018_V1 canonical profile](../../01_Docs/02_Architecture/MATDOG_ST3215_C018_V1_PROFILE.md)
- [Servo allocation](../../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml)
- [ST3215 evidence index](../ST3215_EVIDENCE_INDEX.md)
- [Handoff 2026-08-27](../Development_Log/2026-08-27_ST3215_CANONICAL_ARCHIVE_HANDOFF.md)
