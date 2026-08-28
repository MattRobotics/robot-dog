# Full Leg Calibrator V1 — hardware validation handoff

**Date:** 2026-08-28
**Status:** implementation complete, offline validated, **H1+ NOT executed**
**Tool:** [Full Leg Calibrator V1](../../../05_Firmware/Full_Leg_Calibrator_V1/README.md)

This document is the operating procedure for the physical session. It describes
what each stage does, what must be true before it runs, and where the operator
must explicitly authorize motion.

> **Nothing below has been executed on hardware except H0.**
> H1–H7 are procedure, not evidence.

---

## The stage ladder

```text
H0  ESP32-only, no servos              ✅ executed 2026-08-28
H1  12-servo read-only census          ⬜ requires operator present
H2  manual q0 capture, torque OFF      ⬜
H3  one-joint characterization         🔒 explicit motion authorization
H4  one-joint calibration              🔒
H5  one-leg calibration                🔒
H6  four-leg candidate set             🔒
H7  separate freeze/promotion gate     🔒 not part of the calibrator
```

Each stage is a **separate firmware image**, built by one explicit flag:

```bash
cd 05_Firmware/Full_Leg_Calibrator_V1
./tools/build_stage.sh 1              # H1 census image
./tools/build_stage.sh 3 --bootstrap  # H3 characterization image
./tools/build_stage.sh 4 --bootstrap --upload
```

`tools/build_stage.sh` is the only supported way to raise the stage. Raising the
stage **widens which named operation may be attempted; it does not bypass**
identity, fresh census, measured direction, or any hard servo guard. A test
asserts that H7 alone still refuses motion.

---

## Two independent gates protect first motion

**Gate 1 — build stage.** `FLC_AUTHORIZED_STAGE`, default `H0`, in
`flc_stage_config.h`. H3 is the lowest stage at which anything may move.

**Gate 2 — pre-motion parameters.** Only `CLASS_A` parameters can block motion,
and they are satisfied either by being resolved or by an explicitly approved
bootstrap envelope. Approval requires **both**:

- the build flag `FLC_H3_BOOTSTRAP_APPROVED=1`, and
- the live command `@APPROVE_BOOTSTRAP CONFIRM` in that session.

Neither alone is sufficient. A fresh `@CENSUS` clears the session approval.

### Why H3 is no longer deadlocked

The first version treated all eight contact parameters as pre-motion blockers.
That was circular: H4 needed them, H3 was meant to measure them, and H3 was
blocked by them. They are now classified by *when* a value can exist:

| Parameter | Class | Resolution |
|---|---|---|
| `CONTACT_TORQUE_LIMIT` | **A** pre-motion | bootstrap envelope, refined by H3 |
| `CONTACT_GOAL_SPEED` | **A** pre-motion | bootstrap envelope |
| `CONTACT_ACCELERATION` | **A** pre-motion | bootstrap envelope |
| `CONTACT_RETREAT_TICKS` | **B** measured in H3 | H3 reports what it achieved |
| `CONTACT_CURRENT_THRESHOLD_RAW` | **C** derived | from the per-joint median/MAD baseline — no global constant exists |
| `CONTACT_REPEATABILITY_TOLERANCE_TICKS` | **D** acceptance | derived from the spread H3 measured across two approaches |
| `ENDPOINT_VS_URDF_TOLERANCE_TICKS` | **D** acceptance | judges a result; unknown ⇒ result stays `CANDIDATE` |
| `MANUAL_Q0_VS_DERIVED_Q0_TOLERANCE_TICKS` | **D** acceptance | judges q0; never blocks acquiring q0 data |

A **CLASS_D** tolerance never blocks the measurement it exists to judge. An
unknown acceptance band leaves the result `CANDIDATE` rather than forcing
`ACCEPTED` or refusing to measure.

### The bootstrap envelope is not a measurement

| | Value | Comparison |
|---|---|---|
| TorqueLimit | **200** | below provisioner bench 300, below LF V25 500 |
| GoalSpeed | **60** | far below LF V25 160 |
| Acceleration | **8** | matches the slowest historical value |
| Retreat | **96 ticks** | H3 measures what was actually achieved |

Origin is tagged `H3_BOOTSTRAP_OPERATOR_APPROVED`, never
`CHARACTERIZED_CURRENT_HARDWARE`. It is reported by `@STATUS`, so a session can
never silently be running on it. It never becomes canonical and never touches
EEPROM. It is a candidate for refutation by H3, not a result.

---

## Procedure

### Before anything

1. **Physically support the robot.** Legs must be able to move without bearing
   the body.
2. Connect the 12 leg servos and servo power **only with the operator present**.
3. Confirm the ESP32-S3 port and that no other process holds it.

### H1 — read-only census

```bash
./tools/build_stage.sh 1 --upload
python3 matdog_full_leg_calibrator_runner.py census
```

Verifies all 12 ids, model 777, `PositionOffset = 0`, the 20-register profile
fingerprint, torque OFF, and plausible telemetry. Head ids 51–55 are expected
**absent**; a responder there is a failure.

**Nothing moves. No writes at all.** Review before continuing.

### H2 — manual q0 capture

Operator places the robot in the CAD calibration pose using the square, then:

```bash
./tools/build_stage.sh 2 --upload
python3 matdog_full_leg_calibrator_runner.py capture-q0 --samples 64
```

Torque stays **OFF** and is verified per joint. Produces
`manual_pose_q0_candidate` only — dispersion included, never rounded to 2048,
never promoted.

**Review the residuals.** A joint more than ~120 ticks from 2048 usually means
the pose is wrong, not that the joint is.

### H3 — characterize ONE joint ⚠️ FIRST MOTION

> **Explicit operator authorization required. This is the first time the
> reassembled robot moves under calibrator control.**

```bash
./tools/build_stage.sh 3 --bootstrap --upload
python3 matdog_full_leg_calibrator_runner.py census
python3 matdog_full_leg_calibrator_runner.py approve-bootstrap
python3 matdog_full_leg_calibrator_runner.py characterize-joint --joint 13
```

Start with **one** joint. `13` (LF_HIP, unit M22) is a reasonable first choice
because the hip endstop carrier is the component that was corrected.

H3 performs, in order:

1. **H3A direction** — a small commanded step; the encoder sign is *measured*.
   A joint moving opposite to the plan aborts with `WRONG_DIRECTION`.
2. **H3B baseline** — a bounded free-motion excursion; median/MAD current,
   velocity and tracking behaviour. The contact threshold is derived from this.
3. **H3C first contact** — bounded approach to a real mechanical stop, then a
   verified retreat.
4. **H3D second contact** — an independent re-approach, giving a **measured**
   repeatability spread and the acceptance band for H4.

Outputs, all tagged `CHARACTERIZED_CURRENT_HARDWARE`: direction, baseline,
contact threshold, achieved retreat, observed spread, repeatability band.

**Stop and review.** Compare against expectations before touching another joint.

### H4 — calibrate the same joint

```bash
./tools/build_stage.sh 4 --bootstrap --upload
python3 matdog_full_leg_calibrator_runner.py census
python3 matdog_full_leg_calibrator_runner.py approve-bootstrap
python3 matdog_full_leg_calibrator_runner.py characterize-joint --joint 13
python3 matdog_full_leg_calibrator_runner.py calibrate-joint --joint 13
```

H4 requires characterization **from the same physical session** — a fresh census
clears it deliberately.

For each endpoint: approach → contact → retreat + verify → independent second
approach → repeatability → release. Both endpoints are approached **from
neutral**, never straight across the full span, so each traversal stays around
one joint half-range.

Produces measured MIN/MAX, span vs URDF, derived q0 and direction, at tier
`MEASURED` → `CANDIDATE` → `ACCEPTED`. **`PROMOTED` is never reached here.**

### H5 — one leg

```bash
./tools/build_stage.sh 5 --bootstrap --upload
# characterize all three joints of the leg first, then:
python3 matdog_full_leg_calibrator_runner.py calibrate-leg --leg LF
```

Joints run **distal first** (LOWER → UPPER → HIP), keeping the limb folded and
the swept volume small while proximal joints are still uncalibrated. Any failure
stops the leg and releases every joint.

### H6 — four legs

```bash
./tools/build_stage.sh 6 --bootstrap --upload
python3 matdog_full_leg_calibrator_runner.py calibrate-all
```

Canonical order `LF → RF → RH → LH` from `MATDOG_GEOMETRY.yaml`. One session
result: 12 joint results, 24 endpoints, 12 directions, 12 derived q0 candidates,
all gates, final global torque-OFF verification.

### H7 — promotion

**Not part of the calibrator.** Writing into `MATDOG_JOINT_CALIBRATION.yaml` is
a separate, explicit, reviewed operation after 12/12 validation. No calibrator
path modifies canonical calibration.

---

## Authorization checkpoints

| Point | Required |
|---|---|
| Connect servos and power | operator physically present |
| Before H3 | explicit motion authorization + `@APPROVE_BOOTSTRAP CONFIRM` |
| Before H4 on a new joint | review the H3 evidence for that joint |
| Before H5 | review a complete single-joint calibration |
| Before H6 | review a complete single-leg calibration |
| Before H7 | review the full 12-joint candidate set |

## If something goes wrong

Every abort path releases torque and reports a structured reason. If a joint
will not release, the firmware prints `CUT_SERVO_POWER_NOW` — do that.

`@SAFE_OFF` is unicast, verified per servo and idempotent; it is safe to issue at
any time, including after a fault.

## Related

- [Validation evidence](README.md)
- [Firmware README](../../../05_Firmware/Full_Leg_Calibrator_V1/README.md)
- [Leg reassembly premise](../../Calibration/MATDOG_LEG_REASSEMBLY_CALIBRATION_PREMISE_2026-08-28.md)
- [Calibration reset](../../Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md)
