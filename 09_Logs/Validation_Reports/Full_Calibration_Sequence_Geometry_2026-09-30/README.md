# Full Calibration sequence geometry — 24-contact validation (2026-09-30)

Geometry only: nominal CAD mesh non-intersection. No hardware, no servo, no clearance-policy or
motion-authorization verdict.

Tool: [`06_Software/Matdog_Core/calibration/matdog_full_calibration_sequence_geometry_v5.py`](../../../06_Software/Matdog_Core/calibration/matdog_full_calibration_sequence_geometry_v5.py)
(the four artifacts record its SHA256 in `tool_sha256`).
Model: `03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf`, SHA256 `3890a3f0…81e59`
(= the compiled Geometry V5 provenance); collision-mesh manifest `60fff604…c39e1`.

| File | Leg | Segments | Result |
|---|---|---|---|
| `lf_full_calibration_sequence_geometry_validate.json` | LF | 15 | `sequence_collision_free = true` |
| `rf_full_calibration_sequence_geometry_validate.json` | RF | 16 | `sequence_collision_free = true` |
| `rh_full_calibration_sequence_geometry_validate.json` | RH | 11 | `sequence_collision_free = true` |
| `lh_full_calibration_sequence_geometry_validate.json` | LH | 11 | `sequence_collision_free = true` |

Each segment is swept at 0.5° with bisection refinement (1e-4 rad). The segments are parking,
both probe corridors of each joint up to the calibration guard (URDF limit + 64 ticks), every
prerequisite transition, the return phases and the park restore. Every segment is
`CLEAR_TO_END`, and every segment's start pose is `CLEAR` (only the segment's own modelled joint
stop is excluded). The per-segment minimum non-adjacent clearance is recorded as evidence.

Poses (URDF q): UPPER for the LOWER search 90°; UPPER for HIP MIN / MAX: LF 90° / 84.99°,
RF 84.99° / 90°, RH/LH 90° / 90°; LOWER folded: front −87.01° (V25 `LOWER_FOLDED_DELTA`), rear
−39.99° (−455 ticks, documented deviation `v25_deviations` in the RH/LH artifacts); front rear-UPPER
park 35°.

These four files are the only input of
`05_Firmware/MATDOG_Controller/src/actuator/CalibrationSequencePlanData.h`:

```
python3 06_Software/Matdog_Core/calibration/matdog_full_calibration_sequence_geometry_v5.py \
  --export-header 05_Firmware/MATDOG_Controller/src/actuator/CalibrationSequencePlanData.h \
  --artifacts <lf>,<rf>,<rh>,<lh>
```

`scripts/static_audit.py` re-runs that export and fails on any difference.

Rationale, rear-fold decision and V25 comparison:
[`../../Development_Log/2026-09-30_TRUE_24_CONTACT_FULL_CALIBRATION.md`](../../Development_Log/2026-09-30_TRUE_24_CONTACT_FULL_CALIBRATION.md) §4.
