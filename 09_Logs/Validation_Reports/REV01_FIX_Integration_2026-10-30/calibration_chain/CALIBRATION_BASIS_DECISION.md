# Calibration chain vs REV01-FIX — decision implemented (Option 1)

Date: 2026-10-31. Offline only. No flash, NVS write, q0 capture, calibration, EEPROM or servo access. `REHEARSAL_ANALYSIS.md` in this folder is the analysis that preceded the decision (it describes the three options; Option 1 was approved and implemented here).

## What changed
- `03_CAD/URDF/historical_calibration_basis/rev00/`: preserved REV00 URDF, REV00 `SHA256SUMS.txt`, REV00 `base_link` collision STL, `BASIS.json`, README.
- `06_Software/Matdog_Core/calibration/matdog_calibration_basis.py` (new, ~150 lines): hash-verified access to the preserved files for the closed superseded set, and `verify_delta()` / `--check`.
- `matdog_calibration_geometry_export.py`: reads the REV00 URDF/manifest from the basis; every recorded bundle input must match the live file or, only for the declared superseded files, the preserved copy; refuses if the operational model departs from the declared delta.
- `matdog_full_calibration_sequence_geometry_v5.py`: mesh manifest hash taken from the preserved REV00 manifest (the artifacts were produced under it).
- `static_audit.py`: new `check_calibration_basis`; no existing check weakened, no pin changed.
- Tests: `calibration/tests/test_matdog_calibration_basis.py` (11 tests incl. mutations); `test_calibration_record.cpp` gains `gen-1 identity vs operational geometry` (literal REV00 digests and tag, fails if the profile is rebound).

## Unchanged by design (byte-identical)
`CalibrationGeometryProfileData.h`, `CalibrationSequencePlanData.h`, `CalibrationQ0EvidenceData.h`, `StartupRecoveryReference.*`, the Geometry V5 bundle, the 2026-09-30 validation artifacts, `MATDOG_SERVO_ALLOCATION.yaml`, tag `3713f4ddc43b204e`. Both exporters reproduce the committed headers byte for byte from the preserved inputs.

## Residual constraint (documented, not a new restriction)
- Startup Recovery stays a REV00-certified procedure: its tick boxes and the CAD tolerance certificate (396 PASS / 708 boxes / ≥3.1 mm) were issued on REV00 and the generator is not in the repository, so the certificate is **not re-issued**. A sampling check (114 poses in the startup boxes) gives identical minimum clearance on REV00 and REV01-FIX (`startup_box_spotcheck_*.json`); it is not a qualification.
- Calibration endpoint values for LH HIP max / RH HIP min (diagnostic, beyond the URDF limit) and the half-span of the LH/RH HIP are REV00 numbers; on REV01-FIX they are 68 µrad (0.044 tick) smaller. The tick-domain decisions are identical (513 ticks inside, 514 outside, for both values).
- Sequence validation: gating verdicts identical on REV01-FIX (53/53 segments); evidence-only clearance of the rear legs against the base during HIP sweeps drops from ~0.19–0.21 mm to ~0.09 mm (nominal CAD).
- Motion certifications of dev5 (STAND 150 mm, PairedProne, `collision::k*`) remain REV00-based and separate.
