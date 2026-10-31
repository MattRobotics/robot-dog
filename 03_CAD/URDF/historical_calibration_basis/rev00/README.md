# REV00 calibration basis (historical provenance — not the operational model)

The operational model is `../../matt_robodog_rev00/` (REV01-FIX: new `base_link` collision mesh, four hip origins +2.2 mm in Z, one STL per link for visual and collision).

This directory preserves, byte for byte, the three REV00 inputs that differ from it and that the **hardware calibration chain** is bound to:

| File | SHA-256 |
|---|---|
| `matt_robodog_rev00.urdf` | `3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59` |
| `SHA256SUMS.txt` (REV00 manifest) | `60fff604eae0857c7c61f115bbe3922949ababdd827fab61c74e1673336c39e1` |
| `meshes/collision/base_link.stl` | `0a485e7a1101d457f317b664e52a7a4ef061382e6ecc2c7c87a333c81d5b466f` |

The other 16 REV00 collision meshes are unchanged and are verified in place against the REV00 manifest.

Bound to this basis (preserved unchanged): the Geometry Compiler V5 bundle `2026-08-11_131818_…_REMEDIATION_BENCHMARK_D_W4`, `CalibrationGeometryProfileData.h`, `CalibrationSequencePlanData.h`, the 2026-09-30 sequence validation artifacts, `CalibrationQ0EvidenceData.h`, the startup reference, and the gen-1 NVS record (geometry tag `3713f4ddc43b204e`).

Rules:
- Do not edit these files. `BASIS.json` declares them and `matdog_calibration_basis.py` re-hashes them on every use.
- `python3 06_Software/Matdog_Core/calibration/matdog_calibration_basis.py --check` (also run by `static_audit.py`) proves the operational model equals REV00 plus the declared delta only: the four hip origin Z values, the visual mesh paths and the `base_link` collision mesh. A change to any joint axis/limit/motor metadata, inertial, other collision mesh or hip X/Y fails it.
- Nothing here claims hardware validation on REV01-FIX. Calibration evidence is joint-space and was measured on the physical robot; this file only records which model its provenance hashes refer to.
