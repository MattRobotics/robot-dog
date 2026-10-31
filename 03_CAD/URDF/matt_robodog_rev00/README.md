# MATDOG — URDF REV00 Kinematic Baseline

## Status

Canonical kinematic model (REV00 kinematics with the REV01-FIX base_link and HIP offsets, see below).

This directory is the canonical REV00 robot description used as the starting point for inverse kinematics, stand-pose development, gait generation and later simulation work.

## Canonical Files

- `matt_robodog_rev00.urdf`
  Canonical URDF model.

- `meshes/collision/*.stl`
  The single STL per link (17 files). Since REV01-FIX every `<visual>` and every
  `<collision>` in the URDF references the same file, so what is rendered is
  exactly what the Geometry Compiler V5 checks. The former detailed visual set
  under `meshes/*.stl` was removed (recoverable from Git history), and so was
  the former URDF Studio project archive `urdf_studio/matt_robodog_rev00.usp`
  (it embedded those detailed meshes, the REV00 `base_link` and an out-of-date
  URDF, i.e. a second geometry source; recoverable from Git history). The
  digital-twin render meshes are generated from the 17 canonical STL files by
  the existing tools (viewer staging and decimation) and are never a second
  source.

- `textures/`
  URDF Studio texture assets, when present.

- `reference/Coordinate_Cinematiche_matt_robodog_rev00.xlsx`
  The single canonical workbook (7 sheets, see "Physical Data Status"): joint table, link masses/CoM/inertias, CAD source data, mass reconciliation, weight evidence, and the preliminary ST3215 3S speed/torque reference.

- `SHA256SUMS.txt`
  Integrity manifest for the canonical URDF assets.

## Kinematic Topology

- 1 base link
- 4 legs
- 17 links
- 16 joints
- 12 revolute leg joints
- 4 fixed foot joints

Each leg follows:

base_link
→ hip_joint
→ hip_link
→ upper_leg_joint
→ upper_leg_link
→ lower_leg_joint
→ lower_leg_link
→ foot_joint
→ foot_link

## Coordinate Convention

- X: forward
- Y: left
- Z: up
- Units: metres and radians
- Right-handed coordinate system

## Joint Limits Consolidated in REV00

Hip joints:

- Minimum: -45.0 degrees
- Maximum: +45.0 degrees

Upper-leg joints:

- Minimum: -52.5 degrees
- Maximum: +122.5 degrees

Lower-leg joints:

- Minimum: -92.0 degrees
- Maximum: +37.5 degrees

The lower-leg closing limit was reduced to -92.0 degrees to avoid mechanical interpenetration.

## Mesh Convention

The four upper-leg STL meshes have their final orientation baked directly into the mesh vertices.

Therefore, the URDF visual and collision mesh origins for upper-leg links are intentionally:

origin xyz="0 0 0" rpy="0 0 0"

No compensating mesh transform should be reintroduced without updating the canonical reference workbook.

## Collision Status

Every link uses one STL, `meshes/collision/<link>.stl`, for both `<visual>` and
`<collision>` (same `origin` and `scale="0.001 0.001 0.001"`, verified identical
before unification). The 16 leg meshes are the dedicated collision baseline
approved on 2026-08-10 and are unchanged. `base_link.stl` is the REV01-FIX CAD
export (36,832 triangles, SHA-256 `c2eadf6d6db1b03379cf256b542dadbcd8d63076179bc84ebd7169634e9f0097`),
which replaced the 22,044-triangle REV00 base. The set now totals 134,484
triangles.

The REV00 baseline had 119,696 triangles and 5,986,228 bytes; it reduced the
historical 472,020-triangle collision set by 74.64%.

## REV01-FIX Integration (2026-10-30)

- `base_link` collision/visual mesh replaced by REV01-FIX. Its eight motor pins
  were corrected to a nominal 0.25 mm radial and axial clearance to the HIP
  sockets, which are unchanged.
- The four `hip_joint` origins were raised by 2.2 mm in Z: LF/RF 0.0465 ->
  0.0487 m, RH/LH 0.0265 -> 0.0287 m. The front/rear hip offset is still
  20.0 mm. All other joint origins, axes and limits are unchanged.
- The directory and file names keep the `rev00` label on purpose, so existing
  tools and paths keep working; the revision is identified by the SHA-256 values
  in `SHA256SUMS.txt`.
- Geometry Compiler V5 (offline, unchanged kernel): q=0 12/12 pairs separated,
  24/24 endpoint searches completed. Against REV00, 22 endpoints are identical;
  the RH hip minimum and LH hip maximum differ by 6.8e-5 rad, below the 1e-4 rad
  bisection resolution. Measured pin/socket clearance on the STL meshes is
  0.250 mm axial and 0.250 mm radial by vertex distance (0.203-0.204 mm
  minimum surface distance because the two cylinders are tessellated
  differently).
- Physical properties (2026-10-31): see "Physical Data Status" below.

## Physical Data Status

Single nominal configuration, including the Jetson Orin (175 g, inside `base_link`). The 17 `<inertial>` blocks come from the Solid Edge physical-property reports of 2026-10-30: 17 link masses sum to 2.928 kg against 2.937 kg for the Solid Edge assembly (the 9 g difference lies at the 1 g resolution of the per-link mass reports; it is not missing geometry and is not allocated). CoM origins are in the link frames; tensors are about the CoMs, rebuilt from principal axes and radii of gyration; RF/RH are Y reflections of LF and LH uses the LF frame (verified on the meshes; the rear hip meshes differ slightly, see the report).

The robot weighed on 2026-10-30 (2.807 kg) did not carry the Jetson: it is historical evidence of the hardware assembled that day, not a second configuration. No dynamic hardware validation has been performed on these properties.

Provenance, derivation tools, input hashes and the reconciliation are in `09_Logs/Validation_Reports/REV01_FIX_Physical_Properties_2026-10-31/`. The single canonical workbook `reference/Coordinate_Cinematiche_matt_robodog_rev00.xlsx` has 7 sheets: `Joint_Parameters` (16 joints, motor IDs, origins incl. hip Z 48.7/28.7 mm, limits, and the *design candidate* speed/torque columns), `Reference_Notes` (conventions, model audit, method notes, evidence register), `Inertial_Properties` (the one source of the masses, CoM and inertia tensors of the 17 links, with mesh paths and provenance), `CAD_Source_Data`, `Mass_Reconciliation`, `Weight_Evidence_2026-10-30` (distinta e pesata) and `ST3215_3S_Reference` (preliminary speed/torque estimate with a chart). Earlier sheets `Mesh_Transforms`, `Link_Engineering`, `Mass_Summary`, `Engineering_Weight_Source`, `URDF_Audit`, `Physical_Property_Method` and `Physical_Evidence_Log` were merged into the remaining ones in the approved 2026-10-31 consolidation. Previous versions are kept by Git only (commits `0f8f7ff` and `7d04939`); there is no second workbook.

## Mechanical Segment and Foot-Contact Frame

- `hip_to_knee`: `90.0 mm`, measured between the relevant joint axes.
- `knee_to_foot_interface_center`: `110.0 mm`, measured from the knee axis
  to the center of the lower-leg endpoint where the eccentric rubber foot
  attaches.
- `knee_to_foot_contact_frame`: `118.1 mm`, measured from the knee axis to
  the URDF `foot_joint`, which represents the nominal ground-contact point
  of the eccentric rubber foot.

The lower-leg mechanical interface center and the URDF `foot_joint` are
intentionally different because the rubber foot is eccentric. Use the
`foot_joint` contact-frame distance for IK, stand-pose and gait planning.

## ST3215 URDF Effort and Velocity Semantics

All twelve revolute leg joints currently use:

    effort   = 0.902244 N*m
    velocity = 3.03687289847 rad/s = 29 rpm

These are conservative MATDOG nominal-operation limits for the intended 3S
operating point. They are not an official Feetech torque-speed curve at
exactly 11.1 V.

**Design reference under review (2026-10-31).** The approved workbook
(`ST3215_3S_Reference`) adopts 40 RPM (4.18879 rad/s) as the *nearly no-load
design ceiling* at 11.1 V, scaled from the 45 RPM no-load figure Feetech
declares at 12 V, with a linear torque-speed estimate (about 27.8 RPM at
9.2 kg·cm). That curve is a preliminary estimate, not measured on the robot,
and 40 RPM is not available together with the torque budget. The URDF above
(29 RPM) and anything pinned to it are intentionally **unchanged**: 29 RPM is
neither confirmed nor discarded as the final constraint. Changing the executive
limit, and re-qualifying what depends on it, is a separate Motion task
(`09_Logs/Validation_Reports/REV01_FIX_Alignment_2026-10-31/ASTRA_HANDOFF.md`).

### Calibration provenance (2026-10-31)

The hardware calibration chain (Geometry V5 bundle, `CalibrationGeometryProfileData.h`, sequence plan, startup reference, gen-1 NVS record) stays bound to the REV00 inputs it was measured on. Those inputs are preserved in `../historical_calibration_basis/rev00/` and are not the operational model. `06_Software/Matdog_Core/calibration/matdog_calibration_basis.py --check` (run by `static_audit.py`) proves this model equals REV00 plus the declared delta only: the four hip origin Z values, the visual mesh paths, the `base_link` collision mesh and, since 2026-10-31, the `<inertial>` block of each of the 17 links listed in `BASIS.json`. Each `<inertial>` is checked structurally (exact children, rpy 0) and physically (mass in (0, 5] kg, symmetric positive-definite tensor satisfying the triangle inequality, CoM inside the link collision-mesh bounds, plausible radius of gyration). Any other change — joint axes, limits, motor metadata, other collision meshes, hip X/Y, extra elements — still fails. The calibration logic and chain files are untouched.
