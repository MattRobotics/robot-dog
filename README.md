# MATDOG — Custom Quadruped Robot

MATDOG is a custom quadruped robot developed by Matt Robotics in Italy. The project covers the complete robot stack: mechanical design, 3D-printed structure, power distribution, twelve Feetech ST3215 serial-bus servos, CAD-derived kinematics, calibration, locomotion, embedded control and future perception/autonomy.

## Project scope

MATDOG is not only a calibration project. This repository is the engineering source of truth for:

- mechanical architecture, CAD, URDF and collision geometry;
- electronics, power distribution, battery/BMS integration and servo wiring;
- servo mapping, joint conventions and calibration records;
- forward/inverse kinematics, contact geometry and trajectory validation;
- gait, balance, stand-up and future locomotion control;
- embedded-compute, sensing, watchdog and autonomy planning;
- validation reports, decisions and reproducible project evidence.

## System architecture

```text
MATDOG mechanical platform
→ 12 × Feetech ST3215 serial-bus servos
→ custom power-distribution and protection
→ Waveshare Bus Servo Adapter
→ NormaCore Station/ST3215 runtime
→ MATDOG calibration, kinematics and locomotion software
→ future embedded controller, Jetson, IMU and perception stack
```

### Repository responsibilities

```text
MattRobotics/robot-dog
→ public MATDOG source of truth: CAD, URDF, electronics, calibration evidence,
  kinematics, locomotion, validation and project decisions

MattRobotics/norma-core
→ Station/ST3215 integration fork and native MATDOG calibration runtime
```

Private research material is intentionally excluded from this public repository and is not part of the MATDOG runtime or public technical baseline.

## Current validated state

| Area | Status |
|---|---|
| Mechanical architecture and REV00 CAD/URDF | Validated |
| Twelve-servo bus, sparse IDs, mapping and directions | Validated |
| Digital-home commissioning and EEPROM readback | Validated for all 12 servos |
| Encoder-to-radian conversion and live read-only FK | Validated for all four legs |
| Offline contact, collision, timing and support references | Validated as engineering references |
| Geometry Compiler Phase 1 (24-endpoint offline audit) | Validated 2026-08-07; endpoint metrology **superseded by Phase 1B** |
| Geometry Compiler Phase 1B (adjacent revolute endstop metrology) | **VALIDATED AND MERGED** — 24/24 endpoints, final geometry suite 122/122 PASS, schema v4 current canonical (PR #16, `5b66044`) |
| Collision-mesh motor-pin representation | Corrected in 5 STLs, `rev00` unchanged |
| Mechanical end-stop calibration | **LF only: V25 hardware validated and frozen** |
| RF, RH and LH mechanical calibration | Not yet hardware validated |
| Complete 12-joint persistent profile | Not yet complete |
| Stand-up and locomotion | Pending post-calibration regeneration and validation |
| Embedded/autonomy integration | Planned |

> **Important:** the only mechanically hardware-validated calibration program is **LF V25**. Older V28–V42 experiments and previous “all legs” concepts are historical development records, not current programs and not valid development bases.

## Current milestone — LF V25 calibrated and frozen

On 4 August 2026 the complete left-front leg calibration completed successfully.

```text
58/58 sequence complete
6/6 LF mechanical contacts accepted
URDF affine gate PASS
supervised hardware-witness gate PASS
RAM q0 staging PASS
Station shutdown and serial release PASS
transactional EEPROM freeze PASS
persistent LF profile PASS
global torque OFF verified
```

Canonical technical record:

```text
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
```

Exact validated NormaCore source:

```text
branch: release/matdog-lf-calibrator-v25
source head: f87dd1fbc7e8100d275c74f9af448642f3429680
implementation PR: MattRobotics/norma-core#11
```

### LF final result

| Joint | Motor | MIN contact | MAX contact | Affine q0 before EEPROM | Final displayed q0 |
|---|---:|---:|---:|---:|---:|
| LF hip | M13 | 2535 | 1600 | 2067 | 2048 |
| LF upper | M12 | 1439 | 3443 | 2040 | 2051 |
| LF lower | M11 | 3093 | 1658 | 2074 | 2046 |

Frozen ST3215 Position Offsets:

| Motor | Previous | Frozen |
|---|---:|---:|
| M11 | 101 | 127 |
| M12 | 859 | 851 |
| M13 | -505 | -486 |

LF V25 is now a frozen reference leg. It must not be rerun unless LF mechanics, servo, mounting, URDF or calibration state changes.

## Geometry Compiler — Phase 1 and Phase 1B

The Geometry Compiler is the offline, hardware-free audit that derives each joint's geometric
endpoint from the URDF and the collision meshes, and cross-checks it against hardware evidence
where hardware evidence exists. It never moves a servo.

### Phase 1 (2026-08-07) and what it got wrong

Phase 1 reported **LF 6/6 `MODEL_INCOMPLETE`**: not one of the six LF joints had a mesh finding
corresponding to where V25 hardware actually stopped. The apparent conclusion was that the
collision STLs lacked hardstop geometry.

That conclusion was wrong. Two defects compounded:

1. **A conceptual policy error.** Phase 1 applied one blanket rule, `adjacent pair -> EXCLUDE`,
   treating a REVOLUTE hinge and a FIXED structural attachment identically. The designed
   mechanical hardstop physically lives *on the revolute parent/child pair* — so excluding every
   adjacency made the real endstop **unobservable by construction**.
2. **A mesh representation error.** The assembly STLs modelled the **motor centre pins** in
   nominal contact with the mating screws/pulleys of the adjacent link. That produced a
   permanent, angle-invariant overlap at the joint core, so the adjacent pairs read
   `INTERSECTING` at *every* angle and no `SEPARATED -> INTERSECTING` transition could be located
   even if they had been included.

Fixing either alone changes nothing. Phase 1B fixes both.

### GATE A — is the hardstop really on the adjacent pair?

A read-only diagnostic on the original geometry, production code untouched. All three LF adjacent
pairs were `INTERSECTING` across the entire sweep. The permanent overlap was found to be
**99.4–100 % within 6 mm of the joint axis**, in discrete concentric shells at Ø≈3 mm and
Ø≈5.5 mm spanning ≈±17.5 mm axially — shaft geometry, coaxial with the joint. The link owning
that feature is `base_link` for HIP and `<leg>_upper_leg_link` for UPPER/LOWER, reproduced from
the mesh alone and matching the motor-pin ownership found independently in CAD.

Genuine far-field contact (radial reach up to 96 mm — real external structure) appears within one
1° step of the V25 hardware angle on three endpoints, and not one step earlier.

### GATE B — does removing only the pin interference fix it?

Five collision meshes were corrected in CAD to give the motor pins ≈0.10–0.15 mm clearance.
Verdict **`SUPPORTED`**:

| check | result |
|---|---|
| candidate mesh sanity (geometric, retriangulation-tolerant) | PASS 5/5 |
| q=0 on the three LF adjacent pairs | **3/3 SEPARATED** (were 3/3 INTERSECTING) |
| LF first adjacent contacts, standard kernel, no mask | **6/6 localized** |
| witness geometry | 12/12 on external structure, radial 15.6–106.6 mm |
| historical v3 non-adjacent findings | −47.5000° and −97.9570° both reproduce exactly |

The motor-pin geometry edit itself was made in CAD. The first four upper-leg STL exports then
carried a common proper rigid frame error — each rotated 180° about Y through z = −45 mm
(`x' = -x; y' = y; z' = -z - 90 mm`); `base_link.stl` was unaffected. GATE B STEP 1 diagnosed it
geometrically before any collision physics ran. The frame error was corrected **directly in the
binary STL vertices and normals** by applying the determined rigid transform: no second CAD export
and no retessellation were performed for it, and triangle count, order and attributes were
preserved. The corrected files were reloaded and verified geometrically and by SHA256 before GATE B
physics.

### The five corrected STLs

`base_link.stl`, `lf_upper_leg_link.stl`, `rf_upper_leg_link.stl`, `rh_upper_leg_link.stl`,
`lh_upper_leg_link.stl`. Canonical filenames, same local frame, same scale, same coordinates.

**`rev00` is unchanged and the URDF is byte-identical** — `<visual>` and `<collision>` already
reference these filenames, so nothing in the URDF needed editing. All other STLs are untouched.
The CAD exports retriangulated the surfaces, so triangle counts and IDs changed; integrity was
therefore established geometrically, never by triangle correspondence.

### Phase 1B policy — REVOLUTE vs FIXED

```text
parent-child connected by REVOLUTE joint -> INCLUDE in collision analysis
parent-child connected by FIXED joint    -> structural attachment -> EXCLUDE from endstop metrology
```

12 revolute pairs; 4 fixed pairs (`<leg>_lower_leg_link ↔ <leg>_foot_link`). Adjacency and joint
type are derived from URDF topology, not from a hard-coded list.

### Endstop metrology vs path safety

```text
ENDSTOP METROLOGY = active revolute parent-child pair (exactly one pair per joint)
    HIP   -> base  <-> hip
    UPPER -> hip   <-> upper
    LOWER -> upper <-> lower

PATH SAFETY = all other relevant collision pairs
```

The active pair is excluded from its own path-obstruction set: its contact is the *desired
result*, not an obstruction. Conversely, a `hip ↔ foot` collision met during a LOWER search stays
a path-safety event and is never mistaken for the LOWER endstop — a regression that is
explicitly tested.

### Adjacent clearance policy

A revolute hinge's healthy resting state is sub-millimetre separation, so the generic 3 mm
minimum-clearance gate is **not** applied to revolute adjacent pairs; they contribute to path
safety through the boolean intersection test only. The non-adjacent clearance policy
(`EXACT` / `LOWER_BOUND` / `UNRESOLVED_FOR_THRESHOLD`) is unchanged.

The q=0 micro-clearances measured in GATE B are evidence about the current mesh revision, **not** a
physical assembly clearance and not a threshold to preserve. Their limit is CAD/tessellation/
mechanical significance: they sit far below per-part print tolerance (±0.15 mm), below the observed
CAD/tessellation/model surface differences of O(0.1 mm) between original and corrected meshes, and
below unmodelled assembly stack-up. The regression requirement is therefore semantic — adjacent
revolute at q=0 is SEPARATED — and no policy depends on the value.

### Phase 1B results — 24 endpoints

Run `2026-08-08_231600`, schema v4, exit 0, 59:27, peak RSS 541 MB.

| outcome | v3 (2026-08-07) | Phase 1B |
|---|---:|---:|
| `MODELED_ENDSTOP_CONTACT` | 6 | **21** |
| `NO_MODELED_ENDSTOP` | 14 | **0** |
| `MODEL_INCOMPLETE` | 6 (all LF) | 3 (all LF) |
| path collisions detected | 0 | 6 |

Every endpoint now resolves a first contact on its own active revolute pair.

LF against the V25 hardware oracle — 3 of 6 agree within the 2° band:

| endpoint | mesh contact | V25 hardware | delta |
|---|---:|---:|---:|
| `lf_upper_leg_min` | −52.0391° | −53.5254° | +1.486° |
| `lf_upper_leg_max` | +121.8750° | +122.6074° | −0.732° |
| `lf_lower_leg_min` | −92.0703° | −91.8457° | −0.225° |
| `lf_hip_min` | −46.0117° | −42.8027° | **−3.209°** |
| `lf_hip_max` | +45.2305° | +39.3750° | **+5.856°** |
| `lf_lower_leg_max` | +38.1797° | +34.2773° | **+3.902°** |

The two historical non-adjacent findings reproduce exactly and are now correctly classified as
**path** events rather than endpoints: `base_link ↔ lf_upper_leg_link` at −47.5000° and
`lf_foot_link ↔ lf_upper_leg_link` at −97.9570°. In both cases the joint's own articulation
contact occurs at a smaller |q|, so the endpoint and the obstruction are reported separately.

Mirror geometry is consistent to within 0.004° on upper/lower legs.

Parking is **`REVALIDATED / UNCHANGED FROM v3`** — deliberately not called PASS. All four legs
report `passed=False`, bit-identical to v3 (same clearances, same auxiliary flags). The Phase 1B
changes altered no parking verdict; the gate was already failing in v3 on non-adjacent clearance
and remains a pre-existing open item.

### Still open

The three LF endpoints above with large deltas (`lf_hip_min`, `lf_hip_max`, `lf_lower_leg_max`)
remain an explicit **UNKNOWN**. The model stops later than the hardware did and the reason is
unidentified; they are recorded as `HARDWARE_CONTRADICTED`, never as agreement. RF/RH/LH have no
hardware oracle at all and carry `GEOMETRIC_ENDPOINT_CANDIDATE` — model predictions awaiting
their own hardware validation, not measured hardstops.

Full technical record:

```text
06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_PHASE1B_ADDENDUM_2026-08-08.md
```

## Robot definition

Canonical REV00 package:

```text
03_CAD/URDF/matt_robodog_rev00/
```

The package includes the canonical URDF, visual/collision meshes, workbook, URDF Studio project and integrity manifest.

### Geometry

```text
front-to-rear hip spacing: 225 mm
left-to-right hip spacing: 95 mm
hip-to-knee segment: 90 mm
knee-to-foot mechanical interface: 110 mm
knee-to-contact-frame distance: 118.1 mm
target nominal body height: about 150 mm
```

Coordinate convention:

```text
X = forward
Y = left
Z = up
units = metres and radians
right-handed frame
```

### Servo mapping

```text
LF: M13 hip, M12 upper, M11 lower
RF: M23 hip, M22 upper, M21 lower
RH: M33 hip, M32 upper, M31 lower
LH: M43 hip, M42 upper, M41 lower
```

Canonical leg order and trot diagonals:

```text
[LF, RF, RH, LH]
LF + RH
RF + LH
```

## Permanent control and calibration rules

- Station is the sole ST3215 serial owner during motion.
- ST3215 `GoalPosition` remains unsigned standard `0..4095`; signed-wrap is forbidden.
- Digital-home commissioning remains separate from mechanical endpoint calibration.
- Calibration measures physical endpoints and derives the joint model from the URDF; it does not silently redefine CAD geometry.
- Every contact must satisfy repeatability, model consistency and supervised hardware evidence.
- EEPROM writes are allowed only after complete measurement PASS, verified Station shutdown and serial release.
- EEPROM provisioning is transactional and must include backup, readback, relock and rollback on failure.
- RF/RH/LH must be generalized from merged V25 architecture through data-driven leg profiles, not by reviving old versioned programs.

## Roadmap

### Foundation

- [x] Mechanical architecture and REV00 CAD/URDF
- [x] ST3215 bus, mapping and directions
- [x] Twelve-servo digital-home commissioning
- [x] Encoder/radian conversion and four-leg read-only FK
- [x] Offline contact, collision, timing and support references
- [x] Native Station-mediated calibration foundation

### Mechanical calibration

- [x] LF six-contact calibration
- [x] LF affine q0 derivation and URDF gate
- [x] LF transactional EEPROM freeze
- [x] LF persistent profile
- [ ] Generalize the validated V25 architecture to RF from merged `norma-core/main`
- [ ] Calibrate and freeze RF
- [ ] Calibrate and freeze RH
- [ ] Calibrate and freeze LH
- [ ] Validate the complete twelve-joint persistent profile

### Geometry Compiler

- [x] Phase 1 — 24-endpoint offline audit (2026-08-07, schema v3; superseded for endpoint metrology)
- [x] GATE A — adjacent-link raw baseline diagnostic
- [x] GATE B — motor-pin clearance mesh correction, verdict SUPPORTED
- [x] **Phase 1B — CLOSED**, merged via PR #16 (`5b66044`): joint-aware adjacency, endstop
      metrology vs path safety, schema v4 canonical
- [ ] Resolve the three LF hardware-contradicted endpoints
- [ ] Hardstop-surface-local sensitivity method for adjacent endpoints

## Next milestone — Phase 2 (NOT STARTED)

Phase 1B is closed. The next milestone is a **generic V25-derived full-leg engine in
`norma-core`**. It has **not** been started.

**Phase 2A — offline engine generalization.** Start from current merged `norma-core` `main`.
The historical RF worktree (`matdog/rf-calibrator-from-lf-v25`) is preserved **evidence**, not
the development base. Generalize the proven LF V25 architecture through **data-driven leg
profiles** (LF, RF, RH, LH), preserving the permanent contracts: Station is sole serial owner
during motion; `GoalPosition` unsigned `0..4095` with no signed wrap; contact acquisition RAM-only
until an explicit persistence gate; no Position Offset / EEPROM change without a separately
authorized transactional backup/readback/rollback; torque OFF on every hardware exit and failure
path. Schema-v4 outputs must be consumed correctly — RF/RH/LH endpoints are
`GEOMETRIC_ENDPOINT_CANDIDATE`, i.e. predicted search targets and bounds, **not**
hardware-confirmed stops, and must never silently overwrite URDF or q0. LF V25 remains the
immutable hardware oracle and is not re-run or re-zeroed because three modeled endpoints disagree.

**Phase 2B — path/parking pre-hardware gate.** Before any new real-leg calibration motion, the
parking/path-safety planning whose gate is still `passed=False` on all four legs must be resolved
or explicitly redesigned, and prerequisites/parking validated for the leg being calibrated under
the Phase 1B semantics (active revolute contact = endstop metrology; every other relevant contact
= path safety). No motion is authorized merely because an endpoint candidate exists. Parking being
unchanged from v3 was acceptable for Phase 1B closure; it is **not** acceptable to ignore before
Phase 2 hardware execution.

**Phase 2C — offline validation.** Unit tests, deterministic leg-profile mapping, FRONT/HIND
geometry handling, mirror logic, safe prerequisites, path/parking checks, failure/abort paths,
torque-off contracts, and no hardware imports or access in offline tests.

**Phase 3 — hardware.** Only after Phase 2 offline review and a **new explicit hardware
authorization**: RF → validate/freeze → RH → validate/freeze → LH → validate/freeze → complete
twelve-joint persistent profile. Legs are **not** batch-approved; each remains its own evidence gate.

### Locomotion

- [ ] Recompute four-leg FK with all frozen profiles
- [ ] Regenerate HOME → LOW_STAND → NOMINAL_STAND
- [ ] Repeat collision/contact/support audit with calibrated limits
- [ ] Supervised suspended stand
- [ ] Gradual load transfer and nominal stand
- [ ] Single-foot swing trajectory
- [ ] Trot in place
- [ ] First slow walking tests

### Embedded integration and autonomy

- [ ] Battery and smart BMS integration
- [ ] Embedded motion-controller evaluation
- [ ] Jetson integration
- [ ] IMU, estimator and watchdog integration
- [ ] Depth vision, object detection, voice and autonomous behaviour

## Repository structure

```text
01_Docs/        architecture and stable technical references
02_BOM/         components, suppliers and costs
03_CAD/         CAD, URDF, meshes and mechanical exports
04_Electronics/ wiring, power and servo mapping
05_Firmware/    future low-level firmware
06_Software/    calibration, kinematics, gait and control
07_Media/       images, renders and videos
08_Tests/       repeatable validation procedures
09_Logs/        decisions, reports and historical evidence
```

## Development policy

- `main` is the only active branch in `robot-dog` after milestone cleanup.
- New calibration development starts from the current merged architecture.
- At most one clearly named active calibration branch should exist while a milestone is under development.
- Temporary version-numbered workflows and branches must be removed after closeout.
- Closed PRs preserve historical experiments; obsolete branches are not retained as operational choices.
- Public documentation must describe MATDOG itself and must not expose unrelated private research material.

---

Built and documented by Matt Robotics.
