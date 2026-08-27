# MATDOG Calibration — Canonical Cross-Repository Handoff

**Revision:** 2026-08-07-r1  
**Status:** CANONICAL DEVELOPMENT CONTRACT — PRE-GEOMETRY-COMPILER  
**Hardware execution:** NOT AUTHORIZED BY THIS DOCUMENT  
**Scope:** MATDOG mechanical endpoint calibration, geometry compiler, q=0 derivation, generic NormaCore calibration engine, and future repeatable recalibration.

This document intentionally supersedes the development direction in the 2026-08-05 RF Claude handoff where that direction conflicts with the decisions below. Historical evidence is preserved and must not be deleted.

An identical revision of this handoff is intended to be stored in both repositories:

- `MattRobotics/robot-dog` — project/geometry source of truth;
- `MattRobotics/norma-core` — Station/ST3215 calibration-runtime source of truth.

If the two repositories ever disagree, use the current real repository/file state for its own domain and stop rather than silently reconciling contradictory data.

---

## 1. Current verified repository state and unfinished local checkpoint

### Remote repositories

At the time this contract was written:

```text
MattRobotics/robot-dog
  main = 04da910c98e7e5f17bb284d3c690699807fbdd3e
  active remote branches: main only

MattRobotics/norma-core
  main = b2f7dac2eab7147917fccdfde702360da82ab7de
  immutable validated release: release/matdog-lf-calibrator-v25
  LF V25 reviewed source head: f87dd1fbc7e8100d275c74f9af448642f3429680
  active remote branches: main + release/matdog-lf-calibrator-v25
```

### Current local RF checkpoint on the ASUS

The last Claude Code session created an offline, uncommitted RF worktree:

```text
/home/matteo-manicardi/MATDOG/worktrees/norma-core-rf-calibrator
branch: matdog/rf-calibrator-from-lf-v25
base/HEAD at checkpoint: b2f7dac2eab7147917fccdfde702360da82ab7de
```

Modified only:

```text
software/drivers/st3215/src/auto_calibrate/matdog.rs
software/drivers/st3215/src/auto_calibrate/matdog_test.rs
```

Checkpoint diff:

```text
1176 insertions, 14 deletions
```

Offline validation recorded at the checkpoint:

```text
rustfmt file-scoped: PASS
cargo check --package st3215: PASS
RF witness tests: 5/5 PASS
cargo test --package st3215 --lib: 140/140 PASS
git diff --check: PASS
hardware: NOT RUN
commit/push/PR: NONE
```

This RF patch is evidence and a study checkpoint. Preserve it. Do not destructively reset or delete it. It is **not** the architecture to continue orchestrating unchanged.

---

## 2. Why the RF branch became too large

The 2026-08-05 Claude handoff deliberately instructed the agent to reproduce LF V25 on RF while avoiding a refactor of LF. This was a conservative response to earlier RF false passes and regressions.

That instruction produced a duplicated structure such as:

```text
LfSessionStateMachine
RfSessionStateMachine
```

and RF-specific mirrors of LF role/corridor/session helpers.

That historical decision is now **SUPERSEDED** for future development.

The target architecture is:

```text
Generic V25 Full-Leg Engine
  -> LF LegCalibrationSpec
  -> RF LegCalibrationSpec
  -> RH LegCalibrationSpec
  -> LH LegCalibrationSpec
```

not four independent calibration engines.

LF V25 remains the only full-leg hardware-validated oracle and must remain behaviorally protected by regression tests.

---

## 3. Geometry is the primary model reference

MATDOG is defined by its own CAD, URDF, collision meshes, joint mapping and physical assembly.

Canonical current URDF:

```text
MattRobotics/robot-dog
03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf
```

The collision STL meshes in the URDF are the same nominal geometry used to manufacture the printed structural parts. The current reported PPA+CF dimensional tolerance is approximately +/-0.15 mm per printed part. This is an input to uncertainty analysis, not a hardcoded conversion to encoder ticks.

The current URDF declared joint limits were initially established from 3D CAD observation and then rounded to convenient angular values. Therefore the system must distinguish:

```text
urdf_declared_limit
mesh_predicted_contact
hardware_measured_contact
```

These are related but not automatically identical.

The geometry compiler must never silently force them to be identical.

---

## 4. FRONT and HIND are not the same geometry case

Left/right counterparts are mirror-related, but FRONT and HIND are not interchangeable.

The current REV00 URDF places the front hip axes 20 mm higher than the hind hip axes:

```text
front hip Z ~= 0.0465 m
hind hip Z  ~= 0.0265 m
```

Consequences:

- LF and RF may be mirror-equivalent after validating handedness;
- RH and LH may be mirror-equivalent after validating handedness;
- FRONT calibration paths and HIND calibration paths must be solved from the actual URDF;
- detector logic does not change because of this 20 mm offset;
- prerequisites, path clearance and auxiliary parking may change.

No FRONT prerequisite or parking pose may be copied to HIND merely by convention.

---

## 5. Parking and prerequisites are outputs of geometry planning

The 2026-07-20 geometry checkpoint proved one safe solution:

```text
HIP prerequisite: upper about +50 deg
LOWER prerequisite: upper about +90 deg
LF calibration: LH upper parked +30 deg
RF calibration: RH upper parked +30 deg
RH/LH: no extra parking in that tested plan
```

That checkpoint proved **collision-free paths for those selected poses**. It did not prove that those poses are minimal or always necessary.

New rule:

```text
default = NO AUXILIARY PARKING
```

For every leg and calibration phase, the geometry planner shall:

1. test the active-leg transition/path with the other legs in the nominal safe/home configuration;
2. perform continuous path collision/clearance checks, not endpoint-only checks;
3. if the path is valid with adequate clearance, use no parking;
4. if invalid, search for the smallest justified auxiliary movement;
5. record the chosen auxiliary movement in the generated geometry profile;
6. validate the auxiliary path into and out of that pose.

Historical +30/+50/+90 degree values may be used as search seeds only.

---

## 6. Geometry Compiler — required Phase 1 deliverable

The first new development phase is an **offline geometry compiler** in `robot-dog`.

It shall reuse the already-developed URDF FK / collision infrastructure wherever it is still valid, including the prior AABB, convex-hull and BVH triangle/triangle narrow-phase work. Do not create a second collision kernel merely for stylistic reasons.

For every combination:

```text
4 legs x 3 joints x 2 sides = 24 endpoints
```

it must determine and record:

- joint semantic name and servo mapping;
- `urdf_declared_limit_rad`;
- exact/locally refined `mesh_predicted_contact_rad`;
- contact/collision link pair responsible for the intended endpoint;
- distance/clearance immediately before contact;
- numerical search resolution/error bound;
- manufacturing/assembly uncertainty inputs;
- predicted contact-angle uncertainty band;
- required prerequisite pose;
- auxiliary parking movement, if any;
- complete safe transition path into probing configuration;
- complete restore path;
- minimum modeled clearance along each path;
- geometry validation result and reason.

### Contact search method

Do not assume the rounded URDF limit itself is the exact mesh contact.

Offline only, search around the declared limit inside an explicitly bounded analysis envelope to find the first intended mesh contact, then refine it with an adaptive/bisection-style search.

If the first contact is an unintended collision pair rather than the designed mechanical stop, report the configuration as invalid instead of relabeling that collision as the endpoint.

### Tolerance handling

The stated +/-0.15 mm print tolerance is not automatically the total relative assembly tolerance. Keep uncertainty parameterized and report sensitivity.

Near a predicted contact, estimate how local clearance changes with joint angle. A relationship of the form

```text
Delta q ~= Delta d / |d(clearance)/dq|
```

may be used as a local sensitivity estimate, with explicit numerical validation and bounds.

### Machine-readable artifact

Generate a versioned artifact, conceptually:

```text
MATDOG_CALIBRATION_GEOMETRY_PROFILE
```

It must include at least:

```text
schema/revision
robot-dog commit SHA
URDF SHA256
collision mesh manifest + SHA256
geometry-solver version/hash
manufacturing-tolerance inputs
per-leg/per-joint/per-side predicted contact data
prerequisites
auxiliary moves
safe paths
validation status
```

The file format should fit existing project conventions. Do not introduce a new dependency solely to choose a prettier serialization format.

The compiler must not automatically rewrite the canonical URDF.

If mesh-predicted contacts materially disagree with the declared URDF limits, emit an explicit `MODEL_LIMIT_MISMATCH`-class result for human review.

---

## 7. q=0: corrected canonical interpretation

### Historical manual/visual zero

The original hand-positioned/photo-based home pose was useful as a commissioning seed. It was never a strong metrological definition of the true mathematical `q=0`.

The semantic model home remains the URDF home configuration:

```text
hip   q = 0
upper q = 0
lower q = 0
```

with the physical link orientation defined by the URDF/CAD, not by visual approximation.

### LF V25 evidence

LF V25 already demonstrated the stronger method in practice: it measured both mechanical contacts and derived a candidate affine model and q0 before the EEPROM freeze.

Final accepted LF evidence:

```text
M13 HIP   contacts 2535 / 1600 -> q0 candidate 2067
M12 UPPER contacts 1439 / 3443 -> q0 candidate 2040
M11 LOWER contacts 3093 / 1658 -> q0 candidate 2074
```

Only after this model-based derivation were EEPROM Position Offsets changed so those physical q=0 poses read approximately 2048.

Therefore the generalized project rule is:

> The final q=0 must be derived from model geometry and repeatable real contact evidence; the manual/visual home is only a seed and plausibility check.

### Future q=0 derivation

For each joint, use:

```text
mesh/model contact angle MIN/MAX
+
repeatable hardware contact tick MIN/MAX
+
known encoder direction
+
known or justified transmission/encoder scale
```

to derive the encoder coordinate corresponding to model `q=0`.

Stage this q0 first in RAM/software and verify the resulting physical/model home pose before any persistent EEPROM operation.

### Important scale distinction

Do **not** automatically treat a mismatch in physical endpoint span as proof that the ST3215 encoder-to-joint angular scale itself changed.

The generalized system must distinguish:

1. encoder resolution / known mechanical transmission ratio;
2. q0 offset;
3. geometry/end-stop mismatch;
4. an intentionally defined effective affine normalization.

LF V25 is immutable historical evidence and must not be rewritten retrospectively. However, before copying its free affine-scale behavior to RF/RH/LH, verify that a per-joint scale change represents a physically justified transmission calibration rather than merely forcing two imperfect endpoints onto rounded URDF limits.

At minimum, the new geometry/calibration report must show both:

- model-vs-hardware endpoint error using the nominal/physically justified encoder scale;
- any fitted affine transform separately.

A fitted affine model must not erase the raw discrepancy from the report.

This is essential because FK/IK joint coordinates are physical angles, not arbitrary normalized travel fractions.

### EEPROM policy

EEPROM `Position Offset` is not modified during Phase 1 or Phase 2.

A future EEPROM freeze may be performed only after:

- geometry profile PASS;
- repeatable hardware contacts;
- q0 derivation PASS;
- RAM/software home verification;
- explicit human authorization in the current session;
- transactional backup/readback/rollback procedure;
- verified global torque OFF at exit.

A future CAD/URDF/mechanical change may legitimately require a new calibration and a new approved persistent q0. Existing frozen values are not a universal law independent of geometry; they are validated artifacts for the configuration that produced them.

---

## 8. Contact detector is general and direction-independent

The contact methodology validated on LF must remain one shared detector for all legs.

Core V25 method:

```text
coarse scout       -> locate the region only
backoff
fine #1            -> measurement
backoff
fine #2            -> measurement
repeatability      -> fine #1 vs fine #2
```

Servo ID, encoder direction and physical side only determine how a model-space movement maps into unsigned ST3215 ticks.

They do not define different contact physics.

Permanent rule:

```text
ContactConfirmed -> STOP ADVANCING IMMEDIATELY
```

If a confirmed contact is inside the geometry-predicted band, it is a candidate endpoint and the fine/repeatability procedure continues.

If a confirmed contact is outside that band:

```text
stop pressure
controlled verified backoff
fail closed with diagnostics
upper cleanup
global torque OFF verified
```

Never continue pushing merely because the contact occurred earlier than an LF-derived span prediction.

`FreeMotion` and `ContactSuspected` are not equivalent to confirmed contact and may follow their existing bounded logic.

---

## 9. LF spans remain evidence, not RF/RH/LH coordinates

LF V25 measured spans are:

```text
HIP   = 935 ticks
UPPER = 2004 ticks
LOWER = 1435 ticks
```

They are valuable hardware benchmarks.

They must not be used as mandatory RF/RH/LH second-contact coordinates or as a reason to ignore a real early contact.

After RF/RH/LH are measured, report comparative spans across corresponding joints as diagnostic/manufacturing evidence.

---

## 10. Phase 2 — one generic NormaCore full-leg engine

Only after Phase 1 produces a reviewed geometry profile should `norma-core` be refactored.

Target:

```text
LegSessionStateMachine
+
LegCalibrationSpec
```

not separate LF/RF/RH/LH engines.

`LegCalibrationSpec` (or equivalent) may contain data such as:

- leg identity;
- hip/upper/lower motor IDs;
- encoder directions;
- first/second physical contact side/order per joint;
- prerequisite poses;
- auxiliary moves;
- model-derived corridors/guards;
- geometry-predicted contact bands;
- restore sequence.

The shared engine owns:

- preflight;
- initial recovery;
- transitions;
- low-energy RAM settings;
- coarse/backoff/fine/fine/repeatability;
- telemetry freshness;
- status/current/velocity/tracking safety;
- role/readback validation;
- cleanup;
- verified global torque OFF.

### LF as regression oracle

The generic refactor must preserve LF V25 behavior through direct regression tests.

Every existing LF-specific constant or function introduced during V36/V38/V41 must be classified before moving it:

```text
A. truly generic calibration behavior
B. geometry/profile data
C. global ST3215 hardware/safety parameter
D. historical LF-only evidence
```

Do not change a threshold merely to make another leg pass.

### Current RF patch

Do not continue wiring the existing duplicated `RfSessionStateMachine` into runtime before the generic architecture review. Preserve the patch as evidence; reuse validated ideas/data, not duplication for its own sake.

---

## 11. Phase 3 — hardware completion

Only after Phase 1 and Phase 2 offline gates pass:

```text
RF full leg hardware calibration
-> RH full leg hardware calibration
-> LH full leg hardware calibration
```

RF validates LEFT/RIGHT generalization.

RH is the critical FRONT/HIND generalization test because of the 20 mm hip-height difference.

If RF and RH both validate the same engine with data-driven geometry, LH should require primarily its profile data rather than another engine implementation.

For every leg:

- measure six endpoints;
- retain raw endpoint evidence;
- compare with mesh-predicted contact angles/bands;
- derive q0 candidate from model + contacts;
- report nominal-scale and fitted-affine diagnostics separately;
- stage/verify q0 in RAM/software;
- torque OFF verified;
- only then consider separately authorized EEPROM freeze.

---

## 12. Repeatable future recalibration workflow

The final MATDOG calibration system must support this workflow after a geometry revision:

```text
new CAD / printed geometry
-> new collision STL
-> new URDF
-> Geometry Compiler
-> 24 predicted contacts + paths + prerequisites/parking
-> offline collision/clearance PASS
-> generic V25-derived hardware engine
-> 24 measured contacts
-> model-vs-real report
-> q0 + physical joint mapping
-> approved persistent calibration profile
-> FK / IK / gait consume that profile
```

Pin each calibration profile to its geometry:

```text
robot-dog commit SHA
URDF SHA256
mesh manifest SHA256
geometry-profile SHA256
calibration software SHA
hardware evidence/log hashes
```

The hardware runner must reject a mismatched geometry profile rather than silently calibrating against stale geometry.

---

## 13. Repository responsibilities

### `MattRobotics/robot-dog`

Owns the robot-specific truth:

- CAD/URDF/collision meshes;
- joint semantics and mapping;
- geometry compiler;
- geometry profile;
- model-contact analysis;
- manufacturing tolerance assumptions;
- hardware calibration results/logs/profiles;
- project decisions and canonical handoffs.

Primary paths to inspect:

```text
03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf
03_CAD/URDF/matt_robodog_rev00/meshes/
06_Software/Matdog_Core/calibration/MATDOG_MECHANICAL_ENDSTOP_GEOMETRY_CHECKPOINT_2026-07-20.md
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml
REPOSITORY_VERIFICATION_INDEX.md
```

### `MattRobotics/norma-core`

Owns the current Station/ST3215 implementation truth:

- ST3215 runtime ownership and command path;
- mechanical contact detector implementation;
- calibration state machine;
- RAM write/GoalPosition gating;
- Station integration;
- transactional LF freeze implementation;
- runtime and unit tests.

Primary paths/refs to inspect:

```text
tools/matdog/README.md
software/drivers/st3215/src/auto_calibrate/matdog.rs
software/drivers/st3215/src/auto_calibrate/matdog_test.rs
software/drivers/st3215/src/bin/matdog_lf_freeze.rs
release/matdog-lf-calibrator-v25 @ f87dd1fbc7e8100d275c74f9af448642f3429680
```

Historical handoffs below remain audit evidence, but their development prescriptions are superseded where they conflict with this file:

```text
tools/matdog/MATDOG_RF_CALIBRATOR_CLAUDE_HANDOFF_2026-08-05.md
tools/matdog/CLAUDE_PROMPT_RF_CALIBRATOR_2026-08-05.md
tools/matdog/MATDOG_RF_CALIBRATOR_HANDOFF_STATE_2026-08-05.json
```

Also inspect, without modifying or deleting, the current local RF worktree:

```text
/home/matteo-manicardi/MATDOG/worktrees/norma-core-rf-calibrator
```

when Phase 2 begins.

---

## 14. Permanent constraints

- Respond to Matteo in Italian; preserve technical identifiers/commands in original form.
- Verify real repo/branch/commit/worktree state before modifications.
- Distinguish verified facts, historical evidence, assumptions and proposals.
- No merge into `main` without explicit authorization.
- No force-push or destructive Git operations.
- Do not delete branches, worktrees, logs or verification artifacts.
- Preserve the immutable LF V25 release.
- `ST3215 GoalPosition` remains unsigned `0..4095`; signed-wrap is forbidden.
- Station remains sole owner of the servo serial connection during motion.
- No physical movement without successful offline checks and explicit confirmation from Matteo in the current session.
- Verify global torque OFF on every hardware success/failure exit.
- No EEPROM Position Offset write unless explicitly authorized after the new q0/model gate.
- Do not introduce architecture/refactors/dependencies outside the agreed phase without explaining impact and obtaining approval.
- Do not certify hardware behavior from offline tests.

---

## 15. Immediate next task: Phase 1 only

The next autonomous Claude Code task is **Phase 1: Geometry Compiler / 24 endpoint geometric audit** in `robot-dog`.

It must NOT:

- orchestrate RF;
- modify `norma-core` runtime code;
- move hardware;
- open Station/serial;
- write EEPROM;
- commit/push/PR unless the prompt explicitly authorizes it;
- delete the current RF worktree or patch.

Its deliverables are offline geometry code/tests/profile/report only.

Phase 1 is complete only when a human can review:

1. exact 24 mesh-predicted endpoint contacts;
2. contact link pairs;
3. FRONT/HIND comparison;
4. prerequisite and parking necessity derived from path geometry;
5. safe transition/restore paths;
6. uncertainty/tolerance treatment;
7. machine-readable geometry profile pinned by hashes;
8. tests and real diff;
9. list of unresolved geometry/model ambiguities.

Only then proceed to Phase 2.
