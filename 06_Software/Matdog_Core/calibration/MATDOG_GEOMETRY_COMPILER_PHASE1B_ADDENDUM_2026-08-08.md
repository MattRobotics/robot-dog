# MATDOG — Geometry Compiler Phase 1B addendum

**Date:** 2026-08-08
**Status:** **CLOSED** — reviewed and merged. Validation complete (targeted 39/39, compiler 24/24,
final full suite 122/122). Merged to `main` via PR #16 (squash) as commit
`5b66044e225fcd921e44b98cc710f028da441a64` on 2026-08-09. Schema v4 is the current canonical
profile. Phase 2 is NEXT and NOT STARTED.
**Supersedes for endpoint metrology:** `MATDOG_GEOMETRY_COMPILER_PHASE1_COMPLETION_2026-08-07.md`
(schema v3). That record is **historical, not deleted** — its findings remain valid as a
description of what the v3 policy could see.

---

## 1. What Phase 1B corrects

Phase 1 v3 concluded **LF 6/6 = `MODEL_INCOMPLETE`**: no mesh finding corresponded to where
LF V25 hardware actually stopped, on any of the six joints. The natural reading at the time was
that the collision STLs were missing hardstop geometry.

That reading was wrong, and for two compounding reasons — one in the policy, one in the meshes.

### 1.1 Policy defect — blanket adjacent exclusion

v3 applied a single uniform rule:

```text
adjacent pair -> EXCLUDE
```

It treated a REVOLUTE hinge and a FIXED structural attachment as the same kind of thing. But the
designed mechanical hardstop physically lives **on the revolute parent/child pair**. Excluding
every adjacency therefore made the real endstop event **unobservable by construction**: no
amount of searching the remaining pairs could ever find it.

### 1.2 Mesh defect — motor pins modelled in contact

The adjacent pairs were also permanently `INTERSECTING` at every angle including q=0, which is
what motivated the blanket exclusion in the first place. The cause was not a design interference:
the assembly STLs represented the **motor centre pins** in nominal contact with the mating
screws/pulleys of the adjacent link. A permanent, angle-invariant overlap at the joint core made
the boolean verdict `INTERSECTING` everywhere, so no `SEPARATED -> INTERSECTING` transition could
ever be localized even if the pair had been included.

Both had to be fixed. Fixing either alone changes nothing.

---

## 2. Evidence base — GATE A and GATE B

Full diagnostic archives (code, CSV/JSON, witness patches, resource logs, SHA256SUMS) at:

```text
/home/matteo-manicardi/MATDOG/_archive/geometry-diagnostics/GATE_A_ADJACENT_BASELINE_2026-08-08
/home/matteo-manicardi/MATDOG/_archive/geometry-diagnostics/GATE_B_MOTORPIN_SUPPORTED_2026-08-08
```

### GATE A — adjacent-link raw baseline on the ORIGINAL geometry

Ran the three LF adjacent pairs through the real kernel with no mask and no production change.

- All three pairs `ALWAYS_INTERSECTING` across the full 1° sweep to the declared limit ±10°
  (112/196/166 samples) and at all 11 dense poses — measured along the whole path, not inferred
  from q=0.
- The persistent overlap is **99.4–100 % confined within 6 mm of the joint axis**, in discrete
  concentric shells at Ø≈3 mm and Ø≈5.5 mm, spanning ≈±17.5 mm axially. Concentric, cylindrical,
  shaft-length geometry.
- The link owning the dense concentric feature is `base_link` for HIP and `<leg>_upper_leg_link`
  for UPPER and LOWER — reproduced from the mesh alone, and matching the motor-pin ownership
  found independently in Solid Edge.
- Emergent far-field contact (radial reach up to 96 mm — real external structure) appears within
  one 1° step of the V25 hardware angle for `lf_upper_leg_min`, `lf_upper_leg_max` and
  `lf_lower_leg_min`, with zero far-field triangles one step earlier.

A SAT-based penetration-depth metric was trialled and **withdrawn as invalid**: a flat triangle
projects to a point on its own face normal, so min-overlap is identically 0 for any crossing
pair. No GATE A conclusion rests on it.

### GATE B — motor-pin clearance candidate meshes, verdict `SUPPORTED`

Five candidate STLs, corrected in CAD to give the motor pins ≈0.10–0.15 mm clearance.

#### Candidate frame history — what actually happened

Recorded precisely, because the correction path matters for provenance:

1. The intended **motor-pin geometry edit was made in CAD**.
2. The **first four upper-leg STL exports carried a common proper rigid frame error**: each
   `*_upper_leg_link.stl` was rotated 180° about Y through z = −45 mm
   (`x' = -x; y' = y; z' = -z - 90 mm`; determinant +1, so a rotation, not a reflection).
   `base_link.stl` was unaffected.
3. GATE B STEP 1 **diagnosed the error geometrically**, before any collision physics was run,
   by testing candidate re-orientations against the original surface.
4. The frame error was then corrected **directly in the binary STL vertices and normals**, by
   applying the determined rigid transform to the existing files.
5. **No second CAD export and no retessellation were performed for the frame correction.** The
   candidate **triangle count, triangle order and attribute bytes were preserved**; only the
   vertex/normal coordinates were rigidly transformed.
6. The corrected files were **reloaded from disk and verified geometrically and by SHA256**
   against the supplied manifest **before** any GATE B physics.

The retriangulation noted above therefore belongs to the original CAD export of the pin edit, not
to this frame correction.

Post-correction results:

| step | result |
|---|---|
| STEP 1 mesh sanity | **PASS 5/5** — AABB delta 0.000000 mm, extent ratio 1.0, identity beats every 180° re-orientation |
| STEP 2 q=0 gate | **3/3 SEPARATED** (was 3/3 INTERSECTING on the originals) |
| STEP 3 LF endpoints | **6/6** first contacts localized by the standard kernel, no mask |
| witness localization | 12/12 witnesses at radial 15.6–106.6 mm — external structure, not the pin core |
| STEP 4 historical | `base↔upper` −47.5000° and `upper↔foot` −97.9570° both reproduce exactly |

---

## 3. Production changes made in Phase 1B

### 3.1 Five collision meshes replaced

Canonical filenames, `rev00` unchanged, URDF byte-identical (`<visual>`/`<collision>` paths
untouched — they already point at these filenames).

| mesh | old SHA256 | new SHA256 |
|---|---|---|
| `base_link.stl` | `7b149643a6a71ae8ac37780085c021c5bff0c4491ebd42f1393150cb987e9e6e` | `644a83e98fd116f3fc8e5d8792ca2b60b0bdb09bcafa4fc49140d641079e4b6b` |
| `lf_upper_leg_link.stl` | `dfaf754764ded80743b00ae5d1dde208301ab5a3cebc9e29a38d8513dbe4cd93` | `3c484b110a622274d1f8446b30329b8e16a0ff542d25da2a39334d41e8f4f169` |
| `lh_upper_leg_link.stl` | `dfaf754764ded80743b00ae5d1dde208301ab5a3cebc9e29a38d8513dbe4cd93` | `3c484b110a622274d1f8446b30329b8e16a0ff542d25da2a39334d41e8f4f169` |
| `rf_upper_leg_link.stl` | `3c2508690ac89d006d25cabf3917ce52dc2c0b09e4ec5e051da8f71ff1a7e760` | `08fab5e3229280f21a52c6cfbfdab32f1e8345aae6e10648360ae30fe1e06b16` |
| `rh_upper_leg_link.stl` | `3c2508690ac89d006d25cabf3917ce52dc2c0b09e4ec5e051da8f71ff1a7e760` | `08fab5e3229280f21a52c6cfbfdab32f1e8345aae6e10648360ae30fe1e06b16` |

URDF SHA256 before and after: `5e28da3dba10fd3f2ea6ebf6f5d6271157bda0b12b82d92aedbe3031643089ef`.

The exports were retriangulated (`base_link` 105 330 → 153 080 triangles;
`*_upper_leg_link` 39 450 → 29 238), so triangle IDs, counts and vertex correspondence are **not**
valid integrity criteria. Integrity was established geometrically.

### 3.2 Joint-aware adjacency, derived from the URDF

`adjacent pair -> EXCLUDE` is replaced by four semantic classes, built from URDF
`<parent>`/`<child>`/`type` rather than a hard-coded chain:

```text
CLASS A  ACTIVE REVOLUTE PARENT-CHILD  -> the one primary endpoint-contact pair for this joint
CLASS B  NON-ACTIVE REVOLUTE ADJACENT  -> path-safety obstruction candidate (boolean only)
CLASS C  FIXED ADJACENT                -> structural attachment, excluded entirely
CLASS D  NON-ADJACENT                  -> pre-existing policy unchanged
```

12 revolute pairs, 4 fixed pairs (`<leg>_lower_leg_link ↔ <leg>_foot_link`, `foot_joint` is fixed).

### 3.3 Endstop metrology separated from path safety

```text
ENDSTOP METROLOGY = active revolute parent-child pair
    HIP   -> base_link            <-> <leg>_hip_link
    UPPER -> <leg>_hip_link       <-> <leg>_upper_leg_link
    LOWER -> <leg>_upper_leg_link <-> <leg>_lower_leg_link

PATH SAFETY = all other relevant collision pairs
```

The active pair is excluded from its own path-obstruction set — its contact is the desired
result, not an obstruction. The historical regression is preserved and tested: during a LOWER
probe, a `hip ↔ foot` intersection is a `PATH_COLLISION_BEFORE_ENDPOINT`, never the LOWER endstop.
v3 recognised only cross-leg obstructions; same-leg ones are now reported too.

`same_leg_non_adjacent_pairs()` is retained as a **historical/diagnostic helper only** and is no
longer used to identify a joint limit.

### 3.4 Boolean collision separated from clearance measurement

Two questions, two margins:

- `BOOLEAN_COLLISION_MARGIN_M = 0.0` for intersection tests. Not a weakening: intersecting
  triangles necessarily have overlapping AABBs and always share a grid cell at margin 0.
- `DEFAULT_NARROW_PHASE_MARGIN_M = 0.001` retained for clearance, preserving `EXACT` /
  `LOWER_BOUND` / `UNRESOLVED_FOR_THRESHOLD` semantics unchanged.

Lowering the shared default to 0 would have destroyed the clearance path's ability to report an
exact figure, so the two were split instead.

### 3.5 Clearance gate scoped to NON_ADJACENT

A revolute hinge's healthy resting state is sub-millimetre separation, so the generic 3 mm gate
does not apply to it. Revolute adjacent pairs contribute to path safety through the **boolean**
test only. The non-adjacent clearance policy is untouched.

### 3.6 Supporting kernel/scene changes

| change | reason |
|---|---|
| candidate cap 500 000 → 2 000 000 | measured: `base_link ↔ lf_hip_link` legitimately needs 545 705 candidates at margin 0. Finer grid cells are unavailable — the meshes' largest triangles (radius up to 138 mm) blow through `_MAX_GRID_CELLS_PER_TRIANGLE`. |
| AABB pre-filter before the 11-axis SAT | pure optimisation, cannot change a verdict; halved the dominant per-sample cost |
| pair-result cache keyed on `_pair_relevant_joints` | a pair depends only on the joints BETWEEN its two links, so shared prefix joints cancel; `hip↔upper` is frozen during a hip sweep |
| sensitivity reports NOT APPLICABLE for revolute adjacent contacts | the clearance-gradient method assumes the pair's minimum clearance is the contact feature; on a revolute pair the minimum is the angle-invariant pin fit, so the gradient is meaningless |

---

## 4. Profile schema v4

`matdog.calibration_geometry_profile.v4`. v1/v2/v3 artifacts are preserved unchanged.

New per-endpoint fields: `active_revolute_pair`, `pair_class`, `endpoint_evidence_class`.
New top-level `pair_policy` block stating the rules machine-readably.

`endpoint_evidence_class` exists specifically so a modeled contact is **never silently promoted to
a hardware-confirmed one**:

| class | meaning |
|---|---|
| `HARDWARE_CONFIRMED_CONTACT` | mesh contact agrees with a real hardware oracle (LF V25 only) |
| `HARDWARE_CONTRADICTED` | a hardware oracle exists and disagrees |
| `GEOMETRIC_ENDPOINT_CANDIDATE` | contact localized, no hardware oracle for this leg (RF/RH/LH) — a model prediction awaiting hardware validation |
| `PATH_LIMITED` | a path obstruction precedes the articulation's own contact |
| `NO_MODELED_CONTACT` | nothing found in the analysis envelope |

---

## 5. 24-endpoint run — `2026-08-08_231600`, schema v4

Exit 0, wall clock **59:27**, peak RSS **541 MB**, 24/24 endpoints processed.
`content_sha256` = `928ff09616a1ad80df029a99afe2061e41a0bedec6bfdc9b3f80f74af4867828`.

Every endpoint resolved a first contact **on its own active revolute pair**. Under v3 the
comparable run produced LF 6/6 `MODEL_INCOMPLETE` and 14 `NO_MODELED_ENDSTOP`.

| outcome | count |
|---|---:|
| `MODELED_ENDSTOP_CONTACT` | 21 |
| `MODEL_INCOMPLETE` | 3 (LF only, all hardware-contradicted) |
| `NO_MODELED_ENDSTOP` | 0 |
| path collisions detected | 6 |

Evidence classes: 3 `HARDWARE_CONFIRMED_CONTACT`, 3 `HARDWARE_CONTRADICTED`,
18 `GEOMETRIC_ENDPOINT_CANDIDATE`.

### LF vs V25 hardware

| endpoint | declared | mesh contact | V25 hardware | mesh − hw | verdict |
|---|---:|---:|---:|---:|---|
| `lf_upper_leg_min` | −52.500° | −52.0391° | −53.5254° | **+1.486°** | AGREES |
| `lf_upper_leg_max` | +122.500° | +121.8750° | +122.6074° | **−0.732°** | AGREES |
| `lf_lower_leg_min` | −92.000° | −92.0703° | −91.8457° | **−0.225°** | AGREES |
| `lf_hip_min` | −45.000° | −46.0117° | −42.8027° | **−3.209°** | DISAGREES |
| `lf_hip_max` | +45.000° | +45.2305° | +39.3750° | **+5.856°** | DISAGREES |
| `lf_lower_leg_max` | +37.500° | +38.1797° | +34.2773° | **+3.902°** | DISAGREES |

3/6 within the 2° agreement band. The three that disagree are carried forward as an explicit
UNKNOWN (§6.1) and are **not** reinterpreted as agreement.

### Path safety — historical regressions preserved

The two v3 non-adjacent findings reproduce exactly, and are now correctly classified as PATH
events rather than endpoints:

| endpoint | path collision | angle | own endstop |
|---|---|---:|---:|
| `lf_hip_min` | `base_link ↔ lf_upper_leg_link` | **−47.5000°** | −46.0117° |
| `lf_lower_leg_min` | `lf_foot_link ↔ lf_upper_leg_link` | **−97.9570°** | −92.0703° |

Both mirror onto their counterpart legs (`rf_hip_max` +47.5000°, `rf`/`rh_lower_leg_min`
−98.0039°, `lh_lower_leg_min` −97.9570°). In every case the joint's own articulation contact
occurs at a smaller |q| than the path obstruction, so the endpoint is the articulation contact
and the obstruction is reported separately — exactly the separation Phase 1B was built to make.

### Mirror and FRONT/HIND

| comparison | upper_leg | lower_leg | hip |
|---|---:|---:|---:|
| LF vs RF | 0.000° | 0.004° | 0.781° (min↔max mirror mapping) |
| RH vs LH | 0.000° | 0.004° | 0.855° (min↔max mirror mapping) |

Geometry is mirror-consistent to within 0.004° on upper/lower. The remaining status differences
between LF and RF/RH/LH are **not** geometric: they come from LF being the only leg with a
hardware oracle, so an identical geometric contact is classified `MODEL_INCOMPLETE` on LF and
`MODELED_ENDSTOP_CONTACT` elsewhere. FRONT/HIND hip Z differs by 20 mm by design
(0.0465 m vs 0.0265 m), which does not change detector physics.

### Parking — `REVALIDATED / UNCHANGED FROM v3`

Deliberately **not** described as PASS: all four legs report `passed=False`.

Bit-identical to the v3 run — same `passed=False` on all four legs, same minimum clearances
(LF/RF 0.000117 m, RH/LH 0.001 m), same `auxiliary_parking_required` (LF/RF true, RH/LH false).
The Phase 1B path-planner changes altered **no** parking verdict. The gate was already failing in
v3 on non-adjacent clearance and remains a pre-existing open engineering item, not a Phase 1B
regression and not something Phase 1B claims to have fixed.

### Validation

| stage | result |
|---|---|
| targeted `phase1b_policy` suite (20 required areas) | 39/39 OK, 14:15, 330 MB |
| 24-endpoint compiler | exit 0, 59:27, 541 MB, 24/24 |
| **final full geometry suite** | **122 tests, OK, exit 0, 43:21, peak RSS 842 MB** |

The full suite was run twice. The first run failed with 1 failure and 24 errors, all confined to
`tests/test_matdog_geometry_contact_search.py`: 24 were stale call sites left by the internal
rename `same_leg_found` -> `active_pair_found`, and 1 was a v3 expectation incompatible with
Phase 1B path-safety semantics (same-leg obstructions are now surfaced, where v3 reported only
cross-leg ones). Only that test file was changed; no production module was touched afterwards,
which the profile's `source_file_sha256` manifest independently confirms. The second run,
explicitly authorised, is the result recorded above.

## 6. Open UNKNOWNs

### 6.1 Residual mesh-vs-hardware discrepancy on three LF endpoints

Explicitly **not** resolved by Phase 1B, and explicitly **not** to be reinterpreted as agreement:

| endpoint | mesh first adjacent contact | V25 hardware | delta |
|---|---:|---:|---:|
| `lf_hip_min` | −46.0117° | −42.8027° | **−3.2090°** |
| `lf_hip_max` | +45.2305° | +39.3750° | **+5.8555°** |
| `lf_lower_leg_max` | +38.1797° | +34.2773° | **+3.9023°** |

These do not block Phase 1B and do not invalidate GATE B: the mechanism (pin removal makes the
pairs separable; the standard kernel then localizes real external contacts) is confirmed
independently of how well the angles agree. But the model still stops later than the hardware did
on these three, and the reason is unidentified. Candidate explanations not yet tested: a
servo/bracket internal limit not represented in the STL at all; assembly stack-up; or a hardstop
surface whose CAD representation differs from the printed part.

The same three endpoints are exactly those that showed **no** far-field emergent contact in
GATE A — the two findings are consistent, which is itself evidence that something real and
common is missing there.

### 6.2 Other open items

- **Micro-clearance is not a physical constant.** GATE B's q=0 figures (HIP 0.0372 mm,
  UPPER 0.0026 mm, LOWER 0.0101 mm) characterise the current mesh revision, not an assembly
  clearance. Their limit is CAD/tessellation/mechanical significance, not numerical precision:
  they sit far below per-part print tolerance (±0.15 mm), below the observed
  CAD/tessellation/model surface differences of O(0.1 mm) between the original and corrected
  meshes, and below unmodelled assembly stack-up (bushings, screws, backlash). The regression
  requirement is therefore semantic — **adjacent revolute at q=0 is SEPARATED** — and no
  clearance value is asserted, preserved, or depended on by any policy. The endpoint search
  emits no clearance figure for an adjacent contact at all.
- **No sensitivity estimate for adjacent endpoints.** The clearance-gradient method does not
  apply (§3.6). A hardstop-surface-local sensitivity method is needed.
- **RF/RH/LH have no hardware oracle.** Their endpoints are `GEOMETRIC_ENDPOINT_CANDIDATE`, not
  measured hardstops.
- **Assembly tolerance** (bushings, screws, servo backlash) remains unmodelled.
- **Physical margin at q=0 is small.** Geometrically separated, but little headroom against
  print/assembly tolerance; a physical build could still touch at the pins.

---

## 7. Isolation

No hardware, Station, serial, servo, torque or EEPROM. `norma-core` untouched. RF worktree
untouched. No remote Git operation, no commit, push, PR or merge. Phase 2 NOT started.
