# Geometry Compiler artifacts — index

All runs below are offline-only Geometry Compiler outputs. Historical v1-v4
runs came from `matdog_geometry_compiler.py`; additive V5 runs come from the
model-first V5 modules in `06_Software/Matdog_Core/calibration/`. None were
deleted; each is kept for audit trail per repository hygiene policy.

| Timestamp | Schema | Status | Primary JSON file SHA256 |
|---|---|---|---|
| `2026-08-07_155742` | v1 | **SUPERSEDED** (historical / pre-reconciliation) | `b327aa21fa3ec29b5e1762831883e85d448662134b4ff0892952920d43fbb322` |
| `2026-08-07_193932` | v2 | **SUPERSEDED** (intermediate reconciliation) | `d3db673832bba3803082231717d578c5e15a70d47ce11ba59beded76777fc7d3` |
| `2026-08-07_204107` | v3 | **SUPERSEDED for endpoint metrology** by Phase 1B; retained as historical Phase-1 canonical | `6b438285f7145b2cb4f9fc11f1e9b2342dfbb7360d83976ad9a25cd590a0b7c5` |
| `2026-08-08_231600` | v4 | **HISTORICAL PHASE1B CANONICAL** (closed; superseded by pure V5) | `c2e980bfc49e80b957fc0a326b1ac98724fc1d4ac16121da19fcf6ab2be3eff6` |
| `2026-08-10_164419` | v4 | **G4 REFERENCE** (approved new collision geometry + unchanged Phase1B algorithm; not V5 canonical) | `f1b059a58c51508345ec583cc421cf9ca66ec8e9ac5547649c05bbf755e5c5fa` |
| `2026-08-10_185433` | v5 | **G7 FROZEN PURE-GEOMETRY REFERENCE** | `1a004561ac3213749f28e0da262f211ff6f7021a0230b1415fbe24f46fdf2b01` |
| `2026-08-10_185433` | v5 + parking-v1 | **G9 FROZEN PATH/PARKING REFERENCE** | `418b6dca07475ff0bb2ae131cbf35844eced30c6cb11b9f80d196aa8c2b883de` |
| `2026-08-11_072224 C` | v5 + parking-v2 | **SUPERSEDED PRE-AUDIT V5 CANDIDATE EVIDENCE** | `06f5d1e7f46db1cf0f6084ca75f36a27d76c18e25d26c8addc53a4967e7b41d1` |
| `2026-08-11_072224 D` | v5 + parking-v2 | **SUPERSEDED PRE-AUDIT V5 CANDIDATE EVIDENCE** | `1c112a1af7af484cebc77736155118a924f327364a72e6bdb590980c70786dea` |
| `2026-08-11_131818 C` | v5 + parking-v2 | **CORRECTED WORKERS=1 DETERMINISM REFERENCE** | `23ca4709385019466cfb5782a7e0e6f2053fd0d632b67db6c94bd00c05f5e147` |
| `2026-08-11_131818 D` | v5 + parking-v2 | **CURRENT PURE-GEOMETRY DEPLOYMENT CANONICAL** | `dd8cb42c3b916d067f97a321c5ffcdfb013dde1f2f2a6ba71f73becff360dc0f` |
| `2026-08-11_132758` | G12 consumers | **CURRENT G12 RECONCILER + SAFETY POLICY** | `d7fa04e2b8cde6b1049d4c34c5ba15fa1febde8b2fbf9fd8f14853034b98cc9a` |

Nothing here is ever deleted. A run marked SUPERSEDED remains an accurate record of what the
policy of its day could observe; it is simply no longer the source of truth.

## `2026-08-07_155742` — SUPERSEDED

First Phase 1 run. Same-leg mesh contact found anywhere in the search
envelope was treated as the designed endpoint; a cross-leg path
obstruction blocking the search could be mis-tagged as if it were the
probed joint's own limit (`UNINTENDED_COLLISION_BEFORE_ENDPOINT` result
kind, since removed). No hardware cross-check.

## `2026-08-07_193932` — SUPERSEDED

First reconciliation pass against LF V25 hardware evidence (schema v2).
Fixed the parking-planner seed-acceptance bug and split the
ENDSTOP_CONTACT_POLICY (same-leg) search from the PATH_SELF_COLLISION_POLICY
(full pair set, cross-leg included) search. Introduced
`contact_model_status`. Bug (corrected in v3): `MODEL_INCOMPLETE` for a
hardware-oracle endpoint was decided from the HARDWARE-vs-URDF gap, not
the MESH-vs-HARDWARE gap -- the wrong comparison for that decision. Also
had a sign error in 3 of 6 LF hardware-contact-angle derivations
(hip_min, upper_leg_min, lower_leg_min).

## `2026-08-07_204107` — HISTORICAL PHASE-1 CANONICAL, SUPERSEDED for endpoint metrology (schema v3)

`contact_model_status` for a hardware-oracle endpoint (LF only) is now
driven by MESH-vs-HARDWARE agreement; HARDWARE-vs-URDF agreement is kept
as a separate, informational `hardware_vs_urdf_status` field, never
conflated with the decision. Hardware contact angles stored as the
actual angle (not a pre-computed delta), removing the v2 sign-error
class structurally.

Result: **LF 6/6 = MODEL_INCOMPLETE** (no mesh finding, or absence of
one, corresponds to where LF V25 hardware actually stopped for any of
the six joints). 4 `MODEL_LIMIT_MISMATCH` (RF/RH/LH, no hardware oracle,
never auto-promoted to `MODEL_INCOMPLETE`), 14 `NO_MODELED_ENDSTOP`, 0
`PATH_COLLISION_BEFORE_ENDPOINT`, 0 `UNINTENDED_SELF_COLLISION`.

Full record: `06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_PHASE1_COMPLETION_2026-08-07.md`.

Status: `PASS_GEOMETRY_COMPILER_COMPLETE_WITH_EXPLICIT_MODEL_GAPS`.

**Superseded for endpoint metrology by Phase 1B (2026-08-08).** The v3 LF 6/6
`MODEL_INCOMPLETE` verdict did not establish that hardstop geometry was absent
from the STL. Phase 1B showed that the blanket adjacent-pair exclusion made the
revolute hardstop unobservable, while the original motor-pin representation kept
adjacent pairs permanently INTERSECTING. See the Phase 1B entry below. This run
is retained unchanged as historical evidence.

## `2026-08-08_231600` — HISTORICAL PHASE1B CANONICAL (schema v4)

First run with joint-aware adjacency and the corrected motor-pin collision meshes.

Policy in force:

```text
parent-child REVOLUTE -> INCLUDE in collision analysis        (12 pairs)
parent-child FIXED    -> structural attachment -> EXCLUDE      (4 pairs)

ENDSTOP METROLOGY = active revolute parent-child pair
PATH SAFETY       = all other relevant collision pairs
```

Schema v4 adds `active_revolute_pair`, `pair_class` and `endpoint_evidence_class`
per endpoint, plus a top-level `pair_policy` block. `endpoint_evidence_class`
exists so a modeled adjacent contact is never silently promoted to a
hardware-confirmed mechanical contact: LF has a V25 hardware oracle, RF/RH/LH
carry `GEOMETRIC_ENDPOINT_CANDIDATE` pending their own hardware validation.

Full record:
`06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_PHASE1B_ADDENDUM_2026-08-08.md`.

Status: `PASS_PHASE1B_24_OF_24_ADJACENT_ENDPOINTS_WITH_EXPLICIT_LF_HARDWARE_GAPS`.
Phase 1B is closed; V5 supersedes it as the pure-geometry architecture while
preserving this artifact unchanged.

## `2026-08-10_164419` — G4 NEW-GEOMETRY REFERENCE

Reference replay frozen before the Geometry Compiler V5 refactor. It uses the
approved 17 dedicated collision meshes (119,696 triangles) with the unchanged
validated Phase1B algorithm, one worker and a 6 GiB cgroup envelope. Historical
v4 endpoint equality is deliberately not an acceptance target.

Result: 24/24 mesh contacts found; 20 `MODELED_ENDSTOP_CONTACT`, 2
`MODEL_INCOMPLETE`, 2 `PATH_COLLISION_BEFORE_ENDPOINT`. Wall time 13:29.33,
peak RSS 462,176 KiB, swap 0. Profile content SHA256:
`4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61`.

Full G0-G4 record:
`2026-08-10_164419_MATDOG_NEW_GEOMETRY_G0_G4_BASELINE.md`.

## `2026-08-10_185433` — G7/G9 FROZEN V5 REFERENCES

G7 is the first accepted pure-geometry V5 replay of exactly the G4 contexts.
Its semantic SHA256 is
`d4180810252d43e4cce349e638ef499f55ebb41e057b9fd9d9ced68e206a1ecd`;
the G4/G7 oracle is PASS for all 24 contacts and six path events.

G9 consumes that saved G7 profile and adds topology-driven raw path/parking
plans without rerunning endpoint search. The frozen parking-v1 semantic SHA256
is `3e09aec7ca355f79f43abe803b672958b9d992b1aab9c27b5cc7cdff571b9757`;
the derived-profile semantic SHA256 is
`a37c4e97c47970362e7758edd56c72170148ef4addee97fb330b0f03c0090e07`.
Both remain immutable gate history.

Hardware reconciliation and the historical 3 mm policy are separate consumers
under the same timestamp. They are not embedded in pure geometry.

## `2026-08-11_072224` — SUPERSEDED PRE-AUDIT V5 CANDIDATE EVIDENCE

Retained unchanged for provenance. **Not canonical.** An independent
adversarial audit of PR #19 found that the integrated runner required the
frozen G4 profile and reused one G4-context replay as both replay evidence and
the canonical "pure geometry" endpoint profile. The published canonical
artifact therefore carried the legacy `30 / 50 / 90` degree prerequisite
context on 20 of 24 endpoints, and its `path_obstruction` layer contradicted
the pure `q=0` parking results in the same combined profile on four endpoints.

These semantic hashes are **SUPERSEDED** and must not be cited as canonical:

```text
SUPERSEDED endpoint semantic SHA256:
cad2f194c49d063b5a09ae4602b9acf61a701de48791e5d1aae04f1439db1211

SUPERSEDED parking-v2 semantic SHA256:
3cda03c2c02ba5fbe6def7821ce72d4aa9e4ca66e7ca8655a2b8f0e92dde297c

SUPERSEDED combined semantic SHA256:
e99e2b65ea8d032f94b5d1aa815432a292c1771766e556fc7115dc7a1f5de73e
```

Its four narrative reports (`*_FINAL_VALIDATION_REPORT.md`,
`*_DETERMINISM_REPORT.md`, `*_ABCD_PERFORMANCE_REPORT.md`,
`*_TEST_SUMMARY.md`) and its two `*_G12_*` consumer artifacts are likewise
superseded. Each narrative report carries a banner pointing to its corrected
replacement. Nothing in this bundle was overwritten or deleted.

## `2026-08-11_131818` — CORRECTED FINAL V5 C/D (CANONICAL)

The corrected integrated pipeline was run once with one worker (C) and once
with four spawned processes (D). The canonical endpoint profile is now
context-free and the frozen-G4 replay is a separate, explicitly non-canonical
execution. Endpoint, parking-v2 and combined semantic payloads, the replay
oracle payload, inputs and all three source manifests are exactly equal
between C and D. D is the canonical deployment profile.

```text
endpoint semantic SHA256:
de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e

parking-v2 semantic SHA256:
67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139

combined semantic SHA256:
0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51

C run manifest SHA256: b86b5d35678df4510989ea49fb5d42acaea2e4838226d7017456399dfceb0d81
D run manifest SHA256: 0db86e633599f63a769dbba75db3a54c1e6470e128c007eb24c465f92a428b17
```

Result: 24/24 canonical endpoint search contexts empty; 24/24 geometric
contacts; six canonical direct-target path obstructions and 18 collision-free
direct paths; endpoint/planner path consistency 24/24 with a maximum
precise/refined delta of `3.6703973194107675e-13 rad`; 18 `NOT_NEEDED` and six
`FEASIBLE_1DOF_PLAN_FOUND` parking outcomes with 24/24 complete geometric
sequences and 94 evaluated 1-DOF candidates. D wall time is 1,691.92 s; the
cgroup process-tree peak is 1,798,238,208 bytes, swap is zero, and the 6 GiB
resource gate passes.

The canonical path domain is `q=0` to the geometric target. An obstruction
lying beyond the target is not a canonical path-obstruction result; the
historical full-envelope view lives only in the non-canonical replay artifacts
(`*_G4_G7_ORACLE.*`, `artifact_role: NONCANONICAL_G4_REPLAY_ORACLE`,
`canonical_profile_eligible: false`).

Start with:

- `2026-08-11_132758_MATDOG_GEOMETRY_V5_FINAL_VALIDATION_REPORT.md`;
- `2026-08-11_132758_MATDOG_GEOMETRY_V5_DETERMINISM_REPORT.md`;
- `2026-08-11_132758_MATDOG_GEOMETRY_V5_ABCD_PERFORMANCE_REPORT.md`;
- `2026-08-11_132758_MATDOG_GEOMETRY_V5_TEST_SUMMARY.md`.

## `2026-08-11_132758` — CURRENT G12 CONSUMERS

`*_G12_FINAL_LF_HARDWARE_RECONCILIATION.*` consumes the corrected combined
profile `0a772234…` and reports 3 AGREES, 3 DISAGREES, 18 geometry-only
endpoints and `geometry_modified: false`.

`*_G12_FINAL_EXTERNAL_SAFETY_POLICY.*` consumes the corrected parking artifact
`67c58430…` at the unchanged external 3 mm threshold and reports 16 PASS,
0 FAIL, 8 UNRESOLVED and zero motion authorizations.

The earlier `2026-08-11_072224_*_G12_*` artifacts consume superseded hashes and
are retained as superseded evidence only.
