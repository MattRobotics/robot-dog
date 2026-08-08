# Geometry Compiler artifacts — index

All runs below are offline-only outputs of
`06_Software/Matdog_Core/calibration/matdog_geometry_compiler.py`. None
were deleted; each is kept for audit trail per repository hygiene policy.

| Timestamp | Schema | Status | sha256 (PROFILE.json) |
|---|---|---|---|
| `2026-08-07_155742` | v1 | **SUPERSEDED** (historical / pre-reconciliation) | `b327aa21fa3ec29b5e1762831883e85d448662134b4ff0892952920d43fbb322` |
| `2026-08-07_193932` | v2 | **SUPERSEDED** (intermediate reconciliation) | `d3db673832bba3803082231717d578c5e15a70d47ce11ba59beded76777fc7d3` |
| `2026-08-07_204107` | v3 | **FINAL CANONICAL PHASE-1** | `6b438285f7145b2cb4f9fc11f1e9b2342dfbb7360d83976ad9a25cd590a0b7c5` |

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

## `2026-08-07_204107` — FINAL CANONICAL PHASE-1 (schema v3)

`contact_model_status` for a hardware-oracle endpoint (LF only) is now
driven by MESH-vs-HARDWARE agreement; HARDWARE-vs-URDF agreement is kept
as a separate, informational `hardware_vs_urdf_status` field, never
conflated with the decision. Hardware contact angles are stored as the
actual angle rather than a pre-computed delta, removing the v2 sign-error
class structurally.

Result: **LF 6/6 = MODEL_INCOMPLETE** because no mesh finding corresponds
to where LF V25 hardware actually stopped for any of the six endpoints.
The remaining model-only result is 4 `MODEL_LIMIT_MISMATCH`, 14
`NO_MODELED_ENDSTOP`, 0 `PATH_COLLISION_BEFORE_ENDPOINT`, 0
`UNINTENDED_SELF_COLLISION`.

Full record:

`06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_PHASE1_COMPLETION_2026-08-07.md`

Status:

`PASS_GEOMETRY_COMPILER_COMPLETE_WITH_EXPLICIT_MODEL_GAPS`

---

## Search-envelope interpretation — canonical clarification 2026-08-08

The Geometry Compiler does **not** clamp FK to URDF limits. `RobotScene`
uses `enforce_limits=False`, so collision geometry can be evaluated beyond
the `<limit>` values.

Phase 1 nevertheless used a deliberately bounded outer search margin:

```text
DEFAULT_ENVELOPE_MARGIN_RAD = 10 deg
```

The search therefore reaches:

```text
MAX: declared URDF maximum + 10 deg
MIN: declared URDF minimum - 10 deg
```

This is directly demonstrated by v3 contacts beyond the old limits, e.g.
LF hip MIN `-47.500 deg` vs `-45 deg`, LF lower MIN `-97.957 deg` vs
`-92 deg`, RF hip MAX `+47.500 deg` vs `+45 deg`, and RF/RH lower MIN at
about `-98.004 deg` vs `-92 deg`.

### Exact meaning of `NO_MODELED_ENDSTOP`

In schema v3:

```text
NO_MODELED_ENDSTOP
=
no relevant same-leg mesh contact found inside the documented bounded
analysis envelope
```

It does **not** mean that no STL collision could ever occur at a much
larger angle.

The full clarification and the recommended extended pre-Phase-2 model
sanity check are recorded in:

`06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_ENVELOPE_CLARIFICATION_2026-08-08.md`

### LF remains valid

This bounded-envelope clarification does not change LF 6/6
`MODEL_INCOMPLETE`: LF has real V25 hardware contact angles. A mesh
collision that is absent at the hardware stop or appears several degrees
after hardware has already stopped does not represent the same endpoint.

For RF/RH/LH, which do not yet have equivalent hardware oracles,
`NO_MODELED_ENDSTOP` must always be read with the envelope qualifier.
