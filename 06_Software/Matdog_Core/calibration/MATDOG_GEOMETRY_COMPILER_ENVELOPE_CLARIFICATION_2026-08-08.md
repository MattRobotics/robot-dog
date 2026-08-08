# MATDOG Geometry Compiler — Analysis-envelope clarification

**Date:** 2026-08-08  
**Status:** canonical clarification after Phase 1 closeout  
**Scope:** documentation only; no hardware, no Station, no serial, no EEPROM, no URDF modification.

## Purpose

Phase 1 correctly implemented the canonical requirement to search for mesh contact **around and beyond** the URDF-declared limit instead of assuming the URDF number itself was the physical contact.

After closeout, a human review raised a precise interpretation question:

> If an endpoint reports no mesh contact, did the Geometry Compiler really move past the old URDF limit, or did the URDF limit itself prevent the mesh from ever being tested farther out?

This document records the verified answer and the resulting interpretation rule.

---

## 1. The Geometry Compiler is not clamped by URDF limits

`RobotScene.link_transform()` calls the shared FK implementation with:

```python
enforce_limits=False
```

Therefore arbitrary finite probe angles can be evaluated even when they lie outside the `<limit lower=... upper=...>` values in the canonical URDF.

The URDF limits are read as **declared reference values**, not used as hard clamps during Phase 1 collision evaluation.

---

## 2. Phase 1 nevertheless used a bounded search envelope

The per-endpoint contact search defines:

```python
DEFAULT_ENVELOPE_MARGIN_RAD = 0.17453292519943295
```

which is exactly:

```text
10 degrees
```

For a MAX endpoint:

```text
envelope_end = urdf_declared_max + 10 deg
```

For a MIN endpoint:

```text
envelope_end = urdf_declared_min - 10 deg
```

The coarse scout starts at q=0 and advances toward that envelope end.

Thus the actual Phase 1 question was:

> Is there a relevant first mesh collision/contact between q=0 and a point 10 degrees beyond the URDF-declared limit in the searched direction?

It was **not**:

> Does a mesh collision exist at any arbitrarily large joint angle?

---

## 3. Direct proof that Phase 1 crossed URDF limits

The final v3 report contains multiple mesh contacts beyond the declared URDF limit.

Examples:

| Endpoint | URDF declared | Mesh contact | Beyond declared by |
|---|---:|---:|---:|
| LF hip MIN | -45.000° | -47.500° | 2.500° |
| LF lower MIN | -92.000° | -97.957° | 5.957° |
| RF hip MAX | +45.000° | +47.500° | 2.500° |
| RF lower MIN | -92.000° | -98.004° | 6.004° |
| RH lower MIN | -92.000° | -98.004° | 6.004° |
| LH lower MIN | -92.000° | -97.957° | 5.957° |

Therefore the Geometry Compiler demonstrably did not stop at the old URDF limit.

---

## 4. Exact meaning of `NO_MODELED_ENDSTOP`

For Phase 1 schema v3, the strict interpretation is:

> `NO_MODELED_ENDSTOP` = no relevant same-leg mesh contact was found inside the configured bounded analysis envelope.

Because the envelope margin was 10°, this currently means:

> no relevant same-leg mesh contact was found between q=0 and 10° beyond the declared URDF limit in that direction.

It must **not** be paraphrased as:

> the STL meshes never collide at any larger angle.

That stronger claim was not tested.

The final report itself records the no-contact envelopes explicitly, for example:

```text
hip MAX:       +35° .. +55° around a +45° declared limit
hip MIN:       -55° .. -35° around a -45° declared limit
upper MAX:     +112.5° .. +132.5° around a +122.5° declared limit
upper MIN:     -62.5° .. -42.5° around a -52.5° declared limit
lower MAX:     +27.5° .. +47.5° around a +37.5° declared limit
```

The numerical scout itself starts from q=0 and moves through the relevant direction to the outer envelope boundary; the report prints the 20° window centered on the declared limit for audit readability.

---

## 5. Why LF 6/6 `MODEL_INCOMPLETE` remains valid

This clarification does **not** invalidate the LF reconciliation.

LF has direct hardware V25 contact evidence for all six endpoints.

Examples:

```text
LF hip MIN
hardware contact ≈ -42.803°
mesh contact     ≈ -47.500°
mesh-hardware delta ≈ -4.70°

LF lower MIN
hardware contact ≈ -91.846°
mesh contact     ≈ -97.957°
mesh-hardware delta ≈ -6.11°
```

Those mesh collisions happen **after** the real hardware has already stopped and therefore cannot be the real endpoint event.

For LF hip MAX, upper MIN/MAX and lower MAX, no same-leg collision exists at the real hardware stop or anywhere through the Phase 1 envelope.

Even if an unrelated mesh collision were found much farther out, for example 20° or 40° beyond the declared limit, it would not retroactively become the physical LF endstop that V25 measured at a much earlier angle.

Therefore:

```text
LF 6/6 = MODEL_INCOMPLETE
```

remains the correct endpoint-metrology conclusion.

---

## 6. Why RF/RH/LH need more cautious wording

RF/RH/LH do not yet have direct full-leg hardware contact oracles.

Therefore a no-contact result for those legs cannot distinguish between:

1. no relevant collision feature exists in the current STL geometry at all;
2. a relevant collision feature exists, but only beyond the Phase 1 ±10° envelope around the old declared limit;
3. the real mechanical endstop is an internal servo/bracket feature not represented by the collision STL;
4. a contact exists only under a different physically justified prerequisite configuration.

For these legs, `NO_MODELED_ENDSTOP` is intentionally weaker than LF `MODEL_INCOMPLETE`.

---

## 7. Recommended pre-Phase-2 sanity check

Before treating the current model-only endpoint picture as exhaustive, run a targeted **extended mesh-contact audit** for endpoints currently marked `NO_MODELED_ENDSTOP`.

The new audit should be independent from the old hand-entered URDF limit as the outer search anchor.

It should:

1. remain offline only;
2. keep `enforce_limits=False`;
3. preserve the same endstop-vs-path collision policies;
4. sweep a wider, explicitly justified angular domain;
5. report the first same-leg contact if one eventually appears;
6. separately report first path/cross-leg obstruction;
7. never relabel a far-away incidental collision as a real hardware endpoint;
8. preserve LF hardware reconciliation unchanged;
9. record whether each Phase 1 `NO_MODELED_ENDSTOP` remains no-contact or becomes a far-out mesh collision finding;
10. avoid automatically editing URDF limits.

### Choice of wider domain

The extended range must not be chosen by blindly adding another arbitrary margin to the current URDF limits.

Use a physically justified domain derived from available mechanical/CAD constraints and servo/joint topology, with explicit hard safety bounds for the offline numerical search.

If no such bound is yet authoritative, the audit must say so and use a documented exploratory domain rather than presenting it as a physical limit.

---

## 8. Relationship to Phase 1 completion

Phase 1 remains closed and valid as the bounded Geometry Compiler deliverable specified by the 2026-08-07 canonical handoff, which explicitly required an **explicitly bounded analysis envelope** around the declared limit.

This clarification narrows the interpretation of one result category; it does not rewrite the completed calculations.

Canonical Phase 1 status remains:

```text
PASS_GEOMETRY_COMPILER_COMPLETE_WITH_EXPLICIT_MODEL_GAPS
```

The next planning sequence becomes:

```text
Phase 1 complete
→ extended no-contact geometry sanity check
→ Phase 2 generic V25-derived engine
→ Phase 3 RF/RH/LH hardware calibration
```

---

## 9. Canonical references

- `MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md`
- `MATDOG_GEOMETRY_COMPILER_PHASE1_COMPLETION_2026-08-07.md`
- `09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_PROFILE.json`
- `09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_REPORT.md`
- `matdog_geometry_contact_search.py`
- `matdog_geometry_scene.py`

---

## 10. Permanent wording rule

Use:

```text
NO_MODELED_ENDSTOP = no relevant mesh contact found in the documented analysis envelope
```

Do not use:

```text
NO_MODELED_ENDSTOP = the mesh never contacts at any angle
```

unless a future wider-domain audit actually proves that stronger statement over a clearly defined domain.
