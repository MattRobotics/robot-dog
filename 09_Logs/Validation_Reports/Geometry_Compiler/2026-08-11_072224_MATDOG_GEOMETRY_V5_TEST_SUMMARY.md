# MATDOG Geometry Compiler V5 — offline test summary

> **STATUS: SUPERSEDED PRE-AUDIT V5 CANDIDATE EVIDENCE.**
> This document describes the pre-audit V5 candidate. An independent adversarial
> audit of PR #19 found that the integrated runner reused one frozen-G4-context
> replay as both replay evidence and the canonical "pure geometry" endpoint
> profile, so the published canonical artifact carried legacy 30/50/90 degree
> context on 20 of 24 endpoints and its path-obstruction layer contradicted the
> pure q=0 parking results.
>
> The semantic hashes quoted below (`cad2f194…`, `3cda03c2…`, `e99e2b65…`) are
> **not canonical**. The corrected canonical bundle is
> `2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4_*`; see
> `2026-08-11_132758_MATDOG_GEOMETRY_V5_TEST_SUMMARY.md`.
>
> This file is retained unchanged in substance for provenance. Nothing in the
> `2026-08-11_072224` bundle was overwritten or deleted.

**Date:** 2026-08-11<br>
**G10 result:** **PASS for the approved offline Geometry Compiler contract**<br>
**Hardware access:** none

## Interrupted-run recovery

The model-capacity interruption did not interrupt the G10 process. The process
had exited successfully before conversational execution stopped:

```text
Ran 232 tests in 543.919s
OK
```

Recovery found no live test process, no partial canonical artifact, no stale
staging directory and no changed frozen hash. That completed operation was not
rerun merely because the conversation was interrupted.

G11 subsequently added process, determinism and integrated-run tests, so the
final current implementation was validated again with the suites below.

## Final in-scope suites

| Suite | Result | Duration |
|---|---:|---:|
| `python3 -m unittest discover -s tests -p 'test_matdog_geometry_*.py'` from the calibration directory | 204/204 PASS | 554.376 s |
| Seven remaining offline calibration modules: visual-zero path/static, endstop detector/watch/telemetry, joint math and mechanical endstop | 58/58 PASS | 0.110 s |
| **Approved calibration total** | **262/262 PASS** | — |

All runs used `PYTHONDONTWRITEBYTECODE=1` or `python3 -B`; no bytecode output is
part of the deliverable.

The first post-G11 full geometry attempt exposed 11 errors in one synthetic
safety-test fixture: the stricter validator correctly rejected the fixture's
placeholder combined source digest. The fixture was changed to build the
canonical digest, not to relax production validation. Its targeted 11 tests
then passed, followed by the 204/204 full geometry rerun above.

An independent final fast audit ran 82 V5 tests covering immutability, schemas,
provenance, G4 oracle, parking v1/v2, process workers, integrated C/D runner,
hardware reconciliation and safety policy:

```text
Ran 82 tests in 2.322s
OK
```

## Contract coverage

| Required boundary | Evidence | Result |
|---|---|---|
| URDF collision filename changes loaded geometry without Python edits | Model/scene tests replace filenames and reload | PASS |
| Per-mesh scale and collision origin | Explicit model/scene transform tests | PASS |
| `motorId` and `motorDirection` from URDF | Metadata selection tests | PASS |
| Model-first actuator selection | Renamed-model and topology-order tests | PASS |
| q=0 active pairs | Fresh 12/12 replay in compiler/runner and audit | PASS |
| Pure core does not import LF evidence | Import/source-boundary tests | PASS |
| Separate Hardware Reconciler | Dedicated unit and artifact-validation tests | PASS |
| Separate safety policy | Dedicated 3 mm replay and scope-validation tests | PASS |
| PATH_OBSTRUCTION is not automatically cross-leg | Relation tests and six-event oracle | PASS |
| 3 mm is not geometry feasibility | Parking/safety boundary tests | PASS |
| Semantic metadata exclusions | Two-run/hash mutation tests | PASS |
| Workers=1 vs workers=4 | Spawn probes, ordering/failure tests and real C/D run | PASS |
| Parent-only canonical writes | Writer PID/count and failure/no-output tests | PASS |
| Frozen v1-v4 and G0-G9 artifacts | Explicit SHA immutability tests | PASS |
| G4-to-G7 same-new-geometry oracle | 24/24 endpoints, six path events | PASS |
| Parking 1-DOF then 2-DOF | Candidate ordering, complete path and no-feasible tests | PASS |
| Input/source TOCTOU and no-clobber | Integrated-run tests | PASS |

## Broader kinematics suite

The broader kinematics suite was also executed as diagnostic evidence:

```text
Ran 50 tests in 20.718s
FAILED (errors=16)
```

The 16 errors are four test methods times four leg subtests in
`test_matdog_leg_fk_live.py`:

```text
test_joint_order_servo_mapping_and_direction_for_every_leg
test_visual_zero_encoder_map_becomes_zero_radians_for_every_leg
test_visual_zero_encoder_map_reproduces_urdf_foot_frame_for_every_leg
test_visual_zero_error_is_circular_and_unsigned_for_every_leg
```

Every error is the same pre-existing calibration-state contract mismatch: the
tracked YAML declares
`calibration_status='DIGITAL_ZERO_CALIBRATED_AND_VERIFIED'`, while that live-FK
loader requires `VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION`. The Geometry
Compiler change only updates the pinned URDF SHA in that YAML; it does not
change the status. The other 46 kinematics test cases pass.

This hardware-adjacent, pre-existing mismatch is reported rather than changed
to make the suite green. It is outside the approved V5 geometry refactor and
does not invalidate the 262/262 in-scope offline Geometry Compiler result.

## Runtime validation beyond unit tests

The real integrated runs add end-to-end evidence that mocks cannot provide:

- one-worker C and four-process D both exit 0;
- all worker initializers reload and fingerprint the exact URDF/mesh inputs;
- 24/24 canonical results are returned with no duplicate or missing endpoint;
- D's seven determinism comparisons are all true;
- all 16 C/D canonical artifact file hashes pass their manifests;
- both runs preserve the 12/12 q=0 gate and frozen G4 oracle;
- D observes CPU affinity 0-3, 6 GiB MemoryMax, zero swap and zero OOM;
- workers write no artifact; the parent publishes each complete bundle once.

## Safety and scope statement

No hardware, Station, serial, servo or EEPROM operation was performed by any
test. No threshold or candidate cap was relaxed. No `norma-core`, historical
RF NormaCore worktree, historical v1-v4 artifact or LF evidence file was
modified. No merge was performed.
