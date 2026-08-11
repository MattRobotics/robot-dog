# MATDOG Geometry Compiler V5 — corrected offline test summary

**Date:** 2026-08-11<br>
**G10 result:** **PASS for the corrected offline Geometry Compiler contract**<br>
**Hardware access:** none

Supersedes `2026-08-11_072224_MATDOG_GEOMETRY_V5_TEST_SUMMARY.md`.

## Final approved offline result

Run after every code, document and G12 consumer edit was complete and before
the finalization commit.

| Suite | Result | Duration |
|---|---:|---:|
| `python3 -B -m unittest discover -s tests -p 'test_matdog_geometry_*.py'` from the calibration directory | **241/241 PASS** | 649.338 s |
| Seven adjacent offline calibration modules: visual-zero path/static, endstop detector/watch/telemetry, joint math, mechanical endstop | **58/58 PASS** | 0.119 s |
| **Approved offline calibration total** | **299/299 PASS** | — |

Both runs used `PYTHONDONTWRITEBYTECODE=1` and `python3 -B`; no bytecode output
is part of the deliverable.

The geometry suite grew from 204 to 241 tests. The 37 additional tests are the
remediation's own coverage, including two new modules
(`test_matdog_geometry_compiler_v5.py`, `test_matdog_geometry_contact_search_v5.py`).

Benchmark C and Benchmark D were **not** rerun. Their semantic and execution
source manifests were verified byte-for-byte against current sources
immediately before this run and immediately before commit: zero drift across
the nine-file canonical semantic manifest, the seven-file G4 replay manifest,
the eleven-file execution manifest and the input-file manifest. Everything
changed after the C/D source freeze is documentation, new consumer artifacts,
or test-only.

## Remediation coverage added

| Audit finding | Closing tests |
|---|---|
| **B1** canonical profile contaminated by G4 replay context | `test_compiler_api_cannot_accept_a_g4_profile`, `test_default_tasks_are_exactly_24_context_free_direct_sweeps`, `test_canonical_gate_rejects_even_zero_valued_legacy_context_keys`, `test_all_24_canonical_endpoint_contexts_are_explicitly_empty`, `test_all_24_frozen_replay_records_explicitly_carry_noncanonical_context`, `test_replay_task_rejects_canonical_direct_path_domain`, `test_path_uses_contact_target_and_parking_equal_subdivision_grid`, `test_all_24_direct_sweeps_pass_with_precise_boundary_in_sample_bracket`, `test_context_target_domain_status_pair_relation_and_angle_tamper_stop`, `test_refined_bracket_wider_than_declared_resolution_stops`, `test_parking_task_boundary_does_not_copy_endpoint_search_context` |
| **M1** git commit SHA inside the semantic hash | `test_repository_materialization_metadata_is_nonsemantic`, `test_repository_dirty_audit_distinguishes_clean_dirty_and_git_failure`, `test_semantic_source_hash_change_changes_hash_and_fails_live_provenance` |
| **M2** oracle coverage overstated | `test_complete_same_context_replay_comparison_passes` plus the full 34-check comparator exercised by every replay test |
| **M3** no negative oracle test | `test_contact_angle_beyond_declared_resolution_is_rejected`, `test_outcome_within_declared_resolution_remains_accepted`, `test_endpoint_identity_change_is_rejected`, `test_contact_status_change_is_rejected`, `test_declared_limit_change_is_rejected`, `test_active_pair_change_is_rejected`, `test_contact_pair_change_is_rejected`, `test_missing_expected_path_analysis_is_rejected`, `test_path_pair_change_is_rejected`, `test_contact_bracket_domain_and_bisection_mutations_are_rejected`, `test_path_bracket_domain_and_bisection_mutations_are_rejected` |
| **m1** profile-wide parameter uniformity | `test_every_profile_wide_search_parameter_must_be_uniform`, `test_per_endpoint_direct_path_domain_is_not_misreported_as_envelope_margin` |
| **m2** no-contact report rendering | `test_renderer_handles_legitimate_no_contact_record` |
| **m3** incomplete bundle / manifest-last validity | `test_manifest_is_the_last_visible_link`, `test_complete_manifest_marked_bundle_is_accepted`, `test_visible_data_without_manifest_is_not_a_valid_bundle`, `test_tampered_member_and_unexpected_manifest_member_are_rejected`, `test_manifest_content_hash_and_missing_member_are_rejected`, `test_determinism_snapshot_detects_manifest_and_member_changes` |
| **m4** CPU-affinity documentation semantics | `test_cgroup_limits_require_explicit_cgroup_mode` and the manifest `resource_enforcement` record |
| **m5** misleading test name | renamed to `test_parking_task_boundary_does_not_copy_endpoint_search_context`; direct canonical empty-context test added |

## Contract coverage

| Required boundary | Result |
|---|---|
| Canonical endpoint profile is context-free, 24/24 | PASS |
| Canonical compiler cannot accept a G4 profile (no such parameter, no import) | PASS |
| G4 replay is a separate API with no profile builder or writer | PASS |
| Replay artifacts are marked non-canonical and profile-ineligible | PASS |
| Canonical path domain is `q=0` to geometric target | PASS |
| Endpoint/planner path consistency is a fail-closed gate | PASS |
| Pure `q=0` contacts equal frozen-context replay contacts | PASS |
| Repository materialization excluded from semantic identity | PASS |
| Semantic provenance is URDF + collision meshes + source hashes + content | PASS |
| Oracle rejects perturbed contact/pair/path/bracket/domain/bisection data | PASS |
| Acceptance tolerance remains the declared bisection resolution | PASS |
| Serialized refinement brackets prove their own contract | PASS |
| Bundle without its manifest is not a valid bundle | PASS |
| URDF collision filename changes loaded geometry without Python edits | PASS |
| Per-mesh scale and collision origin honored | PASS |
| `motorId` / `motorDirection` from URDF; model-first selection | PASS |
| q=0 active pairs 12/12 separated | PASS |
| Pure core does not import LF evidence or safety policy | PASS |
| Separate Hardware Reconciler and separate 3 mm safety policy | PASS |
| `PATH_OBSTRUCTION` is topology-classified, not automatically cross-leg | PASS |
| Semantic metadata exclusions | PASS |
| workers=1 vs workers=4 equality; parent-only canonical writes | PASS |
| Frozen v1-v4, G4 and LF evidence immutability | PASS |
| Input/source TOCTOU and no-clobber | PASS |

## Broader kinematics suite — pre-existing, out of scope

```text
branch matdog/geometry-compiler-v5-collision-baseline : 4 failed, 46 passed
main   e71876e80c23c370f9fecf36ddf15f152faf5eb3        : 4 failed, 45 passed
```

The same four `test_matdog_leg_fk_live.py` methods fail identically on `main`.
The tracked `MATDOG_JOINT_CALIBRATION.yaml` declares
`calibration_status: DIGITAL_ZERO_CALIBRATED_AND_VERIFIED` while
`matdog_leg_fk_live.py` requires
`VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION`.

This is **PRE-EXISTING**, **NON-BLOCKING for PR #19**, and remains a separate
next-phase / live-FK issue. PR #19 touches that YAML only to update the pinned
URDF SHA. The branch's one extra pass is the added
`test_collision_path_separation_is_the_only_urdf_text_change`, which proves the
URDF text change is exactly the collision-path separation.

It was reported rather than changed to manufacture a green suite.

## CI

**RECOMMENDED BUT NON-BLOCKING**, and deliberately **not implemented in this
task**. The repository has no `.github/workflows`. Both suites above are
offline, hardware-free and CI-viable; wiring them up is recommended before
Phase 2A.

## Safety and scope statement

No hardware, Station, serial, servo or EEPROM operation was performed by any
test. No threshold or candidate cap was relaxed. No `norma-core`, historical RF
NormaCore worktree, historical v1-v4 artifact, frozen G4 artifact or LF evidence
file was modified. The `2026-08-11_072224` artifact bundle was not overwritten
or deleted. No merge was performed.
