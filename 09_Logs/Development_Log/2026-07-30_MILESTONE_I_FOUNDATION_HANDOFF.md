# MATDOG Milestone I foundation handoff — remediated 2026-07-31

## Publication identity

- Repository: `MattRobotics/robot-dog`
- Branch: `agents/milestone-i-matdog-clean-room-spec`
- Base/main: `a6dc1184f56956dad696b3bcc24d74f375edb5b7`
- Independently reviewed initial head:
  `36c86b74ac344319ff1ff2c9e4c89dd8bb319653`
- Draft PR: `MattRobotics/robot-dog#4`
- PR URL: <https://github.com/MattRobotics/robot-dog/pull/4>
- Foundation classification: `FOUNDATION_PASS_WITH_LIMITS`
- Remediation classification: `REMEDIATION_PASS_WITH_LIMITS`

This handoff is inside the remediated commit and therefore cannot contain the
SHA of its own enclosing commit without a circular hash dependency. The
published branch/PR head and the external remediation report are authoritative
for the final SHA. PR #4 must remain draft and must not be merged by this
handoff.

FOUNDATION_METRICS_JSON: {"claims": 51, "conflicts": 21, "decisions": 9, "frames": 19, "hardware_validated_profiles": 2, "joints": 12, "limits": 50, "materialized_frames": 17, "mutation_cases": 28, "operational_safe_limits": 0, "profiles": 24, "servos": 12, "sources": 66, "unit_tests": 29, "unresolved": 11}

## Original publication commits

1. `7957f8610914126eba05fb82be59ef13c6b0081f` —
   `docs(milestone-i): freeze clean-room source baseline`
2. `39e1e14c1cc9a7cdc5e10193297b38e6fa1cb43c` —
   `docs(milestone-i): define canonical MATDOG contracts`
3. `e0b3c5738be13bd97aa5a0bd67db1ed51b2f7f18` —
   `test(milestone-i): validate clean-room foundation`
4. `36c86b74ac344319ff1ff2c9e4c89dd8bb319653` —
   original publication metadata head reviewed independently.

The remediation commit(s) follow this initial head. Their exact SHAs are
recorded in Git, PR #4 and the external remediation report after publication.

## Independent-review remediation

The 2026-07-31 review demonstrated eight false PASS cases: changed joint
origin, joint effort, frame origin, profile URDF-limit tick, removed claim,
removed unresolved row, changed conflict status and empty source authority.
All eight returned exit code zero at the initial head and are now versioned
negative tests.

Root cause was validation of shape and broad counts without full canonical
comparison or frozen identities/statuses. The remediation adds:

- `foundation_expectations.json` with repository pins, exact IDs,
  distributions/statuses, critical provenance, directions, formula constants
  and acceptance/handoff metrics;
- complete numeric joint-to-URDF checks including origin/RPY,
  effort/velocity and every custom hardware tag;
- complete materialized-frame checks with explicit source joint/link and
  planned handling for `world` / `ground_plane`;
- formula-derived comparison of all 24 NormaCore profiles, including exact
  order, roles, allowed motors, prerequisites, reverse restore order, home,
  limit, guard and software/hardware/evidence state;
- exact inventories and status distributions for sources, claims, conflicts,
  decisions, unresolved rows, joints, frames, servos, profiles and limits;
- 29 standard-library tests: one valid baseline and 28 rejected mutations.

## Provenance changes

- `C-FRAME-BASE` now cites the pinned REV00 ADR section that actually defines
  the coordinate axes.
- `C-UNKNOWN-IMU` was removed because absence was not positive evidence. The
  intentional gap remains in `D-IMU` and `U-IMU`.
- `C-GENERIC-CAL` was split into `C-GENERIC-SO101` and
  `C-GENERIC-ELROBOT`, each with its own source file and locator; the generic
  conflict was split accordingly.
- The historical M11 direction blob remains byte-for-byte unchanged at SHA-256
  `272d5e8e4e9158cd6ac058aaee1282aa132172c24e0faf3775f5b5e472a3afe3`.
  It is explicitly `TEXT_ONLY_NONPARSEABLE` / `PINNED_HUMAN_TEXT`, and the
  claim uses lines 17-20 instead of an implicit YAML parse.

The claim total is still 51 only as the net result of removing one unsupported
claim and replacing one composite claim with two independent claims. The
classification distribution changed to 5 `UNKNOWN` and 2
`NORMACORE_GENERIC_REFERENCE`. Conflicts changed from 20 to 21, with 14 open
and 7 closed. Decisions remain 9 open and unresolved rows remain 11 (8
`UNKNOWN`, 3 `DECISION_REQUIRED`).

## Delivered foundation

Eight policy/contract/acceptance documents remain under
`01_Docs/02_Architecture/Milestone_I/`.

Under `06_Software/Matdog_Core/milestone_i/`:

- schema documentation;
- ten CSV registries;
- the machine-readable expectation manifest;
- the deterministic standard-library validator.

Tests remain in `08_Tests/Milestone_I/test_validate_foundation.py`.

Current canonical counts are 66 sources, 51 claims, 21 conflicts, 9
decisions, 11 unresolved rows, 12 joints, 19 frames (17 materialized), 12
servo mappings, 24 profiles and 50 limit rows. Two profiles have physical
contact evidence. Zero operational safe limits are proven.

## Verification contract

- Validator: exact foundation identity and substantive checks must PASS.
- Tests: 29/29 must PASS; all 28 invalid mutations must produce errors.
- URDF XML, ten CSV files and the JSON expectation manifest must parse.
- `git diff --check` and `git diff --check main...HEAD` must PASS.
- UTF-8, secret/private-key/credential-URL, home-path, symlink, special-file,
  executable, pycache, temporary-artifact and Markdown-link scans must PASS.
- Worktree must be clean after normal non-forced push.

## Limits and next operation

Intentional open work includes the remaining 22 contacts, all 24 safe limits,
URDF custom direction-tag semantics, world placement, collision/runtime
representation, first-stand envelope, bus-owner deployment, controller
placement, real-time rates, MATDOG IMU/estimator and current characterization.

A new independent review is required before merge. Milestone I.2 requires
separate authorization and was not started. No hardware or device was
accessed; no serial port was opened; Station was not started; no torque,
EEPROM, Position Offset, calibration run or firmware operation occurred. No
gait, operational IK, new FK, planner or serial runtime adapter was
implemented. NormaCore and XGoLite remained read-only.
