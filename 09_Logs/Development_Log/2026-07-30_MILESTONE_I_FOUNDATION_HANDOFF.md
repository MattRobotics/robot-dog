# MATDOG Milestone I foundation handoff — 2026-07-30

## Publication identity

- Repository: `MattRobotics/robot-dog`
- Branch: `agents/milestone-i-matdog-clean-room-spec`
- Base/main: `a6dc1184f56956dad696b3bcc24d74f375edb5b7`
- Technical payload head at PR creation:
  `e0b3c5738be13bd97aa5a0bd67db1ed51b2f7f18`
- Draft PR: `MattRobotics/robot-dog#4`
- PR URL: <https://github.com/MattRobotics/robot-dog/pull/4>
- Classification: `FOUNDATION_PASS_WITH_LIMITS`

This handoff is the final versioned metadata file and therefore cannot contain
the SHA of its own enclosing commit without a circular hash dependency. The
published branch head after this handoff is authoritative in Git and PR #4;
the external report records that exact final SHA.

## Commits before this handoff

1. `7957f8610914126eba05fb82be59ef13c6b0081f` —
   `docs(milestone-i): freeze clean-room source baseline`
2. `39e1e14c1cc9a7cdc5e10193297b38e6fa1cb43c` —
   `docs(milestone-i): define canonical MATDOG contracts`
3. `e0b3c5738be13bd97aa5a0bd67db1ed51b2f7f18` —
   `test(milestone-i): validate clean-room foundation`

## Delivered files

Eight documents under `01_Docs/02_Architecture/Milestone_I/`:

- clean-room source policy;
- cross-repository baseline;
- canonical kinematic convention;
- joint and servo contract;
- frame convention;
- calibration and limit semantics;
- Station integration boundary;
- foundation acceptance.

Under `06_Software/Matdog_Core/milestone_i/`:

- schema documentation;
- ten CSV registries: sources, claims, conflicts, joints, frames, servo
  mappings, calibration profiles, limits, decisions and unresolved items;
- deterministic standard-library validator.

Tests are in `08_Tests/Milestone_I/test_validate_foundation.py`. This file is
the versioned handoff.

## Counts and state

- Sources: 66 = 37 robot-dog + 15 NormaCore fork + 13 XGoLite + 1 external
  NormaCore upstream reference.
- Claims: 51 = 11 `MATDOG_VERIFIED`, 3 `MATDOG_DERIVED`, 13
  `HARDWARE_OBSERVATION`, 7 `NORMACORE_MATDOG_FORK_MAIN_FACT`, 1
  `NORMACORE_EXPERIMENTAL_PR`, 1 `NORMACORE_GENERIC_REFERENCE`, 2
  `XGOLITE_ARCHITECTURAL_REFERENCE`, 4 `SUPERSEDED`, 6 `UNKNOWN`, 3
  `DECISION_REQUIRED`.
- Canonical joints / servo mappings: 12 / 12.
- Profiles: 24 software-ready; 2 hardware-validated; 22 hardware-pending.
- Safe limits: 0 proven; 24 explicitly unknown.
- Conflicts: 20 total; 14 open and 6 closed by explicit classification.
- Decisions: 9 open.
- Unresolved: 11 = 8 `UNKNOWN` + 3 `DECISION_REQUIRED`.

The only physically validated contacts are LF upper M12 MIN at 1443/1443
tick with spread 0 and LF upper M12 MAX at 3443/3442 tick with spread 1.
Neither is an operational safe limit and neither is generalized to another
profile.

## Verification

- Foundation validator: PASS — 66 sources, 51 claims, 12 joints, 24 profiles.
- Offline mutation tests: 14/14 PASS.
- Canonical URDF XML parse: PASS.
- Ten CSV parse: PASS.
- UTF-8, relative Markdown links, credential/private-key/credential-URL,
  disallowed home path, symlink, special-file and temporary-artifact scans:
  PASS.
- `git diff --check main...HEAD`: PASS before publication.
- Each technical commit passed `git diff --check HEAD^..HEAD`.

## Limits and next operation

Open work includes the remaining 22 contacts, all 24 safe limits, URDF custom
direction-tag semantics, world placement, collision/runtime representation,
first-stand envelope, bus-owner deployment, controller placement, real-time
rates, MATDOG IMU/estimator and current characterization.

The next proposed operation is human review of draft PR #4. A later offline
I.2 gate requires separate authorization and must carry all registered
unknowns and decisions forward. This handoff does not authorize I.2 or any
hardware work.

No hardware or device was accessed; no serial port was opened; Station was
not started; no torque, EEPROM, Position Offset, calibration run or firmware
operation occurred. No gait, operational IK, new FK, planner, serial runtime
adapter or merge was implemented. NormaCore and XGoLite remained read-only.
