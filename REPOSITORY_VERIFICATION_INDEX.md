# MATDOG Geometry Compiler V5 verification record

> **HISTORICAL / REFERENCE — 2026-08-11 snapshot.**
>
> This file preserves the Phase 2A0 / Geometry Compiler V5 verification record. It does not define
> today's project status, physical population, immediate milestone, repository topology, or
> operational entry point. For those, use the [root `README.md`](README.md) and
> [`ARCHITECTURE.md`](01_Docs/02_Architecture/ARCHITECTURE.md).

**Recorded status on 2026-08-11:** Phase 2A0 CLOSED / Geometry Compiler V5 merged.

**Recorded next step on 2026-08-11:** Phase 2A — Generic V25-derived full-leg calibration engine.

These labels are retained as historical phase evidence, not as current instructions.

## Repository and source snapshot — 2026-08-11

```text
MattRobotics/robot-dog
  default branch: main
  post-Phase-2A0 merge baseline:
  f07aa094a1b78c5670cc36ef3fdb349422a38955
  role: robot-specific CAD / URDF / geometry / calibration evidence / project decisions

MattRobotics/norma-core
  default branch: main
  Phase 2A entry baseline observed 2026-08-11:
  f47b1ba579c623139058a8b0118648015739ab10
  role: Station/ST3215 runtime and native MATDOG calibrator

immutable LF V25 release:
  release/matdog-lf-calibrator-v25
  f87dd1fbc7e8100d275c74f9af448642f3429680
```

Always re-verify live remote/local state before implementation. The SHAs above are entry checkpoints, not permission to ignore newer legitimate commits.

## Phase state recorded on 2026-08-11

```text
Phase 1     CLOSED / historical endpoint-metrology candidate
Phase 1B    CLOSED
Phase 2A0   CLOSED / Geometry Compiler V5 merged via PR #19
Phase 2A    RECORDED NEXT (2026-08-11) / generic V25-derived full-leg engine
Phase 2B    pending / final path-parking safety integration
Phase 2C    pending / complete offline validation
hardware    later: RF -> RH -> LH
```

## Historical Phase 2A operational entry point

Recorded entry document:

```text
06_Software/Matdog_Core/calibration/
MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md
```

Phase 2A0 closeout record:

```text
09_Logs/Development_Log/
2026-08-11_PHASE2A0_GEOMETRY_V5_CLOSEOUT.md
```

Geometry Compiler artifact index:

```text
09_Logs/Validation_Reports/Geometry_Compiler/README.md
```

Within this snapshot, the older Phase1B-closed and 2026-08-07 calibration handoffs were
historical/architectural references rather than its Phase 2A milestone entry point.

## Geometry Compiler V5 canonical evidence record

Merged PR:

```text
robot-dog #19
reviewed head:
2890daf0a8ac6103d3856f208a5f042528fc0da0

squash merge commit on main:
f07aa094a1b78c5670cc36ef3fdb349422a38955
```

Corrected canonical semantic hashes:

```text
endpoint
  de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e
parking
  67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139
combined
  0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51
```

Corrected determinism manifests:

```text
C workers=1
b86b5d35678df4510989ea49fb5d42acaea2e4838226d7017456399dfceb0d81

D workers=4
0db86e633599f63a769dbba75db3a54c1e6470e128c007eb24c465f92a428b17
```

Final canonical evidence:

```text
24/24 geometric contacts
24/24 canonical contexts empty
24/24 endpoint/planner consistency
6 direct-target obstructions
18 collision-free direct paths

parking:
18 NOT_NEEDED
6 FEASIBLE_1DOF_PLAN_FOUND
24/24 complete sequences
94 evaluated 1DOF candidates
0 requiring 2DOF
```

Final tests:

```text
241/241 geometry discovery PASS
58/58 adjacent offline calibration PASS
299/299 total PASS
```

Independent adversarial review findings were all closed before merge:

```text
B1 CLOSED
M1 CLOSED
M2 CLOSED
M3 CLOSED
m1-m5 CLOSED
NO NEW BLOCKER / MAJOR
```

Permanent semantic boundary:

```text
CANONICAL V5 != G4 REPLAY
```

## External safety-policy state

Final reference policy uses the unchanged 3 mm threshold:

```text
16 PASS
0 FAIL
8 UNRESOLVED
0 motion authorizations
```

All eight `UNRESOLVED` are `DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS`.

```text
lf_hip_joint:min
lf_lower_leg_joint:min
rf_hip_joint:max
rf_lower_leg_joint:min
rh_hip_joint:max
rh_lower_leg_joint:min
lh_hip_joint:min
lh_lower_leg_joint:min
```

There are zero FAIL and zero UNRESOLVED results among the eight `EXECUTABLE_URDF_DOMAIN` endpoints.

Never reinterpret:

```text
UNRESOLVED as PASS
DIAGNOSTIC as EXECUTABLE
GEOMETRIC CONTACT as MOTION AUTHORIZATION
```

These eight unresolved lower bounds remain explicit Phase 2B/2C inputs.

## Hardware calibration evidence in this historical snapshot

Only LF V25 is mechanically hardware validated.

```text
HIP   MIN -42.803°   MAX +39.375°
UPPER MIN -53.525°   MAX +122.607°
LOWER MIN -91.846°   MAX +34.277°
```

RF/RH/LH remain geometry-only until future hardware validation.

LF V25 must remain immutable. Older V28–V42 and duplicated all-leg/RF experiments are historical development evidence, not active programs.

## Recorded Phase 2A architecture contract — historical

Target:

```text
LegSessionStateMachine
+
LegCalibrationSpec
```

not four independent state machines.

Before refactoring LF V25, classify every materially relevant LF constant/helper/state transition:

```text
A — generic calibration behavior
B — geometry/profile/spec data
C — global ST3215 hardware/safety parameter
D — historical LF-only evidence
```

In this dated plan, Phase 2A was offline software foundation work. It authorized no physical
movement, Station probing, direct serial work, or EEPROM writes.

## Issues recorded for the planned next phase

1. Pre-existing live-FK calibration-status mismatch:
   `DIGITAL_ZERO_CALIBRATED_AND_VERIFIED` vs `VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION`.
2. 16 diagnostic geometry targets outside operational URDF limits.
3. 8 unresolved conservative clearance lower bounds, all outside executable target domain.
4. RF/RH/LH lack hardware oracle evidence.
5. Do not copy LF measured spans as other-leg coordinates.
6. FRONT/HIND geometry is not interchangeable by convention.
7. A fitted affine diagnostic must never erase raw model-vs-hardware discrepancy.

## Records for this 2026-08-11 verification snapshot

Primary records at the time:

```text
README.md
REPOSITORY_VERIFICATION_INDEX.md
06_Software/Matdog_Core/calibration/MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md
09_Logs/Development_Log/2026-08-11_PHASE2A0_GEOMETRY_V5_CLOSEOUT.md
09_Logs/Validation_Reports/Geometry_Compiler/README.md
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
```

Earlier historical references retained at the time:

```text
06_Software/Matdog_Core/calibration/MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md
06_Software/Matdog_Core/calibration/MATDOG_CALIBRATION_CANONICAL_HANDOFF_PHASE1B_CLOSED_2026-08-09.md
06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_PHASE1B_ADDENDUM_2026-08-08.md
```

Superseded Geometry Compiler candidate evidence remains preserved in its original validation-artifact locations and clearly labelled as superseded.

## Repository hygiene recorded at closeout

- Active development must start from verified current `main`, never from a merged historical branch.
- Merged/retired development branches may be deleted after their unique state is preserved by merged commits, PR history and a closeout log.
- Historical handoffs, validation artifacts and immutable releases must not be deleted merely to make the repository look cleaner.
- No force-push or destructive history rewrite.
- No merge into `main` without explicit authorization.
