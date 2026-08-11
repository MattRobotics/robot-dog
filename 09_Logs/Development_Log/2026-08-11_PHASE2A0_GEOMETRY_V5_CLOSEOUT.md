# MATDOG — Phase 2A0 / Geometry Compiler V5 closeout

**Date:** 2026-08-11  
**Status:** CLOSED / MERGED / ARCHIVED  
**Hardware used:** NO

## Canonical merge record

Repository: `MattRobotics/robot-dog`

Merged PR:

```text
#19 — Geometry Compiler V5: deterministic pure geometry pipeline
```

Final reviewed PR head before merge:

```text
2890daf0a8ac6103d3856f208a5f042528fc0da0
```

Squash merge commit on `main`:

```text
f07aa094a1b78c5670cc36ef3fdb349422a38955
```

Former development branch:

```text
matdog/geometry-compiler-v5-collision-baseline
```

The branch is retired after merge and must never be used as a Phase 2A development base. Its technical history is preserved by this closeout, merged `main`, PR #19, the retained validation artifacts and the superseded pre-audit evidence.

## Independent audit and remediation

The first published V5 candidate passed its original test suite but an independent adversarial review found a real composition-root defect: frozen G4 replay context was reused inside the supposedly pure canonical V5 endpoint/path profile. Endpoint contact angles were unaffected, but path semantics and the purity claim were not acceptable.

The audit also found three MAJOR issues around semantic provenance, oracle coverage/documentation and negative-test coverage.

All findings were remediated before merge:

```text
B1 CLOSED
M1 CLOSED
M2 CLOSED
M3 CLOSED
m1-m5 CLOSED
NO NEW BLOCKER
NO NEW MAJOR
```

Permanent architecture rule:

```text
CANONICAL V5 != G4 REPLAY
```

Canonical V5 uses context-free model-derived tasks. Frozen G4 replay is separate, non-canonical refactor evidence and can never become a canonical profile.

## Corrected canonical V5 results

Corrected C/D semantic hashes, identical for workers=1 and workers=4:

```text
endpoint
  de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e
parking
  67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139
combined
  0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51
```

Corrected run manifests:

```text
C / workers=1
b86b5d35678df4510989ea49fb5d42acaea2e4838226d7017456399dfceb0d81

D / workers=4
0db86e633599f63a769dbba75db3a54c1e6470e128c007eb24c465f92a428b17
```

Canonical profile:

```text
24/24 geometric endpoint contacts
24/24 empty endpoint search contexts
24/24 endpoint/planner path consistency
6 direct-target path obstructions
18 collision-free direct paths
```

Parking:

```text
18 NOT_NEEDED
6 FEASIBLE_1DOF_PLAN_FOUND
24/24 complete sequences
94 evaluated 1DOF candidates
0 requiring 2DOF
```

Target-domain split:

```text
8 EXECUTABLE_URDF_DOMAIN
16 DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS
```

## Final safety-policy status

External reference threshold remains `3 mm`.

```text
16 PASS
0 FAIL
8 UNRESOLVED
0 motion authorizations
```

All eight `UNRESOLVED` are outside the executable URDF target domain:

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

Permanent interpretation:

```text
UNRESOLVED != PASS
DIAGNOSTIC != EXECUTABLE
GEOMETRIC CONTACT != MOTION AUTHORIZATION
```

`UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD` means the conservative evidence is insufficient to prove the external 3 mm threshold. It is not an exact physical-clearance measurement and must never be promoted to PASS.

## Hardware reconciler

Against immutable LF V25 evidence:

```text
3 AGREES
3 DISAGREES
18 geometry-only
0 NO_GEOMETRIC_CONTACT
geometry_modified = false
```

RF/RH/LH remain geometry-only until later hardware validation.

## Final tests

```text
241/241 geometry discovery PASS
58/58 adjacent offline calibration PASS
299/299 total PASS
```

Corrected C/D were not rerun after finalization because their canonical-semantic, G4 replay, execution and input manifests were revalidated byte-for-byte against live source with zero drift.

## Resource validation

```text
workers = 4
spawn process pool
numeric threads = 1
process CPU affinity = [0,1,2,3]
MemoryMax = 6442450944 bytes
peak = 1798238208 bytes
swap peak = 0
OOM = 0
OOM kill = 0
```

## Superseded evidence

The `2026-08-11_072224_*` bundle remains preserved as:

```text
SUPERSEDED PRE-AUDIT V5 CANDIDATE EVIDENCE
```

Its JSON artifacts, manifests and timing evidence were retained. Historical semantic hashes from that candidate are not canonical.

## Known items carried forward

These are not Geometry Compiler V5 correctness blockers, but must remain visible:

1. Pre-existing live-FK calibration-status mismatch: `DIGITAL_ZERO_CALIBRATED_AND_VERIFIED` vs loader requirement `VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION`.
2. 16 diagnostic targets outside operational URDF limits.
3. 8 external 3 mm policy results remain conservative `UNRESOLVED` lower bounds, all outside executable target domain.
4. RF/RH/LH have no hardware oracle yet.
5. Candidate-pair/AABB-survivor totals are not instrumented.
6. Jetson performance is unmeasured.
7. `_atomic_json` remains dead but intentionally retained because removing it would have invalidated corrected C/D source provenance.

## Phase transition

```text
PHASE 1B   CLOSED
PHASE 2A0  CLOSED
      ↓
PHASE 2A   Generic V25-derived full-leg calibration engine — NEXT
      ↓
PHASE 2B   final path/parking safety integration
      ↓
PHASE 2C   complete offline validation
      ↓
future hardware: RF -> RH -> LH
```

Phase 2A must preserve:

```text
MODEL / GEOMETRY
!= PATH
!= HARDWARE EVIDENCE
!= SAFETY POLICY
!= RUNTIME AUTHORIZATION
```

No hardware, Station motion, serial probing or EEPROM activity was used to close Phase 2A0.
