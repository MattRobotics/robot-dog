# MATDOG Milestone I foundation acceptance

## Result

```text
MILESTONE_I_0=COMPLETE
MILESTONE_I_1=COMPLETE
FOUNDATION_CLASSIFICATION=FOUNDATION_PASS_WITH_LIMITS
INDEPENDENT_REVIEW_REMEDIATED=true
REMEDIATION_CLASSIFICATION=REMEDIATION_PASS_WITH_LIMITS
HARDWARE_USED=false
STATION_STARTED=false
MILESTONE_I_2_STARTED=false
```

All blocking findings from the 2026-07-31 independent foundation review are
addressed. The remaining `UNKNOWN` and `DECISION_REQUIRED` rows are intentional
future evidence/decision boundaries and require no change before this
foundation can be reviewed for merge. This acceptance does not authorize a
merge, hardware work, or Milestone I.2.

FOUNDATION_METRICS_JSON: {"claims": 51, "conflicts": 21, "decisions": 9, "frames": 19, "hardware_validated_profiles": 2, "joints": 12, "limits": 50, "materialized_frames": 17, "mutation_cases": 28, "operational_safe_limits": 0, "profiles": 24, "servos": 12, "sources": 66, "unit_tests": 29, "unresolved": 11}

## Frozen references

- `MattRobotics/robot-dog` base:
  `a6dc1184f56956dad696b3bcc24d74f375edb5b7`
- `MattRobotics/norma-core` fork main:
  `32e3222c87016b7f5d7c1c1da497a4cea3e7b80a`
- `MattRobotics/norma-core` draft PR #4 head:
  `b06cc2bf2e36fb5bbaae12e48c5998c7668862ef`
- `MattRobotics/xgolite-low-level-reconstruction` tag
  `xgolite-static-closure-h2-2026-07-30` at
  `a1b34a8594e5bc76c76b1e3ddf89a3aef2b98298`
- `norma-core/norma-core`: external upstream reference only; its main and PR #4
  are not MATDOG gates.

## Machine-readable inventory

| Item | Count/state |
|---|---|
| Source rows | 66: 37 robot-dog; 15 NormaCore fork; 13 XGoLite; 1 external upstream |
| Claims | 51 |
| Claim classes | 11 `MATDOG_VERIFIED`; 3 `MATDOG_DERIVED`; 13 `HARDWARE_OBSERVATION`; 7 fork-main facts; 1 experimental PR; 2 generic references; 2 XGo architecture; 4 superseded; 5 unknown; 3 decision required |
| Joints / servo mappings | 12 / 12 |
| Frames | 19: 17 materialized; `world` and `ground_plane` planned |
| Contact profiles | 24 software-ready; 2 hardware-validated; 22 hardware-pending |
| Limit rows | 50: 24 URDF; 2 mechanical contact; 24 operational-safe unknown |
| Operational safe limits | 0 proven |
| Conflicts | 21 total: 14 open; 7 closed by source/status separation |
| Decisions | 9 open |
| Unresolved | 11: 8 `UNKNOWN`; 3 `DECISION_REQUIRED` |

The claim total remains 51 only because remediation removed the unsupported IMU
claim and split one composite generic claim into separate SO101 and ElRobot
claims. The distribution changed from 6 to 5 `UNKNOWN` and from 1 to 2
`NORMACORE_GENERIC_REFERENCE`; it was not held constant as an acceptance
constraint.

The only physically validated contact profiles are
`LF_UPPER_M12_MIN` (1443/1443 tick; spread 0) and
`LF_UPPER_M12_MAX` (3443/3442 tick; spread 1). They remain mechanical-contact
observations and are not operational safe limits.

## Provenance remediation

- `C-FRAME-BASE` now cites the pinned REV00 ADR section that explicitly
  defines X-forward, Y-left and Z-up.
- The unsupported `C-UNKNOWN-IMU` claim was removed. `D-IMU` and `U-IMU`
  retain the proof gap without treating absence as positive evidence.
- The composite generic calibration claim is split into
  `C-GENERIC-SO101` and `C-GENERIC-ELROBOT`, each with its own pinned file and
  locator.
- Historical M11 evidence remains byte-for-byte unchanged and hash-pinned.
  The manifest marks it `TEXT_ONLY_NONPARSEABLE` /
  `PINNED_HUMAN_TEXT`; `C-DIR-M11` uses the precise lines 17-20 locator.

## Validation

`foundation_expectations.json` freezes repository pins, exact registry ID
sets, canonical counts/distributions, conflict/decision/unresolved states,
critical provenance, joint directions, profile derivation constants and these
document metrics.

The standard-library validator:

- compares every joint topology, origin/RPY, axis, limit, effort, velocity,
  servo ID and custom URDF hardware tag to REV00 with explicit numeric
  tolerance;
- compares all 17 materialized frames to their source joint/link while keeping
  `world` and `ground_plane` planned and non-materialized;
- regenerates all 24 NormaCore profiles from the pinned source hash and
  formula constants, then compares order, roles, directions, allowed motors,
  prerequisites, reverse restore order, home, limit, guard and evidence state;
- enforces exact source/claim/conflict/decision/unresolved identities,
  distributions and statuses, mandatory source metadata and source-class
  compatibility;
- verifies acceptance and handoff metrics against the machine-readable
  baseline.

Invocation:

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPYCACHEPREFIX=/tmp/matdog-milestone-i-remediation-pycache \
python3 06_Software/Matdog_Core/milestone_i/validate_foundation.py --check
```

The suite contains 29 tests: one valid baseline and 28 invalid mutations. It
retains the original 14 cases and adds all eight independent-review false
PASS mutations plus provenance, M11 parse-state, source compatibility, planned
frame, direction-conflict, acceptance and handoff divergence cases. Every
invalid mutation must return at least one validator error.

## Limits and excluded scope

The foundation does not choose world placement, bus-owner deployment,
controller placement, real-time rates, IMU/estimator, current calibration,
safe limits, collision reduction, or first-stand envelope. No gait engine,
operational IK, new FK, planner, firmware, serial runtime adapter, motion, or
hardware calibration has been started. No source or historical evidence file
was retroactively modified.

A new independent review is still required before merge. The PR must remain
draft. Milestone I.2 and hardware activity remain outside this acceptance.
