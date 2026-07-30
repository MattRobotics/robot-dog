# MATDOG Milestone I foundation acceptance

## Result

```text
MILESTONE_I_0=COMPLETE
MILESTONE_I_1=COMPLETE
FOUNDATION_CLASSIFICATION=FOUNDATION_PASS_WITH_LIMITS
HARDWARE_USED=false
STATION_STARTED=false
MILESTONE_I_2_STARTED=false
```

The foundation is complete and internally validated. Correctly registered
`UNKNOWN` and `DECISION_REQUIRED` items prevent an unqualified pass but do not
block further authorized offline work.

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
| Claim classes | 11 `MATDOG_VERIFIED`; 3 `MATDOG_DERIVED`; 13 `HARDWARE_OBSERVATION`; 7 fork-main facts; 1 experimental PR; 1 generic reference; 2 XGo architecture; 4 superseded; 6 unknown; 3 decision required |
| Joints / servo mappings | 12 / 12 |
| Contact profiles | 24 software-ready; 2 hardware-validated; 22 hardware-pending |
| Operational safe limits | 0 proven; 24 explicitly unknown |
| Conflicts | 20 total: 14 open; 6 closed by source/status separation |
| Decisions | 9 open |
| Unresolved | 11: 8 `UNKNOWN`; 3 `DECISION_REQUIRED` |

The only physically validated contact profiles are
`LF_UPPER_M12_MIN` (1443/1443 tick; spread 0) and
`LF_UPPER_M12_MAX` (3443/3442 tick; spread 1). They remain mechanical-contact
observations and are not operational safe limits.

## Validation

The standard-library validator parses all ten CSV registries and the canonical
URDF; checks local hashes and paths; validates repository pins; enforces the
12-joint and 12-servo model; checks all 24 profiles; and prevents the defined
promotion errors. Invocation:

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPYCACHEPREFIX=/tmp/matdog-milestone-i-pycache \
python3 06_Software/Matdog_Core/milestone_i/validate_foundation.py --check
```

Result: `PASS` with 66 sources, 51 claims, 12 joints and 24 profiles.

Fourteen offline mutation tests pass: valid baseline; missing joint; duplicate
servo; invalid direction; invalid classification; invalid confidence; empty
source ref; improper UNKNOWN promotion; mechanical contact used as safe;
XGo source promoted to MATDOG; nonexistent URDF joint; missing conflict;
decision without evidence; and incorrect local checksum.

## Limits and excluded scope

The foundation does not choose world placement, bus-owner deployment,
controller placement, real-time rates, IMU/estimator, current calibration,
safe limits, collision reduction, or first-stand envelope. No gait engine,
operational IK, new FK, planner, firmware, serial runtime adapter, motion, or
hardware calibration has been started. No source or historical evidence file
was retroactively modified.

The next proposed operation is an independently authorized offline I.2 design
gate after human review of the draft PR. This document does not authorize it.
