# MATDOG calibration and limit semantics

Status: `CURRENT_CANONICAL` vocabulary for Milestone I.1.

## Non-interchangeable quantities

| Quantity | Meaning |
|---|---|
| Raw encoder | Physical unoffset tick reading captured at a named time. |
| Circular delta | Local mathematical shortest difference only; never a wrapped command. |
| URDF q | Joint coordinate in radians under the URDF axis and encoder-to-q sign. |
| Visual zero | Historical visually established reference, superseded as command basis by digital recenter. |
| Digital zero | Displayed 2048 q=0 after verified EEPROM Position Offset readback. |
| Position Offset | Signed EEPROM calibration value from the completed 2026-07-10 procedure. |
| Mechanical contact | Observed physical end contact for one profile and one robot state. |
| URDF limit | Model constraint derived into a profile tick; not physical proof. |
| Travel guard | Software stop beyond the modeled limit; not a safe operating limit. |
| Prerequisite pose | Geometry-validated static configuration used to create probe clearance. |
| Calibration corridor | Profile-scoped command allowlist/corridor in the native calibrator. |
| First-stand envelope | Future configuration-level bound; currently absent. |
| Operational safe limit | Future evidence-backed operating boundary; all 24 sides absent. |
| Collision-dependent limit | Configuration-level constraint that cannot be reduced to one joint limit. |

## Current contact state

Pinned NormaCore main generates 24 profiles. Their implementation parameters
and evidence status are in `calibration_registry.csv`. The validator rebuilds
all 24 from the `N-MATDOG` source hash, leg/joint/side order, home tick 2048,
joint delta constants, direction map, 64-tick guard and baseline formulas,
allowed motors, prerequisites and reverse restore order recorded in
`foundation_expectations.json`; it then compares every derived field.

- `LF_UPPER_M12_MIN`: coarse 1443 tick, fine 1443 tick, spread 0; URDF limit
  1451 tick. `validated` only as a mechanical contact.
- `LF_UPPER_M12_MAX`: coarse 3443 tick, fine 3442 tick, spread 1; URDF limit
  3442 tick, travel guard 3506 tick. `validated` only as a mechanical contact.
- Remaining 22: implementation `software-ready`, physical state
  `hardware-pending`.
- Operational safe limits: 0 of 24 demonstrated.

Neither M12 result is extended to another side or joint. The contact current
is diagnostic in the pinned main algorithm; position/velocity/persistence
establish normal contact and a high current is a hard abort. No generic
current unit is asserted for MATDOG.

## Prerequisite and travel semantics

Hip probing uses same-leg upper q=+0.872664625997 rad and lower q=0. Lower
probing uses hip q=0 and upper q=+1.570796326795 rad. Upper probing uses hip
and lower q=0. Front-leg profiles additionally park the ipsilateral rear upper
joint at +0.523598775598 rad: LF parks LH; RF parks RH. These are derived
clearance poses, not joint limits or general safe poses.

The registry records profile order, joint role, allowed motor IDs,
prerequisite motor/target pairs, reverse restore order, home/visual zero,
modeled limit, travel guard, software/hardware states, mechanical evidence and
an explicit false operational-safe flag. The native profile order is LF, RF,
RH, LH; within each leg upper, hip, lower; within each joint MIN then MAX. No
such profile may run under this milestone.
