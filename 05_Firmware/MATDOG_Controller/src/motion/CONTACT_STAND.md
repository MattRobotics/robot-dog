# G2 contact-space stand foundation

G2 adds canonical physical contact geometry, contact FK/IK, a C4 stand
reference, an incremental contact-locked stand trajectory, and a minimal pure
motion lifecycle. All data are metres and semantic URDF radians. Everything
remains offline, fixed-storage C++17 with no actuator or platform dependency.

## G1 review and reuse

G1's public header and meaning are unchanged: `forwardKinematics` and
`inverseKinematics` operate on the **foot_link origin** in base_link. No G1
correctness defect was found. Its verified analytic implementation was
extracted into `LegKinematicsInternal.h` helpers to share candidate enumeration,
limits, seed selection and residual checking. G1 uses zero additional offset;
G2 uses a mathematically derived virtual chain. The G1 oracle suite passes
with unchanged numerical error maxima after extraction.

The internal helper is not an API for arbitrary robot models. Only geometry
validated by the exporters may use its zero-RPY, X/Y/Y reduction.

## Physical contact model

`FootContactData.h` is generated from
`MATDOG_FOOT_CONTACT_GEOMETRY.yaml` by `matdog_contact_stand_export.py`.
The exporter reuses the existing contact loader, URDF binding validator and
G1 geometry checks. It records the YAML hash, verifies the cylinder assumptions
needed by IK, and rejects unsupported geometry. The URDF hash remains in G1's
model. No foot-joint or cylinder offsets are copied by hand.

For the current audited rigid cylinder:

- center in foot_link: `(0,0,0.0149)` m;
- axis in foot_link: `(0,1,0)`;
- radius: `0.0149` m;
- tread width: `0.0139` m, end fillet radius: `0.002` m;
- conservative central support width: `0.0099` m;
- nominal axis tilt from ground: at most `0.034906585` rad.

Given the G1 foot pose `(R,p)`, cylinder-local center `c`, unit axis `a`,
radius `r`, and normalized ground normal `n`, contact FK computes

```text
center = p + R*c
axis   = normalize(R*a)
down   = -(n - axis*dot(axis,n)) / norm(n - axis*dot(axis,n))
reference = center + r*down
strip endpoints = reference +/- axis*(support_width/2)
```

The endpoint lowest along `n` is also returned, as are cylinder center/axis,
radial direction, tilt and nominal/edge-biased mode. A cylinder axis parallel
to the normal is an explicit degenerate result. Contact FK accepts a ground
normal in base_link, so its geometry is useful for later body/terrain work.

`reference` is the canonical Python/C4 continuous **cross-section contact
center**, not the nominal point welded to foot_link. When the finite cylinder
is tilted, `lowestCoreM` is a different point at the support-strip edge. G2
preserves this distinction instead of claiming that the cross-section center
is always the lowest finite-cylinder point. Stand requires nominal strip mode;
the generated C4 stand has hip angles zero within floating-point roundoff.
Friction, compliance and a dynamic pressure patch are not modeled.

## Exact inverse boundary

`contactInverseKinematics` targets `referenceM` for a +Z ground normal in
base_link. No rotated-ground argument is accepted. This explicit boundary
covers the parallel-body C4-A/C4-C task. Arbitrary tilted-body contact IK
requires a later extension; the general FK normal does not imply that IK
supports it.

For the verified cylinder and hip limits, `cos(h)>0`. Thus

```text
R = Rx(h) Ry(u+l)
R*c = Rx(h) Ry(u+l) (0,0,r)
down = Rx(h) (0,0,-1)
```

The exact contact chain is therefore the G1 chain with a **virtual distal
Z component `foot.z+r`**, plus an independent translation `(0,0,-r)` in the
hip-rotated frame. The real URDF model and foot-link origin stay unchanged.
The physical ±1.5 mm foot-joint Y eccentricity is included exactly once.

Let `d=upper.y+foot.y`. The two hip candidates still solve
`h=atan2(z,y) +/- acos(d/hypot(y,z))`, after removing the hip origin. For each
candidate the planar target is `(x,v+r)`, where
`v=-sin(h)*y+cos(h)*z`. The shared analytic solver then uses
`hypot(foot.x,foot.z+r)` and its corresponding phase angle. It enumerates the
planar branches, enforces URDF limits, deduplicates, and selects the in-limit
solution nearest the semantic seed. A second, full orientation/contact FK
check verifies the selected solution independently of the virtual reduction.

The branch label records signs of hip-frame contact Z and planar elbow angle.
At a branch coalescence the sign convention picks +1; it is a diagnostic,
not a physical leg or actuator direction.

Invalid input, invalid seed limits, geometric unreachability, limit exclusion,
degenerate FK, nominal-contact policy rejection and numerical failure are
explicit statuses. As in the existing Python contact IK, the optional nominal
policy checks the selected solution. `CONTACT_MODE` means the selected solution
is geometrically reachable but not accepted; its joints are not returned for
use. Only `status==OK` makes the output usable.

G1 numerical guards/tolerance bounds are retained. Default contact IK residual
tolerance is `1e-9 m`. The contact-FK perpendicular-normal degeneracy threshold
is `1e-9`, matching Python. No hardware velocity or acceleration limit exists.

## Stand definition, targets and trajectory

`StandReferenceData.h` is generated from the pinned C4-A and C4-C JSON reports
and records both hashes. It contains the physical world footprint, C4-A branch
seeds, 0.100/0.150 m body heights and 51 samples. The exporter verifies that all
archived samples have the same footprint, ground Z=0, identity body orientation,
zero body X/Y translation, linear heights and successful archived policy checks.

`generateStandTarget` solves the final C4-A physical contacts. A
`JointTargetFrame` carries all 12 semantic joint angles in LF/RF/RH/LH order,
a sample sequence and an explicit valid flag. No raw positions, actuator IDs,
authority, timing or downstream actuator binding is present. Failure returns
an invalid empty frame, including when a later leg fails after others solve.

`ContactLockedStand::initialize` solves the final reference and then the low
contact-compatible pose using the final solution as seed. `next()` emits the
ascending low-to-stand sequence, solving every sample using the preceding
valid solution. Unlike Python's numerical solver, the exact analytic solution
does not need to solve all frames descending and store/reverse them. The
contacts, body heights and intended branch are the same. Storage is constant
in the sample count. Initialization is not a trajectory from q=0 to the low
pose; that acquisition path remains outside G2.

Each sample includes signed per-joint deltas, per-joint distance to the nearest
URDF limit, contact residual, contact drift from the first emitted contacts,
and branch labels. Aggregate metrics include maximum absolute deltas for every
joint/leg, maximum residual/drift, minimum limit margin and per-leg branch-change
counts. The first emitted sample has zero delta. There are no timestamps or
inferred velocities. A successful end returns COMPLETE with no valid frame;
stop or a sample failure prevents further generation. Initialization failures
clear readiness, preventing accidental continuation of an old plan.

## Minimum motion lifecycle

`MotionStateMachine` owns only this deterministic lifecycle:

| Current state | Event | Next state |
|---|---|---|
| OFF | ENABLE | IDLE |
| IDLE | BEGIN_STAND | STAND_TRANSITION |
| STAND_TRANSITION | STAND_COMPLETE | STAND |
| STAND_TRANSITION or STAND | STOP | STOPPING |
| STOPPING | STOP_COMPLETE | IDLE |
| Any | DISABLE or FAULT | OFF |

All other pairs are rejected without changing state. The caller supplies
completion evidence: BEGIN_STAND follows successful preparation of the low
stance/trajectory, STAND_COMPLETE follows the last valid frame, and
STOP_COMPLETE follows cancellation and establishment of a semantic hold.
The state machine does not itself drive the separate trajectory generator;
callers cancel it with `stop()` on STOP, DISABLE or FAULT. G2 tests exercise this
coordination. STOPPING is not a physical deceleration or rate-controlled stop.
There is no WALK/TROT state, watchdog clock, transport, or actuator authority.

## Offline gate

```sh
python3 06_Software/Matdog_Core/kinematics/matdog_contact_stand_export.py --check
bash 05_Firmware/MATDOG_Controller/scripts/tests/run_motion_host_tests.sh
python3 05_Firmware/MATDOG_Controller/scripts/static_audit.py
```

The existing complete host runner invokes the motion gate. Tests cover G1
regression, all-leg contact geometry and inverse round trips, arbitrary-normal
FK, finite/limit/domain guards, nominal-mode rejection, singular contact FK,
C4-A and all 51 C4-C samples, seed branch continuity, exact metric calculations,
repeatability, atomic failure frames and the exhaustive state/event matrix.

The G2 Python test reuses canonical contact FK and contact DLS. It independently
refines C4-A/start/middle/end IK, checks every archived frame's branch/angles,
and reruns the existing C4 mesh-ground and knee/contact policy on all generated
poses. Archived numerical IK has up to 10 micrometres of contact error; exact
analytic targets can consequently differ slightly in joint angle. Those
bounded differences are reported, not hidden by replaying archived angles.
The C4 policy retains its expected lower-leg fork clearance review and remains
offline evidence, not full dynamic or hardware certification.
