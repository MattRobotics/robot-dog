# G3 startup acquisition and timed stand

G3 is an offline, fixed-storage C++17 model. It produces semantic URDF radians,
rad/s and rad/s². It has no actuator authority, hardware inputs, scheduler or
wall clock. G1 foot-origin IK and G2 physical contact IK retain their contracts.
See [CONTACT_STAND.md](CONTACT_STAND.md) for the exact cylinder/contact model.

## Startup contract

`evaluateStartup` explicitly distinguishes the four starting conditions:

| Physical condition | Result | Generated acquisition target |
|---|---|---|
| A: suspended/unloaded reference pose | `SUSPENDED_PATH_UNPROVEN` | None |
| B: feet placed on floor, pose/support unverified | `FLOOR_ACQUISITION_UNPROVEN` | None |
| C: verified canonical low C4 stance | `READY` only after all checks | Low-stance hold |
| D: unknown pose | `UNKNOWN_POSE` | None |

At q=0 the front physical contact reference is at Z=-0.0934 m in base_link;
the rear is at Z=-0.1134 m. A parallel body cannot place both planes on the
same flat floor. Existing C4 policy also rejects direct q=0 joint interpolation
because lower-leg geometry penetrates the floor at intermediate poses.
Neither a suspended release/contact sequence nor arbitrary floor acquisition
has validated mechanics or support evidence. G3 therefore provides no moving
acquisition from A, B or D. It does not claim that a different tilted pose is
impossible; that pose and an acquisition path are unproven and unsupported.

For C, the caller must supply valid semantic joint observations, body pose and
six explicit evidence assertions: fresh observation, stationary robot, four
confirmed supporting feet, flat ground, reviewed collision clearance and
reviewed support/load. All assertions default to false. These are external
evidence inputs, not sensors, verified physical measurements or permission to
actuate. G3 cannot establish their truth on a robot.

The observation must match the canonical low stance: identity body orientation,
translation (0,0,0.100) m in the chosen world frame, canonical physical footprint,
nominal contact strips and the same in-limit low-stance joint solution. World
origin/yaw are chosen to describe this footprint. Numerical agreement guards
are 1e-9 m, 1e-8 rad and 1e-10 rotation entries. These are model tolerances;
they are **not approved physical measurement tolerances**. Bad evidence, pose,
joint limits, low-stance agreement and contact agreement have distinct statuses.
Failure carries no valid target.

Before a future first autonomous physical stand, the robot must already be
stationary in this verified low C4 support configuration, with the actual four
feet bearing the intended support, the body parallel to the actual flat floor,
the physical contacts at world Z=0, the correct footprint and semantic joints,
and collision clearance and support for the actual load established. The C4-E
base-origin COM proxy is not a measurement of the actual COM. A future physical
observation contract and separately approved actuator/safety execution layer
must establish eligibility; G3 grants none. **q=0 as an autonomous ground-supported
start is NO / NOT PROVEN.**

## Geometry and support segments

1. At t=0: `ACQUISITION_HOLD`, the verified low stance, no intended motion,
   four physical contacts locked. No suspended or free-space segment is emitted.
2. For 0<t<=T: `CONTACT_LOCKED_RISE`, the same four contacts at Z=0 and identity
   body rotation; body height increases from 0.100 to 0.150 m along the C4 path.

`sampleStandPath` exposes the continuous G2 geometric path at progress s. It
uses the same solver, footprint, limits and full physical contact residual check
as G2. It does not interpolate joints. G2's original 51 geometric samples and
the C4-A final target remain unchanged. Each successive timed sample supplies
the preceding valid joint solution as its IK seed. A branch change invalidates
the output and stops sampling; it is never silently accepted.

`BodyPose` stores world-from-base translation and a proper 3x3 rotation:

```text
p_world = translation + R * p_base
p_base  = transpose(R) * (p_world - translation)
```

Generic point transforms accept checked SO(3) rotations. The world contact IK
wrapper accepts only rotations preserving +Z, including yaw. Roll/pitch are
explicitly rejected as `UNSUPPORTED_ORIENTATION`; arbitrary ground-normal IK
requires a later extension. Contact targets are transformed into base_link
then passed to G2 contact IK, never to G1 foot-origin IK directly. No distal
eccentricity or orientation-dependent cylinder offset is removed.

## Time and derivatives

The caller supplies total duration T and interval count N, or a sample period
and N through `timingFromPeriod`. Samples are at t_i=i*T/N, i=0..N. This is
offline sampling, not an execution timer. T must be positive and numerically
representable for the derivative calculations. N is bounded to [2,1000000]
as a computational guard, not an actuator-rate limit. No physical duration is
selected by default.

For u=t/T:

```text
s = 10u^3 - 15u^4 + 6u^5
ds/dt = 30u^2(1-u)^2 / T
d2s/dt2 = 60u(1-u)(1-2u) / T^2
height = low + (stand-low)*s
```

The path is monotonic. Both endpoint velocities and accelerations are zero;
constant holds joined to it are C2. Jerk is not reported or constrained.
Joint rates are analytic derivatives of the exact G2 contact map, not frame
deltas divided by an assumed clock. With contact Jacobian J and directional
Hessian H along q_s:

```text
J*q_s  = (0,0,-height_span)
J*q_ss = -H[q_s,q_s]
q_dot  = q_s * s_dot
q_ddot = q_ss * s_dot^2 + q_s * s_ddot
```

The small fixed-size linear solver uses partial pivoting, rejects pivots below
1e-10 and rejects nonfinite results. This is a numerical derivative guard, not
a mechanical safety margin. Singular/unrepresentable results invalidate the
entire frame. At identical normalized times q is unchanged, q_dot scales as
1/T and q_ddot as 1/T². Tests compare 2, 5 and 10 seconds as mathematical test
parameters, not physical recommendations or imported C4-D actuator limits.

Every timed sample wraps the existing valid/invalid 12-joint `JointTargetFrame`
and adds timestamp, progress, phase, body pose, velocity and acceleration.
Contact residual, branch, joint-limit margin, per-frame joint delta and contact
drift remain available through the embedded geometric sample. Aggregate metrics
report duration, sample count, per-joint sampled peak absolute velocity and
acceleration, minimum URDF limit margin, residual, drift and branch changes.
**Sampled peaks are not certified continuous maxima or hardware eligibility
limits.** A later layer must select sampling and physical limits appropriately.

## Lifecycle ownership

Use `StandTransition` to coordinate the pure lifecycle. `ENABLE` gives IDLE.
`begin(observation,timing)` enters STAND_TRANSITION only after startup READY and
successful path initialization. The first emitted frame is the acquisition hold.
Only successful emission of every expected frame, including final s=1, reaches
STAND. This means completed offline generation, not physical execution feedback.

Public `BEGIN_STAND` and `STAND_COMPLETE` events are now rejected. Their state
changes are private operations accessible only to the coordinator. Other public
events retain G2 meanings. STOP cancels sampling and enters STOPPING; STOP_COMPLETE
returns IDLE; FAULT/DISABLE cancel and go OFF. A sample failure goes OFF and cannot
emit a valid fallback frame or reach STAND. Restart requires new startup evidence.
STOPPING makes no braking/deceleration claim. Standalone `TimedStand` remains a
pure offline geometry sampler; it grants no startup authorization.

## Evidence and validation

Canonical policies are in `06_Software/Matdog_Core/kinematics/`:
`MATDOG_STANCE_FRAME_CONTRACT.md`, `MATDOG_BODY_STANCE_GEOMETRY.yaml`,
`MATDOG_REST_TO_STAND_TRAJECTORY_POLICY.md`, `MATDOG_COLLISION_CONTACT_POLICY.md`,
`MATDOG_STATIC_STABILITY_SUPPORT_POLYGON.md`, and
`MATDOG_TRAJECTORY_TIMING_ENVELOPE.md`. Historical timing/preflight evidence is
context, not authority for hardware execution or imported actuator limits.

The existing host runner/static audit include G3 C++ and Python tests. The
Python test reuses canonical contact FK/IK and existing C4 mesh-ground,
knee/contact and support-polygon evaluators. It checks all 101 distinct poses
for each tested normalized path; corresponding poses at the other durations
are identical. These remain sampled offline checks, not a continuous swept
collision proof or dynamic stability certification. C4's existing lower-leg
fork review and COM-proxy limitations remain in force.

See the [G3 validation report](https://github.com/MattRobotics/robot-dog/blob/e1704719979789cd9c9f18741e4546725558797e/09_Logs/Development_Log/2026-09-28_G3_STARTUP_TIMED_STAND.md)
for numerical results, commands and scope.
