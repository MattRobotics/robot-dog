# G4 offline gait study

G3.5.1 accepted baseline: c4befbe90b3121d60ba1b9ba09dc1082364c8d88.
The G3 acquisition gate remains active. Gait begins only after its completed
canonical stand. Pose graph classifications and REST_GROUND are unchanged.

## Design decisions before implementation

- Leg order LF RF RH LH. WALK offsets 0, 1/2, 3/4, 1/4 give sequential
  RH RF LH LF swings for duty >= 3/4. This alternates rear/front support
  triangles and permits open-loop fore/aft COM positioning in all-stance gaps.
  TROT offsets 0, 1/2, 0, 1/2 pair canonical diagonals. Duty is a parameter.
- Unwrapped cycles determine anchors; fractional phase is always [0,1).
  Stance anchors are immutable world points. Body planar SE(2) motion has
  parameterized translation and yaw per cycle. No elapsed-time accumulation.
- Swing uses quintic XY interpolation and 64 h u^3 (1-u)^3 vertical lift:
  position, velocity and acceleration join stationary anchors exactly.
- Geometry is separate from period. Analytic differentiation of the exact G2
  contact map supplies semantic qdot/qddot. No execution-rate limit is assumed.
- Start prepares height/COM shift with four locked contacts, then increases
  phase speed smoothly. Stop finishes the current cycle, uses a planned final
  cycle to assemble canonical terminal contacts, and recenters to STAND.
- C++ produces kinematic candidates. Canonical full mesh, contact and CAD/URDF
  COM checks remain explicit offline assessments. A candidate is not a
  collision-certified trajectory. TROT dynamics remain NOT YET PROVEN.
- G2 +Z contact reference is retained exactly. Inclined-cylinder reference
  locking does not prove finite-mesh ground clearance. The G3.5 one-micrometre
  collision tolerance is preserved, and such failures are reported.

## XGO evidence and transfer boundary

Read-only archive: origin/main a1b34a8594e5bc76c76b1e3ddf89a3aef2b98298.
The original checkout with unrelated untracked files is untouched.

G/G2/G2.1 establish phase slots, mode dispatch, command/state separation,
Cartesian writer arrays, interpolation and convergence on an IK component.
G2 supersedes G's historical address/timing uncertainties: linked addresses
are historical flat addresses minus eight; a requested two-millisecond task
delay is proven, actual cadence/jitter are not. G2 formulas and continuity
rows supersede G's partially simplified writer formulas. G2.1 host names
(trot/walk/high_walk/slow_trot) are documentary VERIFIED; end-to-end physical
mode/leg binding remains CORROBORATED. Physical signs/units remain UNKNOWN.

Mode 1 has quarter-spaced slots; mode 0 pairs slots 0+2 and 1+3. MATDOG
independently uses its canonical leg names and support analysis. XGO increments
phase per invocation and drops overshoot at strict `phase > period`; MATDOG
uses supplied time and mathematical modulo. XGO interpolation is linear;
MATDOG uses C2 curves. Mark-time and high-walk have explicit piecewise writers;
mode 2 retains X in some ranges and integer rounding can leave endpoint
residuals. They are not templates for MATDOG physical trajectories.

No recovered, fully bound locomotion command-expiry or phase-consistent
stand-stop proof was found in the inspected G/G2/G2.1 material. Host read
response timeouts are not locomotion watchdogs. The MATDOG freshness and
start/stop contracts are independent design choices. No crawl geometry or
lower-controller physical safety guarantee is inferred from names.

No XGO geometry, joint angles, zeros, signs, limits, stride, lift, duty,
period, gain, height or rate is transferred. No XGO code is executed.

## Runtime contracts

`Locomotion` owns a `StandTransition`. Its `LocomotionState` includes G4's
GAIT_START/WALK/TROT while the accepted G3 `MotionState` and startup source
files remain byte-identical. Public completion events cannot establish STAND.
DISABLE/FAULT cancels semantic output; it is not a physical stop command.

A start has one period of four-contact preparation, then two periods of
quintic phase-speed acceleration (one geometric cycle). A stop finishes the
current cycle and uses two periods to decelerate through one final cycle.
Only future swing endpoints are planned to meet the canonical terminal stance;
an in-flight swing is never retargeted. One final period recenters body height
and fore/aft shift with four contacts locked. A stop during preparation returns
through a four-contact hold. A new start retains the terminal world pose.
The preparation and terminal joins have zero joint velocity and acceleration.

The watchdog requires finite monotonic caller time, finite non-future command
stamps, increasing sequence numbers and nondecreasing stamps. Equality at the
configured expiry is fresh; strictly greater age is stale. Zero, stale,
invalid or changed mode/geometry/period requests a semantic stop. A changed
command is not applied mid-cycle; restart must be explicit from STAND.
Invalid time produces an invalid frame. The phase range of one million cycles
is a floating-point/resource boundary, not an actuator-rate limit.

All defaults are study parameters, not approved physical operation values.
`solveGait` yields only a kinematic candidate; `assessGait` consumes explicit
collision/contact/support results. Full triangle and solid-containment checks
are performed by the offline Python model. TROT's static support margin remains
recorded but does not certify or reject its unproven dynamics.

## Reproduction

Use the pinned G3.5 Python environment (`pose_audit/requirements.txt`). From the
repository root:

```sh
python 06_Software/Matdog_Core/gait_audit/xgo_evidence.py
python 06_Software/Matdog_Core/gait_audit/oracle.py
python 06_Software/Matdog_Core/gait_audit/survey.py --stage screen
python 06_Software/Matdog_Core/gait_audit/survey.py --stage full
python 06_Software/Matdog_Core/gait_audit/refine.py
python 06_Software/Matdog_Core/gait_audit/lifecycle_audit.py
python 06_Software/Matdog_Core/gait_audit/render.py
```

Screen results explicitly omit self-collision checks and reject at the first
failure; their metrics cover only that prefix. Full results test every saved
sample against all 17 meshes and all 120 nonadjacent pairs. The 16 assembly
adjacency exclusions remain unchanged. A passing finite set does not prove
continuous collision clearance or physical contact loads. Finer sampling can
expose failures missed by the coarse screen. Envelope bounds are maximum tested
passing values, not proven global limits or a continuous box of valid commands.
