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
