# G1 motion kinematics core

This module maps semantic URDF joint angles to `foot_link` poses and solves
position IK for LF, RF, RH, LH. It is pure C++17 with fixed storage, no heap,
exceptions, mutable global state, transport or controller integration.
`LegKinematics.cpp` and `LegInverseKinematics.cpp` can be linked directly on
host or embedded targets. Only the standard math library is needed.

## Contract

- Frame: `base_link`, right handed, +X forward, +Y left, +Z up.
- Units: metres and radians, `double` arithmetic.
- Order: LF, RF, RH, LH; joints: hip, upper_leg, lower_leg.
- FK returns the origin and orientation of the actual `*_foot_link`.
- IK accepts that same origin in `base_link`; orientation is unconstrained.
- A ground contact point is a separate geometric quantity. Body/world and
  foot-contact transforms must be resolved before using this API.
- Every FK input and IK solution obeys the canonical URDF joint limits.
  These are geometric limits, not downstream operational safety limits.
- `q=0` is a reference pose. Front/rear foot origins differ by 20 mm in Z;
  it is not a flat ground stance.
- Read IK output joints only when `status == OK`. Failure fields are default
  values, not a fallback pose. `OK` establishes kinematic reachability only.

No state machine, gait scheduler, stand interpolation, collision checker,
IMU access, stabilization tuning or actuator integration is introduced.
Calibration and operational authority remain downstream.

## One geometry source

`LegGeometryData.h` is generated from the canonical URDF through the existing
Geometry V5 `load_robot_geometry_model` parser. The exporter is
`06_Software/Matdog_Core/kinematics/matdog_motion_geometry_export.py`.
It emits only chain origins, limits and the source SHA-256. It does not use
the compact calibration profile as a kinematic model.

The exporter verifies topology, fixed-foot binding, zero origin rotations,
X/Y/Y joint axes, the exact zero components needed by the reduction,
nondegenerate dimensions, and joint intervals inside (-pi, pi). It preserves
each leg's own origins and limits. Any unsupported URDF change fails before
export; geometry changes require review and oracle validation.

From the repository root:

```sh
python3 06_Software/Matdog_Core/kinematics/matdog_motion_geometry_export.py --check
# After an intentional geometry change, regenerate without --check and review.
bash 05_Firmware/MATDOG_Controller/scripts/tests/run_motion_host_tests.sh
```

The normal `run_host_tests.sh`, and thus `scripts/static_audit.py`, also run
this gate. Tests need Python 3 and PyYAML, already used by the contact oracle,
and a C++17 compiler. They run entirely offline.

## Exact analytic reduction

The verified chain is

```text
T_hip Rx(h) T_upper Ry(u) T_lower Ry(l) T_foot
```

Let `a = -lowerOrigin.z`, `b = hypot(footOrigin.x, footOrigin.z)`,
`d = upperOrigin.y + footOrigin.y`, and
`phi = atan2(footOrigin.x, -footOrigin.z)`.
After removing the hip translation, the foot point satisfies

```text
x = -a sin(u) + foot.x cos(u+l) + foot.z sin(u+l)
y = d cos(h) - v sin(h)
z = d sin(h) + v cos(h)
v = -a cos(u) - foot.x sin(u+l) + foot.z cos(u+l)
```

Two hip candidates follow from
`h = atan2(z,y) +/- acos(d / hypot(y,z))`.
For each, compute `v = -sin(h)y + cos(h)z`, then

```text
cos(beta) = (x*x + v*v - a*a - b*b) / (2*a*b)
beta = +/- acos(cos(beta))
u = atan2(-x,-v) - atan2(b*sin(beta), a+b*cos(beta))
l = beta + phi
```

All four candidates are considered in a fixed order. Equivalent angles are
normalized to (-pi, pi), checked against that leg's limits, and verified by
FK. Coincident solutions are deduplicated. Among valid solutions the smallest
squared distance to the supplied in-limit seed wins, with first-candidate tie
breaking. The default seed is zero. A caller following a trajectory should
supply the preceding solution; no history is stored inside the module.

This retains the URDF's 107 mm / 49.9 mm distal components and ±1.5 mm lateral
foot eccentricity. It is not a nominal two-link length approximation.
The solver uses no Jacobian inversion or convergence budget. At hip branch
coalescence or planar extension/folding it evaluates the same finite formulas;
there is no division by a Jacobian determinant. The exporter excludes zero
lateral radius and equal planar lengths (the fully folded indeterminate case).

## Status and numerical policy

| Status | Reachability | Meaning |
|---|---|---|
| `OK` | REACHABLE | In-limit solution passes FK residual check |
| `INVALID_INPUT` | UNKNOWN | Invalid leg, nonfinite data or tolerance |
| `JOINT_LIMIT` | UNKNOWN | Supplied seed violates a URDF limit |
| `UNREACHABLE_GEOMETRY` | UNREACHABLE | Outside exact geometric domain |
| `UNREACHABLE_LIMITS` | UNREACHABLE | Geometric branches exist; none meet limits |
| `NUMERICAL_FAILURE` | UNKNOWN | In-limit candidates fail numerical FK verification |

For FK, `JOINT_LIMIT` describes the supplied angles. Geometry and domain checks
use `1e-12` metre or dimensionless roundoff guards as appropriate; angle endpoint
normalization/deduplication uses `1e-12` rad. These guards only absorb floating
point roundoff. Every returned angle is clamped inside the exact URDF interval,
and every successful solution must meet the requested residual tolerance.
The default is `1e-9 m`; accepted option range is `[1e-12, 1e-3] m`.
Nonfinite inputs are rejected before trigonometry. Large finite targets are
rejected with overflow-safe norms before squaring.

Determinism means fixed candidate order, no state and identical results for
repeated inputs in the same binary/math environment. Bitwise equivalence
across different CPU/libm implementations is not claimed. ESP32-S3 latency,
stack usage and floating point cost remain unmeasured; no device build or
hardware benchmark is part of G1.

## Validation scope

The C++ suite covers q=0 goldens, all four legs, exact joint boundaries,
symmetry, front/rear offsets, singular/near-singular points, determinism,
nonfinite input, unreachable domains and joint-limit exclusion.

`test_motion_oracle.py` uses the existing Python FK, LF DLS IK and quadruped
contact DLS. It compares full FK position/orientation and checks C++ IK results
through canonical Python FK, including deterministic random samples spanning
the entire limit box. It also mutates unsupported URDF features to prove the
exporter fails closed and checks the core's include boundary.

C4-A and all 51 C4-C frames are replayed using their archived angles to obtain
foot-origin targets from canonical FK. C++ IK receives zero or the preceding
frame's seed. Its recovered angles are evaluated by the existing Python foot
contact model at each archived body pose; target contact, achieved contact and
nominal strip mode must match. Python contact IK is independently rerun on
C4-A and C4-C start/middle/end with perturbed seeds.

This validates the kinematic/contact meaning of those references. It does not
implement contact-locked trajectory generation in C++, recertify mesh collision
policy, or authorize execution. The existing C4-C policy and archived collision
evidence remain unchanged. Direct q=0-to-stand interpolation remains rejected.
