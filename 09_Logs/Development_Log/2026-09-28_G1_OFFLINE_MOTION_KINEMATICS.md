# G1 — offline motion kinematics core

## Result

G1 is implemented and its offline gates pass. The core provides canonical
four-leg FK and analytic position IK, with explicit identities, statuses,
reachability, finite/domain guards and URDF joint limits. No gait scheduler or
actuator integration was started.

The broader existing Python suite has a reproduced baseline calibration-loader
failure, recorded below. It does not affect the G1 oracles or host gate.

## Repository checkpoint

| Item | Observed value |
|---|---|
| Gait worktree | `/home/matteo-manicardi/MATDOG/worktrees/robot-dog-gait-engine` |
| Local gait branch | `feat/gait-engine-offline-v1` |
| Fetched `origin/main` base | `bff2ae4517d88fc437213f3bb53427b800b784e7` |
| Calibration remote branch | `origin/feat/calibration-motion-full-cal-v1` |
| Calibration SHA | `2603ee789e7619b79c37e5f8bdadfbb06da8761e` |
| Calibration divergence from main | 0 main-only / 9 calibration-only commits |
| Canonical XGO repository | `/home/matteo-manicardi/robotics-reverse/xgolite-low-level-reconstruction` |
| XGO `origin/main` used for evidence | `a1b34a8594e5bc76c76b1e3ddf89a3aef2b98298` |
| XGO checkout HEAD observed | `72234e03739727356470834ab55daee721ed1864` |

Both origin remotes were fetched with pruning before implementation. MATDOG's
primary checkout was clean on main. Neither the gait branch nor its worktree
existed, so a new worktree was created from live `origin/main`. The local gait
branch has no upstream and nothing was pushed.

XGO's canonical checkout is on the historical research branch, with pre-existing
untracked `analysis/`, `references/intake/`, and `tools/`. It was left unchanged.
The G, H.2 transfer-boundary and H closeout documents were read with
`git show origin/main:<path>` in that repository. No historical worktree was
used, and no XGO physical constants entered the implementation.

## Files

Added under `05_Firmware/MATDOG_Controller/src/motion/`:

- `LegKinematics.h`: semantic data types and FK/IK API.
- `LegGeometryData.h`: generated per-leg origins, limits and URDF hash.
- `LegKinematics.cpp`: canonical four-leg FK and foot orientation.
- `LegInverseKinematics.cpp`: bounded analytic position IK.
- `README.md`: coordinate contract, derivation, statuses and validation scope.

Added under `05_Firmware/MATDOG_Controller/scripts/tests/`:

- `test_motion_kinematics.cpp`: 1,228 host assertions.
- `motion_oracle_driver.cpp`: host text adapter for differential validation.
- `test_motion_oracle.py`: existing Python oracle and C4 comparison.
- `run_motion_host_tests.sh`: lean compiler/oracle gate.

Also added:

- `06_Software/Matdog_Core/kinematics/matdog_motion_geometry_export.py`.
- `09_Logs/Development_Log/2026-09-28_G1_OFFLINE_MOTION_KINEMATICS.md` (this report).

Modified only:

- `05_Firmware/MATDOG_Controller/scripts/tests/run_host_tests.sh`: invokes the
  new motion gate, which is therefore also covered by `static_audit.py`.

## Architecture and geometry

The core is `matdog::motion`, C++17, with fixed-size storage and double precision.
It uses no heap allocation, exceptions, mutable globals or platform framework.
It outputs hip/upper/lower angles in URDF radians. No calibration metadata,
raw positions, authority logic or transport ownership is duplicated.

The model exporter reuses Geometry Compiler V5's topology/parser and emits a
small motion-only subset from the canonical URDF. It verifies all assumptions
needed by the analytic equations and fails on unsupported transforms or axes.
The compact calibration profile is not used as an FK model.

Verified hip origins in metres:

| Leg | X | Y | Z |
|---|---:|---:|---:|
| LF | 0.1125 | 0.0475 | 0.0465 |
| RF | 0.1125 | -0.0475 | 0.0465 |
| RH | -0.1125 | -0.0475 | 0.0265 |
| LH | -0.1125 | 0.0475 | 0.0265 |

The full distal offsets are retained: foot-joint X=0.107 m, Z=-0.0499 m,
Y=-0.0015 m on LF/LH and +0.0015 m on RF/RH. Actual moving joint names are
`*_hip_joint`, `*_upper_leg_joint`, `*_lower_leg_joint`. The fixed
`*_foot_joint` leads to `*_foot_link`.

IK targets the foot-link origin in base_link. The exact X/Y/Y chain admits
four analytic branch candidates, verified through FK and filtered by URDF
limits. The nearest valid seed branch wins deterministically. This avoids
numerical Jacobian estimation, iterative convergence ambiguity and variable
iteration cost. The existing Python DLS remains an independent reference.
The derivation and degeneracy exclusions are in the module README.

Geometric unreachability, joint-limit unreachability, invalid input and
numerical verification failure have distinct statuses. Singular branch
coalescence uses bounded formulas without matrix inversion. Repeated-run
identity is tested; cross-platform libm bitwise identity is not claimed.
Embedded latency and stack cost have not been measured.

### Source hashes (SHA-256)

| Artifact | Hash |
|---|---|
| Canonical `03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf` | `3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59` |
| `MATDOG_FOOT_CONTACT_GEOMETRY.yaml` | `43dc72abdc132ef2492d383efe4ed12dfe2e18cf17705e69fc21f4166c4f0966` |
| `2026-07-08_175245_C4A_offline_safe_stand_candidate.json` | `221b6551ccf1bf36cd10b2b4136da38d1d2a73af18d65e0f3486de1faa72991b` |
| `2026-07-08_190405_C4C_contact_locked_rest_to_stand_trajectory.json` | `4b667a5995e55122b9578c39042914a373aa6bb4e0be84de11381e7f2398b7b2` |

C4 reports do not embed a canonical URDF hash. Their provenance was checked
by replaying their joint angles with the current canonical FK/contact model
and comparing the recorded achieved contacts (maximum drift below).

## Validation executed

Environment: g++ 13.3.0, Python 3.12.3, Ubuntu host.

Repository-root commands:

```sh
python3 06_Software/Matdog_Core/kinematics/matdog_motion_geometry_export.py --check
python3 05_Firmware/MATDOG_Controller/scripts/static_audit.py
PYTHONPATH=06_Software/Matdog_Core/kinematics python3 -m unittest discover \
  -s 06_Software/Matdog_Core/kinematics/tests -p 'test_*.py'
git diff --check
```

`static_audit.py` invoked the complete existing `run_host_tests.sh`, which
invoked `run_motion_host_tests.sh`. The latter compiled the production motion
translation units and both test executables with
`-std=c++17 -Wall -Wextra -Werror -O1 -fno-exceptions -fno-rtti`, then ran the
C++ assertions and `test_motion_oracle.py` against the compiled driver.
The C++ unit executable and differential script were also run directly during
implementation. Initial test-vector errors were corrected before these gates.

Additional compilation/execution:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -fno-exceptions -fno-rtti -fsyntax-only \
  05_Firmware/MATDOG_Controller/src/motion/LegKinematics.cpp \
  05_Firmware/MATDOG_Controller/src/motion/LegInverseKinematics.cpp

g++ -std=c++17 -Wall -Wextra -Werror -O1 -g -fno-exceptions -fno-rtti \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -o /tmp/matdog-g1-sanitized \
  05_Firmware/MATDOG_Controller/scripts/tests/test_motion_kinematics.cpp \
  05_Firmware/MATDOG_Controller/src/motion/LegKinematics.cpp \
  05_Firmware/MATDOG_Controller/src/motion/LegInverseKinematics.cpp
/tmp/matdog-g1-sanitized
```

| Check | Result |
|---|---|
| Geometry regeneration / freshness | PASS |
| Complete controller static audit and host runner | PASS; 118 source files scanned |
| G1 C++ suite | 1,228 checks, 0 failures |
| AddressSanitizer + UndefinedBehaviorSanitizer | 1,228 checks, 0 failures; no diagnostics |
| Python differential vectors | 2,380 passed; repeated C++ output identical |
| Sample composition | 4 q=0, 108 limit-grid, 2,048 random, 12 LF DLS, 208 C4 |
| C4-A/C4-C | 208/208 contact checks; all 51 trajectory frames |
| Independent Python solver runs | 12 LF DLS + 16 contact DLS passed |
| Unsupported-model mutations | 7/7 rejected |
| Include boundary | PASS; only local motion headers and standard math/types |
| Extended existing Python suite | 50 tests: 46 passed; 4 baseline tests produced 16 subtest errors |
| Final whitespace/diff checks | PASS |

### Numerical results and tolerances

| Quantity | Maximum measured | Acceptance |
|---|---:|---:|
| C++ FK vs canonical Python position | 8.442e-17 m | 1e-12 m |
| C++ FK vs canonical Python rotation entry | 3.331e-16 | 1e-12 |
| Python FK of C++ IK vs target | 1.279e-16 m | 1e-9 m |
| Python LF DLS residual | 6.878e-9 m | 1e-8 m |
| Rerun Python contact DLS residual | 1.044e-7 m | 1e-5 m |
| C++ solution's C4 contact vs target | 9.930e-6 m | existing 1e-5 m |
| C++ solution's contact vs archived achieved contact | 9.679e-17 m | 1e-10 m |

The nearly 10 micrometre C4 target residual is inherited from the archived
contact DLS trajectory, not introduced by C++ IK. All nominal strip-contact
classifications match. C4 mesh collision certification was not rerun or
reimplemented. These are kinematic/contact replay checks, not a C++ stand
trajectory generator or execution approval.

Production roundoff guards are 1e-12 m for distance-domain checks, 1e-12 for
cosine-domain checks, and 1e-12 rad for endpoint normalization/deduplication.
The default IK residual tolerance is 1e-9 m; its accepted configuration range
is [1e-12, 1e-3] m. Returned joints remain strictly inside the URDF intervals.
Nearest-seed recovery tests allow 1e-8 rad. No q=0 geometry was altered.

### Existing discrepancy

`test_matdog_leg_fk_live.py` has four tests, each with four leg subcases, that
fail because `load_leg_calibration` expects
`VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION`, while the current calibration
file declares `DIGITAL_ZERO_CALIBRATED_AND_VERIFIED`.

The same 16 subtest errors were reproduced in the clean primary main checkout
with bytecode writes disabled:

```sh
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=06_Software/Matdog_Core/kinematics \
  python3 -m unittest discover -s 06_Software/Matdog_Core/kinematics/tests \
  -p 'test_matdog_leg_fk_live.py'
```

Those three source/config/test files have no diff against the base. This
calibration compatibility issue was left untouched, respecting workstream
ownership. No geometric discrepancy was found in the G1 comparisons.

## Local history and safety boundary

Implementation commits:

- `ab9159984020422972dd5a483e0d8f5a0f865df0` — verified model and FK.
- `6b88c9594c2035d2d6d9002f678b9e0f3cb36a9c` — bounded analytic IK.
- `3887d5324e46e75db21ba60c28693f59d4624f0f` — tests, host gate and API documentation.

This report is a separate local documentation commit. The implementation
worktree was clean after the three implementation commits; final status is
checked after committing this report.

Final source/include review found no servo conversion, q0 assumption,
motor-direction handling, actuator/serial/Arduino dependency or hardware
access in the motion core. There was no hardware access, servo command,
firmware build/upload/flash, calibration change, main modification or merge.
The calibration branch was inspected only through Git references/diffs.
No commits were pushed. The next milestone remains outside this G1 change.
