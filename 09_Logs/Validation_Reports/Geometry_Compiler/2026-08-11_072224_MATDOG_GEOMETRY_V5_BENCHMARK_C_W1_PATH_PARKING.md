# MATDOG Geometry Compiler V5 — geometry-driven path/parking

input geometry profile: `09_Logs/Validation_Reports/Geometry_Compiler/2026-08-11_072224_MATDOG_GEOMETRY_V5_BENCHMARK_C_W1_ENDPOINT_PROFILE.json`
geometry file SHA256: `e9fa23d5c5ee98c56d622064b7ed7577ea9867f6db498909afbfc61d11156079`
geometry semantic SHA256: `cad2f194c49d063b5a09ae4602b9acf61a701de48791e5d1aae04f1439db1211`
parking semantic SHA256: `3cda03c2c02ba5fbe6def7821ce72d4aa9e4ca66e7ca8655a2b8f0e92dde297c`
endpoint plans: 24
baseline obstructions: 6
complete geometric sequences: 24
targets inside URDF limits: 8
diagnostic targets outside URDF limits: 16

Search is normalized to URDF limits: 1-DOF first, 2-DOF only if no 1-DOF plan is feasible.
The declared search domain is the finite grid serialized per plan; it is not a continuous-domain proof.
Path feasibility is evaluated at every serialized-step configuration, not as continuous swept volume.
A DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS target is not an executable robot motion claim.
No fixed historical parking-angle seed list and no safety threshold is used.

| Endpoint | Target domain | Baseline | First sampled blocker | Relation | Relevant joints | Outcome | In/task/return/out | Candidates 1D/2D | Parking rad | Min clearance |
|---|---|---|---|---|---|---|---|---:|---|---:|
| lf_hip_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 1.58536288960634e-05 |
| lf_hip_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.00409210112521681 |
| lf_upper_leg_joint:min | EXECUTABLE_URDF_DOMAIN | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.011457888362754924 |
| lf_upper_leg_joint:max | EXECUTABLE_URDF_DOMAIN | PATH_OBSTRUCTION | lf_foot_link ↔ lh_foot_link | cross_branch | lf_hip_joint, lf_lower_leg_joint, lh_hip_joint, lh_upper_leg_joint, lh_lower_leg_joint | FEASIBLE_1DOF_PLAN_FOUND | COLLISION_FREE/COLLISION_FREE/COLLISION_FREE/COLLISION_FREE | 33/0 | `{'lh_upper_leg_joint': 0.610865238198}` | 0.0085 |
| lf_lower_leg_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | PATH_OBSTRUCTION | lf_hip_link ↔ lf_lower_leg_link | same_branch | lf_upper_leg_joint | FEASIBLE_1DOF_PLAN_FOUND | COLLISION_FREE/COLLISION_FREE/COLLISION_FREE/COLLISION_FREE | 7/0 | `{'lf_upper_leg_joint': 1.119919603363}` | 1.5175329199840215e-05 |
| lf_lower_leg_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.013787832260131826 |
| rf_hip_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.00409210112521681 |
| rf_hip_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 2.0393984694687395e-05 |
| rf_upper_leg_joint:min | EXECUTABLE_URDF_DOMAIN | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.011457888362754924 |
| rf_upper_leg_joint:max | EXECUTABLE_URDF_DOMAIN | PATH_OBSTRUCTION | rf_foot_link ↔ rh_foot_link | cross_branch | rf_hip_joint, rf_lower_leg_joint, rh_hip_joint, rh_upper_leg_joint, rh_lower_leg_joint | FEASIBLE_1DOF_PLAN_FOUND | COLLISION_FREE/COLLISION_FREE/COLLISION_FREE/COLLISION_FREE | 33/0 | `{'rh_upper_leg_joint': 0.610865238198}` | 0.0085 |
| rf_lower_leg_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | PATH_OBSTRUCTION | rf_hip_link ↔ rf_lower_leg_link | same_branch | rf_upper_leg_joint | FEASIBLE_1DOF_PLAN_FOUND | COLLISION_FREE/COLLISION_FREE/COLLISION_FREE/COLLISION_FREE | 7/0 | `{'rf_upper_leg_joint': 1.119919603363}` | 1.5175329199840215e-05 |
| rf_lower_leg_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.013787832260131826 |
| rh_hip_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.004123949869189562 |
| rh_hip_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 2.0393984694687395e-05 |
| rh_upper_leg_joint:min | EXECUTABLE_URDF_DOMAIN | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.0085 |
| rh_upper_leg_joint:max | EXECUTABLE_URDF_DOMAIN | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.013787832260131826 |
| rh_lower_leg_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | PATH_OBSTRUCTION | rh_hip_link ↔ rh_lower_leg_link | same_branch | rh_upper_leg_joint | FEASIBLE_1DOF_PLAN_FOUND | COLLISION_FREE/COLLISION_FREE/COLLISION_FREE/COLLISION_FREE | 7/0 | `{'rh_upper_leg_joint': 1.628973968528}` | 0.001 |
| rh_lower_leg_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.013787832260131826 |
| lh_hip_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 1.58536288960634e-05 |
| lh_hip_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.004123949869189562 |
| lh_upper_leg_joint:min | EXECUTABLE_URDF_DOMAIN | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.0085 |
| lh_upper_leg_joint:max | EXECUTABLE_URDF_DOMAIN | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.013787832260131826 |
| lh_lower_leg_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | PATH_OBSTRUCTION | lh_hip_link ↔ lh_lower_leg_link | same_branch | lh_upper_leg_joint | FEASIBLE_1DOF_PLAN_FOUND | COLLISION_FREE/COLLISION_FREE/COLLISION_FREE/COLLISION_FREE | 7/0 | `{'lh_upper_leg_joint': 1.628973968528}` | 0.001 |
| lh_lower_leg_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.013787832260131826 |

Geometric feasibility is intersection/no intersection. Clearance is a measured quantity.
Safety acceptance is intentionally absent and belongs to the separate policy artifact.
