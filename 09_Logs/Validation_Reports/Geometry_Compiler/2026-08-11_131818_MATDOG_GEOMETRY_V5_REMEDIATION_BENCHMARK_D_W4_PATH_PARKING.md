# MATDOG Geometry Compiler V5 — geometry-driven path/parking

input geometry profile: `09_Logs/Validation_Reports/Geometry_Compiler/2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4_ENDPOINT_PROFILE.json`
geometry file SHA256: `dd8cb42c3b916d067f97a321c5ffcdfb013dde1f2f2a6ba71f73becff360dc0f`
geometry semantic SHA256: `de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e`
parking semantic SHA256: `67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139`
endpoint plans: 24
baseline obstructions: 6
complete geometric sequences: 24
targets inside URDF limits: 8
diagnostic targets outside URDF limits: 16

Search is normalized to URDF limits: 1-DOF first, 2-DOF only if no 1-DOF plan is feasible.
The declared search domain is the finite grid serialized per plan; it is not a continuous-domain proof.
Path feasibility is evaluated at every serialized-step configuration, not as continuous swept volume.
The first baseline obstruction is additionally bisected to the serialized refinement resolution; sampled and refined evidence remain distinct.
A DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS target is not an executable robot motion claim.
No fixed historical parking-angle seed list and no safety threshold is used.

| Endpoint | Target domain | Baseline | First blocker (refined when available) | Relation | Refined contact / bracket / resolution rad | Relevant joints | Outcome | In/task/return/out | Candidates 1D/2D | Parking rad | Min clearance |
|---|---|---|---|---|---|---|---|---|---:|---|---:|
| lf_hip_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 1.5853628948354903e-05 |
| lf_hip_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.004092101125299265 |
| lf_upper_leg_joint:min | EXECUTABLE_URDF_DOMAIN | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.01145788836294163 |
| lf_upper_leg_joint:max | EXECUTABLE_URDF_DOMAIN | PATH_OBSTRUCTION | lf_foot_link ↔ lh_foot_link | cross_branch | `1.279050784` / `6.81070705e-05` / `0.0001` | lf_hip_joint, lf_lower_leg_joint, lh_hip_joint, lh_upper_leg_joint, lh_lower_leg_joint | FEASIBLE_1DOF_PLAN_FOUND | COLLISION_FREE/COLLISION_FREE/COLLISION_FREE/COLLISION_FREE | 33/0 | `{'lh_upper_leg_joint': 0.610865238198}` | 0.0085 |
| lf_lower_leg_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | PATH_OBSTRUCTION | lf_hip_link ↔ lf_lower_leg_link | same_branch | `-1.470854308` / `6.74982474e-05` / `0.0001` | lf_upper_leg_joint | FEASIBLE_1DOF_PLAN_FOUND | COLLISION_FREE/COLLISION_FREE/COLLISION_FREE/COLLISION_FREE | 7/0 | `{'lf_upper_leg_joint': 1.119919603363}` | 1.5175329199840215e-05 |
| lf_lower_leg_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.013787832260131826 |
| rf_hip_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.004092101125299265 |
| rf_hip_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 2.0393984746930327e-05 |
| rf_upper_leg_joint:min | EXECUTABLE_URDF_DOMAIN | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.01145788836294163 |
| rf_upper_leg_joint:max | EXECUTABLE_URDF_DOMAIN | PATH_OBSTRUCTION | rf_foot_link ↔ rh_foot_link | cross_branch | `1.279050784` / `6.81070705e-05` / `0.0001` | rf_hip_joint, rf_lower_leg_joint, rh_hip_joint, rh_upper_leg_joint, rh_lower_leg_joint | FEASIBLE_1DOF_PLAN_FOUND | COLLISION_FREE/COLLISION_FREE/COLLISION_FREE/COLLISION_FREE | 33/0 | `{'rh_upper_leg_joint': 0.610865238198}` | 0.0085 |
| rf_lower_leg_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | PATH_OBSTRUCTION | rf_hip_link ↔ rf_lower_leg_link | same_branch | `-1.470854308` / `6.74982474e-05` / `0.0001` | rf_upper_leg_joint | FEASIBLE_1DOF_PLAN_FOUND | COLLISION_FREE/COLLISION_FREE/COLLISION_FREE/COLLISION_FREE | 7/0 | `{'rf_upper_leg_joint': 1.119919603363}` | 1.5175329199840215e-05 |
| rf_lower_leg_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.013787832260131826 |
| rh_hip_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.0041239498692445045 |
| rh_hip_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 2.0393984746930327e-05 |
| rh_upper_leg_joint:min | EXECUTABLE_URDF_DOMAIN | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.0085 |
| rh_upper_leg_joint:max | EXECUTABLE_URDF_DOMAIN | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.013787832260131826 |
| rh_lower_leg_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | PATH_OBSTRUCTION | rh_hip_link ↔ rh_lower_leg_link | same_branch | `-1.470854308` / `6.74982474e-05` / `0.0001` | rh_upper_leg_joint | FEASIBLE_1DOF_PLAN_FOUND | COLLISION_FREE/COLLISION_FREE/COLLISION_FREE/COLLISION_FREE | 7/0 | `{'rh_upper_leg_joint': 1.628973968528}` | 0.001 |
| rh_lower_leg_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.013787832260131826 |
| lh_hip_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 1.5853628948354903e-05 |
| lh_hip_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.0041239498692445045 |
| lh_upper_leg_joint:min | EXECUTABLE_URDF_DOMAIN | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.0085 |
| lh_upper_leg_joint:max | EXECUTABLE_URDF_DOMAIN | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.013787832260131826 |
| lh_lower_leg_joint:min | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | PATH_OBSTRUCTION | lh_hip_link ↔ lh_lower_leg_link | same_branch | `-1.470854308` / `6.74982474e-05` / `0.0001` | lh_upper_leg_joint | FEASIBLE_1DOF_PLAN_FOUND | COLLISION_FREE/COLLISION_FREE/COLLISION_FREE/COLLISION_FREE | 7/0 | `{'lh_upper_leg_joint': 1.628973968528}` | 0.001 |
| lh_lower_leg_joint:max | DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS | COLLISION_FREE | - | - | - | - | NOT_NEEDED | -/COLLISION_FREE/COLLISION_FREE/- | 0/0 | `{}` | 0.013787832260131826 |

Geometric feasibility is intersection/no intersection. Clearance is a measured quantity.
Safety acceptance is intentionally absent and belongs to the separate policy artifact.
