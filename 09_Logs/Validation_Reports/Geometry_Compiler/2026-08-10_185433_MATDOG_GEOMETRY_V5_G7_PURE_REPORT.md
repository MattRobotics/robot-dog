# MATDOG Geometry Compiler V5 — pure geometry report

schema: `matdog.calibration_geometry_profile.v5`
semantic content SHA256: `d4180810252d43e4cce349e638ef499f55ebb41e057b9fd9d9ced68e206a1ecd`
URDF: `03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf`
URDF SHA256: `3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59`
selected actuated revolute joints: 12
topology-derived articulated branches: 4

This artifact reports geometry only. Geometric contact, path obstruction, external empirical evidence and safety-policy acceptance are distinct facts.

## Outcome summary

- contact `GEOMETRIC_CONTACT_FOUND`: 24
- path `NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN`: 18
- path `PATH_OBSTRUCTION`: 6
- obstruction relation `body_vs_branch`: 2
- obstruction relation `same_branch`: 4

## Endpoint searches

| Joint | Side | Declared deg | Contact status | Contact deg | Delta deg | Active pair | Path status | Path deg | Relation | Precedes contact |
|---|---|---:|---|---:|---:|---|---|---:|---|---|
| lf_hip_joint | min | -45.000000 | GEOMETRIC_CONTACT_FOUND | -46.011719 | -1.011719 | base_link ↔ lf_hip_link | PATH_OBSTRUCTION | -45.296875 | body_vs_branch | True |
| lf_hip_joint | max | +45.000000 | GEOMETRIC_CONTACT_FOUND | +45.222656 | +0.222656 | base_link ↔ lf_hip_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| lf_upper_leg_joint | min | -52.500000 | GEOMETRIC_CONTACT_FOUND | -52.132813 | +0.367187 | lf_hip_link ↔ lf_upper_leg_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| lf_upper_leg_joint | max | +122.500000 | GEOMETRIC_CONTACT_FOUND | +121.875000 | -0.625000 | lf_hip_link ↔ lf_upper_leg_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| lf_lower_leg_joint | min | -92.000000 | GEOMETRIC_CONTACT_FOUND | -92.074219 | -0.074219 | lf_upper_leg_link ↔ lf_lower_leg_link | PATH_OBSTRUCTION | -97.968750 | same_branch | False |
| lf_lower_leg_joint | max | +37.500000 | GEOMETRIC_CONTACT_FOUND | +38.179688 | +0.679688 | lf_upper_leg_link ↔ lf_lower_leg_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| rf_hip_joint | min | -45.000000 | GEOMETRIC_CONTACT_FOUND | -45.222656 | -0.222656 | base_link ↔ rf_hip_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| rf_hip_joint | max | +45.000000 | GEOMETRIC_CONTACT_FOUND | +46.011719 | +1.011719 | base_link ↔ rf_hip_link | PATH_OBSTRUCTION | +45.296875 | body_vs_branch | True |
| rf_upper_leg_joint | min | -52.500000 | GEOMETRIC_CONTACT_FOUND | -52.132813 | +0.367187 | rf_hip_link ↔ rf_upper_leg_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| rf_upper_leg_joint | max | +122.500000 | GEOMETRIC_CONTACT_FOUND | +121.875000 | -0.625000 | rf_hip_link ↔ rf_upper_leg_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| rf_lower_leg_joint | min | -92.000000 | GEOMETRIC_CONTACT_FOUND | -92.074219 | -0.074219 | rf_upper_leg_link ↔ rf_lower_leg_link | PATH_OBSTRUCTION | -97.972656 | same_branch | False |
| rf_lower_leg_joint | max | +37.500000 | GEOMETRIC_CONTACT_FOUND | +38.179688 | +0.679688 | rf_upper_leg_link ↔ rf_lower_leg_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| rh_hip_joint | min | -45.000000 | GEOMETRIC_CONTACT_FOUND | -45.156250 | -0.156250 | base_link ↔ rh_hip_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| rh_hip_joint | max | +45.000000 | GEOMETRIC_CONTACT_FOUND | +46.011719 | +1.011719 | base_link ↔ rh_hip_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| rh_upper_leg_joint | min | -52.500000 | GEOMETRIC_CONTACT_FOUND | -52.132813 | +0.367187 | rh_hip_link ↔ rh_upper_leg_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| rh_upper_leg_joint | max | +122.500000 | GEOMETRIC_CONTACT_FOUND | +121.875000 | -0.625000 | rh_hip_link ↔ rh_upper_leg_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| rh_lower_leg_joint | min | -92.000000 | GEOMETRIC_CONTACT_FOUND | -92.074219 | -0.074219 | rh_upper_leg_link ↔ rh_lower_leg_link | PATH_OBSTRUCTION | -97.972656 | same_branch | False |
| rh_lower_leg_joint | max | +37.500000 | GEOMETRIC_CONTACT_FOUND | +38.179688 | +0.679688 | rh_upper_leg_link ↔ rh_lower_leg_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| lh_hip_joint | min | -45.000000 | GEOMETRIC_CONTACT_FOUND | -46.011719 | -1.011719 | base_link ↔ lh_hip_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| lh_hip_joint | max | +45.000000 | GEOMETRIC_CONTACT_FOUND | +45.156250 | +0.156250 | base_link ↔ lh_hip_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| lh_upper_leg_joint | min | -52.500000 | GEOMETRIC_CONTACT_FOUND | -52.132813 | +0.367187 | lh_hip_link ↔ lh_upper_leg_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| lh_upper_leg_joint | max | +122.500000 | GEOMETRIC_CONTACT_FOUND | +121.875000 | -0.625000 | lh_hip_link ↔ lh_upper_leg_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |
| lh_lower_leg_joint | min | -92.000000 | GEOMETRIC_CONTACT_FOUND | -92.074219 | -0.074219 | lh_upper_leg_link ↔ lh_lower_leg_link | PATH_OBSTRUCTION | -97.968750 | same_branch | False |
| lh_lower_leg_joint | max | +37.500000 | GEOMETRIC_CONTACT_FOUND | +38.179688 | +0.679688 | lh_upper_leg_link ↔ lh_lower_leg_link | NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN | - | - | False |

## Topology-derived branches

| Branch ID | Root motorId | Joint chain | Link set |
|---|---:|---|---|
| lf_hip_joint | 13 | lf_hip_joint → lf_upper_leg_joint → lf_lower_leg_joint | lf_hip_link, lf_upper_leg_link, lf_lower_leg_link, lf_foot_link |
| rf_hip_joint | 23 | rf_hip_joint → rf_upper_leg_joint → rf_lower_leg_joint | rf_hip_link, rf_upper_leg_link, rf_lower_leg_link, rf_foot_link |
| rh_hip_joint | 33 | rh_hip_joint → rh_upper_leg_joint → rh_lower_leg_joint | rh_hip_link, rh_upper_leg_link, rh_lower_leg_link, rh_foot_link |
| lh_hip_joint | 43 | lh_hip_joint → lh_upper_leg_joint → lh_lower_leg_joint | lh_hip_link, lh_upper_leg_link, lh_lower_leg_link, lh_foot_link |

## Collision mesh provenance

| Link | URDF collision filename | SHA256 | Triangles | Scale | Origin xyz | Origin rpy |
|---|---|---|---:|---|---|---|
| base_link | `meshes/collision/base_link.stl` | `0a485e7a1101d457f317b664e52a7a4ef061382e6ecc2c7c87a333c81d5b466f` | 22044 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| lf_hip_link | `meshes/collision/lf_hip_link.stl` | `b2e430aa99998791879f1ab80c46787a310c80f77a3df9116d1084e0a0e27d97` | 6016 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| lf_upper_leg_link | `meshes/collision/lf_upper_leg_link.stl` | `0830fc10d8873b6a45f2f58c2ca61f08a7f38092984a96b911ac6beed274c30c` | 3802 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| lf_lower_leg_link | `meshes/collision/lf_lower_leg_link.stl` | `bbb40400571fab554395d5671ad86b1b106ec3fe24fce733ef0be179a739508a` | 12336 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| lf_foot_link | `meshes/collision/lf_foot_link.stl` | `e4d35cd4107bd7fad2d3fe511926ec4fc5211f15f978581e469787266ebe5357` | 2248 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| rf_hip_link | `meshes/collision/rf_hip_link.stl` | `83a7622f990228e6c2b6bb0431e682fc462a76feb3ce2fe190b7553be879dc33` | 6016 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| rf_upper_leg_link | `meshes/collision/rf_upper_leg_link.stl` | `fbf43f047a943188a6c7b9a7a3a45763c22ddb35704a517b7b47b03dcb68cb91` | 3806 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| rf_lower_leg_link | `meshes/collision/rf_lower_leg_link.stl` | `6b40148edad22670c420d1bcca2325c8befd72e27f4390569a292250df3e96c8` | 12352 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| rf_foot_link | `meshes/collision/rf_foot_link.stl` | `18dfa7908e7410cc2920d5d37b0c0cce13cb2c683f765e7d073b3d5777c00d33` | 2248 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| rh_hip_link | `meshes/collision/rh_hip_link.stl` | `04b205e9be3683b5238a035970b56ee319e4e7fca55ba7860a19fdb549c4efea` | 6018 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| rh_upper_leg_link | `meshes/collision/rh_upper_leg_link.stl` | `fbf43f047a943188a6c7b9a7a3a45763c22ddb35704a517b7b47b03dcb68cb91` | 3806 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| rh_lower_leg_link | `meshes/collision/rh_lower_leg_link.stl` | `6b40148edad22670c420d1bcca2325c8befd72e27f4390569a292250df3e96c8` | 12352 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| rh_foot_link | `meshes/collision/rh_foot_link.stl` | `18dfa7908e7410cc2920d5d37b0c0cce13cb2c683f765e7d073b3d5777c00d33` | 2248 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| lh_hip_link | `meshes/collision/lh_hip_link.stl` | `5350010e3623ec36301d2c0592e3209e1b06da83e823215caf55cca5aafe86da` | 6018 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| lh_upper_leg_link | `meshes/collision/lh_upper_leg_link.stl` | `0830fc10d8873b6a45f2f58c2ca61f08a7f38092984a96b911ac6beed274c30c` | 3802 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| lh_lower_leg_link | `meshes/collision/lh_lower_leg_link.stl` | `bbb40400571fab554395d5671ad86b1b106ec3fe24fce733ef0be179a739508a` | 12336 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |
| lh_foot_link | `meshes/collision/lh_foot_link.stl` | `e4d35cd4107bd7fad2d3fe511926ec4fc5211f15f978581e469787266ebe5357` | 2248 | `[0.001, 0.001, 0.001]` | `[0.0, 0.0, 0.0]` | `[0.0, 0.0, 0.0]` |

## Geometry UNKNOWNs

- {'code': 'ADJACENT_HARDSTOP_LOCAL_SENSITIVITY_NOT_COMPUTED', 'description': 'minimum adjacent-pair clearance is dominated by joint-core fit and is not a hardstop-feature derivative'}
- {'code': 'NOMINAL_COLLISION_GEOMETRY_ONLY', 'description': 'assembly stack-up, bushings, screws, backlash and fit clearances are not represented'}

No Station, serial, servo, EEPROM or other hardware access occurred.
No norma-core file was modified. No merge was performed.
