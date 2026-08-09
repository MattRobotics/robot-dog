# MATDOG collision mesh hash manifest — Phase 1B

- **Profile:** `2026-08-08_231600_MATDOG_CALIBRATION_GEOMETRY_PROFILE.json`
- **Profile content_sha256:** `928ff09616a1ad80df029a99afe2061e41a0bedec6bfdc9b3f80f74af4867828`
- **Schema:** `matdog.calibration_geometry_profile.v4`

- **URDF:** `03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf`
- **URDF sha256:** `5e28da3dba10fd3f2ea6ebf6f5d6271157bda0b12b82d92aedbe3031643089ef` — **unchanged in Phase 1B**
- **Model revision:** rev00 (unchanged)

## Changed in Phase 1B (motor-pin representation, 5 meshes)

| mesh | old sha256 | new sha256 |
|---|---|---|
| `base_link.stl` | `7b149643a6a71ae8ac37780085c021c5bff0c4491ebd42f1393150cb987e9e6e` | `644a83e98fd116f3fc8e5d8792ca2b60b0bdb09bcafa4fc49140d641079e4b6b` |
| `lf_upper_leg_link.stl` | `dfaf754764ded80743b00ae5d1dde208301ab5a3cebc9e29a38d8513dbe4cd93` | `3c484b110a622274d1f8446b30329b8e16a0ff542d25da2a39334d41e8f4f169` |
| `lh_upper_leg_link.stl` | `dfaf754764ded80743b00ae5d1dde208301ab5a3cebc9e29a38d8513dbe4cd93` | `3c484b110a622274d1f8446b30329b8e16a0ff542d25da2a39334d41e8f4f169` |
| `rf_upper_leg_link.stl` | `3c2508690ac89d006d25cabf3917ce52dc2c0b09e4ec5e051da8f71ff1a7e760` | `08fab5e3229280f21a52c6cfbfdab32f1e8345aae6e10648360ae30fe1e06b16` |
| `rh_upper_leg_link.stl` | `3c2508690ac89d006d25cabf3917ce52dc2c0b09e4ec5e051da8f71ff1a7e760` | `08fab5e3229280f21a52c6cfbfdab32f1e8345aae6e10648360ae30fe1e06b16` |

## All collision meshes at this run

| mesh | sha256 | changed |
|---|---|---|
| `base_link.stl` | `644a83e98fd116f3fc8e5d8792ca2b60b0bdb09bcafa4fc49140d641079e4b6b` | **yes** |
| `lf_foot_link.stl` | `e43737c4cbc5d618431ecf2ce9ba20a4803f5a7b7f800974a6d8b2c7534091bc` | no |
| `lf_hip_link.stl` | `7d28351aba669cc1a8e11ad60b89db001f9689b728e4e8eb05849af69e1e4879` | no |
| `lf_lower_leg_link.stl` | `97f2bb775768fdbcef59638916eaef2cd34b21bf01e6197ef3900b0df85d7ae2` | no |
| `lf_upper_leg_link.stl` | `3c484b110a622274d1f8446b30329b8e16a0ff542d25da2a39334d41e8f4f169` | **yes** |
| `lh_foot_link.stl` | `e43737c4cbc5d618431ecf2ce9ba20a4803f5a7b7f800974a6d8b2c7534091bc` | no |
| `lh_hip_link.stl` | `543f6f04a4dc45a0b176eba8db8ab7819ab20a328aa4f7fd9c9d4b77c1612094` | no |
| `lh_lower_leg_link.stl` | `97f2bb775768fdbcef59638916eaef2cd34b21bf01e6197ef3900b0df85d7ae2` | no |
| `lh_upper_leg_link.stl` | `3c484b110a622274d1f8446b30329b8e16a0ff542d25da2a39334d41e8f4f169` | **yes** |
| `rf_foot_link.stl` | `e43737c4cbc5d618431ecf2ce9ba20a4803f5a7b7f800974a6d8b2c7534091bc` | no |
| `rf_hip_link.stl` | `ebd664b3db2065ac3341184a6b12700caca6cdece57adae72f935882ed9df4c0` | no |
| `rf_lower_leg_link.stl` | `c52ae662c09455f53db5bc8efc13332919942c7f938ada5a8c0e987f54ac3bc8` | no |
| `rf_upper_leg_link.stl` | `08fab5e3229280f21a52c6cfbfdab32f1e8345aae6e10648360ae30fe1e06b16` | **yes** |
| `rh_foot_link.stl` | `e43737c4cbc5d618431ecf2ce9ba20a4803f5a7b7f800974a6d8b2c7534091bc` | no |
| `rh_hip_link.stl` | `2e9f31952e51b5748546c803c06d72316a3a776226f1f72775c1e8d383e1f325` | no |
| `rh_lower_leg_link.stl` | `c52ae662c09455f53db5bc8efc13332919942c7f938ada5a8c0e987f54ac3bc8` | no |
| `rh_upper_leg_link.stl` | `08fab5e3229280f21a52c6cfbfdab32f1e8345aae6e10648360ae30fe1e06b16` | **yes** |

Canonical filenames were preserved, so the URDF `<visual>`/`<collision>` references
required no edit. Local frame, scale and coordinates are unchanged; the CAD exports
retriangulated the surfaces, so triangle counts and IDs differ and are NOT integrity
criteria — integrity was established geometrically (GATE B STEP 1).
