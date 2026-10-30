# RETURN TO PRONE — offline reproduction

Production candidate: **IMPLEMENTED NOT VALIDATED**. The command descends and
keeps torque ON at `PRONE_CONTACT_UNVERIFIED`; it does not establish physical
support and never requests automatic SAFE_OFF. No autonomous fault recovery.

Run from `05_Firmware/MATDOG_Controller`, with a separate artifact directory:

```sh
python3 scripts/offline_prone/run_geometry.py /absolute/artifact/directory
python3 scripts/offline_prone/flat_bound.py /absolute/artifact/directory/flat_bound.json
MATDOG_TEST_OUTPUT=/absolute/artifact/directory/tests bash scripts/tests/run_operational_host_tests.sh test_prone_descent test_prone_height_residuals test_prone_admission
```

Dependencies: host g++, Python 3, NumPy, SciPy. These scripts access local files
and compile/run host programs only. They have no serial, robot HTTP or flash path.
Run the host tests before the geometry runner to include their final encoder
snapshots in the CAD endpoint study.

`path_probe.cpp` uses the production ViaPlan, frozen route knots and canonical
53 mm target. `locked_contact_probe.cpp` uses production contact FK/IK to examine
an alternative with fixed foot references; `analyze_path.py` additionally checks
ALL CAD collision meshes against the floor. IK success alone is not a PASS.
`geometry.py` provides both the original symmetric sagittal model and a 3D
support-hull model for asymmetric recorded residuals. Quasi-static rigid support
is an assumption, not a measured contact or a dynamics/friction simulation.

`flat_bound.py` covers each unchanged box intersected with gen1 limits using
numerical interval subdivision. A negative upper bound for a leg's lowest vertex
excludes a flat body at ground level in that box. Floating-point arithmetic uses
a 1e-10 m guard, not formally outward-rounded intervals. This is not a theorem
excluding every possible tilted resting pose or a compliant passive settlement.
`search_landing.py` is an earlier exploratory optimizer; its iteration-limit
result is not used as a proof or acceptance test.

The fixtures retain original source paths and SHA256. `load` in the historical
CSV is SIGNED telemetry; `extract_residuals.py` reconstructs the servo direction
bit, then transforms it with the hardware-witness encoder directions for the
positive-sign host port. It never treats a negative CSV value as a raw bit word.
Holding the measured STAND100 errors/load values constant during descent is a
stress scenario, not a measured descent and not a motor model.

See the final report under `09_Logs/Validation_Reports/Prone_Completion_2026-10-29`.

Post-build study only: `rear_settle_study.py OUTPUT_DIR` evaluates paired rear
UPPER, then UPPER+LOWER changes without changing the firmware. Its two-variable
optimizer did not attest convergence; neither variant removes rear support.
