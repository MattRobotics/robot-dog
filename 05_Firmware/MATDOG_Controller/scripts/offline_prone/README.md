# PRONE dev.3 / rc.11 integration — offline only

Current firmware: IMPLEMENTED NOT VALIDATED, terminal torque ON at
PRONE_CONTACT_UNVERIFIED. No autonomous recovery or SAFE_OFF.

Current reproduction from the firmware directory:

```sh
python3 scripts/offline_prone/extract_all_residuals.py
MATDOG_TEST_OUTPUT=/absolute/output/tests-corrected bash scripts/tests/run_operational_host_tests.sh test_prone_hardware_replay test_prone_descent test_temperature_confirm test_prone_temperature
python3 scripts/offline_prone/integration_geometry.py /absolute/output/geometry
```

The integration geometry runner reads the replay log in the sibling
`tests-corrected` directory. It uses `integration_path_probe.cpp`: existing
ViaPlan/route/53 mm target, with the dev.3 PRONE-only +20 mrad front LOWER floor.
It checks the production floor identity before running. Geometry is rigid
quasi-static full-mesh support, not friction, compliance, contact sensing or dynamics.
The 69 cases preserve recorded STAND HOLD residuals/load as constant synthetic
responses during descent. They are not physical descent recordings. CSV load is
signed; generators encode the native sign bit after hardware direction conversion.

The other study scripts (run_geometry.py, path_probe.cpp, analyze_path.py,
flat_bound.py, rear_settle_study.py) retain the historical dev.2 study. They are
NOT evidence for the modified dev.3 path. Do not repeat those searches as part
of integration qualification. See the 2026-10-30 integration report for actual
results, limitations, provenance and the supervised STAND50-only protocol.

All tools above operate on local files and host compilation; none accesses a
serial device, robot HTTP endpoint, flash or motion command path.
