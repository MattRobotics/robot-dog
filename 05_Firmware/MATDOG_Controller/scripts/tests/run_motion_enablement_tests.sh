#!/usr/bin/env bash
# Offline-only integration suite. Output traces go outside Git by default.
set -euo pipefail
TESTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$(cd "$TESTS/../.." && pwd)"
export PYTHONDONTWRITEBYTECODE=1
bash "$TESTS/run_calibration_restore_tests.sh"
bash "$TESTS/run_motion_service_executor_tests.sh"
bash "$TESTS/run_motion_lifecycle_tests.sh"
python3 "$TESTS/run_adaptive_motion_tests.py" --trace "${MATDOG_TEST_TRACE:-/tmp/matdog-adaptive-rc.jsonl}"
python3 "$TESTS/test_ap_only_transport.py"
python3 "$TESTS/test_motion_http_transport.py"
python3 "$TESTS/test_motion_transport_mode.py"
python3 "$TESTS/test_motion_safety_boundaries.py"
python3 "$TESTS/test_motion_rc_receipt.py"
node "$TESTS/test_portal_dom.cjs"
node "$TESTS/test_motion_dashboard_dom.cjs"
python3 "$FW/scripts/static_audit.py"
echo "MOTION_ENABLEMENT_OFFLINE = PASS; hardware qualification=false; flash/motion authorization=false"
