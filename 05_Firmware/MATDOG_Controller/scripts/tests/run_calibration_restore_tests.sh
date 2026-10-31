#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
OUT="$(mktemp -d /tmp/matdog-runtime-restore.XXXXXX)"
trap 'rm -rf "$OUT"' EXIT
CXX="${CXX:-g++}"
SOURCES=(
  CalibrationRecord CalibrationRecordStore CalibrationSaveMarker
  CalibrationPersistenceState CalibrationPersistenceService CalibrationRuntimeRestore
  FullLegCalibrationFinalizer FullLegCalibrationPlan FullLegCalibrationExecutor
  ContactProbeEngine CalibrationExecutionEngine CalibrationManager
  CalibrationMotionPermit CalibrationPopulationEvidence CalibrationDomain
  StartupRecoveryReference RestoreAttestationPolicy
)
FILES=()
for SOURCE in "${SOURCES[@]}"; do FILES+=("$SKETCH_DIR/src/calibration/$SOURCE.cpp"); done
for SOURCE in OperationalEnvelope MotionDeadman ActuatorRuntime ActuatorWritePolicy CalibrationSequencePlan CalibrationTargetResolver CalibrationGeometryProfile; do
  FILES+=("$SKETCH_DIR/src/actuator/$SOURCE.cpp")
done
FILES+=("$SKETCH_DIR/src/servo/ServoPopulation.cpp")
for SOURCE in SystemState ActuatorAuthority OperatingMode; do FILES+=("$SKETCH_DIR/src/core/$SOURCE.cpp"); done
TESTS=(test_calibration_runtime_restore test_calibration_persistence_service test_issue48_restore)
if [[ "${MATDOG_RESTORE_REGRESSION_ONLY:-0}" == "1" ]]; then TESTS=(test_calibration_persistence_service); fi
for TEST in "${TESTS[@]}"; do
  "$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -o "$OUT/$TEST" "$SCRIPT_DIR/$TEST.cpp" "${FILES[@]}"
  "$OUT/$TEST"
done
