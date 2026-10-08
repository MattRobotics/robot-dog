#!/usr/bin/env bash
# Compiles and runs the offline C++ host test suites: the G2 servo
# population / hardware profile logic, the DALY wire protocol / KEY
# probe (frames, CRC, decoders, bus scheduling), and the W1 Wi-Fi runtime
# policy (credential gate, two-phase radio start, connect deadline, backoff
# ladder, link loss, wraparound). No hardware, no Arduino toolchain, no
# device I/O — they link the real firmware translation units, which is why
# those were kept Arduino-free. OTA-A adds a fourth: the update state
# machine, the first-boot rollback guard and the streaming SHA-256, driven
# against a fake OtaBackend so every flash-failure path is reachable offline.
#
# Invoked by scripts/static_audit.py so there is one gate command, matching
# how the OTA partition parser's Python suite is already run.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

CXX="${CXX:-g++}"
if ! command -v "$CXX" >/dev/null 2>&1; then
  echo "HOST_TESTS = FAIL (no C++ compiler found: CXX=$CXX)" >&2
  exit 1
fi

OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 \
  -I "$SCRIPT_DIR/servo_read_stub" \
  -o "$OUT/test_servo_read_validation" "$SCRIPT_DIR/test_servo_read_validation.cpp"

"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 \
  -o "$OUT/test_servo_population" \
  "$SCRIPT_DIR/test_servo_population.cpp" \
  "$SKETCH_DIR/src/servo/ServoPopulation.cpp" \
  "$SKETCH_DIR/src/core/Availability.cpp" \
  "$SKETCH_DIR/src/core/SystemState.cpp"

# The MATDOG_C018_V1 contract links the REAL generated register table, so the
# twenty values under test are the ones the firmware carries. The preflight
# SERVICE is device-only (it holds a ServoBus); what is host-testable is the
# contract, the comparison semantics and the leg selection.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 \
  -o "$OUT/test_servo_profile" \
  "$SCRIPT_DIR/test_servo_profile.cpp" \
  "$SKETCH_DIR/src/servo/ServoProfile.cpp" \
  "$SKETCH_DIR/src/servo/ServoPopulation.cpp" \
  "$SKETCH_DIR/src/core/Availability.cpp" \
  "$SKETCH_DIR/src/core/SystemState.cpp"

# -DDISABLED=0x00 reproduces the Arduino-ESP32 core macro (esp32-hal-gpio.h)
# so an identifier clash with it fails here, not only in the device build.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_daly_protocol" \
  "$SCRIPT_DIR/test_daly_protocol.cpp" \
  "$SKETCH_DIR/src/power/DalyProtocol.cpp"

# -DDISABLED=0x00 here too: WifiPolicy.h documents why its first state is
# INACTIVE and not DISABLED, and this flag makes that reasoning enforceable
# on the host instead of only discoverable on the device build.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_wifi_policy" \
  "$SCRIPT_DIR/test_wifi_policy.cpp" \
  "$SKETCH_DIR/src/network/WifiPolicy.cpp" \
  "$SKETCH_DIR/src/network/NetworkConfig.cpp"

"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_domain" \
  "$SCRIPT_DIR/test_calibration_domain.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp"

# CR1 formal population evidence is a pure adapter over existing structured
# census + preflight results. No ServoBus or device I/O is linked.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_population_evidence" \
  "$SCRIPT_DIR/test_calibration_population_evidence.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationPopulationEvidence.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/servo/ServoPopulation.cpp"

# The calibration manager links the REAL arbiter, so the authority
# integration it exercises is the shipped one rather than a mock.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_manager" \
  "$SCRIPT_DIR/test_calibration_manager.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationManager.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_actuator_authority" \
  "$SCRIPT_DIR/test_actuator_authority.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# The Safe Actuator Layer policy links the REAL arbiter and the REAL
# calibration domain model: the authority binding and the limit-provenance
# rules it enforces are the shipped ones, not a mock's idea of them.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_actuator_write_policy" \
  "$SCRIPT_DIR/test_actuator_write_policy.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# The calibration bootstrap geometry suite links the REAL generated profile
# table, so the 24 endpoints, 6 parking plans and per-joint envelopes it
# checks are the exact ones the firmware would carry - reduced from the
# canonical Geometry Compiler V5 bundle, never retyped.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_geometry" \
  "$SCRIPT_DIR/test_calibration_geometry.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# CR2 q0 bootstrap is pure evidence reduction: real geometry profile + real
# calibration domain, no ServoBus/Arduino/device I/O.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_q0_bootstrap" \
  "$SCRIPT_DIR/test_calibration_q0_bootstrap.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0Bootstrap.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp"

# CR2-B same-session acquisition coordinator is pure. It links the real
# CR1 population producer and CR2-A q0 reducer plus current generated geometry,
# but no Arduino or device I/O.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_q0_capture_session" \
  "$SCRIPT_DIR/test_calibration_q0_capture_session.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationQ0CaptureSession.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationPopulationEvidence.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0Bootstrap.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/servo/ServoPopulation.cpp"

# CR3-M1/M2: pure q0 acceptance/promotion plus the one checked q<->raw
# resolver. Links real generated Geometry V5 data; no ServoBus/Arduino/device IO.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_cr3_q0_transform" \
  "$SCRIPT_DIR/test_cr3_q0_transform.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0Promotion.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0Bootstrap.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp"

# CR3 persisted q0 evidence rehydration. The frozen source geometry is
# compared to current Geometry V5 before the real M1 accept/promote path runs.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_cr3_q0_evidence_preparation" \
  "$SCRIPT_DIR/test_cr3_q0_evidence_preparation.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0EvidencePreparation.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0Promotion.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0Bootstrap.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# The CURRENT-BOOT q0 promotion: a real CalibrationQ0CaptureSession capture is
# promoted (not the frozen CR2-C package), admitted into the real
# JointTransformTable (a second promotion replaces the first) and consumed by
# the real four-leg plan resolver. The frozen package is only a contrast oracle.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_cr3_q0_fresh_promotion" \
  "$SCRIPT_DIR/test_cr3_q0_fresh_promotion.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0EvidencePreparation.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0Promotion.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0Bootstrap.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationQ0CaptureSession.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationPopulationEvidence.cpp" \
  "$SKETCH_DIR/src/calibration/FullLegCalibrationPlan.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/servo/ServoPopulation.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# CR3-M4 session-scoped calibration motion permit is pure policy: no
# ServoBus, Arduino or device IO. It deliberately remains un-wired in production
# until the first-motion hardware authorization gate.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_motion_permit" \
  "$SCRIPT_DIR/test_calibration_motion_permit.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationMotionPermit.cpp"

# CR3 continuation Objective A: the session-start orchestrator is pure.
# It links the REAL CalibrationManager + REAL ActuatorAuthority arbiter.
# No ServoBus, Arduino or physical backend is present in this binary.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_session_orchestrator" \
  "$SCRIPT_DIR/test_calibration_session_orchestrator.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationSessionOrchestrator.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationManager.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# CR3 Priority 4: the telemetry/deadman monitor shared by the first-motion
# executor and the contact-probe engine. Pure: no ServoBus/Arduino/device IO,
# every sample is synthetic.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 \
  -o "$OUT/test_motion_deadman" \
  "$SCRIPT_DIR/test_motion_deadman.cpp" \
  "$SKETCH_DIR/src/actuator/MotionDeadman.cpp"

# The LF V25 runtime PresentTemperature over-limit confirmation (port.rs),
# driven through the real classification with a scripted direct-read port.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_thermal_confirmation" \
  "$SCRIPT_DIR/test_thermal_confirmation.cpp" \
  "$SKETCH_DIR/src/calibration/ThermalConfirmation.cpp"

# The OTA suite links the REAL OTA-B gate and the REAL arbiter, so the
# authorization path it exercises is the shipped one, not a stub.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_ota_policy" \
  "$SCRIPT_DIR/test_ota_policy.cpp" \
  "$SKETCH_DIR/src/update/OtaPolicy.cpp" \
  "$SKETCH_DIR/src/update/OtaBootGuard.cpp" \
  "$SKETCH_DIR/src/update/Sha256.cpp" \
  "$SKETCH_DIR/src/update/OtaAuthorityGate.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# The firmware-side MATDOG flash layout contract (pure, ESP-IDF-free).
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 \
  -o "$OUT/test_ota_layout_contract" \
  "$SCRIPT_DIR/test_ota_layout_contract.cpp" \
  "$SKETCH_DIR/src/update/OtaLayoutContract.cpp"

# The Safe Actuator runtime adapter suite links the REAL adapter, the REAL
# policy and the REAL arbiter, so the "no ACCEPT -> no backend call" and
# "ACCEPT -> exactly one backend call" properties under test are the shipped
# ones. Only the backend is fake - the one thing this adapter is meant to be
# tested against, the same contract as the OTA suite's fake OtaBackend.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_actuator_runtime" \
  "$SCRIPT_DIR/test_actuator_runtime.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorRuntime.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# The Calibration Execution boundary suite links the REAL engine, the REAL
# policy, the REAL runtime adapter and the REAL arbiter against a fake
# backend, the same contract as test_actuator_runtime.cpp. LF V25's 18-phase
# sequence is never linked here (V3 handoff Sec 15.11) - only the current
# generated geometry profile data.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_execution_engine" \
  "$SCRIPT_DIR/test_calibration_execution_engine.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationExecutionEngine.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorRuntime.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# CR3 Priority 3: the first-motion executor. Links the REAL policy, runtime,
# arbiter and checked target resolver against a fake backend and synthetic
# telemetry - the same contract as test_calibration_execution_engine.cpp.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_first_motion_executor" \
  "$SCRIPT_DIR/test_first_motion_executor.cpp" \
  "$SKETCH_DIR/src/calibration/FirstMotionExecutor.cpp" \
  "$SKETCH_DIR/src/actuator/MotionDeadman.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorRuntime.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# CR3 Priority 5: the contact-probe engine. Links the REAL policy, runtime,
# CalibrationExecutionEngine, arbiter and checked target resolver against a
# fake backend and synthetic telemetry - no fabricated physical measurement.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_contact_probe_engine" \
  "$SCRIPT_DIR/test_contact_probe_engine.cpp" \
  "$SKETCH_DIR/src/calibration/ContactProbeEngine.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationExecutionEngine.cpp" \
  "$SKETCH_DIR/src/actuator/MotionDeadman.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorRuntime.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# CR3 Priority 6: the operational-envelope builder. Links the REAL geometry
# profile, checked target resolver and calibration domain predicates - no
# fabricated contact measurement anywhere in this suite.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_operational_envelope" \
  "$SCRIPT_DIR/test_operational_envelope.cpp" \
  "$SKETCH_DIR/src/actuator/OperationalEnvelope.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp"

# The 24-contact Full Calibration sequence, end to end on all four legs: the
# REAL policy (Geometry V5 + the geometry-validated sequence plan), runtime,
# CalibrationExecutionEngine, ContactProbeEngine, arbiter, checked resolvers
# and the REAL plan resolver against a twelve-joint kinematic servo model -
# no fabricated physical measurement anywhere here.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_full_leg_calibration_executor" \
  "$SCRIPT_DIR/test_full_leg_calibration_executor.cpp" \
  "$SKETCH_DIR/src/calibration/FullLegCalibrationExecutor.cpp" \
  "$SKETCH_DIR/src/calibration/ThermalConfirmation.cpp" \
  "$SKETCH_DIR/src/calibration/FullLegCalibrationPlan.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationPopulationEvidence.cpp" \
  "$SKETCH_DIR/src/servo/ServoPopulation.cpp" \
  "$SKETCH_DIR/src/calibration/ContactProbeEngine.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationExecutionEngine.cpp" \
  "$SKETCH_DIR/src/actuator/OperationalEnvelope.cpp" \
  "$SKETCH_DIR/src/actuator/MotionDeadman.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorRuntime.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# The pure four-leg plan resolver (canonical identity/bus, Geometry V5
# endpoints, parking matrix, generic backoff rule). Links the REAL checked
# target resolver, geometry profile and canonical allocation.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_full_leg_calibration_plan" \
  "$SCRIPT_DIR/test_full_leg_calibration_plan.cpp" \
  "$SKETCH_DIR/src/calibration/FullLegCalibrationPlan.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationPopulationEvidence.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp" \
  "$SKETCH_DIR/src/servo/ServoPopulation.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# The Full Leg evidence lifecycle (recordContact, envelope build, operational
# limit admission, completeSession, permit/authority cleanup) and the RAM
# evidence store/export, against the REAL manager, arbiter, permit, policy
# tables and Geometry V5 profile.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_full_leg_calibration_finalizer" \
  "$SCRIPT_DIR/test_full_leg_calibration_finalizer.cpp" \
  "$SKETCH_DIR/src/calibration/FullLegCalibrationFinalizer.cpp" \
  "$SKETCH_DIR/src/calibration/FullLegCalibrationPlan.cpp" \
  "$SKETCH_DIR/src/calibration/FullLegCalibrationExecutor.cpp" \
  "$SKETCH_DIR/src/calibration/ContactProbeEngine.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationExecutionEngine.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationManager.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationMotionPermit.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationPopulationEvidence.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp" \
  "$SKETCH_DIR/src/actuator/OperationalEnvelope.cpp" \
  "$SKETCH_DIR/src/actuator/MotionDeadman.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorRuntime.cpp" \
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/servo/ServoPopulation.cpp" \
  "$SKETCH_DIR/src/core/SystemState.cpp" \
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp" \
  "$SKETCH_DIR/src/core/OperatingMode.cpp"

# Calibration Persistence V1 (P2): record codec/validation, A/B store with fault
# injection, and the NVS adapter against a host stand-in of <nvs.h>. The record
# validator re-derives joint diagnostics through the production
# deriveFullLegJointDiagnostics, so the executor chain is linked as above.
CALREC_SRCS=(
  "$SKETCH_DIR/src/calibration/CalibrationRecord.cpp"
  "$SKETCH_DIR/src/calibration/CalibrationRecordStore.cpp"
  "$SKETCH_DIR/src/calibration/CalibrationSaveMarker.cpp"
  "$SKETCH_DIR/src/calibration/CalibrationPersistenceState.cpp"
  "$SKETCH_DIR/src/calibration/CalibrationPersistenceService.cpp"
  "$SKETCH_DIR/src/calibration/FullLegCalibrationFinalizer.cpp"
  "$SKETCH_DIR/src/calibration/FullLegCalibrationPlan.cpp"
  "$SKETCH_DIR/src/calibration/FullLegCalibrationExecutor.cpp"
  "$SKETCH_DIR/src/calibration/ContactProbeEngine.cpp"
  "$SKETCH_DIR/src/calibration/CalibrationExecutionEngine.cpp"
  "$SKETCH_DIR/src/calibration/CalibrationManager.cpp"
  "$SKETCH_DIR/src/calibration/CalibrationMotionPermit.cpp"
  "$SKETCH_DIR/src/calibration/CalibrationPopulationEvidence.cpp"
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp"
  "$SKETCH_DIR/src/actuator/OperationalEnvelope.cpp"
  "$SKETCH_DIR/src/actuator/MotionDeadman.cpp"
  "$SKETCH_DIR/src/actuator/ActuatorRuntime.cpp"
  "$SKETCH_DIR/src/actuator/ActuatorWritePolicy.cpp"
  "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp"
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp"
  "$SKETCH_DIR/src/actuator/CalibrationTargetResolver.cpp"
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp"
  "$SKETCH_DIR/src/servo/ServoPopulation.cpp"
  "$SKETCH_DIR/src/core/SystemState.cpp"
  "$SKETCH_DIR/src/core/ActuatorAuthority.cpp"
  "$SKETCH_DIR/src/core/OperatingMode.cpp"
)
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_record" \
  "$SCRIPT_DIR/test_calibration_record.cpp" "${CALREC_SRCS[@]}"
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_save_marker" \
  "$SCRIPT_DIR/test_calibration_save_marker.cpp" "${CALREC_SRCS[@]}"
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_persistence_state" \
  "$SCRIPT_DIR/test_calibration_persistence_state.cpp" "${CALREC_SRCS[@]}"
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_record_store" \
  "$SCRIPT_DIR/test_calibration_record_store.cpp" "${CALREC_SRCS[@]}"
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -I"$SCRIPT_DIR/nvs_stub" \
  -o "$OUT/test_calibration_record_nvs_backend" \
  "$SCRIPT_DIR/test_calibration_record_nvs_backend.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationRecordNvsBackend.cpp" "${CALREC_SRCS[@]}"

# Controller integration of the persistence (P3a): the service (boot LOAD
# verdicts, SAVE/ACK/RECONCILE entry points) against a fake storage, and the
# SAVE gate against the REAL Q0 capture session, promotion and transform table.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_persistence_service" \
  "$SCRIPT_DIR/test_calibration_persistence_service.cpp" "${CALREC_SRCS[@]}"
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_calibration_save_gate" \
  "$SCRIPT_DIR/test_calibration_save_gate.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationSaveGate.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationQ0CaptureSession.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0EvidencePreparation.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0Promotion.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0Bootstrap.cpp" \
  "${CALREC_SRCS[@]}"

# P3a.1/P3a.2: both REAL CommandRouter translation units, including USB framing,
# dispatch and explicit Q0 promotion. Platform transports / unrelated device
# service methods are fakes; the calibration and persistence code stays real.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -ffunction-sections -fdata-sections -Wl,--gc-sections \
  -DMATDOG_ACTIVE_HARDWARE_PROFILE=::matdog::config::HardwareProfile::ROBOT_POWERED \
  -DMATDOG_OTA_INGEST_ENABLED=0 -I"$SCRIPT_DIR/router_stubs" -I"$SCRIPT_DIR/nvs_stub" \
  -o "$OUT/test_command_router_persistence" \
  "$SCRIPT_DIR/test_command_router_persistence.cpp" \
  "$SCRIPT_DIR/router_fake_hardware.cpp" \
  "$SCRIPT_DIR/router_nvs_stub.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationRecordNvsBackend.cpp" \
  "$SKETCH_DIR/src/core/CommandRouter.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryQualification.cpp" \
  "$SKETCH_DIR/src/core/CommandRouterPersistence.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationSaveGate.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationQ0CaptureSession.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0EvidencePreparation.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0Promotion.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationQ0Bootstrap.cpp" \
  "$SKETCH_DIR/src/calibration/FirstMotionExecutor.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationSessionOrchestrator.cpp" \
  "$SKETCH_DIR/src/core/Availability.cpp" \
  "$SKETCH_DIR/src/core/PowerState.cpp" \
  "$SKETCH_DIR/src/core/ServiceReadiness.cpp" \
  "$SKETCH_DIR/src/servo/ServoProfile.cpp" \
  "$SKETCH_DIR/src/network/WifiPolicy.cpp" \
  "$SKETCH_DIR/src/network/NetworkConfig.cpp" \
  "$SKETCH_DIR/src/update/OtaPolicy.cpp" \
  "$SKETCH_DIR/src/update/OtaBootGuard.cpp" \
  "$SKETCH_DIR/src/status/LedStatusPolicy.cpp" \
  "$SKETCH_DIR/src/power/DalyProtocol.cpp" \
  "${CALREC_SRCS[@]}"

# The HostLink readiness classifier suite links the REAL pure classifier -
# no module pointer, no hardware call - I6 (2026-09-25 objective change).
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_service_readiness" \
  "$SCRIPT_DIR/test_service_readiness.cpp" \
  "$SKETCH_DIR/src/core/ServiceReadiness.cpp"

# The LED status suite links the REAL decision core; LedStatusPolicy.* has
# no <Arduino.h> and no LedRing dependency, so this drives the whole
# priority/effect table from a synthetic clock, the same contract as the
# Wi-Fi and OTA policy suites above.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_led_status_policy" \
  "$SCRIPT_DIR/test_led_status_policy.cpp" \
  "$SKETCH_DIR/src/status/LedStatusPolicy.cpp"

# Link the actual driver, manager and policy under both hardware profiles.
# Only the Arduino/NeoPixel transport is replaced with host observations.
for LED_PROFILE in USB_ONLY ROBOT_POWERED; do
  "$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
    "-DMATDOG_ACTIVE_HARDWARE_PROFILE=::matdog::config::HardwareProfile::$LED_PROFILE" \
    -I"$SCRIPT_DIR/led_stubs" \
    -o "$OUT/test_led_ring_manager_$LED_PROFILE" \
    "$SCRIPT_DIR/test_led_ring_manager.cpp" \
    "$SKETCH_DIR/src/status/LedRing.cpp" \
    "$SKETCH_DIR/src/status/LedStatusManager.cpp" \
    "$SKETCH_DIR/src/status/LedStatusPolicy.cpp" \
    "$SKETCH_DIR/src/core/Availability.cpp"
done

# HMAC-SHA256 (I7, 2026-09-25 correction), verified against the RFC 4231
# vectors, not self-consistency only. Links the REAL Sha256 it is built on.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 \
  -o "$OUT/test_hmac256" \
  "$SCRIPT_DIR/test_hmac256.cpp" \
  "$SKETCH_DIR/src/update/Hmac256.cpp" \
  "$SKETCH_DIR/src/update/Sha256.cpp"

# The OTA transport's authentication/session layer (I7). Links the REAL
# Hmac256/Sha256 it is built on. OtaPolicy.h is included only for the
# OtaImageMetadata type, so OtaPolicy.cpp is deliberately not linked here.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -DDISABLED=0x00 \
  -o "$OUT/test_ota_session" \
  "$SCRIPT_DIR/test_ota_session.cpp" \
  "$SKETCH_DIR/src/update/OtaSession.cpp" \
  "$SKETCH_DIR/src/update/Hmac256.cpp" \
  "$SKETCH_DIR/src/update/Sha256.cpp"

# HttpTransport's single-slot mailbox correlation logic (I7/I8 hardening,
# 2026-09-25). Header-only and pure: no .cpp to link.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 \
  -o "$OUT/test_http_mailbox" \
  "$SCRIPT_DIR/test_http_mailbox.cpp"

"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 \
  -o "$OUT/test_startup_recovery_qualification" \
  "$SCRIPT_DIR/test_startup_recovery_qualification.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryQualification.cpp" \
  "$SKETCH_DIR/src/calibration/StartupRecoveryReference.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationGeometryProfile.cpp" \
  "$SKETCH_DIR/src/actuator/CalibrationSequencePlan.cpp" \
  "$SKETCH_DIR/src/calibration/CalibrationDomain.cpp"

"$OUT/test_servo_population"
"$OUT/test_servo_profile"
"$OUT/test_daly_protocol"
"$OUT/test_wifi_policy"
"$OUT/test_actuator_authority"
"$OUT/test_actuator_write_policy"
"$OUT/test_calibration_geometry"
"$OUT/test_calibration_q0_bootstrap"
"$OUT/test_calibration_q0_capture_session"
"$OUT/test_cr3_q0_transform"
"$OUT/test_cr3_q0_evidence_preparation"
"$OUT/test_cr3_q0_fresh_promotion"
"$OUT/test_calibration_motion_permit"
"$OUT/test_motion_deadman"
"$OUT/test_thermal_confirmation"
"$OUT/test_servo_read_validation"
"$OUT/test_ota_policy"
"$OUT/test_ota_layout_contract"
"$OUT/test_calibration_domain"
"$OUT/test_calibration_population_evidence"
"$OUT/test_calibration_manager"
"$OUT/test_calibration_session_orchestrator"
"$OUT/test_actuator_runtime"
"$OUT/test_calibration_execution_engine"
"$OUT/test_first_motion_executor"
"$OUT/test_contact_probe_engine"
"$OUT/test_operational_envelope"
"$OUT/test_full_leg_calibration_executor"
"$OUT/test_startup_recovery_qualification"
"$OUT/test_full_leg_calibration_plan"
"$OUT/test_full_leg_calibration_finalizer"
"$OUT/test_calibration_record"
"$OUT/test_calibration_save_marker"
"$OUT/test_calibration_persistence_state"
"$OUT/test_calibration_record_store"
"$OUT/test_calibration_persistence_service"
"$OUT/test_calibration_save_gate"
"$OUT/test_command_router_persistence"
python3 "$SCRIPT_DIR/test_command_router_framing_mutations.py"
"$OUT/test_calibration_record_nvs_backend"
"$OUT/test_service_readiness"
"$OUT/test_led_status_policy"
"$OUT/test_led_ring_manager_USB_ONLY"
"$OUT/test_led_ring_manager_ROBOT_POWERED"
"$OUT/test_hmac256"
"$OUT/test_ota_session"
"$OUT/test_http_mailbox"

python3 "$SCRIPT_DIR/test_matdog_startup_reference.py"
python3 "$SCRIPT_DIR/test_static_audit_startup.py"

# V3: real configuration codec, NVS owner, transactions, USB fallback/roam,
# portal input/auth and concurrent mailbox delivery/abandon. No device I/O.
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -pthread \
  -I "$SCRIPT_DIR/nvs_stub" -o "$OUT/test_network_v3" \
  "$SCRIPT_DIR/test_network_v3.cpp" \
  "$SKETCH_DIR/src/network/NetworkConfig.cpp" \
  "$SKETCH_DIR/src/network/NetworkConfigNvs.cpp" \
  "$SKETCH_DIR/src/network/PortalSecurity.cpp" \
  "$SKETCH_DIR/src/update/Sha256.cpp"
"$OUT/test_network_v3"

"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 \
  -I "$SCRIPT_DIR/network_stubs" -I "$SCRIPT_DIR/nvs_stub" \
  -o "$OUT/test_wifi_runtime_v3" "$SCRIPT_DIR/test_wifi_runtime_v3.cpp" \
  "$SKETCH_DIR/src/network/WifiManager.cpp" "$SKETCH_DIR/src/network/WifiPolicy.cpp" \
  "$SKETCH_DIR/src/network/NetworkConfig.cpp" "$SKETCH_DIR/src/network/NetworkConfigNvs.cpp" \
  "$SKETCH_DIR/src/update/Sha256.cpp"
"$OUT/test_wifi_runtime_v3"
python3 "$SCRIPT_DIR/test_ota_tls_client.py"
python3 "$SCRIPT_DIR/test_static_audit_network_v3.py"

echo "HOST_TESTS = PASS"
