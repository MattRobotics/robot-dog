#include "ServiceReadiness.h"

namespace matdog {
namespace core {

ServiceReadiness classifyCapability(ServiceCapability capability,
                                    const ServiceReadinessInputs& inputs) {
  switch (capability) {
    case ServiceCapability::ACTUATOR_TORQUE_ENABLE:
    case ServiceCapability::ACTUATOR_POSITION_COMMAND:
    case ServiceCapability::CALIBRATION_CONTACT_PROBE:
    case ServiceCapability::CALIBRATION_AUXILIARY_MOVE:
      // Every one of these resolves to REJECT_NO_ACCEPTED_LIMITS /
      // REJECT_NO_ACCEPTED_TRANSFORM / a hardware_motion_authorized==0
      // refusal today — structurally blocked, not merely untested. See
      // ActuatorWritePolicy.cpp / CalibrationExecutionEngine.cpp. Even once
      // hardware motion is authorized, these still need current q0/limits
      // this build does not have, so the honest answer is TO_TEST, not
      // READY.
      return inputs.hardware_motion_authorized ? ServiceReadiness::TO_TEST
                                               : ServiceReadiness::BLOCKED;
    case ServiceCapability::CALIBRATION_DIRECTION_VERIFY:
      // Optional diagnostic; its budget defaults to 0 (not authorised) but
      // is not gated on hardware_motion_authorized the way a calibration
      // move is — see CalibrationBootstrapContext.direction_verify_tick_budget.
      return ServiceReadiness::TO_TEST;
    case ServiceCapability::WIFI_HARDWARE_ASSOCIATION:
      return inputs.wifi_hardware_validated ? ServiceReadiness::READY
                                            : ServiceReadiness::TO_TEST;
    case ServiceCapability::OTA_END_TO_END:
      return inputs.ota_hardware_validated ? ServiceReadiness::READY
                                           : ServiceReadiness::TO_TEST;
    case ServiceCapability::WEB_READ_ONLY_DASHBOARD:
      // DEVELOPMENT_GATES.md's UI-1 entry is Wi-Fi runtime PASS — until
      // then this is a harder refusal than "untested", it is BLOCKED by
      // the same entry condition I8 deferred on.
      return inputs.wifi_hardware_validated ? ServiceReadiness::TO_TEST
                                            : ServiceReadiness::BLOCKED;
  }
  return ServiceReadiness::BLOCKED;
}

const char* toString(ServiceReadiness readiness) {
  switch (readiness) {
    case ServiceReadiness::READY:   return "READY";
    case ServiceReadiness::TO_TEST: return "TO_TEST";
    case ServiceReadiness::BLOCKED: return "BLOCKED";
  }
  return "UNKNOWN";
}

const char* toString(ServiceCapability capability) {
  switch (capability) {
    case ServiceCapability::ACTUATOR_TORQUE_ENABLE:       return "ACTUATOR_TORQUE_ENABLE";
    case ServiceCapability::ACTUATOR_POSITION_COMMAND:    return "ACTUATOR_POSITION_COMMAND";
    case ServiceCapability::CALIBRATION_CONTACT_PROBE:    return "CALIBRATION_CONTACT_PROBE";
    case ServiceCapability::CALIBRATION_AUXILIARY_MOVE:   return "CALIBRATION_AUXILIARY_MOVE";
    case ServiceCapability::CALIBRATION_DIRECTION_VERIFY: return "CALIBRATION_DIRECTION_VERIFY";
    case ServiceCapability::WIFI_HARDWARE_ASSOCIATION:    return "WIFI_HARDWARE_ASSOCIATION";
    case ServiceCapability::OTA_END_TO_END:               return "OTA_END_TO_END";
    case ServiceCapability::WEB_READ_ONLY_DASHBOARD:      return "WEB_READ_ONLY_DASHBOARD";
  }
  return "UNKNOWN";
}

}  // namespace core
}  // namespace matdog
