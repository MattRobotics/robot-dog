#include "SystemState.h"

namespace matdog {
namespace core {

const char* toString(ModuleHealth health) {
  switch (health) {
    case ModuleHealth::NOT_INITIALIZED: return "NOT_INITIALIZED";
    case ModuleHealth::OK:              return "OK";
    case ModuleHealth::DEGRADED:        return "DEGRADED";
    case ModuleHealth::OFFLINE:         return "OFFLINE";
    case ModuleHealth::FAULT:           return "FAULT";
  }
  return "UNKNOWN";
}

const char* toString(SystemHealth health) {
  switch (health) {
    case SystemHealth::BOOTING:     return "BOOTING";
    case SystemHealth::READY:       return "READY";
    case SystemHealth::DEGRADED:    return "DEGRADED";
    case SystemHealth::MAINTENANCE: return "MAINTENANCE";
    case SystemHealth::FAULT:       return "FAULT";
  }
  return "UNKNOWN";
}

void SystemState::beginBoot(uint32_t now_ms) {
  boot_millis_ = now_ms;
  system_health_ = SystemHealth::BOOTING;
}

SystemHealth SystemState::update() {
  const ModuleHealth modules[] = {imu_health_, servo_health_, bms_health_, led_health_};

  bool any_fault = false;
  bool any_degraded_or_offline = false;
  bool any_not_initialized = false;

  for (ModuleHealth h : modules) {
    if (h == ModuleHealth::FAULT) any_fault = true;
    if (h == ModuleHealth::DEGRADED || h == ModuleHealth::OFFLINE) any_degraded_or_offline = true;
    if (h == ModuleHealth::NOT_INITIALIZED) any_not_initialized = true;
  }

  if (any_fault) {
    system_health_ = SystemHealth::FAULT;
  } else if (any_not_initialized) {
    system_health_ = SystemHealth::BOOTING;
  } else if (any_degraded_or_offline) {
    system_health_ = SystemHealth::DEGRADED;
  } else {
    system_health_ = SystemHealth::READY;
  }

  return system_health_;
}

}  // namespace core
}  // namespace matdog
