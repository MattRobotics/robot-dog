#ifndef MATDOG_CORE_SYSTEM_STATE_H
#define MATDOG_CORE_SYSTEM_STATE_H

#include <stdint.h>

namespace matdog {
namespace core {

// Per-module health. Deliberately small: this is not the final locomotion
// safety state machine, only enough to answer "is this peripheral usable".
enum class ModuleHealth : uint8_t {
  NOT_INITIALIZED = 0,
  OK              = 1,
  DEGRADED        = 2,
  OFFLINE         = 3,  // expected-absent hardware (bench profile) or lost comms
  FAULT           = 4,
};

// System-wide aggregated health.
enum class SystemHealth : uint8_t {
  BOOTING     = 0,
  READY       = 1,
  DEGRADED    = 2,
  MAINTENANCE = 3,
  FAULT       = 4,
};

const char* toString(ModuleHealth health);
const char* toString(SystemHealth health);

// Aggregates module health into one system health value.
// Rule: a single OFFLINE/DEGRADED noncritical module degrades the system,
// it never faults it. FAULT only propagates from a module that reports FAULT.
class SystemState {
 public:
  // Takes the current time rather than calling millis() itself (G2): it
  // keeps this whole translation unit free of the Arduino runtime, so the
  // offline host tests can link the REAL aggregation in update() instead
  // of a reimplementation. Controller already threads now_ms everywhere
  // else, so this matches the surrounding style.
  void beginBoot(uint32_t now_ms);

  void setImuHealth(ModuleHealth health)   { imu_health_ = health; }
  void setServoHealth(ModuleHealth health) { servo_health_ = health; }
  void setBmsHealth(ModuleHealth health)   { bms_health_ = health; }
  void setLedHealth(ModuleHealth health)   { led_health_ = health; }

  ModuleHealth imuHealth() const   { return imu_health_; }
  ModuleHealth servoHealth() const { return servo_health_; }
  ModuleHealth bmsHealth() const   { return bms_health_; }
  ModuleHealth ledHealth() const   { return led_health_; }

  // Recomputes and returns the aggregated system health.
  SystemHealth update();

  SystemHealth systemHealth() const { return system_health_; }
  uint32_t bootMillis() const { return boot_millis_; }
  uint32_t uptimeMillis(uint32_t now) const { return now - boot_millis_; }

 private:
  ModuleHealth imu_health_   = ModuleHealth::NOT_INITIALIZED;
  ModuleHealth servo_health_ = ModuleHealth::NOT_INITIALIZED;
  ModuleHealth bms_health_   = ModuleHealth::NOT_INITIALIZED;
  ModuleHealth led_health_   = ModuleHealth::NOT_INITIALIZED;

  SystemHealth system_health_ = SystemHealth::BOOTING;
  uint32_t boot_millis_ = 0;
};

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_SYSTEM_STATE_H
