#ifndef MATDOG_EXECUTION_REVIEWED_MOTION_RUNTIME_H
#define MATDOG_EXECUTION_REVIEWED_MOTION_RUNTIME_H
#include "MotionCoordinator.h"
namespace matdog { namespace execution {
// Local compiled qualification boundary. No network deserializer, default
// artifact or inferred approval exists. RESTORE/limits must already be admitted.
struct ReviewedMotionRuntime {
 TrialScope scope=TrialScope::FIXTURE_MICRO;
 uint32_t calibration_generation=0,policy_epoch=0;
 char route_sha256[65]{};
 motion::FeedbackPolicy feedback{};
 motion::WalkSpecification walk{};
 ExecutionEnvelope execution{};
 uint32_t max_imu_age_ms=0;
 int max_servo_temperature_c=0,min_servo_voltage_dv=0,max_servo_voltage_dv=0;
 bool fixture_bus_timing_qualified=false,autonomous_bus_timing_qualified=false;
};
} }
#endif
