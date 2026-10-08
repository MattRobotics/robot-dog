#include "StartupRecoveryQualification.h"
#include "ThermalConfirmation.h"
namespace matdog { namespace calibration {
bool StartupRecoveryQualification::start(uint32_t now_ms) {
  if (phase_!=StartupQualificationPhase::IDLE) return false;
  started_ms_=now_ms;phase_=StartupQualificationPhase::CENSUS_START;return true;
}
void StartupRecoveryQualification::population(const PopulationEvidenceBuildResult& r) {
  if (phase_!=StartupQualificationPhase::PREFLIGHT_WAIT || r.status!=PopulationEvidenceBuildStatus::PASS ||
      !populationIsCurrentPass(r.evidence)) { refuse("STARTUP_POPULATION_OR_PROFILE_FAILED");return; }
  population_=r.evidence;phase_=StartupQualificationPhase::OBSERVING;
}
void StartupRecoveryQualification::observe(const actuator::TelemetrySample& s,uint32_t now_ms) {
  if (phase_!=StartupQualificationPhase::OBSERVING) return;
  if (!s.read_ok) { refuse("STARTUP_UART_OR_READBACK_UNAVAILABLE");return; }
  if (s.sampled_at_ms>now_ms || now_ms-s.sampled_at_ms>kSequenceMaxTelemetryAgeMs) { refuse("STARTUP_READBACK_STALE");return; }
  if (s.present_position<0 || s.present_position>=4096) { refuse("STARTUP_POSITION_INVALID");return; }
  if (s.torque_enable!=0) { refuse("STARTUP_TORQUE_NOT_OFF");return; }
  if (s.present_speed<0 || (s.present_speed&0x7FFF)>kSequenceSettleMaxSpeedRaw) { refuse("STARTUP_SPEED_NOT_STABLE");return; }
  if (s.present_current<0 || (s.present_current&0x7FFF)>=kSearchHardCurrentAbortRaw) { refuse("STARTUP_CURRENT_UNAVAILABLE_OR_HIGH");return; }
  if (s.present_temperature<0 || s.present_temperature>kThermalLimitC) { refuse("STARTUP_THERMAL_UNAVAILABLE_OR_HIGH");return; }
  if (s.servo_status!=0) { refuse("STARTUP_SERVO_STATUS_FAULT");return; }
  if (pass_==0) first_[index_]=s.present_position;
  const int32_t delta=s.present_position-first_[index_];
  if (delta < -4 || delta > 4) { refuse("STARTUP_POSE_NOT_STABLE");return; }
  last_[index_]=s.present_position;
  if (++index_<12) return;
  index_=0;
  if (++pass_<3) return;
  nominal_=true;bool residual=true;uint8_t bad_bus=0;
  for (uint8_t i=0;i<12;++i) {
    const int32_t d=last_[i]-kStartupReference[i].q0;
    nominal_=nominal_ && d>=-10 && d<=10;
    const bool compatible=startupPositionInBand(kStartupReference[i].bus,last_[i],CalibrationPhase::PREFLIGHT);
    if (!compatible && !bad_bus) bad_bus=kStartupReference[i].bus;
    residual=residual && compatible;
  }
  if (!nominal_ && !residual) { refuse("STARTUP_POSE_UNRECOGNIZED",bad_bus);return; }
  completed_ms_=now_ms;phase_=StartupQualificationPhase::READY;
}
bool StartupRecoveryQualification::ready(uint32_t now_ms) const {
  return phase_==StartupQualificationPhase::READY && now_ms-completed_ms_<=kSequenceMaxTelemetryAgeMs;
}
bool StartupRecoveryQualification::consumeForExecution(uint32_t now_ms) {
  if (!ready(now_ms) || nominal_) { refuse("STARTUP_QUALIFICATION_STALE_OR_NO_RECOVERY_REQUIRED");return false; }
  phase_=StartupQualificationPhase::EXECUTING;return true;
}
} }
