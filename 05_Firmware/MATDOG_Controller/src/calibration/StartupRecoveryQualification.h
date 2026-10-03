#ifndef MATDOG_CALIBRATION_STARTUP_RECOVERY_QUALIFICATION_H
#define MATDOG_CALIBRATION_STARTUP_RECOVERY_QUALIFICATION_H
#include "StartupRecoveryReference.h"
#include "CalibrationPopulationEvidence.h"

namespace matdog { namespace calibration {
enum class StartupQualificationPhase : uint8_t { IDLE, CENSUS_START, CENSUS_WAIT,
  PREFLIGHT_START, PREFLIGHT_WAIT, OBSERVING, READY, EXECUTING, COMPLETE, REFUSED };
class StartupRecoveryQualification {
 public:
  bool start(uint32_t now_ms);
  void censusStarted() { phase_=StartupQualificationPhase::CENSUS_WAIT; }
  void censusComplete() { phase_=StartupQualificationPhase::PREFLIGHT_START; }
  void preflightStarted() { phase_=StartupQualificationPhase::PREFLIGHT_WAIT; }
  void population(const PopulationEvidenceBuildResult& result);
  void observe(const actuator::TelemetrySample& sample,uint32_t now_ms);
  void refuse(const char* reason,uint8_t failed_bus=0) {
    failed_bus_=failed_bus ? failed_bus : phase_==StartupQualificationPhase::OBSERVING ? bus() : 0;
    reason_=reason;phase_=StartupQualificationPhase::REFUSED;
  }
  bool ready(uint32_t now_ms) const;
  bool consumeForExecution(uint32_t now_ms);
  void complete(bool ok) { phase_=ok?StartupQualificationPhase::COMPLETE:StartupQualificationPhase::REFUSED; }
  StartupQualificationPhase phase() const { return phase_; }
  const char* reason() const { return reason_; }
  uint8_t failedBus() const { return failed_bus_; }
  bool nominal() const { return nominal_; }
  uint8_t bus() const { return kStartupReference[index_].bus; }
  uint32_t startedMs() const { return started_ms_; }
  const LegPopulationEvidence& populationEvidence() const { return population_; }
 private:
  StartupQualificationPhase phase_=StartupQualificationPhase::IDLE;
  const char* reason_="NONE";
  uint32_t started_ms_=0,completed_ms_=0;
  uint8_t index_=0,pass_=0;
  uint8_t failed_bus_=0;
  int32_t first_[12]={0},last_[12]={0};
  bool nominal_=false;
  LegPopulationEvidence population_{};
};
} }
#endif
