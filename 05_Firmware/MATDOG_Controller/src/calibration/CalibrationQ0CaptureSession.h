#ifndef MATDOG_CALIBRATION_CALIBRATION_Q0_CAPTURE_SESSION_H
#define MATDOG_CALIBRATION_CALIBRATION_Q0_CAPTURE_SESSION_H

#include <stdint.h>

#include "CalibrationPopulationEvidence.h"
#include "../actuator/CalibrationQ0Bootstrap.h"
#include "../actuator/CalibrationGeometryProfile.h"
#include "../servo/ServoPopulation.h"
#include "../servo/ServoPreflight.h"

namespace matdog {
namespace calibration {

// CR2-B: one read-only acquisition transaction spanning census, preflight and
// round-robin q0 observations. Pure orchestration only: no bus transport, no UART,
// no Arduino clock and no command surface live here.
enum class Q0CaptureState : uint8_t {
  IDLE = 0,
  NEED_CENSUS_START = 1,
  WAIT_CENSUS = 2,
  NEED_PREFLIGHT_START = 3,
  WAIT_PREFLIGHT = 4,
  SAMPLING = 5,
  COMPLETE = 6,
  FAILED = 7,
};

enum class Q0CaptureFailure : uint8_t {
  NONE = 0,
  INVALID_CONFIG = 1,
  WRONG_STATE = 2,
  POPULATION_REJECTED = 3,
  GEOMETRY_REJECTED = 4,
  READ_FAILED = 5,
  TORQUE_NOT_OFF = 6,
  RAW_DOMAIN = 7,
  CANDIDATE_REJECTED = 8,
  MODE_NOT_MAINTENANCE = 9,
  CENSUS_START_REFUSED = 10,
  PREFLIGHT_START_REFUSED = 11,
  EXTERNAL_ABORT = 12,
};

struct Q0CaptureConfig {
  uint8_t samples_per_joint = 0;
  bool stability_budget_specified = false;
  uint16_t max_stability_spread_ticks = 0;
  bool nominal_zero_pose_confirmed = false;
  uint32_t started_at_ms = 0;
};

struct Q0ReadRequest {
  bool valid = false;
  uint8_t bus_id = 0;
  JointIdentity identity{};
  uint8_t joint_index = 0;
  uint8_t sample_pass = 0;
};

struct Q0ReadObservation {
  uint8_t bus_id = 0;
  bool read_ok = false;
  int32_t raw_tick = -1;
  int32_t torque_enable = -1;
};

struct Q0CaptureStatus {
  Q0CaptureState state = Q0CaptureState::IDLE;
  Q0CaptureFailure failure = Q0CaptureFailure::NONE;
  uint32_t capture_session_id = 0;
  uint8_t samples_per_joint = 0;
  uint8_t completed_sample_passes = 0;
  uint8_t next_joint_index = 0;
  PopulationEvidenceBuildStatus population_status =
      PopulationEvidenceBuildStatus::NOT_EVALUATED;
  uint8_t candidates_complete = 0;
};

class CalibrationQ0CaptureSession {
 public:
  bool start(const Q0CaptureConfig& config);

  bool markCensusStarted();
  bool submitCensus(const servo::CensusResult& census);
  bool markPreflightStarted();
  bool submitPreflight(const servo::PreflightResult& preflight);

  bool nextReadRequest(Q0ReadRequest* out) const;
  bool recordRead(const Q0ReadObservation& observation);

  // Used only by the Controller orchestration boundary when an external
  // prerequisite disappears (for example MAINTENANCE mode is left).
  void fail(Q0CaptureFailure failure);
  void reset();

  bool active() const;
  bool terminal() const {
    return status_.state == Q0CaptureState::COMPLETE ||
           status_.state == Q0CaptureState::FAILED;
  }

  const Q0CaptureStatus& status() const { return status_; }
  const PopulationEvidenceBuildResult& populationResult() const { return population_; }
  const actuator::Q0BootstrapCandidate* candidates() const { return candidates_; }

  // The CR3 promotion view of this capture. `complete` is true only for a
  // failure-free COMPLETE capture whose 12 candidates were all built.
  actuator::FreshQ0Capture freshCapture() const;

 private:
  bool buildReadRequest(uint8_t joint_index, uint8_t sample_pass,
                        Q0ReadRequest* out) const;
  bool finalizeCandidates();
  void clearTransactionState();
  uint32_t allocateSessionId();

  uint32_t next_session_id_ = 1;
  Q0CaptureConfig config_{};
  Q0CaptureStatus status_{};
  servo::CensusResult census_{};
  servo::PreflightResult preflight_{};
  PopulationEvidenceBuildResult population_{};
  actuator::CalibrationGeometryProfile profile_{};
  actuator::Q0CaptureSample
      samples_[kLegServoSlotCount][actuator::kQ0BootstrapMaxSamples]{};
  actuator::Q0BootstrapCandidate candidates_[kLegServoSlotCount]{};
};

const char* toString(Q0CaptureState state);
const char* toString(Q0CaptureFailure failure);

}  // namespace calibration
}  // namespace matdog

#endif  // MATDOG_CALIBRATION_CALIBRATION_Q0_CAPTURE_SESSION_H
