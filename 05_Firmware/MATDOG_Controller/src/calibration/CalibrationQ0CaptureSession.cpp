#include "CalibrationQ0CaptureSession.h"

#include "../actuator/CalibrationGeometryProfileData.h"

namespace matdog {
namespace calibration {

namespace {

constexpr int32_t kRawTickMin = 0;
constexpr int32_t kRawTickMax = 4095;

bool validConfig(const Q0CaptureConfig& config) {
  return config.samples_per_joint >= actuator::kQ0BootstrapMinSamples &&
         config.samples_per_joint <= actuator::kQ0BootstrapMaxSamples &&
         config.stability_budget_specified &&
         config.max_stability_spread_ticks < 2048 &&
         config.nominal_zero_pose_confirmed;
}

}  // namespace

uint32_t CalibrationQ0CaptureSession::allocateSessionId() {
  uint32_t id = next_session_id_++;
  if (id == 0) id = next_session_id_++;
  if (next_session_id_ == 0) next_session_id_ = 1;
  return id;
}

void CalibrationQ0CaptureSession::clearTransactionState() {
  config_ = Q0CaptureConfig{};
  status_ = Q0CaptureStatus{};
  census_ = servo::CensusResult{};
  preflight_ = servo::PreflightResult{};
  population_ = PopulationEvidenceBuildResult{};
  profile_.clear();
  for (uint8_t j = 0; j < kLegServoSlotCount; ++j) {
    candidates_[j] = actuator::Q0BootstrapCandidate{};
    for (uint8_t s = 0; s < actuator::kQ0BootstrapMaxSamples; ++s) {
      samples_[j][s] = actuator::Q0CaptureSample{};
    }
  }
}

void CalibrationQ0CaptureSession::reset() {
  clearTransactionState();
}

bool CalibrationQ0CaptureSession::active() const {
  switch (status_.state) {
    case Q0CaptureState::NEED_CENSUS_START:
    case Q0CaptureState::WAIT_CENSUS:
    case Q0CaptureState::NEED_PREFLIGHT_START:
    case Q0CaptureState::WAIT_PREFLIGHT:
    case Q0CaptureState::SAMPLING:
      return true;
    default:
      return false;
  }
}

bool CalibrationQ0CaptureSession::start(const Q0CaptureConfig& config) {
  if (active()) return false;
  clearTransactionState();
  if (!validConfig(config)) {
    status_.state = Q0CaptureState::FAILED;
    status_.failure = Q0CaptureFailure::INVALID_CONFIG;
    return false;
  }

  config_ = config;
  status_.capture_session_id = allocateSessionId();
  status_.samples_per_joint = config.samples_per_joint;
  profile_.bind(&actuator::geometry_data::kProvenance,
                actuator::geometry_data::kJoints,
                actuator::geometry_data::kJointCount,
                actuator::geometry_data::kEndpoints,
                actuator::geometry_data::kEndpointCount);
  if (!profile_.bound() ||
      !profile_.provenanceMatches(actuator::geometry_data::kProvenance)) {
    status_.state = Q0CaptureState::FAILED;
    status_.failure = Q0CaptureFailure::GEOMETRY_REJECTED;
    return false;
  }

  status_.state = Q0CaptureState::NEED_CENSUS_START;
  return true;
}

bool CalibrationQ0CaptureSession::markCensusStarted() {
  if (status_.state != Q0CaptureState::NEED_CENSUS_START) return false;
  status_.state = Q0CaptureState::WAIT_CENSUS;
  return true;
}

bool CalibrationQ0CaptureSession::submitCensus(const servo::CensusResult& census) {
  if (status_.state != Q0CaptureState::WAIT_CENSUS) return false;
  census_ = census;
  status_.state = Q0CaptureState::NEED_PREFLIGHT_START;
  return true;
}

bool CalibrationQ0CaptureSession::markPreflightStarted() {
  if (status_.state != Q0CaptureState::NEED_PREFLIGHT_START) return false;
  status_.state = Q0CaptureState::WAIT_PREFLIGHT;
  return true;
}

bool CalibrationQ0CaptureSession::submitPreflight(const servo::PreflightResult& preflight) {
  if (status_.state != Q0CaptureState::WAIT_PREFLIGHT) return false;
  preflight_ = preflight;

  PopulationEvidenceBuildContext context{};
  context.current_observation_bundle = true;
  context.session_ms = config_.started_at_ms;
  population_ = buildCurrentLegPopulationEvidence(census_, preflight_, context);
  status_.population_status = population_.status;
  if (population_.status != PopulationEvidenceBuildStatus::PASS ||
      !populationIsCurrentPass(population_.evidence)) {
    status_.state = Q0CaptureState::FAILED;
    status_.failure = Q0CaptureFailure::POPULATION_REJECTED;
    return false;
  }

  status_.state = Q0CaptureState::SAMPLING;
  status_.completed_sample_passes = 0;
  status_.next_joint_index = 0;
  return true;
}

bool CalibrationQ0CaptureSession::buildReadRequest(uint8_t joint_index,
                                                   uint8_t sample_pass,
                                                   Q0ReadRequest* out) const {
  if (out == nullptr || joint_index >= kLegServoSlotCount ||
      sample_pass >= config_.samples_per_joint) return false;

  const servo::CanonicalServo* canonical = servo::legServoAt(joint_index);
  if (canonical == nullptr) return false;
  JointIdentity identity{};
  if (!semanticIdentityFromCanonical(*canonical, &identity)) return false;
  const actuator::GeometryJointRecord* geometry = profile_.findJoint(identity);
  if (geometry == nullptr || geometry->bus_id != canonical->bus_id) return false;

  *out = Q0ReadRequest{};
  out->valid = true;
  out->bus_id = canonical->bus_id;
  out->identity = identity;
  out->joint_index = joint_index;
  out->sample_pass = sample_pass;
  return true;
}

bool CalibrationQ0CaptureSession::nextReadRequest(Q0ReadRequest* out) const {
  if (status_.state != Q0CaptureState::SAMPLING) return false;
  return buildReadRequest(status_.next_joint_index,
                          status_.completed_sample_passes, out);
}

bool CalibrationQ0CaptureSession::recordRead(const Q0ReadObservation& observation) {
  if (status_.state != Q0CaptureState::SAMPLING) return false;

  Q0ReadRequest expected{};
  if (!nextReadRequest(&expected) || observation.bus_id != expected.bus_id) {
    status_.state = Q0CaptureState::FAILED;
    status_.failure = Q0CaptureFailure::WRONG_STATE;
    return false;
  }
  if (!observation.read_ok) {
    status_.state = Q0CaptureState::FAILED;
    status_.failure = Q0CaptureFailure::READ_FAILED;
    return false;
  }
  if (observation.torque_enable != 0) {
    status_.state = Q0CaptureState::FAILED;
    status_.failure = Q0CaptureFailure::TORQUE_NOT_OFF;
    return false;
  }
  if (observation.raw_tick < kRawTickMin || observation.raw_tick > kRawTickMax) {
    status_.state = Q0CaptureState::FAILED;
    status_.failure = Q0CaptureFailure::RAW_DOMAIN;
    return false;
  }

  actuator::Q0CaptureSample& sample =
      samples_[expected.joint_index][expected.sample_pass];
  sample.read_ok = true;
  sample.raw_tick = observation.raw_tick;
  sample.torque_enable = observation.torque_enable;

  status_.next_joint_index++;
  if (status_.next_joint_index >= kLegServoSlotCount) {
    status_.next_joint_index = 0;
    status_.completed_sample_passes++;
  }

  if (status_.completed_sample_passes >= config_.samples_per_joint) {
    return finalizeCandidates();
  }
  return true;
}

bool CalibrationQ0CaptureSession::finalizeCandidates() {
  uint8_t completed = 0;
  for (uint8_t i = 0; i < kLegServoSlotCount; ++i) {
    const servo::CanonicalServo* canonical = servo::legServoAt(i);
    if (canonical == nullptr) {
      status_.state = Q0CaptureState::FAILED;
      status_.failure = Q0CaptureFailure::GEOMETRY_REJECTED;
      return false;
    }
    JointIdentity identity{};
    if (!semanticIdentityFromCanonical(*canonical, &identity)) {
      status_.state = Q0CaptureState::FAILED;
      status_.failure = Q0CaptureFailure::GEOMETRY_REJECTED;
      return false;
    }

    actuator::Q0BootstrapRequest request{};
    request.identity = identity;
    request.bus_id = canonical->bus_id;
    request.capture_session_id = status_.capture_session_id;
    request.nominal_zero_pose_confirmed = config_.nominal_zero_pose_confirmed;
    request.stability_budget_specified = config_.stability_budget_specified;
    request.max_stability_spread_ticks = config_.max_stability_spread_ticks;

    candidates_[i] = actuator::buildQ0BootstrapCandidate(
        profile_, actuator::geometry_data::kProvenance, population_.evidence,
        request, samples_[i], config_.samples_per_joint);
    if (candidates_[i].status != actuator::Q0BootstrapStatus::CANDIDATE) {
      status_.candidates_complete = completed;
      status_.state = Q0CaptureState::FAILED;
      status_.failure = Q0CaptureFailure::CANDIDATE_REJECTED;
      return false;
    }
    ++completed;
  }

  status_.candidates_complete = completed;
  status_.state = Q0CaptureState::COMPLETE;
  status_.failure = Q0CaptureFailure::NONE;
  return true;
}

actuator::FreshQ0Capture CalibrationQ0CaptureSession::freshCapture() const {
  actuator::FreshQ0Capture view{};
  view.complete = status_.state == Q0CaptureState::COMPLETE &&
                  status_.failure == Q0CaptureFailure::NONE &&
                  status_.candidates_complete == kLegServoSlotCount;
  view.population_pass = population_.status == PopulationEvidenceBuildStatus::PASS &&
                         populationIsCurrentPass(population_.evidence);
  view.capture_session_id = status_.capture_session_id;
  view.candidates = candidates_;
  view.candidate_count = kLegServoSlotCount;
  return view;
}

bool CalibrationQ0CaptureSession::notePromotionCompleted(
    uint32_t capture_session_id, uint8_t admitted_joints,
    actuator::GeometryProvenanceTag geometry) {
  status_.promoted_capture_session_id = 0;
  status_.promoted_geometry = actuator::kNoGeometryProvenance;
  if (!freshCapture().complete || capture_session_id == 0 ||
      capture_session_id != status_.capture_session_id ||
      admitted_joints != kLegServoSlotCount || geometry == actuator::kNoGeometryProvenance ||
      geometry != profile_.provenanceTag()) {
    return false;
  }
  status_.promoted_capture_session_id = capture_session_id;
  status_.promoted_geometry = geometry;
  return true;
}

void CalibrationQ0CaptureSession::fail(Q0CaptureFailure failure) {
  if (!active()) return;
  status_.state = Q0CaptureState::FAILED;
  status_.failure = failure == Q0CaptureFailure::NONE
                        ? Q0CaptureFailure::EXTERNAL_ABORT
                        : failure;
}

const char* toString(Q0CaptureState state) {
  switch (state) {
    case Q0CaptureState::IDLE:                 return "IDLE";
    case Q0CaptureState::NEED_CENSUS_START:    return "NEED_CENSUS_START";
    case Q0CaptureState::WAIT_CENSUS:          return "WAIT_CENSUS";
    case Q0CaptureState::NEED_PREFLIGHT_START: return "NEED_PREFLIGHT_START";
    case Q0CaptureState::WAIT_PREFLIGHT:       return "WAIT_PREFLIGHT";
    case Q0CaptureState::SAMPLING:             return "SAMPLING";
    case Q0CaptureState::COMPLETE:             return "COMPLETE";
    case Q0CaptureState::FAILED:               return "FAILED";
  }
  return "UNKNOWN";
}

const char* toString(Q0CaptureFailure failure) {
  switch (failure) {
    case Q0CaptureFailure::NONE:                return "NONE";
    case Q0CaptureFailure::INVALID_CONFIG:      return "INVALID_CONFIG";
    case Q0CaptureFailure::WRONG_STATE:         return "WRONG_STATE";
    case Q0CaptureFailure::POPULATION_REJECTED: return "POPULATION_REJECTED";
    case Q0CaptureFailure::GEOMETRY_REJECTED:   return "GEOMETRY_REJECTED";
    case Q0CaptureFailure::READ_FAILED:         return "READ_FAILED";
    case Q0CaptureFailure::TORQUE_NOT_OFF:      return "TORQUE_NOT_OFF";
    case Q0CaptureFailure::RAW_DOMAIN:          return "RAW_DOMAIN";
    case Q0CaptureFailure::CANDIDATE_REJECTED:   return "CANDIDATE_REJECTED";
    case Q0CaptureFailure::MODE_NOT_MAINTENANCE: return "MODE_NOT_MAINTENANCE";
    case Q0CaptureFailure::CENSUS_START_REFUSED: return "CENSUS_START_REFUSED";
    case Q0CaptureFailure::PREFLIGHT_START_REFUSED: return "PREFLIGHT_START_REFUSED";
    case Q0CaptureFailure::EXTERNAL_ABORT:       return "EXTERNAL_ABORT";
  }
  return "UNKNOWN";
}

}  // namespace calibration
}  // namespace matdog
