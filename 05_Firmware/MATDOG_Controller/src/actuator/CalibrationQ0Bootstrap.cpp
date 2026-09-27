#include "CalibrationQ0Bootstrap.h"

namespace matdog {
namespace actuator {

namespace {

constexpr int32_t kRawMin = 0;
constexpr int32_t kRawMax = 4095;
constexpr int32_t kRawModulus = 4096;
constexpr int32_t kRawHalfTurn = 2048;

int32_t circularEncoderDelta(int32_t value, int32_t reference) {
  int32_t delta = value - reference;
  // Match the historical read-only capture semantics at the wrap boundary:
  // +2048 is represented as -2048. This is encoder measurement arithmetic,
  // never GoalPosition target arithmetic.
  while (delta >= kRawHalfTurn) delta -= kRawModulus;
  while (delta < -kRawHalfTurn) delta += kRawModulus;
  return delta;
}

uint16_t wrapRawTick(int32_t unwrapped) {
  int32_t v = unwrapped % kRawModulus;
  if (v < 0) v += kRawModulus;
  return static_cast<uint16_t>(v);
}

uint16_t absDeltaMagnitude(int32_t a, int32_t b) {
  int32_t d = circularEncoderDelta(a, b);
  if (d < 0) d = -d;
  return static_cast<uint16_t>(d);
}

void insertionSort(int32_t* values, uint8_t count) {
  for (uint8_t i = 1; i < count; ++i) {
    const int32_t value = values[i];
    int j = static_cast<int>(i) - 1;
    while (j >= 0 && values[j] > value) {
      values[j + 1] = values[j];
      --j;
    }
    values[j + 1] = value;
  }
}

bool requestWellFormed(const Q0BootstrapRequest& request) {
  if (!request.identity.valid() || !request.identity.unitKnown()) return false;
  if (request.capture_session_id == 0) return false;
  // 2048 would make any circular sample cloud stable by definition.
  if (request.max_stability_spread_ticks >= kRawHalfTurn) return false;
  return true;
}

}  // namespace

Q0BootstrapCandidate buildQ0BootstrapCandidate(
    const CalibrationGeometryProfile& profile,
    const GeometryProvenance& expected_provenance,
    const calibration::LegPopulationEvidence& population,
    const Q0BootstrapRequest& request,
    const Q0CaptureSample* samples,
    uint8_t sample_count) {
  Q0BootstrapCandidate out{};

  if (!calibration::populationIsCurrentPass(population)) {
    out.status = Q0BootstrapStatus::REJECT_POPULATION_NOT_CURRENT;
    return out;
  }

  if (!profile.bound()) {
    out.status = Q0BootstrapStatus::REJECT_GEOMETRY_UNBOUND;
    return out;
  }
  if (!profile.provenanceMatches(expected_provenance) ||
      profile.provenanceTag() == kNoGeometryProvenance) {
    out.status = Q0BootstrapStatus::REJECT_GEOMETRY_PROVENANCE;
    return out;
  }

  if (!requestWellFormed(request)) {
    out.status = Q0BootstrapStatus::REJECT_INVALID_REQUEST;
    return out;
  }

  const GeometryJointRecord* joint = profile.findJoint(request.identity);
  if (joint == nullptr) {
    out.status = Q0BootstrapStatus::REJECT_JOINT_NOT_IN_PROFILE;
    return out;
  }
  if (joint->bus_id != request.bus_id) {
    out.status = Q0BootstrapStatus::REJECT_BUS_ID_MISMATCH;
    return out;
  }

  if (!request.nominal_zero_pose_confirmed) {
    out.status = Q0BootstrapStatus::REJECT_POSE_NOT_CONFIRMED;
    return out;
  }

  if (samples == nullptr ||
      sample_count < kQ0BootstrapMinSamples ||
      sample_count > kQ0BootstrapMaxSamples) {
    out.status = Q0BootstrapStatus::REJECT_SAMPLE_COUNT;
    return out;
  }

  int32_t unwrapped[kQ0BootstrapMaxSamples] = {0};
  int32_t raw[kQ0BootstrapMaxSamples] = {0};
  int32_t reference = -1;

  for (uint8_t i = 0; i < sample_count; ++i) {
    const Q0CaptureSample& sample = samples[i];
    if (!sample.read_ok) {
      out.status = Q0BootstrapStatus::REJECT_SAMPLE_READ;
      return out;
    }
    if (sample.torque_enable != 0) {
      out.status = Q0BootstrapStatus::REJECT_TORQUE_NOT_OFF;
      return out;
    }
    if (sample.raw_tick < kRawMin || sample.raw_tick > kRawMax) {
      out.status = Q0BootstrapStatus::REJECT_RAW_DOMAIN;
      return out;
    }

    raw[i] = sample.raw_tick;
    if (reference < 0) reference = sample.raw_tick;
    unwrapped[i] = reference + circularEncoderDelta(sample.raw_tick, reference);
  }

  insertionSort(unwrapped, sample_count);
  // Historical read-only capture used ordered[len/2], including for even
  // counts; preserve that deterministic median convention without importing
  // any historical q0 value or tolerance.
  const uint16_t candidate_tick = wrapRawTick(unwrapped[sample_count / 2]);

  uint16_t spread = 0;
  for (uint8_t i = 0; i < sample_count; ++i) {
    const uint16_t d = absDeltaMagnitude(raw[i], candidate_tick);
    if (d > spread) spread = d;
  }
  if (spread > request.max_stability_spread_ticks) {
    out.status = Q0BootstrapStatus::REJECT_UNSTABLE;
    out.sample_count = sample_count;
    out.stability_spread_ticks = spread;
    return out;
  }

  out.status = Q0BootstrapStatus::CANDIDATE;
  out.geometry = profile.provenanceTag();
  out.bus_id = request.bus_id;
  out.capture_session_id = request.capture_session_id;
  out.sample_count = sample_count;
  out.stability_spread_ticks = spread;

  out.evidence.measured = true;
  out.evidence.estimator = calibration::Q0Estimator::MANUAL_ZERO_POSE;
  out.evidence.state = calibration::EvidenceState::CANDIDATE;
  out.evidence.origin = calibration::CalibrationOrigin::LIVE_SESSION;
  out.evidence.identity = request.identity;
  out.evidence.tick = candidate_tick;
  out.evidence.shift_from_digital_home_ticks =
      absDeltaMagnitude(candidate_tick, calibration::kServoRawCenter);
  out.evidence.endpoint_disagreement_ticks = 0;
  out.evidence.scale_permille = 0;
  out.evidence.accepted_by_gate = false;

  return out;
}

const char* toString(Q0BootstrapStatus status) {
  switch (status) {
    case Q0BootstrapStatus::NOT_EVALUATED:                 return "NOT_EVALUATED";
    case Q0BootstrapStatus::REJECT_POPULATION_NOT_CURRENT: return "REJECT_POPULATION_NOT_CURRENT";
    case Q0BootstrapStatus::REJECT_GEOMETRY_UNBOUND:       return "REJECT_GEOMETRY_UNBOUND";
    case Q0BootstrapStatus::REJECT_GEOMETRY_PROVENANCE:    return "REJECT_GEOMETRY_PROVENANCE";
    case Q0BootstrapStatus::REJECT_INVALID_REQUEST:        return "REJECT_INVALID_REQUEST";
    case Q0BootstrapStatus::REJECT_JOINT_NOT_IN_PROFILE:   return "REJECT_JOINT_NOT_IN_PROFILE";
    case Q0BootstrapStatus::REJECT_BUS_ID_MISMATCH:        return "REJECT_BUS_ID_MISMATCH";
    case Q0BootstrapStatus::REJECT_POSE_NOT_CONFIRMED:     return "REJECT_POSE_NOT_CONFIRMED";
    case Q0BootstrapStatus::REJECT_SAMPLE_COUNT:           return "REJECT_SAMPLE_COUNT";
    case Q0BootstrapStatus::REJECT_SAMPLE_READ:            return "REJECT_SAMPLE_READ";
    case Q0BootstrapStatus::REJECT_TORQUE_NOT_OFF:         return "REJECT_TORQUE_NOT_OFF";
    case Q0BootstrapStatus::REJECT_RAW_DOMAIN:             return "REJECT_RAW_DOMAIN";
    case Q0BootstrapStatus::REJECT_UNSTABLE:               return "REJECT_UNSTABLE";
    case Q0BootstrapStatus::CANDIDATE:                     return "CANDIDATE";
  }
  return "UNKNOWN";
}

}  // namespace actuator
}  // namespace matdog
