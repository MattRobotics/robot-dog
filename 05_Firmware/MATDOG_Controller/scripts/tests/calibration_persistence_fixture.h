#ifndef MATDOG_TEST_CALIBRATION_PERSISTENCE_FIXTURE_H
#define MATDOG_TEST_CALIBRATION_PERSISTENCE_FIXTURE_H

#include <cstring>
#include "calibration_record_golden.h"
#include "../../src/actuator/CalibrationQ0EvidencePreparation.h"
#include "../../src/calibration/CalibrationPersistenceService.h"
#include "../../src/calibration/CalibrationQ0CaptureSession.h"
#include "../../src/calibration/CalibrationSaveGate.h"
#include "../../src/servo/ServoProfileData.h"

namespace persistence_fixture {
using namespace matdog;
using namespace matdog::calibration;
using namespace matdog::servo;
using golden::boundProfile;
using golden::goldenRecord;

// ---- the 24/24 evidence of 2026-10-01, as a FullLegEvidenceStore ---------------

inline FullLegEvidenceStore syntheticEvidence(const CalibrationRecord& g) {
  FullLegEvidenceStore store;
  store.reset();
  const actuator::GeometryProvenanceTag tag = g.geometry_tag;
  for (uint8_t l = 0; l < 4; ++l) {
    FullLegRecord rec;
    rec.leg = static_cast<Leg>(l);
    rec.present = true;
    rec.attempts = g.leg[l].attempts;
    rec.session_id = l + 1;
    rec.geometry = tag;
    rec.has_rear_park = g.leg[l].has_rear_park != 0;
    if (rec.has_rear_park) {
      rec.park_leg = static_cast<Leg>(g.leg[l].park_leg);
      rec.park_joint = static_cast<JointKind>(g.leg[l].park_joint);
      rec.park_bus_id = g.leg[l].park_bus_id;
      rec.park_target_urad = g.leg[l].park_target_urad;
    }
    rec.contacts_measured = 6;
    rec.contacts_accepted = 6;
    rec.diagnostics_accepted = true;
    rec.parameters_approved = false;
    rec.contact_margin_ticks = g.contact_margin_ticks;
    rec.session_completed = true;
    rec.permit_revoked = true;
    rec.authority_released = true;
    rec.hardware_contact_calibrated = true;
    rec.verdict = FullLegVerdict::HARDWARE_CONTACT_CALIBRATED;
    for (uint8_t k = 0; k < 3; ++k) {
      const CalibrationRecordJointV1& gj = g.joint[l * 3 + k];
      FullLegJointRecord& j = rec.joints[k];
      j.identity.leg = static_cast<Leg>(l);
      j.identity.joint = static_cast<JointKind>(k);
      setPhysicalUnit(&j.identity, gj.unit);
      j.bus_id = gj.bus_id;
      j.q0_present = true;
      j.q0_state = EvidenceState::PROMOTED;
      j.q0_origin = CalibrationOrigin::LIVE_SESSION;
      j.q0_geometry = tag;
      j.q0_tick = gj.q0_tick;
      for (uint8_t s = 0; s < 2; ++s) {
        ContactEvidence& e = j.contact[s];
        e.key.leg = static_cast<Leg>(l);
        e.key.joint = static_cast<JointKind>(k);
        e.key.side = static_cast<ContactSide>(s);
        e.state = EvidenceState::PROMOTED;
        e.origin = CalibrationOrigin::LIVE_SESSION;
        e.detection = ContactState::CONTACT_CONFIRMED;
        e.witness = makeContactWitness(0, gj.contact[s].repeatability_ticks,
                                       kFullLegRepeatabilityToleranceTicks);
        e.coarse_tick = gj.contact[s].scout_tick;
        e.fine_tick_1 = gj.contact[s].fine_tick_1;
        e.fine_tick_2 = gj.contact[s].fine_tick_2;
        e.repeatability_ticks = gj.contact[s].repeatability_ticks;
        e.has_measurement = true;
        j.contact_recorded[s] = true;
      }
      j.diagnostics.evaluated = true;
      j.diagnostics.ordered = true;
      j.diagnostics.accepted = true;
      j.diagnostics.min_contact_tick = gj.diagnostics.min_contact_tick;
      j.diagnostics.max_contact_tick = gj.diagnostics.max_contact_tick;
      j.diagnostics.expected_span_ticks = gj.diagnostics.expected_span_ticks;
      j.diagnostics.measured_span_ticks = gj.diagnostics.measured_span_ticks;
      j.diagnostics.scale_permille = gj.diagnostics.scale_permille;
      j.diagnostics.affine_zero_tick = gj.diagnostics.affine_zero_tick;
      j.diagnostics.affine_shift_from_q0_ticks = gj.diagnostics.affine_shift_from_q0_ticks;
      j.diagnostics.fixed_endpoint_disagreement_ticks = gj.diagnostics.fixed_endpoint_disagreement_ticks;
    }
    store.put(rec);
  }
  return store;
}

// ---- a q0 capture whose ticks are the evidence's q0 ticks -----------------------

struct Scenario {
  actuator::CalibrationGeometryProfile profile = boundProfile();
  CalibrationRecord golden = goldenRecord(1);
  FullLegEvidenceStore evidence = syntheticEvidence(golden);
  actuator::Q0BootstrapCandidate candidates[kLegServoSlotCount];
  actuator::FreshQ0Capture capture;
  actuator::JointTransformTable transforms;
  SaveGateFacts facts;
  uint32_t promoted_capture_session_id = 0;
  actuator::GeometryProvenanceTag promoted_geometry = actuator::kNoGeometryProvenance;

  Scenario() {
    const actuator::GeometryProvenanceTag tag = profile.provenanceTag();
    for (uint8_t i = 0; i < kLegServoSlotCount; ++i) {
      const CalibrationRecordJointV1& gj = golden.joint[i];
      actuator::Q0BootstrapCandidate x{};
      x.status = actuator::Q0BootstrapStatus::CANDIDATE;
      x.geometry = tag;
      x.bus_id = gj.bus_id;
      x.capture_session_id = 5;
      x.sample_count = 9;
      x.stability_spread_ticks = 0;
      x.evidence.measured = true;
      x.evidence.estimator = Q0Estimator::MANUAL_ZERO_POSE;
      x.evidence.state = EvidenceState::CANDIDATE;
      x.evidence.origin = CalibrationOrigin::LIVE_SESSION;
      x.evidence.identity.leg = static_cast<Leg>(gj.leg);
      x.evidence.identity.joint = static_cast<JointKind>(gj.joint);
      setPhysicalUnit(&x.evidence.identity, gj.unit);
      x.evidence.tick = gj.q0_tick;
      const int d = gj.q0_tick > kServoRawCenter ? gj.q0_tick - kServoRawCenter
                                                 : kServoRawCenter - gj.q0_tick;
      x.evidence.shift_from_digital_home_ticks = static_cast<uint16_t>(d);
      candidates[legSlotIndex(x.evidence.identity.leg, x.evidence.identity.joint)] = x;
    }
    capture.complete = true;
    capture.population_pass = true;
    capture.capture_session_id = 5;
    capture.candidates = candidates;
    capture.candidate_count = kLegServoSlotCount;
  }

  // The test (never the gate) promotes: this is the existing fresh-Q0 path.
  void setTick(uint8_t index, uint16_t tick) {
    candidates[index].evidence.tick = tick;
    const int d = tick > kServoRawCenter ? tick - kServoRawCenter : kServoRawCenter - tick;
    candidates[index].evidence.shift_from_digital_home_ticks = static_cast<uint16_t>(d);
  }

  bool promote() {
    const actuator::Q0EvidencePreparation p = actuator::prepareFreshQ0Evidence(
        profile, actuator::geometry_data::kProvenance, capture, true);
    if (!p.ready()) return false;
    uint8_t admitted = 0;
    for (uint8_t i = 0; i < p.transform_count; ++i) {
      if (transforms.admit(p.transforms[i])) ++admitted;
    }
    if (admitted != p.transform_count) return false;
    promoted_capture_session_id = capture.capture_session_id;
    promoted_geometry = profile.provenanceTag();
    return true;
  }

  // Every fact proven.
  void allGood() {
    facts.persistence_ready = true;
    facts.storage_save_allowed = true;
    facts.writes_blocked = false;
    facts.maintenance_mode = true;
    facts.session_live = false;
    facts.leg_run_armed = false;
    facts.motion_executor_active = false;
    facts.q0_capture_active = false;
    facts.servo_diagnostic_busy = false;
    facts.authority_none = true;
    facts.motion_permit_active = false;
    facts.operator_authorized = false;
    facts.first_motion_safe_off_proven = true;
    facts.evidence = &evidence;
    facts.profile = &profile;
    facts.geometry_tag = profile.provenanceTag();
    facts.build_id = "golden-test";
    facts.q0_capture_state_complete = true;
    facts.q0_capture = capture;
    facts.promoted_capture_session_id = promoted_capture_session_id;
    facts.promoted_geometry = promoted_geometry;
    facts.transforms = &transforms;
  }
};

class MemStorage : public CalibrationRecordStorage {
 public:
  bool present[2] = {false, false};
  uint8_t data[2][kCalibrationRecordV1EncodedBytes];
  size_t size[2] = {0, 0};
  bool marker_present = false;
  uint8_t marker[kSaveMarkerV1Bytes];
  size_t marker_size = 0;
  int slot_writes = 0;
  int marker_writes = 0;

  StorageIoStatus read(CalibrationSlot slot, uint8_t* buffer, size_t capacity, size_t* length) override {
    const int i = static_cast<int>(slot);
    *length = 0;
    if (!present[i]) return StorageIoStatus::ABSENT;
    if (size[i] > capacity) return StorageIoStatus::BUFFER_TOO_SMALL;
    std::memcpy(buffer, data[i], size[i]);
    *length = size[i];
    return StorageIoStatus::OK;
  }
  StorageIoStatus readMarker(uint8_t* buffer, size_t capacity, size_t* length) override {
    *length = 0;
    if (!marker_present) return StorageIoStatus::ABSENT;
    if (marker_size > capacity) return StorageIoStatus::BUFFER_TOO_SMALL;
    std::memcpy(buffer, marker, marker_size);
    *length = marker_size;
    return StorageIoStatus::OK;
  }
  StorageIoStatus write(CalibrationSlot slot, const uint8_t* bytes, size_t length) override {
    const int i = static_cast<int>(slot);
    ++slot_writes;
    std::memcpy(data[i], bytes, length);
    size[i] = length;
    present[i] = true;
    return StorageIoStatus::OK;
  }
  StorageIoStatus writeMarker(const uint8_t* bytes, size_t length) override {
    ++marker_writes;
    std::memcpy(marker, bytes, length);
    marker_size = length;
    marker_present = true;
    return StorageIoStatus::OK;
  }
};

inline CensusResult goodCensus() {
  CensusResult c{};
  c.canonical_allocated = canonicalAllocatedCount();
  c.expected_now = expectedNowCount();
  c.present_expected = c.expected_now;
  c.absent_by_design = absentByDesignCount();
  c.scan_lo = kCanonicalScanLo;
  c.scan_hi = kCanonicalScanHi;
  c.verdict = CensusVerdict::PASS;
  return c;
}

inline PreflightResult goodPreflight() {
  PreflightResult p{};
  p.complete = true;
  p.joints_evaluated = kLegPreflightCount;
  p.pass_count = kLegPreflightCount;
  for (uint8_t i = 0; i < kLegPreflightCount; ++i) {
    const CanonicalServo* c = legServoAt(i);
    JointPreflightRecord& r = p.joints[i];
    r.expected_bus_id = c->bus_id;
    r.joint = c->joint;
    r.expected_physical_unit = c->physical_unit;
    r.observed_bus_id = c->bus_id;
    r.model = profile_data::kInvariants.model_expected;
    r.position_offset_read = true;
    r.position_offset = 0;
    r.torque_enable = 0;
    r.present_position = 2000 + i;
    r.profile = ProfileVerdict::MATCH;
    r.profile_registers_read = profile_data::kPersistentRegisterCount;
    r.result = JointPreflightResult::PASS;
  }
  return p;
}

inline bool startCapture(CalibrationQ0CaptureSession& session) {
  Q0CaptureConfig c{};
  c.samples_per_joint = 9;
  c.stability_budget_specified = true;
  c.max_stability_spread_ticks = 16;
  c.nominal_zero_pose_confirmed = true;
  c.started_at_ms = 1000;
  return session.start(c) && session.markCensusStarted() &&
         session.submitCensus(goodCensus()) && session.markPreflightStarted() &&
         session.submitPreflight(goodPreflight());
}

inline bool finishCapture(CalibrationQ0CaptureSession& session, const CalibrationRecord& record) {
  while (session.active()) {
    Q0ReadRequest req{};
    if (!session.nextReadRequest(&req)) return false;
    Q0ReadObservation obs{};
    obs.bus_id = req.bus_id;
    obs.read_ok = true;
    obs.raw_tick = record.joint[legSlotIndex(req.identity.leg, req.identity.joint)].q0_tick;
    obs.torque_enable = 0;
    if (!session.recordRead(obs)) return false;
  }
  return session.freshCapture().complete;
}

inline bool promoteCapture(CalibrationQ0CaptureSession& session,
                           const actuator::CalibrationGeometryProfile& profile,
                           actuator::JointTransformTable& transforms,
                           bool confirmation = true, uint8_t admit_limit = kLegServoSlotCount) {
  const auto capture = session.freshCapture();
  const auto prepared = actuator::prepareFreshQ0Evidence(
      profile, actuator::geometry_data::kProvenance, capture, confirmation);
  if (!prepared.ready()) return false;
  uint8_t admitted = 0;
  for (uint8_t i = 0; i < prepared.transform_count && i < admit_limit; ++i) {
    if (transforms.admit(prepared.transforms[i])) ++admitted;
  }
  return session.notePromotionCompleted(capture.capture_session_id, admitted, profile.provenanceTag());
}

inline void captureFacts(SaveGateFacts& f, const CalibrationQ0CaptureSession& session) {
  f.q0_capture_state_complete = session.status().state == Q0CaptureState::COMPLETE;
  f.q0_capture = session.freshCapture();
  f.promoted_capture_session_id = session.status().promoted_capture_session_id;
  f.promoted_geometry = session.status().promoted_geometry;
}

}  // namespace persistence_fixture
#endif
