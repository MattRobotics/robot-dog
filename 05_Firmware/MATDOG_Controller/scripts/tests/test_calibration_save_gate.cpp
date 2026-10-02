// Offline tests for the SAVE gate (src/calibration/CalibrationSaveGate.*): every
// prerequisite of a persistent SAVE of the Full Calibration, against the REAL
// pure pieces - the Q0 capture session, prepareFreshQ0Evidence(), the
// JointTransformTable and the record builder/validator. No hardware, no NVS.
//
// What is proven here: the gate passes only with every fact proven; every fact
// that is missing, stale or not provable names its own reason, in the documented
// order; a fact left at its default refuses; a q0 that is not the promoted q0 of
// the capture refuses; the gate never admits a transform and never changes the
// facts; a gate pass feeds the persistence service into AWAITING_ACK and an
// explicit ACK makes it VALID_ACKNOWLEDGED - and neither step moves anything.

#include <cstdio>
#include <cstring>

#include "calibration_record_golden.h"
#include "../../src/actuator/CalibrationQ0EvidencePreparation.h"
#include "../../src/calibration/CalibrationPersistenceService.h"
#include "../../src/calibration/CalibrationQ0CaptureSession.h"
#include "../../src/calibration/CalibrationSaveGate.h"
#include "../../src/servo/ServoProfileData.h"
#include "../../src/servo/ServoPopulation.h"

using namespace matdog;
using namespace matdog::calibration;
using namespace matdog::servo;

static int g_checks = 0;
static int g_failures = 0;
static const char* g_case = "";

#define CHECK(cond)                                                              \
  do {                                                                           \
    ++g_checks;                                                                  \
    if (!(cond)) {                                                               \
      ++g_failures;                                                              \
      std::printf("  FAIL [%s] %s:%d: %s\n", g_case, __FILE__, __LINE__, #cond); \
    }                                                                            \
  } while (0)

#define CHECK_EQ(actual, expected)                                         \
  do {                                                                     \
    ++g_checks;                                                            \
    const long a_ = (long)(actual);                                        \
    const long e_ = (long)(expected);                                      \
    if (a_ != e_) {                                                        \
      ++g_failures;                                                        \
      std::printf("  FAIL [%s] %s:%d: %s == %ld, expected %ld\n", g_case,  \
                  __FILE__, __LINE__, #actual, a_, e_);                    \
    }                                                                      \
  } while (0)

#define CHECK_REASON(result, expected)                                            \
  do {                                                                            \
    ++g_checks;                                                                   \
    if ((result).reason != (expected)) {                                          \
      ++g_failures;                                                               \
      std::printf("  FAIL [%s] %s:%d: reason %s, expected %s\n", g_case, __FILE__, \
                  __LINE__, toString((result).reason), toString(expected));       \
    }                                                                             \
  } while (0)

namespace {

using golden::boundProfile;
using golden::goldenRecord;
using SGR = SaveGateReason;

// ---- the 24/24 evidence of 2026-10-01, as a FullLegEvidenceStore ---------------

FullLegEvidenceStore syntheticEvidence(const CalibrationRecord& g) {
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
    return admitted == p.transform_count;
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
    facts.transforms = &transforms;
  }
};

SaveGateResult evaluate(const SaveGateFacts& f) {
  static CalibrationRecord scratch;
  return evaluateSaveGate(f, &scratch);
}

// ---------------------------------------------------------------------------

void test_defaults_refuse() {
  g_case = "defaults refuse";
  SaveGateFacts f;
  CHECK_REASON(evaluate(f), SGR::PERSISTENCE_NOT_READY);
  CHECK(!f.persistence_ready && !f.storage_save_allowed && f.writes_blocked);
  CHECK(!f.maintenance_mode && f.session_live && f.leg_run_armed);
  CHECK(f.motion_executor_active && f.q0_capture_active && f.servo_diagnostic_busy);
  CHECK(!f.authority_none && f.motion_permit_active && f.operator_authorized);
  CHECK(!f.first_motion_safe_off_proven);
  CHECK(f.evidence == nullptr && f.profile == nullptr && f.transforms == nullptr);
  CHECK(!f.q0_capture_state_complete && !f.q0_capture.complete);
  SaveGateResult r;
  CHECK(!r.ok());  // a default result is a refusal

  CalibrationRecord record;
  CHECK_REASON(evaluateSaveGate(f, nullptr), SGR::RECORD_NOT_BUILDABLE);
}

void test_gate_passes_with_everything_proven() {
  g_case = "gate passes";
  Scenario s;
  CHECK(s.promote());
  s.allGood();
  CalibrationRecord record;
  const SaveGateResult r = evaluateSaveGate(s.facts, &record);
  CHECK_REASON(r, SGR::OK);
  CHECK(r.ok());
  CHECK_EQ(record.generation, 0);  // the store assigns it
  CHECK(validateCalibrationRecord(record, s.profile) != CalibrationRecordStatus::OK);  // generation 0 is not storable
  record.generation = 1;
  CHECK(validateCalibrationRecord(record, s.profile) == CalibrationRecordStatus::OK);
  CHECK_EQ(record.calibration_accepted, 1);
  CHECK_EQ(record.parameters_approved, 0);
  for (uint8_t i = 0; i < kRecordJointCount; ++i) {
    CHECK_EQ(record.joint[i].q0_tick, s.golden.joint[i].q0_tick);
  }
}

// One refusal per fact, in the documented order: each case starts from "all good"
// and breaks exactly one fact; the reason must be that fact's.
void test_each_prerequisite_refuses() {
  g_case = "each prerequisite";
  struct Case {
    const char* name;
    SGR expected;
    void (*break_it)(Scenario&);
  };
  static const Case kCases[] = {
      {"persistence", SGR::PERSISTENCE_NOT_READY, [](Scenario& s) { s.facts.persistence_ready = false; }},
      {"writes blocked", SGR::WRITES_BLOCKED, [](Scenario& s) { s.facts.writes_blocked = true; }},
      {"storage state", SGR::STORAGE_STATE_NOT_SAVEABLE, [](Scenario& s) { s.facts.storage_save_allowed = false; }},
      {"maintenance", SGR::NOT_IN_MAINTENANCE_MODE, [](Scenario& s) { s.facts.maintenance_mode = false; }},
      {"session live", SGR::CALIBRATION_SESSION_LIVE, [](Scenario& s) { s.facts.session_live = true; }},
      {"leg run armed", SGR::LEG_RUN_NOT_FINALIZED, [](Scenario& s) { s.facts.leg_run_armed = true; }},
      {"executor", SGR::MOTION_EXECUTOR_ACTIVE, [](Scenario& s) { s.facts.motion_executor_active = true; }},
      {"q0 capture active", SGR::Q0_CAPTURE_ACTIVE, [](Scenario& s) { s.facts.q0_capture_active = true; }},
      {"servo diagnostic", SGR::SERVO_DIAGNOSTIC_BUSY, [](Scenario& s) { s.facts.servo_diagnostic_busy = true; }},
      {"authority", SGR::AUTHORITY_NOT_NONE, [](Scenario& s) { s.facts.authority_none = false; }},
      {"permit", SGR::MOTION_PERMIT_ACTIVE, [](Scenario& s) { s.facts.motion_permit_active = true; }},
      {"operator authorization", SGR::OPERATOR_AUTHORIZATION_ACTIVE, [](Scenario& s) { s.facts.operator_authorized = true; }},
      {"safe off", SGR::SAFE_OFF_NOT_PROVEN, [](Scenario& s) { s.facts.first_motion_safe_off_proven = false; }},
      {"no evidence", SGR::GEOMETRY_NOT_CURRENT, [](Scenario& s) { s.facts.evidence = nullptr; }},
      {"no profile", SGR::GEOMETRY_NOT_CURRENT, [](Scenario& s) { s.facts.profile = nullptr; }},
      {"no transforms", SGR::GEOMETRY_NOT_CURRENT, [](Scenario& s) { s.facts.transforms = nullptr; }},
      {"no tag", SGR::GEOMETRY_NOT_CURRENT, [](Scenario& s) { s.facts.geometry_tag = actuator::kNoGeometryProvenance; }},
      {"foreign tag", SGR::GEOMETRY_NOT_CURRENT, [](Scenario& s) { s.facts.geometry_tag ^= 1; }},
      {"capture state", SGR::Q0_CAPTURE_NOT_COMPLETE, [](Scenario& s) { s.facts.q0_capture_state_complete = false; }},
      {"capture view", SGR::Q0_CAPTURE_NOT_COMPLETE, [](Scenario& s) { s.facts.q0_capture.complete = false; }},
      {"capture candidates", SGR::Q0_CAPTURE_NOT_COMPLETE, [](Scenario& s) { s.facts.q0_capture.candidates = nullptr; }},
      {"capture count", SGR::Q0_CAPTURE_NOT_COMPLETE, [](Scenario& s) { s.facts.q0_capture.candidate_count = 11; }},
  };
  for (const Case& c : kCases) {
    Scenario s;
    CHECK(s.promote());
    s.allGood();
    c.break_it(s);
    g_case = c.name;
    CHECK_REASON(evaluate(s.facts), c.expected);
  }
  g_case = "each prerequisite";
}

void test_full_calibration_must_be_24_of_24() {
  g_case = "24/24";
  {
    Scenario s;
    CHECK(s.promote());
    s.allGood();
    FullLegEvidenceStore three;
    three.reset();
    for (uint8_t l = 0; l < 3; ++l) three.put(*s.evidence.find(static_cast<Leg>(l)));
    s.facts.evidence = &three;
    CHECK_REASON(evaluate(s.facts), SGR::FULL_CALIBRATION_NOT_24_OF_24);
  }
  {
    Scenario s;
    CHECK(s.promote());
    s.allGood();
    FullLegEvidenceStore empty;
    empty.reset();
    s.facts.evidence = &empty;
    CHECK_REASON(evaluate(s.facts), SGR::FULL_CALIBRATION_NOT_24_OF_24);
  }
  {  // a leg that is not hardware-contact-calibrated
    Scenario s;
    CHECK(s.promote());
    s.allGood();
    FullLegEvidenceStore partial = s.evidence;
    FullLegRecord leg = *s.evidence.find(Leg::LH);
    leg.joints[1].contact_recorded[0] = false;
    leg.hardware_contact_calibrated = false;
    partial.put(leg);
    s.facts.evidence = &partial;
    CHECK(!evaluate(s.facts).ok());
  }
}

void test_leg_run_must_be_closed() {
  g_case = "leg run closed";
  const struct {
    const char* name;
    void (*mutate)(FullLegRecord*);
  } kCases[] = {
      {"session not completed", [](FullLegRecord* r) { r->session_completed = false; }},
      {"permit not revoked", [](FullLegRecord* r) { r->permit_revoked = false; }},
      {"authority not released", [](FullLegRecord* r) { r->authority_released = false; }},
  };
  for (const auto& c : kCases) {
    Scenario s;
    CHECK(s.promote());
    s.allGood();
    FullLegEvidenceStore store = s.evidence;
    FullLegRecord leg = *s.evidence.find(Leg::RH);
    c.mutate(&leg);
    store.put(leg);
    s.facts.evidence = &store;
    g_case = c.name;
    CHECK_REASON(evaluate(s.facts), SGR::LEG_RUN_NOT_CLOSED);
  }
  {  // a leg run under another geometry
    Scenario s;
    CHECK(s.promote());
    s.allGood();
    FullLegEvidenceStore store = s.evidence;
    FullLegRecord leg = *s.evidence.find(Leg::LF);
    leg.geometry ^= 1;
    store.put(leg);
    s.facts.evidence = &store;
    g_case = "leg other geometry";
    CHECK(!evaluate(s.facts).ok());
  }
}

void test_q0_must_be_current_and_promoted() {
  g_case = "q0 promoted";
  {  // nothing promoted
    Scenario s;
    s.allGood();
    CHECK(s.transforms.empty());
    CHECK_REASON(evaluate(s.facts), SGR::Q0_NOT_PROMOTED);
  }
  {  // 11 of 12 promoted
    Scenario s;
    const actuator::Q0EvidencePreparation p = actuator::prepareFreshQ0Evidence(
        s.profile, actuator::geometry_data::kProvenance, s.capture, true);
    CHECK(p.ready());
    for (uint8_t i = 0; i + 1 < p.transform_count; ++i) CHECK(s.transforms.admit(p.transforms[i]));
    s.allGood();
    CHECK_REASON(evaluate(s.facts), SGR::Q0_NOT_PROMOTED);
  }
  {  // promoted from an earlier capture with other ticks: stale
    Scenario old;
    for (uint8_t i = 0; i < kLegServoSlotCount; ++i) old.setTick(i, static_cast<uint16_t>(old.candidates[i].evidence.tick + 7));
    CHECK(old.promote());
    Scenario s;
    s.transforms = old.transforms;
    s.allGood();
    CHECK_REASON(evaluate(s.facts), SGR::Q0_NOT_PROMOTED);
  }
  {  // promoted, but the Full Calibration measured another q0: the record disagrees
    Scenario s;
    CHECK(s.promote());
    s.allGood();
    FullLegEvidenceStore store = s.evidence;
    FullLegRecord leg = *s.evidence.find(Leg::RF);
    leg.joints[1].q0_tick = static_cast<uint16_t>(leg.joints[1].q0_tick + 5);
    store.put(leg);
    s.facts.evidence = &store;
    const SaveGateResult r = evaluate(s.facts);
    CHECK(!r.ok());
    CHECK(r.reason == SGR::RECORD_Q0_MISMATCH || r.reason == SGR::RECORD_NOT_VALID ||
          r.reason == SGR::RECORD_NOT_BUILDABLE);
  }
  {  // the real capture session: a fresh capture promoted, but its ticks are not the evidence's
    CalibrationQ0CaptureSession session;
    Q0CaptureConfig cfg{};
    cfg.samples_per_joint = 9;
    cfg.stability_budget_specified = true;
    cfg.max_stability_spread_ticks = 16;
    cfg.nominal_zero_pose_confirmed = true;
    cfg.started_at_ms = 1000;
    CHECK(session.start(cfg));
    CensusResult census{};
    census.canonical_allocated = canonicalAllocatedCount();
    census.expected_now = expectedNowCount();
    census.present_expected = census.expected_now;
    census.absent_by_design = absentByDesignCount();
    census.scan_lo = kCanonicalScanLo;
    census.scan_hi = kCanonicalScanHi;
    census.verdict = CensusVerdict::PASS;
    PreflightResult pre{};
    pre.complete = true;
    pre.joints_evaluated = kLegPreflightCount;
    pre.pass_count = kLegPreflightCount;
    for (uint8_t i = 0; i < kLegPreflightCount; ++i) {
      const CanonicalServo* c = legServoAt(i);
      if (c == nullptr) continue;
      JointPreflightRecord& r = pre.joints[i];
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
    CHECK(session.markCensusStarted() && session.submitCensus(census));
    CHECK(session.markPreflightStarted() && session.submitPreflight(pre));
    for (uint8_t pass = 0; pass < 9; ++pass) {
      for (uint8_t joint = 0; joint < kLegServoSlotCount; ++joint) {
        Q0ReadRequest req{};
        CHECK(session.nextReadRequest(&req));
        Q0ReadObservation obs{};
        obs.bus_id = req.bus_id;
        obs.read_ok = true;
        obs.raw_tick = 2050 + joint;
        obs.torque_enable = 0;
        CHECK(session.recordRead(obs));
      }
    }
    CHECK(session.status().state == Q0CaptureState::COMPLETE);

    Scenario s;
    const actuator::FreshQ0Capture real = session.freshCapture();
    CHECK(real.complete);
    const actuator::Q0EvidencePreparation p = actuator::prepareFreshQ0Evidence(
        s.profile, actuator::geometry_data::kProvenance, real, true);
    CHECK(p.ready());
    for (uint8_t i = 0; i < p.transform_count; ++i) CHECK(s.transforms.admit(p.transforms[i]));
    s.allGood();
    s.facts.q0_capture = real;
    CHECK(actuator::freshQ0CaptureIsPromoted(real, s.transforms, s.facts.geometry_tag));
    // the evidence of the Full Calibration carries other q0 ticks than this capture
    const SaveGateResult r = evaluate(s.facts);
    CHECK_REASON(r, SGR::RECORD_Q0_MISMATCH);
    CHECK(r.joint_index < kRecordJointCount);
  }
}

void test_record_q0_matches_capture_per_joint() {
  g_case = "q0 per joint";
  for (uint8_t joint = 0; joint < kRecordJointCount; ++joint) {
    Scenario s;
    CHECK(s.promote());
    s.allGood();
    // capture and promoted transform agree with each other but not with the
    // evidence of the Full Calibration on this one joint
    Scenario shifted;
    shifted.setTick(joint, static_cast<uint16_t>(shifted.candidates[joint].evidence.tick + 3));
    s.transforms.clear();
    shifted.capture.candidates = shifted.candidates;
    const actuator::Q0EvidencePreparation p = actuator::prepareFreshQ0Evidence(
        s.profile, actuator::geometry_data::kProvenance, shifted.capture, true);
    CHECK(p.ready());
    for (uint8_t i = 0; i < p.transform_count; ++i) CHECK(s.transforms.admit(p.transforms[i]));
    s.facts.q0_capture = shifted.capture;
    const SaveGateResult r = evaluate(s.facts);
    CHECK_REASON(r, SGR::RECORD_Q0_MISMATCH);
    CHECK_EQ(r.joint_index, joint);
  }
}

void test_gate_has_no_side_effects() {
  g_case = "no side effects";
  Scenario s;
  CHECK(s.promote());
  s.allGood();
  const actuator::JointTransformTable before = s.transforms;
  const uint8_t size_before = s.transforms.size();
  CalibrationRecord record;
  CHECK(evaluateSaveGate(s.facts, &record).ok());
  CHECK_EQ(s.transforms.size(), size_before);
  CHECK(std::memcmp(&before, &s.transforms, sizeof(before)) == 0);
  CHECK(s.facts.q0_capture_state_complete);
  CHECK(s.facts.authority_none && !s.facts.motion_permit_active);

  // refusing does not admit anything either
  Scenario empty;
  empty.allGood();
  const uint8_t n = empty.transforms.size();
  CHECK(!evaluate(empty.facts).ok());
  CHECK_EQ(empty.transforms.size(), n);
  CHECK(empty.transforms.empty());
}

// ---- SAVE through the service, then ACK ---------------------------------------

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

void fillGateFacts(Scenario& s, const CalibrationPersistenceService& svc) {
  s.allGood();
  s.facts.persistence_ready = svc.snapshot().nvs == NvsInitStatus::READY;
  s.facts.writes_blocked = svc.writesBlocked();
  s.facts.storage_save_allowed = svc.snapshot().assessment.save_allowed;
}

void test_gate_save_ack_flow() {
  g_case = "gate save ack";
  Scenario s;
  CHECK(s.promote());
  MemStorage mem;
  CalibrationPersistenceService svc(&mem);
  svc.begin(NvsInitStatus::READY, 0);
  svc.load(s.profile);
  CHECK(svc.snapshot().verdict == PersistenceVerdict::NO_RECORD);
  CHECK(!svc.calibrationAvailable());

  fillGateFacts(s, svc);
  const SaveGateResult gate = evaluateSaveGate(s.facts, svc.scratchRecord());
  CHECK_REASON(gate, SGR::OK);
  CHECK_EQ(mem.slot_writes + mem.marker_writes, 0);  // the gate wrote nothing

  const ServiceSaveResult saved = svc.save(*svc.scratchRecord(), s.profile);
  CHECK(saved.guard == ServiceGuard::OK);
  CHECK(saved.save.status == SaveStatus::OK);
  CHECK_EQ(saved.save.generation, 1);
  CHECK(svc.snapshot().verdict == PersistenceVerdict::AWAITING_ACK);
  CHECK(!svc.calibrationAvailable());  // SAVE is not concluded before the ACK

  // while the SAVE awaits its ACK, a second SAVE is not allowed
  fillGateFacts(s, svc);
  CHECK_REASON(evaluateSaveGate(s.facts, svc.scratchRecord()), SGR::STORAGE_STATE_NOT_SAVEABLE);

  const ServiceAckResult wrong = svc.acknowledge(2, s.profile);
  CHECK(wrong.ack.status != AckStatus::OK);
  CHECK(!svc.calibrationAvailable());

  const ServiceAckResult ack = svc.acknowledge(1, s.profile);
  CHECK(ack.guard == ServiceGuard::OK);
  CHECK(ack.ack.status == AckStatus::OK);
  CHECK(svc.snapshot().verdict == PersistenceVerdict::VALID_ACKNOWLEDGED);
  CHECK(svc.calibrationAvailable());

  // the repeat is recognised, not re-applied
  const int marker_writes = mem.marker_writes;
  const ServiceAckResult again = svc.acknowledge(1, s.profile);
  CHECK(again.ack.status == AckStatus::ALREADY_ACKNOWLEDGED);
  CHECK_EQ(mem.marker_writes, marker_writes);

  // SAVE, gate and ACK together never touched the transform table
  CHECK_EQ(s.transforms.size(), 12);

  // the next Full Calibration may be saved again
  fillGateFacts(s, svc);
  CHECK_REASON(evaluateSaveGate(s.facts, svc.scratchRecord()), SGR::OK);
}

void test_gate_blocks_save_when_backend_not_ready() {
  g_case = "backend not ready";
  Scenario s;
  CHECK(s.promote());
  MemStorage mem;
  CalibrationPersistenceService svc(&mem);
  svc.begin(NvsInitStatus::PARTITION_MISSING, 0);
  svc.load(s.profile);
  fillGateFacts(s, svc);
  CHECK(!s.facts.persistence_ready);
  CHECK_REASON(evaluateSaveGate(s.facts, svc.scratchRecord()), SGR::PERSISTENCE_NOT_READY);
  const ServiceSaveResult r = svc.save(*svc.scratchRecord(), s.profile);
  CHECK(r.guard == ServiceGuard::NVS_NOT_READY);
  CHECK_EQ(mem.slot_writes + mem.marker_writes, 0);
}

}  // namespace

int main() {
  test_defaults_refuse();
  test_gate_passes_with_everything_proven();
  test_each_prerequisite_refuses();
  test_full_calibration_must_be_24_of_24();
  test_leg_run_must_be_closed();
  test_q0_must_be_current_and_promoted();
  test_record_q0_matches_capture_per_joint();
  test_gate_has_no_side_effects();
  test_gate_save_ack_flow();
  test_gate_blocks_save_when_backend_not_ready();
  std::printf("test_calibration_save_gate: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
