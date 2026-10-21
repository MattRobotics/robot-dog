// Independent motion boundary tests: REAL executor, semantic service, JSON,
// mapping, Safe Actuator policy/runtime and sole authority arbiter. The backend
// and planner are observations only. Synthetic approvals are TEST DATA, never
// a hardware qualification, production limit artifact or operator permission.
#include <cstdio>
#include <cstring>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <functional>

#include "calibration_record_golden.h"
#include "../../src/execution/MotionExecutor.h"
#include "../../src/execution/MotionCoordinator.h"
#include "../../src/core/MotionService.h"
#include "../../src/servo/ServoPopulation.h"
#include "../../src/core/MotionJson.h"

using namespace matdog;
using namespace matdog::actuator;
using namespace matdog::core;
using namespace matdog::execution;
using namespace matdog::calibration;
static int checks = 0, failures = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) { ++failures; std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (0)
#define REASON(actual, expected) CHECK(std::strcmp((actual), (expected)) == 0)

namespace {
class Backend : public ActuatorBackend {
 public:
  struct Write { uint8_t bus; uint16_t tick; };
  std::vector<Write> writes;
  BackendWriteOutcome outcome = BackendWriteOutcome::VERIFIED_APPLIED;
  int torque_calls = 0;
  // Injected single-joint faults: the next fail_next goal/torque writes return
  // fail_outcome and change nothing (models a lost reply / refused write).
  int fail_next = 0, attempts = 0; BackendWriteOutcome fail_outcome = BackendWriteOutcome::UNCERTAIN;
  std::function<bool()> preempt;
  bool injected() { ++attempts; if (preempt && preempt()) return true; if (fail_next > 0) { --fail_next; return true; } return false; }
  BackendWriteOutcome enableTorque(uint8_t) override { if (injected()) return fail_outcome; ++torque_calls; return outcome; }
  BackendWriteOutcome writeCalibrationTorqueLimit(uint8_t) override { ++torque_calls; return outcome; }
  BackendWriteOutcome writeGoalPosition(uint8_t bus, uint16_t tick, actuator::MotionProfile) override {
    if (injected()) return fail_outcome;
    writes.push_back({bus, tick}); return outcome;
  }
  // Issue #52 frame: one SYNC_WRITE double. TX_OK = every byte left (no ACK
  // exists, so UNCERTAIN + tx_complete); REFUSE = zero bytes; PARTIAL = TX
  // started but not completed.
  enum class FrameMode { TX_OK, REFUSE, PARTIAL } frame_mode = FrameMode::TX_OK;
  unsigned frames = 0; uint32_t tx_us = 1040, frame_clock_us = 0;
  bool absolute_frame_clock = false;
  BackendWriteOutcome writeGoalPositionFrame(const uint8_t bus[12], const uint16_t tick[12], FrameTxReport* r) override {
    if (r) *r = FrameTxReport{};
    if (frame_mode == FrameMode::REFUSE) return BackendWriteOutcome::VERIFIED_NOT_APPLIED;
    ++frames; for (unsigned i = 0; i < 12; ++i) writes.push_back({bus[i], tick[i]});
    if (r) { r->controller_clock = absolute_frame_clock; r->tx_started = true; r->tx_complete = frame_mode == FrameMode::TX_OK; r->tx_started_us = frame_clock_us; r->tx_complete_us = frame_clock_us+tx_us; r->bytes = 104; }
    return BackendWriteOutcome::UNCERTAIN;
  }
};
uint8_t gCanonicalIds[12];

class Planner : public SemanticMotionPlanner {
 public:
  int accepts = 0, stops = 0;
  bool stopping = false;
  const char* reject = nullptr;
  motion::MotionCommand last{};
  const char* accept(const motion::MotionCommand& c, uint32_t) override { ++accepts; last = c; return reject; }
  void stop(uint32_t) override { ++stops; }
  const char* state() const override { return "STAND"; }
  bool coordinatedStopping() const override { return stopping; }
};

struct Rig {
  CalibrationGeometryProfile geometry = golden::boundProfile();
  ActuatorAuthorityArbiter arbiter;
  SafeActuatorPolicy policy;
  Backend backend;
  ActuatorRuntime runtime;
  MotionExecutor executor;
  MotionExecutionFacts facts{};
  ExecutionEnvelope envelope{};
  JointFeedback feedback{};
  Rig() {
    arbiter.reset(AuthorityClearReason::BOOT);
    policy.begin(&arbiter); policy.bindGeometry(&geometry, &geometry_data::kProvenance);
    auto record = golden::goldenRecord();
    for (unsigned i = 0; i < 12; ++i) {
      const auto& joint = record.joint[i]; JointTransform t;
      t.identity.leg = static_cast<Leg>(joint.leg); t.identity.joint = static_cast<JointKind>(joint.joint);
      setPhysicalUnit(&t.identity, joint.unit);
      t.q0_tick = joint.q0_tick; t.geometry = geometry.provenanceTag();
      t.origin = CalibrationOrigin::LIVE_SESSION; t.state = EvidenceState::PROMOTED; t.present = true;
      CHECK(policy.transforms().admit(t));
      JointLimit l; l.identity = t.identity; l.geometry = t.geometry;
      l.origin = t.origin; l.state = t.state; l.present = true;
      l.min_tick = t.q0_tick - 100; l.max_tick = t.q0_tick + 100;
      CHECK(policy.limits().admit(l));
      feedback.read_ok[i] = feedback.torque_on[i] = true; feedback.sampled_at_ms[i] = 100;
    }
    CHECK(arbiter.request(ActuatorAuthority::MOTION, OperatingMode::RUN, &facts.lease) == AuthorityResult::GRANTED);
    facts.mode = OperatingMode::RUN;
    facts.calibration_accepted = facts.envelopes_approved = facts.hardware_qualified = true;
    facts.operator_permit = facts.bms_ok = facts.power_ok = facts.sequential_bus_qualified = true;
    facts.prime_profiles_verified=true; // explicit synthetic register readbacks
    facts.permit_generation = 1; facts.permit_expires_at_ms = 10000;
    envelope.approved = true; envelope.max_velocity_rad_s = 1;
    envelope.max_acceleration_rad_s2 = 10; envelope.tracking_tolerance_rad = .02;
    envelope.feedback_max_age_ms = 100; envelope.batch_timeout_ms = 100;
    envelope.tracking_timeout_ms = 100;
    runtime.begin(&policy, &backend);
    for (unsigned k = 0; k < 12; ++k) gCanonicalIds[k] = servo::legServoAt(k)->bus_id;
    runtime.bindCanonicalBusIds(gCanonicalIds, 12);
    executor.begin(&policy, &runtime, &geometry, &geometry_data::kProvenance, &arbiter);
  }
  bool arm() { return executor.arm(facts, envelope, feedback, 100); }
  motion::JointTargetFrame frame(uint32_t sequence = 1, double q = .01) const {
    motion::JointTargetFrame f; f.valid = true; f.sequence = sequence;
    for (auto& leg : f.legs) leg.hip = leg.upper = leg.lower = q;
    return f;
  }
  FrameValidation validation(uint32_t sequence = 1) const {
    FrameValidation v; v.sequence = sequence; v.policy_epoch = policy.epoch(); v.geometry = geometry.provenanceTag();
    v.collision_clear = v.support_valid = v.path_continuous = v.configuration_valid = true;
    return v;
  }
  bool submit(const motion::JointTargetFrame& f, double dt = 1) {
    return executor.submit(f, validation(f.sequence), 100, 1000, dt, facts, feedback, 100);
  }
  void fresh(uint32_t now, double q = 0) { for (unsigned i = 0; i < 12; ++i) { feedback.actual_rad[i] = q; feedback.sampled_at_ms[i] = now; } }
  void writeAll() { for (uint32_t now = 101; now <= 112; ++now) executor.update(facts, feedback, now); }
};

MotionReadiness ready() {
  MotionReadiness r; r.calibration_restored = r.calibration_accepted = r.limits_approved = true;
  r.hardware_approved = r.operational_permit = r.support_observable = true;
  r.walk_qualified=true;r.max_vx=.03;r.max_vy=.02;r.max_yaw_rate=.15;
  r.power_ok = r.bms_ok = r.authority_live = true; r.mode = OperatingMode::RUN;
  return r;
}
motion::MotionCommand command(motion::Intent intent, uint32_t sequence = 1, motion::ClientOwner owner = motion::ClientOwner::DASHBOARD) {
  motion::MotionCommand c; std::snprintf(c.id, sizeof(c.id), "command-%lu", static_cast<unsigned long>(sequence));
  c.intent = intent; c.sequence = sequence; c.owner = owner; c.issued_at_ms = 100; c.ttl_ms = 500;
  return c;
}

void entireFramePrevalidationAndMapping() {
  for (unsigned index = 0; index < 12; ++index) {
    Rig rig; CHECK(rig.arm()); auto f = rig.frame();
    if (index % 3 == 0) f.legs[index / 3].hip = 1;
    if (index % 3 == 1) f.legs[index / 3].upper = 1;
    if (index % 3 == 2) f.legs[index / 3].lower = 1;
    CHECK(!rig.submit(f)); CHECK(rig.backend.writes.empty());
    CHECK(rig.executor.status().safe_off_required);
    CHECK(rig.executor.status().completed_sequence == 0);
  }
  Rig rig; CHECK(rig.arm()); CHECK(rig.backend.writes.empty() && rig.backend.torque_calls == 0);
  CHECK(rig.submit(rig.frame())); CHECK(rig.backend.writes.empty());
  rig.writeAll(); CHECK(rig.backend.writes.size() == 12);
  CHECK(rig.executor.status().state == ExecutorState::TRACKING);
  for (unsigned i = 0; i < 12 && i < rig.backend.writes.size(); ++i) {
    const auto& g = golden::kJoints[i];
    JointIdentity id; id.leg = static_cast<Leg>(g.leg); id.joint = static_cast<JointKind>(g.joint); setPhysicalUnit(&id, g.unit);
    const auto* t = rig.policy.transforms().find(id, rig.geometry.provenanceTag()); uint16_t raw = 0;
    CHECK(resolveUrdfQToRaw(rig.geometry, geometry_data::kProvenance, *t, 10000, &raw) == TargetResolveStatus::OK);
    CHECK(rig.backend.writes[i].bus == g.bus && rig.backend.writes[i].tick == raw);
  }
  rig.fresh(112, .01); rig.executor.update(rig.facts, rig.feedback, 112);
  CHECK(rig.executor.status().state == ExecutorState::TRACKING); // equals last write, not fresh completion
  rig.fresh(113, .01); rig.executor.update(rig.facts, rig.feedback, 113);
  CHECK(rig.executor.status().state == ExecutorState::READY && rig.executor.status().completed_sequence == 1);
  CHECK(!rig.submit(rig.frame())); REASON(rig.executor.status().reason, "FRAME_SEQUENCE");
}

void runtimeGatesAndRevocation() {
  for (int mutation = 0; mutation < 15; ++mutation) {
    Rig rig;
    switch (mutation) {
      case 0: rig.facts.calibration_accepted = false; break;
      case 1: rig.facts.envelopes_approved = false; break;
      case 2: rig.facts.hardware_qualified = false; break;
      case 3: rig.facts.sequential_bus_qualified = false; break;
      case 4: rig.facts.operator_permit = false; break;
      case 5: rig.facts.permit_expires_at_ms = 100; break;
      case 6: rig.facts.bms_ok = false; break;
      case 7: rig.facts.power_ok = false; break;
      case 8: rig.facts.mode = OperatingMode::MAINTENANCE; break;
      case 9: rig.facts.fault = true; break;
      case 10: ++rig.facts.lease.generation; break;
      case 11: rig.arbiter.forceClear(AuthorityClearReason::FATAL_FAULT); break;
      case 12: rig.facts.permit_generation = 0; break;
      case 13: rig.envelope.approved = false; break;
      case 14: rig.feedback.torque_on[11] = false; break;
    }
    CHECK(!rig.arm()); CHECK(rig.backend.writes.empty() && rig.backend.torque_calls == 0);
  }
  for (int mutation = 0; mutation < 6; ++mutation) {
    Rig rig; CHECK(rig.arm()); CHECK(rig.submit(rig.frame()));
    switch (mutation) {
      case 0: rig.arbiter.forceClear(AuthorityClearReason::FATAL_FAULT); break;
      case 1: ++rig.facts.permit_generation; break;
      case 2: rig.facts.operator_permit = false; break;
      case 3: rig.facts.mode = OperatingMode::MAINTENANCE; break;
      case 4: rig.policy.reset(); break;
      case 5: {
        rig.arbiter.forceClear(AuthorityClearReason::FATAL_FAULT); InhibitLease inhibit;
        CHECK(rig.arbiter.requestInhibit(InhibitReason::FIRMWARE_UPDATE, &inhibit) == AuthorityResult::GRANTED); break;
      }
    }
    rig.executor.update(rig.facts, rig.feedback, 101);
    CHECK(rig.executor.status().safe_off_required && rig.backend.writes.empty());
  }
}

void safetyOffAndFailureOutcomes() {
  for (const auto outcome : {BackendWriteOutcome::VERIFIED_NOT_APPLIED, BackendWriteOutcome::UNCERTAIN}) {
    Rig rig; CHECK(rig.arm()); CHECK(rig.submit(rig.frame())); rig.backend.outcome = outcome;
    rig.executor.update(rig.facts, rig.feedback, 101);
    CHECK(rig.backend.writes.size() == 1 && rig.executor.status().safe_off_required);
    REASON(rig.executor.status().reason, "BACKEND_UNCERTAIN_OR_REJECTED");
    rig.executor.update(rig.facts, rig.feedback, 102); CHECK(rig.backend.writes.size() == 1);
    rig.executor.confirmSafeOff(false); CHECK(rig.executor.status().safe_off_required);
    CHECK(!rig.arm()); rig.executor.confirmSafeOff(true);
    CHECK(rig.executor.status().state == ExecutorState::OFF && !rig.executor.status().safe_off_required);
  }
  Rig timeout; CHECK(timeout.arm()); CHECK(timeout.submit(timeout.frame())); timeout.fresh(201);
  timeout.executor.update(timeout.facts, timeout.feedback, 201);
  REASON(timeout.executor.status().reason, "BUS_BATCH_TIMEOUT"); CHECK(timeout.backend.writes.empty());
  Rig tracking; CHECK(tracking.arm()); CHECK(tracking.submit(tracking.frame())); tracking.writeAll();
  tracking.fresh(212, .1); tracking.executor.update(tracking.facts, tracking.feedback, 212);
  CHECK(tracking.executor.status().safe_off_required); REASON(tracking.executor.status().reason, "TRACKING_TIMEOUT");
  Rig expiry; CHECK(expiry.arm());
  CHECK(expiry.executor.submit(expiry.frame(), expiry.validation(), 100, 10, 1, expiry.facts, expiry.feedback, 100));
  expiry.executor.update(expiry.facts, expiry.feedback, 110); CHECK(expiry.backend.writes.empty());
  REASON(expiry.executor.status().reason, "COMMAND_TIMEOUT");
}

void rateTrackingFeedbackAndProofGates() {
  for (int mutation = 0; mutation < 15; ++mutation) {
    Rig rig; CHECK(rig.arm()); auto f = rig.frame(); auto v = rig.validation();
    uint32_t issued = 100, ttl = 1000; double dt = 1;
    switch (mutation) {
      case 0: dt = .001; break;
      case 1: rig.feedback.actual_rad[11] = .1; break;
      case 2: rig.feedback.sampled_at_ms[11] = 101; break;
      case 3: rig.feedback.read_ok[11] = false; break;
      case 4: rig.feedback.torque_on[11] = false; break;
      case 5: rig.feedback.actual_rad[11] = std::numeric_limits<double>::quiet_NaN(); break;
      case 6: v.collision_clear = false; break;
      case 7: v.support_valid = false; break;
      case 8: v.path_continuous = false; break;
      case 9: v.configuration_valid = false; break;
      case 10: issued = 101; break;
      case 11: ttl = 0; break;
      case 12: v.sequence = 2; break;
      case 13: ++v.policy_epoch; break;
      case 14: v.geometry ^= 1; break;
    }
    CHECK(!rig.executor.submit(f, v, issued, ttl, dt, rig.facts, rig.feedback, 100));
    CHECK(rig.backend.writes.empty()); CHECK(rig.executor.status().safe_off_required);
  }
  Rig stale; CHECK(stale.arm()); CHECK(stale.submit(stale.frame()));
  stale.executor.update(stale.facts, stale.feedback, 201); REASON(stale.executor.status().reason, "STALE_FEEDBACK");
  Rig acceleration; acceleration.envelope.max_velocity_rad_s = 100; acceleration.envelope.max_acceleration_rad_s2 = .001;
  CHECK(acceleration.arm()); CHECK(!acceleration.submit(acceleration.frame(), 1));
  REASON(acceleration.executor.status().reason, "RATE_LIMIT"); CHECK(acceleration.backend.writes.empty());
  // Stay inside the synthetic approved envelope while exceeding tracking tolerance.
  // Out-of-envelope feedback is covered independently by the lifecycle suite.
  Rig hold; CHECK(hold.arm()); hold.fresh(101, .05); hold.executor.update(hold.facts, hold.feedback, 101);
  CHECK(hold.executor.status().safe_off_required && hold.backend.writes.empty());
  REASON(hold.executor.status().reason, "HOLD_TRACKING_ERROR");
  Rig divergence; CHECK(divergence.arm()); CHECK(divergence.submit(divergence.frame()));
  divergence.fresh(101, .5); divergence.executor.update(divergence.facts, divergence.feedback, 101);
  CHECK(divergence.executor.status().safe_off_required && divergence.backend.writes.empty());
}

void serviceContractReplayAndOwners() {
  Rig rig; Planner planner; MotionService service; service.begin(&rig.executor, &planner);
  auto r = ready(); service.refresh(r, 100);
  auto c = command(motion::Intent::STAND);
  auto ack = service.command(c, 100); CHECK(ack.outcome == motion::CommandOutcome::ACCEPTED && planner.accepts == 1);
  CHECK(service.command(c, 100).outcome == motion::CommandOutcome::ACCEPTED && planner.accepts == 1);
  auto mismatch = c; mismatch.target_height_m = .14;
  REASON(service.command(mismatch, 100).reason, "COMMAND_REPLAY_MISMATCH"); CHECK(planner.accepts == 1);
  auto reused = c; reused.sequence = 2;
  REASON(service.command(reused, 100).reason, "COMMAND_ID_REUSED");
  auto other = command(motion::Intent::STAND, 1, motion::ClientOwner::JETSON);
  REASON(service.command(other, 100).reason, "COMMAND_OWNER_BUSY");
  auto stop = command(motion::Intent::STOP, 1, motion::ClientOwner::JETSON);
  CHECK(service.command(stop, 100).outcome == motion::CommandOutcome::COMPLETED && planner.stops == 1);
  other = command(motion::Intent::STAND, 2, motion::ClientOwner::JETSON);
  CHECK(service.command(other, 100).outcome == motion::CommandOutcome::ACCEPTED);
  auto replay = command(motion::Intent::STATUS, 1, motion::ClientOwner::JETSON);
  REASON(service.command(replay, 100).reason, "SEQUENCE_REPLAY");
  for (int mutation = 0; mutation < 7; ++mutation) {
    c = command(motion::Intent::STAND, 3 + mutation);
    if (mutation == 0) c.version = 2;
    if (mutation == 1) c.owner = motion::ClientOwner::UNKNOWN;
    if (mutation == 2) c.issued_at_ms = 101;
    if (mutation == 3) c.ttl_ms = 0;
    if (mutation == 4) c.sequence = 0;
    if (mutation == 5) c.target_height_m = std::numeric_limits<double>::infinity();
    if (mutation == 6) c.id[0] = '"';
    CHECK(service.command(c, 100).outcome == motion::CommandOutcome::REJECTED);
  }
}

void serviceUnavailableActionsAndTeleopStop() {
  Rig rig; Planner planner; MotionService service; service.begin(&rig.executor, &planner);
  auto c = command(motion::Intent::STAND); REASON(service.command(c, 100).reason, "RESTORE_REQUIRED");
  CHECK(planner.accepts == 0 && rig.backend.writes.empty());
  auto r = ready(); service.refresh(r, 100);
  for (size_t i = 0; i < motion::kMotionActionCount; ++i) {
    const auto& action = motion::actionCatalog()[i];
    if (action.implemented) continue;
    c = command(motion::Intent::ACTION_EXECUTE, static_cast<uint32_t>(i + 2));
    std::strcpy(c.action_id, action.id);
    CHECK(!service.available(action)); REASON(service.command(c, 100).reason, "ACTION_NOT_VALIDATED");
  }
  c = command(motion::Intent::GAIT_SELECT, 100); std::strcpy(c.gait, "TROT");
  REASON(service.command(c, 100).reason, "GAIT_NOT_VALIDATED");
  c = command(motion::Intent::ACTION_EXECUTE, 101); std::strcpy(c.action_id, "gait.walk");
  CHECK(service.command(c, 100).outcome == motion::CommandOutcome::COMPLETED);
  CHECK(planner.last.intent == motion::Intent::GAIT_SELECT && std::strcmp(planner.last.gait, "WALK") == 0);
  service.disconnected(100);
  CHECK(rig.arm());
  c = command(motion::Intent::BASE_VELOCITY, 102); c.vx = .01; c.vy = -.01; c.yaw_rate = .1; c.ttl_ms = 50;
  CHECK(service.command(c, 100).outcome == motion::CommandOutcome::ACCEPTED);
  const int stops = planner.stops; service.refresh(r, 149); CHECK(planner.stops == stops);
  service.refresh(r, 150); CHECK(planner.stops == stops + 1 && rig.executor.status().safe_off_required);
  c = command(motion::Intent::BASE_VELOCITY, 103); c.vx = .031;
  REASON(service.command(c, 100).reason, "VELOCITY_OUTSIDE_SOFTWARE_ENVELOPE");
  c = command(motion::Intent::BASE_VELOCITY, 104); c.ttl_ms = 1001;
  REASON(service.command(c, 100).reason, "TTL_RANGE");
  for (int gate = 0; gate < 11; ++gate) {
    auto bad = r;
    switch (gate) {
      case 0: bad.calibration_restored = false; break;
      case 1: bad.calibration_accepted = false; break;
      case 2: bad.limits_approved = false; break;
      case 3: bad.hardware_approved = false; break;
      case 4: bad.operational_permit = false; break;
      case 5: bad.support_observable = false; break;
      case 6: bad.power_ok = false; break;
      case 7: bad.bms_ok = false; break;
      case 8: bad.fault = true; break;
      case 9: bad.inhibited = true; break;
      case 10: bad.authority_live = false; break;
    }
    service.refresh(bad, 100); c = command(motion::Intent::STAND, 110 + gate);
    CHECK(service.command(c, 100).outcome == motion::CommandOutcome::REJECTED);
    CHECK(!service.available(*motion::findAction("posture.stand")));
    c = command(motion::Intent::STOP, 110 + gate); CHECK(service.command(c, 100).outcome != motion::CommandOutcome::REJECTED);
  }
}

void localSessionOwnershipSurvivesCommandBoundaries(){
 Rig rig;Planner planner;MotionService service;service.begin(&rig.executor,&planner);auto qualified=ready();qualified.local_operator_owns_session=true;qualified.first_stride_only=true;service.refresh(qualified,100);CHECK(rig.arm());
 unsigned seq=140;for(auto intent:{motion::Intent::STAND,motion::Intent::BASE_VELOCITY,motion::Intent::GAIT_SELECT}){auto web=command(intent,seq++);web.vx=.001;strcpy(web.gait,"WALK");auto a=service.command(web,100);CHECK(a.outcome==motion::CommandOutcome::REJECTED);REASON(a.reason,"LOCAL_OPERATOR_CONTROLS_SESSION");}
 auto local=command(motion::Intent::STAND,1);local.owner=motion::ClientOwner::LOCAL;CHECK(service.command(local,100).outcome==motion::CommandOutcome::ACCEPTED);service.refresh(qualified,101);CHECK(service.activeAck().outcome==motion::CommandOutcome::COMPLETED);
 auto web=command(motion::Intent::STAND,160);web.issued_at_ms=101;CHECK(service.command(web,101).outcome==motion::CommandOutcome::REJECTED);CHECK(!service.available(*motion::findAction("posture.stand")));CHECK(service.available(*motion::findAction("motion.stop")));
}

void coordinatedStopIsPendingAndEmergencyStillDeescalates() {
  Rig rig; Planner planner; MotionService service; service.begin(&rig.executor, &planner);
  service.refresh(ready(), 100); CHECK(rig.arm());
  auto velocity = command(motion::Intent::BASE_VELOCITY); velocity.vx = .01;
  CHECK(service.command(velocity, 100).outcome == motion::CommandOutcome::ACCEPTED);
  planner.stopping = true; const int accepts = planner.accepts;
  auto stop = command(motion::Intent::STOP, 2);
  CHECK(service.command(stop, 100).outcome == motion::CommandOutcome::ACCEPTED);
  CHECK(planner.accepts == accepts && planner.stops == 1);
  CHECK(rig.executor.status().state == ExecutorState::READY && !rig.executor.status().safe_off_required);
  auto fault = ready(); fault.fault = true; service.refresh(fault, 101);
  CHECK(rig.executor.status().safe_off_required); // coordinated stop cannot suppress emergency stop
}

motion::FeedbackPolicy coordinatorPolicy(motion::FeedbackMode mode) {
  motion::FeedbackPolicy p; p.mode = mode; p.envelopesApproved = true; p.loadedRatesVerified = true;
  p.maxJointStepRad = .1; p.maxTrackingErrorRad = .02;
  p.observationMaxAgeS = p.assessmentMaxAgeS = .15; p.segmentTimeoutS = .5;
  p.heightToleranceM = p.contactToleranceM = .001; p.samplePeriodS = .04;
  for (unsigned i = 0; i < 4; ++i) {
    const auto* model = motion::legModel(static_cast<motion::LegId>(i));
    for (unsigned j = 0; j < 3; ++j) p.jointLimits[i][j] = model->limits[j];
    p.maxVelocityRadS[i] = {2, 2, 2}; p.maxAccelerationRadS2[i] = {12, 12, 12};
  }
  return p;
}
motion::WalkSpecification coordinatorWalk() {
  motion::WalkSpecification w; w.periodS = 6; w.rampDurationS = 3; w.liftM = .005; w.duty = .8;
  w.maxVxMps = .004; w.maxVyMps = .003; w.maxYawRateRadS = .03; w.maxCommandTtlS = .5;
  return w;
}
motion::AdaptiveObservation coordinatorObservation(double height, uint32_t now, motion::EvidenceSource source) {
  motion::GaitParameters p; const auto cartesian = motion::gaitHold(p, 0, height, 0);
  const auto solution = motion::solveGait(p, cartesian); CHECK(solution.target.valid);
  motion::AdaptiveObservation o; o.actual = solution.target; o.body = cartesian.body;
  o.stampS = now / 1000.; o.heightSource = o.supportSource = source;
  o.confirmedFeet = 15; o.flatPlaneConfirmed = o.stationary = true;
  return o;
}
void setCoordinatorFeedback(Rig& rig, const motion::AdaptiveObservation& o, uint32_t now) {
  // This unrestricted raw interval is a deliberately synthetic fixture so the
  // test can reach the integration boundary; it is never a production approval.
  for (unsigned i = 0; i < 12; ++i) {
    const auto& g = golden::kJoints[i]; JointIdentity id;
    id.leg = static_cast<Leg>(g.leg); id.joint = static_cast<JointKind>(g.joint); setPhysicalUnit(&id, g.unit);
    auto limit = *rig.policy.limits().find(id, rig.geometry.provenanceTag());
    limit.min_tick = 0; limit.max_tick = 4095; CHECK(rig.policy.limits().admit(limit));
    const auto q = o.actual.legs[i / 3];
    rig.feedback.actual_rad[i] = i % 3 == 0 ? q.hip : i % 3 == 1 ? q.upper : q.lower;
    rig.feedback.sampled_at_ms[i] = now;
  }
}
motion::SegmentAssessment coordinatorAssessment(const motion::FeedbackFrame& frame, uint32_t now, motion::EvidenceSource source) {
  motion::SegmentAssessment a; a.sequence = frame.target.sequence; a.stampS = now / 1000.; a.source = source;
  a.geometryIdentityVerified = a.sweptCollisionClear = a.supportCompatible = a.continuousBoundsConfirmed = true;
  a.loadedTrackingQualified = true; return a;
}

void realCoordinatorRejectsSyntheticRuntimeAndRestartsFreshSessions() {
  // A fully qualified fake executor is intentionally armed. The SIMULATION
  // coordinator can generate and assess frames, but cannot send one to runtime.
  Rig simulated; MotionCoordinator simulation; simulation.begin(&simulated.executor, &simulated.policy);
  CHECK(simulation.configure(coordinatorPolicy(motion::FeedbackMode::SIMULATION), coordinatorWalk(), nullptr));
  auto o = coordinatorObservation(.125, 100, motion::EvidenceSource::SYNTHETIC);
  setCoordinatorFeedback(simulated, o, 100); CHECK(simulated.arm()); simulation.observe(o);
  CHECK(simulation.accept(command(motion::Intent::STAND), 100) == nullptr);
  simulation.update(simulated.facts, simulated.feedback, 100);
  o.stampS = .101; simulation.observe(o); simulation.update(simulated.facts, simulated.feedback, 101);
  CHECK(simulation.frame().target.valid && simulation.frame().target.sequence == 1);
  CHECK(simulation.assess(coordinatorAssessment(simulation.frame(), 101, motion::EvidenceSource::SYNTHETIC), 101));
  simulation.update(simulated.facts, simulated.feedback, 101);
  CHECK(simulation.frame().executable && !simulation.frame().physicalReadiness);
  CHECK(simulated.executor.status().accepted_sequence == 0 && simulated.backend.writes.empty());

  // Exercise one real coordinator frame, then safe-off/re-arm and start again.
  // The second firstSequence1 must be unassessed and then reach a NEW submit.
  Rig physical; MotionCoordinator coordinator; coordinator.begin(&physical.executor, &physical.policy);
  CHECK(coordinator.configure(coordinatorPolicy(motion::FeedbackMode::PHYSICAL_REVIEW), coordinatorWalk(), nullptr));
  o = coordinatorObservation(.125, 100, motion::EvidenceSource::EXTERNAL_MEASURED);
  setCoordinatorFeedback(physical, o, 100); CHECK(physical.arm()); coordinator.observe(o);
  const uint32_t first_arm_generation = physical.executor.status().arm_generation;
  CHECK(first_arm_generation != 0);
  CHECK(coordinator.accept(command(motion::Intent::STAND), 100) == nullptr);
  coordinator.update(physical.facts, physical.feedback, 100);
  o.stampS = .101; coordinator.observe(o); coordinator.update(physical.facts, physical.feedback, 101);
  CHECK(coordinator.frame().target.valid && !coordinator.frame().executable);
  CHECK(coordinator.assess(coordinatorAssessment(coordinator.frame(), 101, motion::EvidenceSource::EXTERNAL_MEASURED), 101));
  coordinator.update(physical.facts, physical.feedback, 101);
  CHECK(physical.executor.status().accepted_sequence == 1 && physical.executor.status().state == ExecutorState::WRITING);
  const auto first = coordinator.frame();
  for (uint32_t now = 102; now <= 113; ++now) physical.executor.update(physical.facts, physical.feedback, now);
  CHECK(physical.backend.writes.size() == 12);
  o.actual = first.target; o.body = first.body; o.stampS = .114;
  setCoordinatorFeedback(physical, o, 114); physical.executor.update(physical.facts, physical.feedback, 114);
  CHECK(physical.executor.status().completed_sequence == 1);
  coordinator.stop(114); physical.executor.stop("TEST_REARM"); physical.executor.confirmSafeOff(true);
  CHECK(physical.executor.arm(physical.facts, physical.envelope, physical.feedback, 150));
  CHECK(physical.executor.status().arm_generation > first_arm_generation);
  o.stampS = .150; coordinator.observe(o);
  CHECK(coordinator.accept(command(motion::Intent::STAND, 2), 150) == nullptr);
  coordinator.update(physical.facts, physical.feedback, 150);
  o.stampS = .151; coordinator.observe(o); coordinator.update(physical.facts, physical.feedback, 151);
  CHECK(coordinator.frame().target.valid && coordinator.frame().target.sequence == 1);
  CHECK(!coordinator.frame().executable && physical.executor.status().accepted_sequence == 0);
  CHECK(physical.backend.writes.size() == 12); // old assessment cannot authorize restart
  CHECK(coordinator.assess(coordinatorAssessment(coordinator.frame(), 151, motion::EvidenceSource::EXTERNAL_MEASURED), 151));
  coordinator.update(physical.facts, physical.feedback, 151);
  CHECK(physical.executor.status().accepted_sequence == 1 && physical.executor.status().state == ExecutorState::WRITING);
  physical.executor.update(physical.facts, physical.feedback, 152); CHECK(physical.backend.writes.size() == 13);

  // A separate prior WALK session must also disappear when executor re-arms.
  Rig walked; MotionCoordinator walking; walking.begin(&walked.executor, &walked.policy);
  CHECK(walking.configure(coordinatorPolicy(motion::FeedbackMode::PHYSICAL_REVIEW), coordinatorWalk(), nullptr));
  o = coordinatorObservation(.150, 100, motion::EvidenceSource::EXTERNAL_MEASURED);
  setCoordinatorFeedback(walked, o, 100); CHECK(walked.arm()); walking.observe(o);
  CHECK(walking.accept(command(motion::Intent::STAND), 100) == nullptr);
  walking.update(walked.facts, walked.feedback, 100);
  o.stampS = .101; walking.observe(o); walking.update(walked.facts, walked.feedback, 101);
  REASON(walking.state(), "STAND");
  auto velocity = command(motion::Intent::BASE_VELOCITY, 2); velocity.vx = .001; velocity.issued_at_ms = 101;
  CHECK(walking.accept(velocity, 101) == nullptr); REASON(walking.state(), "WALK_STARTING");
  walked.executor.stop("TEST_WALK_REARM"); walked.executor.confirmSafeOff(true);
  CHECK(walked.executor.arm(walked.facts, walked.envelope, walked.feedback, 150));
  o.stampS = .150; walking.observe(o);
  walking.update(walked.facts, walked.feedback, 150);
  REASON(walking.state(), "STOPPED"); CHECK(!walking.frame().target.valid);
  CHECK(!walking.coordinatedStopping() && walked.backend.writes.empty());
  CHECK(walking.accept(command(motion::Intent::STAND, 3), 150) == nullptr);
  REASON(walking.state(), "OBSERVE"); CHECK(!walking.coordinatedStopping());
}

void measuredSupportRevocationPrecedesTheNextWrite() {
  for (unsigned leg = 0; leg < 4; ++leg) {
    Rig rig; MotionCoordinator coordinator;
    coordinator.begin(&rig.executor, &rig.policy);
    CHECK(coordinator.configure(coordinatorPolicy(motion::FeedbackMode::PHYSICAL_REVIEW), coordinatorWalk(), nullptr));
    auto observed = coordinatorObservation(.125, 100, motion::EvidenceSource::EXTERNAL_MEASURED);
    setCoordinatorFeedback(rig, observed, 100); CHECK(rig.arm()); coordinator.observe(observed);
    CHECK(coordinator.accept(command(motion::Intent::STAND), 100) == nullptr);
    coordinator.update(rig.facts, rig.feedback, 100);
    observed.stampS = .101; coordinator.observe(observed);
    coordinator.update(rig.facts, rig.feedback, 101);
    CHECK(coordinator.frame().target.valid);
    CHECK(coordinator.assess(coordinatorAssessment(coordinator.frame(), 101, motion::EvidenceSource::EXTERNAL_MEASURED), 101));
    coordinator.update(rig.facts, rig.feedback, 101);
    CHECK(rig.executor.status().state == ExecutorState::WRITING);
    coordinator.update(rig.facts, rig.feedback, 102);
    rig.executor.update(rig.facts, rig.feedback, 102);
    CHECK(rig.backend.writes.size() == 1);
    // The ACK is deliberately absent. Loss of a held support must still revoke
    // the remaining 11 writes in the same ordering used by Controller.
    observed.confirmedFeet &= static_cast<uint8_t>(~(1u << leg));
    observed.stampS = .103; observed.acknowledged = false;
    coordinator.observe(observed);
    coordinator.update(rig.facts, rig.feedback, 103);
    rig.executor.update(rig.facts, rig.feedback, 103);
    REASON(coordinator.state(), "REJECTED");
    REASON(rig.executor.status().reason, "SUPPORT_INVALID");
    CHECK(rig.executor.status().safe_off_required && rig.backend.writes.size() == 1);
  }
}

std::string envelopeJson(const char* type = "POSTURE.STAND", const char* params = "{\"target_height_m\":0.15}") {
  return std::string("{\"version\":1,\"id\":\"json-1\",\"sequence\":1,\"issued_at_ms\":100,\"ttl_ms\":500,\"owner\":\"dashboard\",\"type\":\"") + type + "\",\"params\":" + params + "}";
}
void jsonStrictBoundary() {
  const auto good = envelopeJson(); motion::MotionCommand parsed; const char* reason = nullptr;
  CHECK(parseMotionJson(good.data(), good.size(), &parsed, &reason));
  CHECK(parsed.intent == motion::Intent::STAND && parsed.target_height_m == .15);
  const std::vector<std::string> invalid = {
    good + "x", good.substr(0, good.size() - 1) + ",", // missing final top-level closure
    "{\"extra\":1," + good.substr(1),
    "{\"version\":1," + good.substr(1),
    envelopeJson("POSTURE.STAND", "{\"target_height_m\":0.15,\"target_height_m\":0.1}"),
    envelopeJson("POSTURE.STAND", "{\"target_height_m\":0.15,\"raw_tick\":2048}"),
    envelopeJson("POSTURE.STAND", "{}"),
    envelopeJson("BASE_VELOCITY", "{\"vx\":0.01,\"vy\":0,\"yaw_rate\":0,\"authority\":\"motion\"}"),
    envelopeJson("BASE_VELOCITY", "{\"vx\":0.01,\"vy\":0}"),
    envelopeJson("POSTURE.STAND", "{\"target_height_m\":1e999}"),
    envelopeJson("POSTURE.STAND", "{\"target_height_m\":NaN}"),
    envelopeJson("POSTURE.STAND", "{\"target_height_m\":01}"),
    envelopeJson("POSTURE.STAND", "{\"target_height_m\":0.15,}"),
    envelopeJson("MOTION.STATUS", "{\"target_height_m\":0.15}"),
    envelopeJson("SERVO.WRITE", "{}")
  };
  for (const auto& text : invalid) CHECK(!parseMotionJson(text.data(), text.size(), &parsed, &reason));
  CHECK(!parseMotionJson(nullptr, 10, &parsed, &reason));
  auto nul = good; nul.insert(nul.begin() + 10, '\0'); CHECK(!parseMotionJson(nul.data(), nul.size(), &parsed, &reason));
  CHECK(parseMotionJson(envelopeJson("BASE_VELOCITY", "{\"vx\":0.01,\"vy\":-0.02,\"yaw_rate\":0.1}").c_str(), envelopeJson("BASE_VELOCITY", "{\"vx\":0.01,\"vy\":-0.02,\"yaw_rate\":0.1}").size(), &parsed, &reason));
}

void jsonCapabilityVisibilityAndStaleTelemetry() {
  Rig rig; Planner planner; MotionService service; service.begin(&rig.executor, &planner);
  std::vector<char> buffer(24000); CHECK(service.capabilitiesJson(buffer.data(), buffer.size()));
  const std::string capabilities = buffer.data();
  for (size_t i = 0; i < motion::kMotionActionCount; ++i) {
    const auto& a = motion::actionCatalog()[i]; auto pos = capabilities.find(std::string("\"id\":\"") + a.id + "\""); CHECK(pos != std::string::npos);
    if (pos == std::string::npos) continue;
    const auto object = capabilities.substr(pos, capabilities.find('}', pos) - pos);
    if (a.intent != motion::Intent::STOP && a.intent != motion::Intent::STATUS) CHECK(object.find("\"available\":false") != std::string::npos);
  }
  CHECK(service.statusJson(buffer.data(), buffer.size(), 100)); CHECK(std::strstr(buffer.data(), "\"stale\":true"));
  MotionTelemetry telemetry; telemetry.joints_valid = true; telemetry.quaternion_valid = true;
  telemetry.sampled_at_ms = telemetry.quaternion_at_ms = 100; service.publishTelemetry(telemetry);
  CHECK(service.statusJson(buffer.data(), buffer.size(), 101)); CHECK(std::strstr(buffer.data(), "\"stale\":false"));
  CHECK(service.statusJson(buffer.data(), buffer.size(), 601)); CHECK(std::strstr(buffer.data(), "\"stale\":true"));
  CHECK(std::strstr(buffer.data(), "\"quaternion\":null"));
  char tiny[2]; CHECK(!service.capabilitiesJson(tiny, sizeof(tiny))); CHECK(tiny[1] == 0);
}
}  // namespace

int main() {
 localSessionOwnershipSurvivesCommandBoundaries();
  entireFramePrevalidationAndMapping(); runtimeGatesAndRevocation(); safetyOffAndFailureOutcomes();
  rateTrackingFeedbackAndProofGates(); serviceContractReplayAndOwners();
  serviceUnavailableActionsAndTeleopStop(); jsonStrictBoundary(); jsonCapabilityVisibilityAndStaleTelemetry();
  coordinatedStopIsPendingAndEmergencyStillDeescalates();
  realCoordinatorRejectsSyntheticRuntimeAndRestartsFreshSessions();
  measuredSupportRevocationPrecedesTheNextWrite();
  std::printf("MotionServiceExecutor: %d checks, %d failures (synthetic approvals; no hardware)\n", checks, failures);
  return failures ? 1 : 0;
}
