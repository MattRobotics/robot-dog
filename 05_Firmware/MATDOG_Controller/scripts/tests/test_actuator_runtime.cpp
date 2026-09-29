// Offline host tests for the Safe Actuator runtime adapter
// (src/actuator/ActuatorRuntime.*) — I4, V3 handoff §12.
//
// Links the REAL adapter AND the REAL policy AND the REAL arbiter, the same
// contract as test_actuator_write_policy.cpp: a mocked policy here would
// test the mock's idea of the decision, not the shipped one. Only the
// backend is fake — that is the one thing this adapter is meant to be
// tested against, exactly like update/OtaPolicy's fake OtaBackend.
//
// NO HOST TEST IN THIS FILE IS HARDWARE VALIDATION. There is still no
// production ActuatorBackend anywhere in this firmware — ServoBus exposes
// exactly one write, safeOff() (torque OFF) — and nothing in Controller.cpp
// constructs or owns an ActuatorRuntime; scripts/static_audit.py enforces
// that structurally.
//
// Same conventions as the other suites: no framework, a CHECK macro and a
// pass/fail tally. Run via scripts/tests/run_host_tests.sh.

#include <cstdio>
#include <cstring>

#include "../../src/actuator/ActuatorRuntime.h"
#include "../../src/actuator/CalibrationGeometryProfileData.h"

using namespace matdog;
using namespace matdog::actuator;
using matdog::calibration::CalibrationOrigin;
using matdog::calibration::EvidenceState;
using matdog::calibration::JointIdentity;
using matdog::calibration::JointKind;
using matdog::calibration::Leg;
using matdog::core::ActuatorAuthority;
using matdog::core::ActuatorAuthorityArbiter;
using matdog::core::AuthorityClearReason;
using matdog::core::AuthorityLease;
using matdog::core::AuthorityResult;
using matdog::core::OperatingMode;

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

#define CHECK_STR(actual, expected)                                            \
  do {                                                                         \
    ++g_checks;                                                                \
    if (std::strcmp((actual), (expected)) != 0) {                              \
      ++g_failures;                                                            \
      std::printf("  FAIL [%s] %s:%d: %s == \"%s\", expected \"%s\"\n", g_case, \
                  __FILE__, __LINE__, #actual, (actual), (expected));          \
    }                                                                          \
  } while (0)

namespace {

// ---------------------------------------------------------------------------
// Fixtures — mirrors test_actuator_write_policy.cpp exactly, so the ACCEPT
// paths under test here are reached the same way the policy's own suite
// reaches them.
// ---------------------------------------------------------------------------

JointIdentity joint(Leg leg, JointKind kind, const char* unit) {
  JointIdentity id{};
  id.leg = leg;
  id.joint = kind;
  calibration::setPhysicalUnit(&id, unit);
  return id;
}

JointIdentity lfLower() { return joint(Leg::LF, JointKind::LOWER, "M33"); }

ActuatorCommand command(ActuatorOperation operation, JointIdentity id, uint16_t target = 0) {
  ActuatorCommand cmd{};
  cmd.operation = operation;
  cmd.joint = id;
  cmd.target_tick = target;
  return cmd;
}

GeometryProvenanceTag testGeometry() { return geometryProvenanceTag(geometry_data::kProvenance); }

CalibrationGeometryProfile boundProfile() {
  CalibrationGeometryProfile profile;
  profile.bind(&geometry_data::kProvenance, geometry_data::kJoints, geometry_data::kJointCount,
              geometry_data::kEndpoints, geometry_data::kEndpointCount);
  return profile;
}

JointLimit acceptedLimit(JointIdentity id, uint16_t lo, uint16_t hi) {
  JointLimit limit{};
  limit.identity = id;
  limit.geometry = testGeometry();
  limit.state = EvidenceState::PROMOTED;
  limit.origin = CalibrationOrigin::LIVE_SESSION;
  limit.min_tick = lo;
  limit.max_tick = hi;
  limit.present = true;
  return limit;
}

AuthorityLease grant(ActuatorAuthorityArbiter& arbiter, ActuatorAuthority owner,
                     OperatingMode mode) {
  AuthorityLease lease{};
  const AuthorityResult result = arbiter.request(owner, mode, &lease);
  CHECK(result == AuthorityResult::GRANTED);
  return lease;
}

// Records every call it receives; each method's return value is
// independently steerable so VERIFIED_APPLIED, VERIFIED_NOT_APPLIED and
// UNCERTAIN are all reachable without a second fixture.
class FakeActuatorBackend : public ActuatorBackend {
 public:
  BackendWriteOutcome enableTorque(uint8_t bus_id) override {
    ++enable_torque_calls;
    last_bus_id = bus_id;
    return enable_torque_result;
  }
  BackendWriteOutcome writeGoalPosition(uint8_t bus_id, uint16_t target_tick,
                                        actuator::MotionProfile profile) override {
    ++write_goal_position_calls;
    last_bus_id = bus_id;
    last_target_tick = target_tick;
    last_profile = profile;
    return write_goal_position_result;
  }
  actuator::MotionProfile last_profile = actuator::MotionProfile::BOUNDED_DEFAULT;

  int enable_torque_calls = 0;
  int write_goal_position_calls = 0;
  uint8_t last_bus_id = 0;
  uint16_t last_target_tick = 0;
  BackendWriteOutcome enable_torque_result = BackendWriteOutcome::VERIFIED_APPLIED;
  BackendWriteOutcome write_goal_position_result = BackendWriteOutcome::VERIFIED_APPLIED;

  int totalCalls() const { return enable_torque_calls + write_goal_position_calls; }
};

// ---------------------------------------------------------------------------
// backendCallFor() — pure, total, exhaustively testable without an ACCEPT
// ---------------------------------------------------------------------------

void testBackendCallForCoversEveryOperation() {
  g_case = "backend_call_for";
  CHECK_EQ((int)backendCallFor(ActuatorOperation::TORQUE_ENABLE),
          (int)BackendCallKind::ENABLE_TORQUE);
  CHECK_EQ((int)backendCallFor(ActuatorOperation::POSITION_COMMAND),
          (int)BackendCallKind::WRITE_GOAL_POSITION);
  CHECK_EQ((int)backendCallFor(ActuatorOperation::CALIBRATION_CONTACT_PROBE),
          (int)BackendCallKind::WRITE_GOAL_POSITION);
  CHECK_EQ((int)backendCallFor(ActuatorOperation::DIRECTION_VERIFY),
          (int)BackendCallKind::WRITE_GOAL_POSITION);
  CHECK_EQ((int)backendCallFor(ActuatorOperation::CALIBRATION_AUXILIARY_MOVE),
          (int)BackendCallKind::WRITE_GOAL_POSITION);
  CHECK_EQ((int)backendCallFor(ActuatorOperation::NONE), (int)BackendCallKind::NONE);
}

// ---------------------------------------------------------------------------
// execute() — the integration: policy decision -> at most one backend call
// ---------------------------------------------------------------------------

void testNoPolicyIsRefusedAndNeverTouchesBackend() {
  g_case = "no policy bound";
  ActuatorRuntime runtime;
  FakeActuatorBackend backend;
  runtime.begin(/*policy=*/nullptr, &backend);

  ActuatorTransaction txn{};
  txn.command = command(ActuatorOperation::TORQUE_ENABLE, lfLower());
  WriteDecision decision = WriteDecision::ACCEPT;
  const ExecuteResult result = runtime.execute(&txn, /*bus_id=*/11, &decision);

  CHECK_EQ((int)result, (int)ExecuteResult::NOT_EXECUTED);
  CHECK_EQ((int)decision, (int)WriteDecision::REJECT_NO_ARBITER);
  CHECK_EQ(backend.totalCalls(), 0);
}

// The safety property that matters most: a REJECTing policy decision must
// never reach the backend, regardless of which command was attempted.
void testPolicyRejectionNeverReachesBackend() {
  g_case = "policy rejection never reaches backend";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  ActuatorRuntime runtime;
  FakeActuatorBackend backend;
  runtime.begin(&policy, &backend);

  // No authority was ever granted — REJECT_NO_AUTHORITY at commit time
  // (plan() never ran, so the transaction was never PLANNED either;
  // commit() must refuse it on transaction state, not on authority — the
  // point under test is only that no state reaches ACCEPT).
  ActuatorTransaction txn{};
  txn.command = command(ActuatorOperation::TORQUE_ENABLE, lfLower());
  WriteDecision decision = WriteDecision::ACCEPT;
  const ExecuteResult result = runtime.execute(&txn, /*bus_id=*/11, &decision);

  CHECK_EQ((int)result, (int)ExecuteResult::NOT_EXECUTED);
  CHECK(decision != WriteDecision::ACCEPT);
  CHECK_EQ(backend.totalCalls(), 0);
}

void testTorqueEnableAcceptWrittenThroughToBackend() {
  g_case = "torque enable written";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease =
      grant(arbiter, ActuatorAuthority::MOTION, OperatingMode::RUN);

  ActuatorTransaction txn{};
  CHECK_EQ((int)policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
                            OperatingMode::RUN, &txn),
          (int)WriteDecision::ACCEPT);

  ActuatorRuntime runtime;
  FakeActuatorBackend backend;
  runtime.begin(&policy, &backend);
  WriteDecision decision = WriteDecision::REJECT_NO_ARBITER;
  const ExecuteResult result = runtime.execute(&txn, /*bus_id=*/13, &decision);

  CHECK_EQ((int)decision, (int)WriteDecision::ACCEPT);
  CHECK_EQ((int)result, (int)ExecuteResult::WRITTEN);
  CHECK_EQ(backend.enable_torque_calls, 1);
  CHECK_EQ(backend.write_goal_position_calls, 0);
  CHECK_EQ(backend.last_bus_id, 13);
}

void testPositionCommandAcceptWrittenThroughToBackendWithExactTick() {
  g_case = "position command written";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  CalibrationGeometryProfile profile = boundProfile();
  policy.begin(&arbiter);
  policy.bindGeometry(&profile, &geometry_data::kProvenance);
  CHECK(policy.limits().admit(acceptedLimit(lfLower(), 1800, 2300)));
  const AuthorityLease lease =
      grant(arbiter, ActuatorAuthority::MOTION, OperatingMode::RUN);

  ActuatorTransaction txn{};
  CHECK_EQ((int)policy.plan(command(ActuatorOperation::POSITION_COMMAND, lfLower(), 2048), lease,
                            OperatingMode::RUN, &txn),
          (int)WriteDecision::ACCEPT);

  ActuatorRuntime runtime;
  FakeActuatorBackend backend;
  runtime.begin(&policy, &backend);
  WriteDecision decision = WriteDecision::REJECT_NO_ARBITER;
  const ExecuteResult result = runtime.execute(&txn, /*bus_id=*/11, &decision);

  CHECK_EQ((int)result, (int)ExecuteResult::WRITTEN);
  CHECK_EQ(backend.write_goal_position_calls, 1);
  CHECK_EQ(backend.enable_torque_calls, 0);
  CHECK_EQ(backend.last_bus_id, 11);
  CHECK_EQ(backend.last_target_tick, 2048);  // exact pass-through, no rescaling
}

void testAcceptWithNoBackendIsFailClosed() {
  g_case = "no backend installed";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease =
      grant(arbiter, ActuatorAuthority::MOTION, OperatingMode::RUN);

  ActuatorTransaction txn{};
  CHECK_EQ((int)policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
                            OperatingMode::RUN, &txn),
          (int)WriteDecision::ACCEPT);

  ActuatorRuntime runtime;
  runtime.begin(&policy, /*backend=*/nullptr);
  WriteDecision decision = WriteDecision::REJECT_NO_ARBITER;
  const ExecuteResult result = runtime.execute(&txn, /*bus_id=*/13, &decision);

  CHECK_EQ((int)decision, (int)WriteDecision::ACCEPT);  // the policy still decided
  CHECK_EQ((int)result, (int)ExecuteResult::NO_BACKEND);  // but nothing could be sent
}

void testBackendFailureIsReportedNotSwallowed() {
  g_case = "backend rejects";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease =
      grant(arbiter, ActuatorAuthority::MOTION, OperatingMode::RUN);

  ActuatorTransaction txn{};
  CHECK_EQ((int)policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
                            OperatingMode::RUN, &txn),
          (int)WriteDecision::ACCEPT);

  ActuatorRuntime runtime;
  FakeActuatorBackend backend;
  backend.enable_torque_result = BackendWriteOutcome::VERIFIED_NOT_APPLIED;  // readback proved it off
  runtime.begin(&policy, &backend);
  WriteDecision decision = WriteDecision::REJECT_NO_ARBITER;
  const ExecuteResult result = runtime.execute(&txn, /*bus_id=*/13, &decision);

  CHECK_EQ((int)decision, (int)WriteDecision::ACCEPT);
  CHECK_EQ((int)result, (int)ExecuteResult::BACKEND_REJECTED);
  CHECK_EQ(backend.enable_torque_calls, 1);  // it WAS attempted, just failed
}

// The safety property Priority 2 exists for: a backend that cannot verify
// its write either way must surface as a DISTINCT result from a verified
// failure, never silently collapse into "rejected" (which a caller could
// read as "safely did nothing"). The actuator may be energized/moved and
// the caller owning the real transport must escalate to SAFE_OFF on this.
void testUncertainBackendOutcomeIsDistinctFromRejection() {
  g_case = "uncertain write is not swallowed as rejection";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  policy.begin(&arbiter);
  const AuthorityLease lease =
      grant(arbiter, ActuatorAuthority::MOTION, OperatingMode::RUN);

  ActuatorTransaction txn{};
  CHECK_EQ((int)policy.plan(command(ActuatorOperation::TORQUE_ENABLE, lfLower()), lease,
                            OperatingMode::RUN, &txn),
          (int)WriteDecision::ACCEPT);

  ActuatorRuntime runtime;
  FakeActuatorBackend backend;
  backend.enable_torque_result = BackendWriteOutcome::UNCERTAIN;  // no readback response
  runtime.begin(&policy, &backend);
  WriteDecision decision = WriteDecision::REJECT_NO_ARBITER;
  const ExecuteResult result = runtime.execute(&txn, /*bus_id=*/13, &decision);

  CHECK_EQ((int)decision, (int)WriteDecision::ACCEPT);  // the policy still decided
  CHECK_EQ((int)result, (int)ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF);
  CHECK(result != ExecuteResult::BACKEND_REJECTED);
  CHECK(result != ExecuteResult::WRITTEN);
  CHECK_EQ(backend.enable_torque_calls, 1);
}

void testUncertainGoalPositionOutcomeIsDistinctFromRejection() {
  g_case = "uncertain goal-position write is not swallowed as rejection";
  ActuatorAuthorityArbiter arbiter;
  arbiter.reset(AuthorityClearReason::BOOT);
  SafeActuatorPolicy policy;
  CalibrationGeometryProfile profile = boundProfile();
  policy.begin(&arbiter);
  policy.bindGeometry(&profile, &geometry_data::kProvenance);
  CHECK(policy.limits().admit(acceptedLimit(lfLower(), 1800, 2300)));
  const AuthorityLease lease =
      grant(arbiter, ActuatorAuthority::MOTION, OperatingMode::RUN);

  ActuatorTransaction txn{};
  CHECK_EQ((int)policy.plan(command(ActuatorOperation::POSITION_COMMAND, lfLower(), 2048), lease,
                            OperatingMode::RUN, &txn),
          (int)WriteDecision::ACCEPT);

  ActuatorRuntime runtime;
  FakeActuatorBackend backend;
  backend.write_goal_position_result = BackendWriteOutcome::UNCERTAIN;
  runtime.begin(&policy, &backend);
  WriteDecision decision = WriteDecision::REJECT_NO_ARBITER;
  const ExecuteResult result = runtime.execute(&txn, /*bus_id=*/11, &decision);

  CHECK_EQ((int)result, (int)ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF);
  CHECK_EQ(backend.write_goal_position_calls, 1);
}

// ---------------------------------------------------------------------------
// toString()
// ---------------------------------------------------------------------------

void testToStringCoversEveryValue() {
  g_case = "to_string";
  CHECK_STR(toString(ExecuteResult::NOT_EXECUTED), "NOT_EXECUTED");
  CHECK_STR(toString(ExecuteResult::NO_BACKEND), "NO_BACKEND");
  CHECK_STR(toString(ExecuteResult::NO_RAW_TARGET), "NO_RAW_TARGET");
  CHECK_STR(toString(ExecuteResult::WRITTEN), "WRITTEN");
  CHECK_STR(toString(ExecuteResult::BACKEND_REJECTED), "BACKEND_REJECTED");
  CHECK_STR(toString(ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF), "UNCERTAIN_REQUIRES_SAFE_OFF");
  CHECK_STR(toString(static_cast<ExecuteResult>(200)), "UNKNOWN");

  CHECK_STR(toString(BackendCallKind::NONE), "NONE");
  CHECK_STR(toString(BackendCallKind::ENABLE_TORQUE), "ENABLE_TORQUE");
  CHECK_STR(toString(BackendCallKind::WRITE_GOAL_POSITION), "WRITE_GOAL_POSITION");
  CHECK_STR(toString(static_cast<BackendCallKind>(200)), "UNKNOWN");

  CHECK_STR(toString(BackendWriteOutcome::VERIFIED_APPLIED), "VERIFIED_APPLIED");
  CHECK_STR(toString(BackendWriteOutcome::VERIFIED_NOT_APPLIED), "VERIFIED_NOT_APPLIED");
  CHECK_STR(toString(BackendWriteOutcome::UNCERTAIN), "UNCERTAIN");
  CHECK_STR(toString(static_cast<BackendWriteOutcome>(200)), "UNKNOWN");
}

}  // namespace

int main() {
  testBackendCallForCoversEveryOperation();
  testNoPolicyIsRefusedAndNeverTouchesBackend();
  testPolicyRejectionNeverReachesBackend();
  testTorqueEnableAcceptWrittenThroughToBackend();
  testPositionCommandAcceptWrittenThroughToBackendWithExactTick();
  testAcceptWithNoBackendIsFailClosed();
  testBackendFailureIsReportedNotSwallowed();
  testUncertainBackendOutcomeIsDistinctFromRejection();
  testUncertainGoalPositionOutcomeIsDistinctFromRejection();
  testToStringCoversEveryValue();

  std::printf("test_actuator_runtime: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
