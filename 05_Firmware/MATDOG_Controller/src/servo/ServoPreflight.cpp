#include "ServoPreflight.h"

#include "ServoProfileData.h"

namespace matdog {
namespace servo {

// isLegServo()/legServoAt() live in ServoPopulation: the selection is pure
// config policy and is proven offline there, not re-derived here.

bool ServoPreflight::start() {
  if (bus_ == nullptr) return false;
  if (state_ == State::RUNNING) return false;

  result_ = PreflightResult{};
  next_index_ = 0;
  state_ = State::RUNNING;
  return true;
}

void ServoPreflight::evaluateJoint(const CanonicalServo& expected,
                                   JointPreflightRecord* record) {
  record->expected_bus_id = expected.bus_id;
  record->joint = expected.joint;
  // Configuration, carried through untouched. The servo is never asked for it
  // and can never confirm it.
  record->expected_physical_unit = expected.physical_unit;

  const int id = static_cast<int>(expected.bus_id);

  // Ping first. An absent unit then costs one bounded timeout instead of
  // twenty-three, which is what keeps a single update() tick short.
  if (!bus_->ping(id)) {
    record->observed_bus_id = 0;
    record->result = JointPreflightResult::NO_RESPONSE;
    result_.no_response_count++;
    return;
  }
  // The only identity a servo can supply: it answered at this address.
  record->observed_bus_id = expected.bus_id;

  bool mismatch = false;
  bool incomplete = false;

  int model = 0;
  if (bus_->readModel(id, &model)) {
    record->model = model;
    if (static_cast<uint16_t>(model) != profile_data::kInvariants.model_expected) {
      mismatch = true;
    }
  } else {
    incomplete = true;
  }

  int16_t offset = 0;
  if (bus_->readPositionOffset(id, &offset)) {
    record->position_offset_read = true;
    record->position_offset = offset;
    if (offset != profile_data::kInvariants.position_offset_expected) mismatch = true;
  } else {
    // A failed read is never reported as an offset of zero.
    incomplete = true;
  }

  ProfileVerdict verdict = ProfileVerdict::NOT_RUN;
  for (uint8_t r = 0; r < profile_data::kPersistentRegisterCount; ++r) {
    const ProfileRegister& reg = profile_data::kPersistentRegisters[r];
    int32_t observed = -1;
    const bool ok = bus_->readProfileRegister(id, reg.address, reg.width, &observed);
    const RegisterCheck check = ok ? checkRegister(reg, observed) : RegisterCheck::NO_ANSWER;
    if (ok) record->profile_registers_read++;
    if (check == RegisterCheck::MISMATCH) {
      if (record->profile_mismatch_count == 0) {
        record->first_mismatch_address = reg.address;
        record->first_mismatch_expected = reg.expected;
        record->first_mismatch_observed = observed;
      }
      record->profile_mismatch_count++;
    }
    verdict = foldRegisterCheck(verdict, check);
  }
  record->profile = verdict;
  if (verdict == ProfileVerdict::MISMATCH) mismatch = true;
  if (verdict != ProfileVerdict::MATCH && verdict != ProfileVerdict::MISMATCH) {
    incomplete = true;
  }

  ServoBus::RuntimeState runtime;
  if (bus_->readRuntimeState(id, &runtime)) {
    record->torque_enable = runtime.torque_enable;
    // Raw tick, liveness evidence only. NOT q0.
    record->present_position = runtime.present_position;
  } else {
    incomplete = true;
  }

  // A proven disagreement outranks a missing read: it is worse news.
  if (mismatch) {
    record->result = JointPreflightResult::MISMATCH;
    result_.mismatch_count++;
  } else if (incomplete) {
    record->result = JointPreflightResult::INCOMPLETE;
    result_.incomplete_count++;
  } else {
    record->result = JointPreflightResult::PASS;
    result_.pass_count++;
  }
}

void ServoPreflight::update() {
  if (state_ != State::RUNNING) return;
  if (bus_ == nullptr) return;

  if (next_index_ >= kLegPreflightCount) {
    result_.complete = true;
    state_ = State::COMPLETE;
    return;
  }

  const CanonicalServo* expected = legServoAt(next_index_);
  if (expected == nullptr) {
    // The canonical table does not hold twelve leg servos. Fail closed rather
    // than report a short pass.
    result_.complete = true;
    state_ = State::COMPLETE;
    return;
  }

  evaluateJoint(*expected, &result_.joints[next_index_]);
  result_.joints_evaluated++;
  ++next_index_;

  if (next_index_ >= kLegPreflightCount) {
    result_.complete = true;
    state_ = State::COMPLETE;
  }
}

const char* toString(JointPreflightResult result) {
  switch (result) {
    case JointPreflightResult::NOT_RUN:     return "NOT_RUN";
    case JointPreflightResult::PASS:        return "PASS";
    case JointPreflightResult::NO_RESPONSE: return "NO_RESPONSE";
    case JointPreflightResult::MISMATCH:    return "MISMATCH";
    case JointPreflightResult::INCOMPLETE:  return "INCOMPLETE";
  }
  return "UNKNOWN";
}

}  // namespace servo
}  // namespace matdog
