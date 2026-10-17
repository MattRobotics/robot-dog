#include "ServoBus.h"

#include "../config/Pins.h"
#include "ServoProfile.h"
#include "ServoPacket.h"

namespace matdog {
namespace servo {

const char* toString(SafeOffResult result) {
  switch (result) {
    case SafeOffResult::VERIFIED_OFF:           return "VERIFIED_OFF";
    case SafeOffResult::UNVERIFIED_NO_RESPONSE: return "UNVERIFIED_NO_RESPONSE";
    case SafeOffResult::VERIFY_FAILED:          return "VERIFY_FAILED";
  }
  return "UNKNOWN";
}

bool ServoBus::begin() {
  servo_uart_.begin(
      build::kServoBusBaud,
      SERIAL_8N1,
      pins::kServoRx,
      pins::kServoTx);

  st_.pSerial = &servo_uart_;

  // IOTimeOut is deliberately NOT touched here — see kDiagnosticTimeoutMs/
  // kOperationalTimeoutMs and ScopedIOTimeout in ServoBus.h (Session 2.2
  // Finding C, refined Session 2.3 Finding 1). Every method below names
  // its own timeout explicitly via the guard; nothing relies on whatever
  // IOTimeOut happens to already be set to.

  // ServoBus::begin() itself performs no ping, scan, torque or write.
  // Under ROBOT_POWERED the Controller may subsequently ARM the reviewed
  // read-only startup census; its first Ping occurs later from update().
  // No automatic motion, torque enable or persistent write exists here.
  init_ = core::InitializationState::INITIALIZED;
  return true;
}

core::AvailabilityStatus ServoBus::availability() const {
  core::AvailabilityStatus a;
  a.init = init_;
  a.detected = last_detected_;
  // G2: one profile authority, one derivation (core/Availability.h).
  // Under ROBOT_POWERED this becomes REQUIRED and the SAME classify() turns
  // an unchanged NO_RESPONSE into FAULT — exactly what the V0.1 Availability
  // model was designed for. No logic here changed shape.
  a.expected = core::expectedStateForServoBus(build::kProfileExpectations);
  return a;
}

bool ServoBus::ping(int id) {
  if (id < 0 || id > 253) return false;
  ScopedIOTimeout guard(st_, kDiagnosticTimeoutMs);
  int result = st_.Ping(static_cast<uint8_t>(id));
  last_detected_ = (result >= 0) ? core::DetectedState::ONLINE : core::DetectedState::NO_RESPONSE;
  return result >= 0;
}

bool ServoBus::readModel(int id, int* model_out) {
  if (id < 0 || id > 253 || model_out == nullptr) return false;
  ScopedIOTimeout guard(st_, kDiagnosticTimeoutMs);
  int model = st_.readWord(static_cast<uint8_t>(id), SMS_STS_MODEL_L);
  if (model < 0) return false;
  *model_out = model;
  return true;
}

bool ServoBus::startScan(int lo, int hi) {
  if (scan_state_ == ScanState::RUNNING) return false;

  if (lo > hi) {
    int tmp = lo;
    lo = hi;
    hi = tmp;
  }
  lo = constrain(lo, 0, 253);
  hi = constrain(hi, 0, 253);
  if (hi - lo + 1 > kMaxScanRange) {
    hi = lo + kMaxScanRange - 1;
  }

  scan_result_ = ScanResult{};
  scan_result_.lo = lo;
  scan_result_.hi = hi;
  scan_next_id_ = lo;
  scan_state_ = ScanState::RUNNING;
  scan_started_ms_ = millis();
  return true;
}

void ServoBus::update(uint32_t now_ms) {
  (void)now_ms;
  if (scan_state_ != ScanState::RUNNING) return;

  // Exactly one Ping() per tick — see the ScanState comment in ServoBus.h
  // for why this is "incremental with bounded per-ID blocking", not
  // non-blocking. Measures its own duration (micros()) so the actual
  // per-ID and total cost is evidence, not a claim — see
  // ScanResult::max_ping_us / elapsed_ms. ScopedIOTimeout bounds this
  // single Ping() to the diagnostic timeout and restores the previous
  // value immediately after, every tick.
  const int id = scan_next_id_;
  const uint32_t ping_start_us = micros();
  bool responded;
  {
    ScopedIOTimeout guard(st_, kDiagnosticTimeoutMs);
    responded = st_.Ping(static_cast<uint8_t>(id)) >= 0;
  }
  const uint32_t ping_us = micros() - ping_start_us;
  if (ping_us > scan_result_.max_ping_us) {
    scan_result_.max_ping_us = ping_us;
  }

  if (responded) {
    if (scan_result_.found_count < kMaxScanIds) {
      scan_result_.found_ids[scan_result_.found_count] = id;
    }
    scan_result_.found_count++;
  }

  scan_next_id_++;
  if (scan_next_id_ > scan_result_.hi) {
    scan_state_ = ScanState::COMPLETE;
    scan_result_.elapsed_ms = millis() - scan_started_ms_;
    last_detected_ = (scan_result_.found_count > 0) ? core::DetectedState::ONLINE
                                                       : core::DetectedState::NO_RESPONSE;
  }
}

SafeOffResult ServoBus::safeOff(int id) {
  if (id < 0 || id > 253) return SafeOffResult::UNVERIFIED_NO_RESPONSE;

  // kOperationalTimeoutMs, NOT kDiagnosticTimeoutMs (Session 2.3 Finding 1)
  // — this is the safety de-escalation path, reachable from any
  // OperatingMode, and must not inherit a timeout tuned for
  // MAINTENANCE-only absence detection.
  ScopedIOTimeout guard(st_,validation_timeout_ms_?validation_timeout_ms_:kOperationalTimeoutMs);

  // The write's own ACK (SCS::Ack(), via EnableTorque -> writeByte) is
  // informational only — see the SafeOffResult comment in ServoBus.h for
  // why it must never be the safety claim by itself. Issue it, then always
  // verify with an independent, read-only TorqueEnable readback regardless
  // of what the write's ACK reported.
  st_.EnableTorque(static_cast<uint8_t>(id), 0);

  int torque_enable = st_.readByte(static_cast<uint8_t>(id), SMS_STS_TORQUE_ENABLE);

  if (torque_enable < 0) {
    // No readback response at all: we cannot prove anything, so we must
    // not claim success — regardless of whether the write itself ACKed.
    last_detected_ = core::DetectedState::NO_RESPONSE;
    return SafeOffResult::UNVERIFIED_NO_RESPONSE;
  }

  last_detected_ = core::DetectedState::ONLINE;  // readback proves the servo is present

  return (torque_enable == 0) ? SafeOffResult::VERIFIED_OFF : SafeOffResult::VERIFY_FAILED;
}

ServoWriteVerifyResult ServoBus::enableTorqueOn(int id) {
  if(prime_diagnostic_inhibit_)return ServoWriteVerifyResult::VERIFIED_NOT_APPLIED;
  if (id < 0 || id > 253) return ServoWriteVerifyResult::VERIFIED_NOT_APPLIED;

  ScopedIOTimeout guard(st_,validation_timeout_ms_?validation_timeout_ms_:kOperationalTimeoutMs);
  st_.EnableTorque(static_cast<uint8_t>(id), 1);

  // Independent readback is the sole authority, matching safeOff()'s rule:
  // the write's own ACK/status is informational only and is never the
  // verdict - see ServoWriteVerifyResult's file comment for why a lost ACK
  // on this half-duplex bus cannot be read as "not applied".
  const int torque_enable = st_.readByte(static_cast<uint8_t>(id), SMS_STS_TORQUE_ENABLE);
  last_detected_ = (torque_enable < 0) ? core::DetectedState::NO_RESPONSE
                                       : core::DetectedState::ONLINE;
  return classifyServoWriteVerify(torque_enable, 1);
}

ServoWriteVerifyResult ServoBus::writeGoalPosition(int id, uint16_t target_tick,
                                                   GoalMotionProfile profile) {
  if (id < 0 || id > 253) return ServoWriteVerifyResult::VERIFIED_NOT_APPLIED;
  if (target_tick >= 4096u) return ServoWriteVerifyResult::VERIFIED_NOT_APPLIED;
  if (profile != GoalMotionProfile::BOUNDED && profile != GoalMotionProfile::SEARCH_ENVELOPE) {
    return ServoWriteVerifyResult::VERIFIED_NOT_APPLIED;  // corrupted value: write nothing
  }
  const bool search = profile == GoalMotionProfile::SEARCH_ENVELOPE;

  ScopedIOTimeout guard(st_, validation_timeout_ms_?validation_timeout_ms_:kOperationalTimeoutMs);
  goal_capture_=diagnostics::GoalWriteCapture{};
  goal_capture_.attempted=true;goal_capture_.id=static_cast<uint8_t>(id);goal_capture_.position=target_tick;
  goal_capture_.speed=search?kSearchEnvelopeSpeed:validation_speed_?validation_speed_:kBoundedWriteSpeed;
  goal_capture_.acc=search?kSearchEnvelopeAcceleration:validation_acceleration_?validation_acceleration_:kBoundedWriteAcceleration;
  goal_capture_.started_ms=millis();
  goal_capture_.ack=st_.WritePosEx(
      static_cast<uint8_t>(id),
      static_cast<s16>(target_tick),
      search ? kSearchEnvelopeSpeed : validation_speed_?validation_speed_:kBoundedWriteSpeed,
      search ? kSearchEnvelopeAcceleration : validation_acceleration_?validation_acceleration_:kBoundedWriteAcceleration);
  goal_capture_.write_done_ms=millis();

  if(!search&&validation_speed_){
    // The explicit validation profile requires independent SRAM readback of
    // the complete holding command before MotionSession can enable torque.
    uint8_t control[10]{};
    if(st_.ReadCaptured(static_cast<uint8_t>(id),SMS_STS_TORQUE_ENABLE,control,sizeof(control),&goal_capture_.response)!=static_cast<int>(sizeof(control))){
      goal_capture_.finished_ms=millis();
      last_detected_=core::DetectedState::NO_RESPONSE;
      return ServoWriteVerifyResult::UNVERIFIED_NO_RESPONSE;
    }
    auto readGoalWord=[&](unsigned address){const unsigned i=address-SMS_STS_TORQUE_ENABLE;return static_cast<uint16_t>(control[i]|(control[i+1]<<8));};
    last_detected_=core::DetectedState::ONLINE;
    goal_capture_.position_read=readGoalWord(SMS_STS_GOAL_POSITION_L);goal_capture_.speed_read=readGoalWord(SMS_STS_GOAL_SPEED_L);
    goal_capture_.acc_read=control[SMS_STS_ACC-SMS_STS_TORQUE_ENABLE];
    goal_capture_.mismatch=(goal_capture_.position_read!=target_tick?1u:0u)|(goal_capture_.speed_read!=validation_speed_?2u:0u)|(goal_capture_.acc_read!=validation_acceleration_?4u:0u);
    goal_capture_.finished_ms=millis();
    return readGoalWord(SMS_STS_GOAL_POSITION_L)==target_tick&&readGoalWord(SMS_STS_GOAL_SPEED_L)==validation_speed_&&control[SMS_STS_ACC-SMS_STS_TORQUE_ENABLE]==validation_acceleration_?
        ServoWriteVerifyResult::VERIFIED_APPLIED:ServoWriteVerifyResult::VERIFIED_NOT_APPLIED;
  }

  // Verified by reading back the internal GoalPosition register itself, not
  // present_position: present_position only catches up to the target over
  // the joint's physical travel time, so it cannot prove anything about a
  // write that was just issued. GoalPosition is a register latch with no
  // travel delay, exactly like PositionOffset's own read-back verification.
  const int goal_readback = st_.readWord(static_cast<uint8_t>(id), SMS_STS_GOAL_POSITION_L);
  last_detected_ = (goal_readback < 0) ? core::DetectedState::NO_RESPONSE
                                       : core::DetectedState::ONLINE;
  return classifyServoWriteVerify(goal_readback, static_cast<int32_t>(target_tick));
}

ServoWriteVerifyResult ServoBus::writeReviewedRamTorqueLimit(int id) {
  if (id < 0 || id > 253) return ServoWriteVerifyResult::VERIFIED_NOT_APPLIED;

  ScopedIOTimeout guard(st_, kOperationalTimeoutMs);
  // RAM register 48 (SRAM, read/write); never the EEPROM Max Torque.
  st_.writeWord(static_cast<uint8_t>(id), SMS_STS_TORQUE_LIMIT_L, kReviewedRamTorqueLimit);

  // Independent readback is the sole verdict - see ServoWriteVerifyResult.
  const int torque_limit_readback =
      st_.readWord(static_cast<uint8_t>(id), SMS_STS_TORQUE_LIMIT_L);
  last_detected_ = (torque_limit_readback < 0) ? core::DetectedState::NO_RESPONSE
                                               : core::DetectedState::ONLINE;
  return classifyServoWriteVerify(torque_limit_readback,
                                  static_cast<int32_t>(kReviewedRamTorqueLimit));
}

ServoWriteVerifyResult ServoBus::reduceValidationOutputWhileOff(int id,uint16_t ceiling){
 if(id<0||id>253||!ceiling||ceiling>1000)return ServoWriteVerifyResult::VERIFIED_NOT_APPLIED;
 ScopedIOTimeout guard(st_,validation_timeout_ms_?validation_timeout_ms_:kOperationalTimeoutMs);
 // De-escalation only, SRAM address 48. No authority acquisition, no EEPROM.
 if(st_.readByte(id,SMS_STS_TORQUE_ENABLE)!=0)return ServoWriteVerifyResult::VERIFIED_NOT_APPLIED;
 const int current=st_.readWord(id,SMS_STS_TORQUE_LIMIT_L);
 if(current<0||current>1023||ceiling>current)return ServoWriteVerifyResult::VERIFIED_NOT_APPLIED;
 if(current!=ceiling)st_.writeWord(id,SMS_STS_TORQUE_LIMIT_L,ceiling);
 return classifyServoWriteVerify(st_.readWord(id,SMS_STS_TORQUE_LIMIT_L),ceiling);
}
bool ServoBus::readControlFeedback(int id, ControlFeedbackSnapshot* out) {
 return readControlFeedbackBudgeted(id,out,kOperationalTimeoutMs);
}
bool ServoBus::readControlFeedbackBudgeted(int id, ControlFeedbackSnapshot* out,uint32_t timeout_ms) {
  if (id < 0 || id > 253 || out == nullptr) return false;
  *out = ControlFeedbackSnapshot{};

  if(timeout_ms<2||timeout_ms>kOperationalTimeoutMs)return false;
  ScopedIOTimeout guard(st_,timeout_ms);

  // Block 1: SMS_STS_TORQUE_ENABLE (40) .. SMS_STS_TORQUE_LIMIT_H (49).
  uint8_t control[SMS_STS_TORQUE_LIMIT_H - SMS_STS_TORQUE_ENABLE + 1] = {0};
  if (st_.Read(static_cast<uint8_t>(id), SMS_STS_TORQUE_ENABLE, control, sizeof(control)) !=
      static_cast<int>(sizeof(control))) {
    last_detected_ = core::DetectedState::NO_RESPONSE;
    return false;
  }
  // Block 2: SMS_STS_PRESENT_POSITION_L (56) .. SMS_STS_PRESENT_CURRENT_H (70).
  uint8_t feedback[SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L + 1] = {0};
  if (st_.Read(static_cast<uint8_t>(id), SMS_STS_PRESENT_POSITION_L, feedback,
               sizeof(feedback)) != static_cast<int>(sizeof(feedback))) {
    last_detected_ = core::DetectedState::NO_RESPONSE;
    return false;
  }
  last_detected_ = core::DetectedState::ONLINE;

  // SMS_STS is little-endian (End = 0): low byte first.
  auto le16 = [](const uint8_t* block, int offset) {
    return static_cast<int>(block[offset]) | (static_cast<int>(block[offset + 1]) << 8);
  };
  const int c0 = SMS_STS_TORQUE_ENABLE;
  const int f0 = SMS_STS_PRESENT_POSITION_L;
  out->torque_enable = control[SMS_STS_TORQUE_ENABLE - c0];
  out->goal_position = le16(control, SMS_STS_GOAL_POSITION_L - c0);
  out->goal_speed=le16(control,SMS_STS_GOAL_SPEED_L-c0);out->goal_acceleration=control[SMS_STS_ACC-c0];
  out->torque_limit = le16(control, SMS_STS_TORQUE_LIMIT_L - c0);
  out->present_position = le16(feedback, SMS_STS_PRESENT_POSITION_L - f0);
  out->present_speed = le16(feedback, SMS_STS_PRESENT_SPEED_L - f0);
  out->present_load = le16(feedback, SMS_STS_PRESENT_LOAD_L - f0);
  out->present_voltage = feedback[SMS_STS_PRESENT_VOLTAGE - f0];
  out->present_temperature = feedback[SMS_STS_PRESENT_TEMPERATURE - f0];
  out->status = feedback[kServoStatusRegister - f0];
  out->present_current = le16(feedback, SMS_STS_PRESENT_CURRENT_L - f0);
  return true;
}

bool ServoBus::syncWriteGoalPositions(const uint8_t ids[12], const uint16_t ticks[12], FrameTx* report) {
  FrameTx local{};FrameTx& r=report?*report:local;r=FrameTx{};
  if(prime_diagnostic_inhibit_)return false;
  if(!validation_speed_||!validation_acceleration_)return false;   // no reviewed drive pair: nothing sent
  uint8_t packet[packet::kGoalFrameBytes];
  const size_t n=packet::buildGoalFrame(ids,ticks,12,validation_speed_,validation_acceleration_,packet,sizeof(packet));
  if(n!=packet::kGoalFrameBytes)return false;
  // Bounded drain of stale RX (a late reply must not be parsed later as ours).
  for(unsigned k=0;k<64&&servo_uart_.available();++k)servo_uart_.read();
  r.started=true;r.started_us=micros();
  const size_t written=servo_uart_.write(packet,n);
  servo_uart_.flush();                       // returns when the TX FIFO and shifter are empty
  r.complete_us=micros();r.bytes=static_cast<uint16_t>(written);r.complete=written==n;
  return r.complete;
}
uint16_t ServoBus::syncTorqueOff(const uint8_t* ids, unsigned count) {
  uint8_t packet[packet::kTorqueOffBytesMax];
  const size_t n=packet::buildTorqueOff(ids,count,packet,sizeof(packet));
  if(!n)return 0;
  const size_t written=servo_uart_.write(packet,n);
  servo_uart_.flush();
  return static_cast<uint16_t>(written);
}
SafeOffResult ServoBus::verifyTorqueOff(int id, uint32_t timeout_ms) {
  if(id<0||id>253||timeout_ms<2||timeout_ms>kOperationalTimeoutMs)return SafeOffResult::UNVERIFIED_NO_RESPONSE;
  ScopedIOTimeout guard(st_,timeout_ms);
  const int torque_enable=st_.readByte(static_cast<uint8_t>(id),SMS_STS_TORQUE_ENABLE);
  if(torque_enable<0){last_detected_=core::DetectedState::NO_RESPONSE;return SafeOffResult::UNVERIFIED_NO_RESPONSE;}
  last_detected_=core::DetectedState::ONLINE;
  return torque_enable==0?SafeOffResult::VERIFIED_OFF:SafeOffResult::VERIFY_FAILED;
}
bool ServoBus::readFastFeedback(int id, FastFeedback* out, uint32_t timeout_ms) {
  if(id<0||id>253||out==nullptr)return false;
  *out=FastFeedback{};
  if(timeout_ms<2||timeout_ms>kOperationalTimeoutMs)return false;
  ScopedIOTimeout guard(st_,timeout_ms);
  uint8_t feedback[SMS_STS_PRESENT_CURRENT_H - SMS_STS_PRESENT_POSITION_L + 1] = {0};
  if(st_.Read(static_cast<uint8_t>(id),SMS_STS_PRESENT_POSITION_L,feedback,sizeof(feedback))!=static_cast<int>(sizeof(feedback))){
    last_detected_=core::DetectedState::NO_RESPONSE;return false;}
  last_detected_=core::DetectedState::ONLINE;
  auto le16=[&](int a){const int o=a-SMS_STS_PRESENT_POSITION_L;return static_cast<int>(feedback[o])|(static_cast<int>(feedback[o+1])<<8);};
  out->present_position=le16(SMS_STS_PRESENT_POSITION_L);out->present_speed=le16(SMS_STS_PRESENT_SPEED_L);
  out->present_load=le16(SMS_STS_PRESENT_LOAD_L);out->present_voltage=feedback[SMS_STS_PRESENT_VOLTAGE-SMS_STS_PRESENT_POSITION_L];
  out->present_temperature=feedback[SMS_STS_PRESENT_TEMPERATURE-SMS_STS_PRESENT_POSITION_L];
  out->status=feedback[kServoStatusRegister-SMS_STS_PRESENT_POSITION_L];out->present_current=le16(SMS_STS_PRESENT_CURRENT_L);
  return true;
}

bool ServoBus::readRuntimeState(int id, RuntimeState* out) {
  if (id < 0 || id > 253 || out == nullptr) return false;

  // kOperationalTimeoutMs, NOT kDiagnosticTimeoutMs (Session 2.3 Finding 1)
  // — a future motion controller will naturally reuse this for operational
  // state reads; it must not silently inherit a timeout tuned for
  // MAINTENANCE-only absence detection just because @SERVO READ happens to
  // be MAINTENANCE-gated today.
  ScopedIOTimeout guard(st_, kOperationalTimeoutMs);

  int ping = st_.Ping(static_cast<uint8_t>(id));
  if (ping < 0) {
    last_detected_ = core::DetectedState::NO_RESPONSE;
    return false;
  }
  last_detected_ = core::DetectedState::ONLINE;

  out->present_position    = st_.readWord(static_cast<uint8_t>(id), SMS_STS_PRESENT_POSITION_L);
  out->present_speed       = st_.readWord(static_cast<uint8_t>(id), SMS_STS_PRESENT_SPEED_L);
  out->present_load        = st_.readWord(static_cast<uint8_t>(id), SMS_STS_PRESENT_LOAD_L);
  out->present_voltage     = st_.readByte(static_cast<uint8_t>(id), SMS_STS_PRESENT_VOLTAGE);
  out->present_temperature = st_.readByte(static_cast<uint8_t>(id), SMS_STS_PRESENT_TEMPERATURE);
  out->torque_enable       = st_.readByte(static_cast<uint8_t>(id), SMS_STS_TORQUE_ENABLE);
  out->present_current     = st_.readWord(static_cast<uint8_t>(id), SMS_STS_PRESENT_CURRENT_L);

  return true;
}

bool ServoBus::readPresentTemperatureDirect(int id, int* celsius_out) {
  if (id < 0 || id > 253 || celsius_out == nullptr) return false;
  // An operational safety read, like readRuntimeState(): operational timeout.
  ScopedIOTimeout guard(st_, kOperationalTimeoutMs);
  const int celsius = st_.readByte(static_cast<uint8_t>(id), SMS_STS_PRESENT_TEMPERATURE);
  if (celsius < 0) {
    last_detected_ = core::DetectedState::NO_RESPONSE;
    return false;
  }
  last_detected_ = core::DetectedState::ONLINE;
  *celsius_out = celsius;
  return true;
}

bool ServoBus::readPositionOffset(int id, int16_t* out) {
  if (id < 0 || id > 253 || out == nullptr) return false;

  // Diagnostic timeout: this is provisioning verification, not an operational
  // control read, and an absent unit must fail fast rather than stall the loop.
  ScopedIOTimeout guard(st_, kDiagnosticTimeoutMs);

  const int raw = st_.readWord(static_cast<uint8_t>(id), SMS_STS_OFS_L);
  if (raw < 0) {
    last_detected_ = core::DetectedState::NO_RESPONSE;
    return false;  // no answer is not an offset of zero
  }
  last_detected_ = core::DetectedState::ONLINE;
  *out = decodePositionOffset(static_cast<uint16_t>(raw));
  return true;
}

bool ServoBus::readProfileRegister(int id, uint8_t address, uint8_t width, int32_t* out) {
  if (id < 0 || id > 253 || out == nullptr) return false;
  if (width != 1 && width != 2) return false;  // refuse, never guess

  ScopedIOTimeout guard(st_, kDiagnosticTimeoutMs);

  const int raw = (width == 1) ? st_.readByte(static_cast<uint8_t>(id), address)
                               : st_.readWord(static_cast<uint8_t>(id), address);
  if (raw < 0) {
    last_detected_ = core::DetectedState::NO_RESPONSE;
    return false;
  }
  last_detected_ = core::DetectedState::ONLINE;
  *out = raw;
  return true;
}

}  // namespace servo
}  // namespace matdog
