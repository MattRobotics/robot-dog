#ifndef MATDOG_SERVO_SERVO_PREFLIGHT_H
#define MATDOG_SERVO_SERVO_PREFLIGHT_H

#include <stdint.h>

#include "ServoBus.h"
#include "ServoPopulation.h"
#include "ServoProfile.h"

namespace matdog {
namespace servo {

// Controller-owned semantic service: "verify the twelve leg servos against the
// canonical allocation and the MATDOG_C018_V1 persistent profile, and hold the
// structured result". H0.
//
// PERMANENT CAPABILITY, NOT A ONE-SHOT GATE. H0 is its first consumer; the
// same service answers post-servo-replacement verification, post-repair
// verification and routine MAINTENANCE checks. It is re-runnable, it holds no
// gate state, and it decides nothing about motion.
//
//   ServoBus       -> what did the physical bus answer?     (transport)
//   ServoPreflight -> run one pass, hold the meaning        (service, THIS)
//   ServoProfile   -> what does MATDOG_C018_V1 require?     (pure contract)
//   ServoPopulation-> what does the allocation expect?      (pure config)
//   CommandRouter  -> formats result() for USB CDC          (adapter)
//
// Same split as ServoCensus, and deliberately a SEPARATE service: a census
// answers "who is on the bus", a preflight answers "is each one the unit we
// expect, configured the way we provisioned it". Neither re-implements the
// other, and this class contains no Serial and no printing.
//
// SAFETY
// ------
// Strictly read-only: Ping plus register READS. No torque, no GoalPosition,
// no register write, no EEPROM write, no PositionOffset write. It is never
// started automatically - not at boot, not on a timer.
//
// BOUNDED BLOCKING. One joint per update() tick, and each joint is pinged
// first: an absent unit costs one timeout rather than twenty-three. Like the
// census it is therefore MAINTENANCE-only at the command surface.

// Why one joint failed, or that it did not.
enum class JointPreflightResult : uint8_t {
  NOT_RUN    = 0,
  PASS       = 1,  // answered, model matches, offset read and zero, profile MATCH
  NO_RESPONSE = 2,  // did not answer the ping
  // The unit answered but is not the unit we expect, or is not provisioned the
  // way we provisioned it. Deliberately one verdict with detail fields rather
  // than a combinatorial enum.
  MISMATCH   = 3,
  // Something could not be read. Fail-closed: never reported as PASS.
  INCOMPLETE = 4,
};

struct JointPreflightRecord {
  // --- expected, from configuration. NEVER an observation. ----------------
  uint8_t expected_bus_id = 0;
  const char* joint = "";
  const char* expected_physical_unit = "";

  // --- observed, from the bus ---------------------------------------------
  // The bus id is the ONLY identity a servo can actually supply: it answered
  // at this address, or it did not. 0 means no answer.
  uint8_t observed_bus_id = 0;
  int32_t model = -1;              // -1 = not read
  bool position_offset_read = false;
  int16_t position_offset = 0;     // meaningless unless position_offset_read
  int32_t torque_enable = -1;      // -1 = not read
  int32_t present_position = -1;   // -1 = not read; NOT q0, see below

  // --- persistent profile --------------------------------------------------
  ProfileVerdict profile = ProfileVerdict::NOT_RUN;
  uint8_t profile_registers_read = 0;
  uint8_t profile_mismatch_count = 0;
  uint8_t first_mismatch_address = 0;
  uint16_t first_mismatch_expected = 0;
  int32_t first_mismatch_observed = -1;

  JointPreflightResult result = JointPreflightResult::NOT_RUN;
};

// `present_position` is a raw encoder tick captured for liveness evidence.
// IT IS NOT q0. q0 is measured in a separate, later step with the robot
// manually aligned to the nominal URDF q=0 pose and torque confirmed off, and
// it carries its own provenance. Nothing here may be promoted into calibration.

// Derived from the canonical allocation, never a second literal.
constexpr uint8_t kLegPreflightCount = kLegServoCount;

struct PreflightResult {
  bool complete = false;
  uint8_t joints_evaluated = 0;
  uint8_t pass_count = 0;
  uint8_t no_response_count = 0;
  uint8_t mismatch_count = 0;
  uint8_t incomplete_count = 0;
  JointPreflightRecord joints[kLegPreflightCount]{};

  // 12/12 and nothing else. Partial success is not a gate result.
  bool allPass() const {
    return complete && joints_evaluated == kLegPreflightCount &&
           pass_count == kLegPreflightCount;
  }
};

class ServoPreflight {
 public:
  enum class State : uint8_t { IDLE = 0, RUNNING = 1, COMPLETE = 2 };

  void begin(ServoBus* bus) { bus_ = bus; }

  // Starts one pass over the twelve leg joints. Returns false if a pass is
  // already running or no bus is bound. Re-runnable: a previous result is
  // discarded, never merged.
  bool start();

  // Call once per Controller tick. Evaluates at most ONE joint per call.
  void update();

  State state() const { return state_; }
  const PreflightResult& result() const { return result_; }

 private:
  void evaluateJoint(const CanonicalServo& expected, JointPreflightRecord* record);

  ServoBus* bus_ = nullptr;
  State state_ = State::IDLE;
  uint8_t next_index_ = 0;
  PreflightResult result_{};
};

const char* toString(JointPreflightResult result);

}  // namespace servo
}  // namespace matdog

#endif  // MATDOG_SERVO_SERVO_PREFLIGHT_H
