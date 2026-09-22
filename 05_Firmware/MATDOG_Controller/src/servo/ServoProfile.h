#ifndef MATDOG_SERVO_SERVO_PROFILE_H
#define MATDOG_SERVO_SERVO_PROFILE_H

#include <stdint.h>

// MATDOG_C018_V1 — the persistent EEPROM profile contract, as the Controller
// sees it.
//
// Pure: <stdint.h> only. No Arduino, no bus, no Serial. The expected values
// come from the generated ServoProfileData.h, reduced by
// 06_Software/Matdog_Core/config/matdog_servo_profile_export.py from the
// reviewed YAML. Nothing here is retyped.
//
// THREE LAYERS, KEPT APART. Merging any two of them is how a provisioning
// contract quietly becomes a motion setting:
//
//   persistent profile   the 20 EEPROM registers. A unit matches all twenty
//                        or it is not provisioned. Read-only here.
//   servo invariants     model word, PositionOffset, baud, raw centre.
//                        Properties of the unit, verified but NOT part of the
//                        twenty-register delta set.
//   runtime RAM state    TorqueLimit, GoalSpeed, Acc. Written per motion,
//                        never persistent, and deliberately ABSENT from this
//                        header - the YAML's own runtime_policy says so, and
//                        the exporter fails if one appears in the profile.
//
// EVERYTHING THIS HEADER DESCRIBES IS A READ. There is no write path to any
// of these registers anywhere in the firmware, and scripts/static_audit.py
// fails the build if one appears.

namespace matdog {
namespace servo {

// One register of the persistent contract.
struct ProfileRegister {
  uint8_t address;
  uint8_t width;    // 1 or 2 bytes
  uint16_t expected;
  const char* name;
};

// Properties of the unit rather than of the provisioning contract.
struct ServoInvariants {
  uint8_t model_address;
  uint16_t model_expected;       // 777 for the ST-3215-C018
  uint8_t position_offset_address;
  int16_t position_offset_expected;  // 0; int16 LE two's complement on the wire
  uint8_t baud_address;
  uint8_t baud_expected;         // 0 = 1 Mbps; VERIFICATION ONLY, never written
  uint16_t raw_center;           // 2048
  uint8_t center_tolerance_ticks;
};

// What one register comparison produced.
enum class RegisterCheck : uint8_t {
  NOT_READ  = 0,  // the verifier never got this far
  NO_ANSWER = 1,  // the read timed out; proves nothing, so it is never a match
  MATCH     = 2,
  MISMATCH  = 3,
};

struct RegisterObservation {
  uint8_t address = 0;
  uint16_t expected = 0;
  int32_t observed = -1;  // negative means no valid read
  RegisterCheck check = RegisterCheck::NOT_READ;
};

// The verdict for one unit's persistent profile.
enum class ProfileVerdict : uint8_t {
  NOT_RUN   = 0,
  MATCH     = 1,  // all twenty registers read and matched
  MISMATCH  = 2,  // at least one register read and disagreed
  // At least one register did not answer. Fail-closed: an unread register is
  // never assumed to hold its expected value, so this is NOT a MATCH even if
  // every register that DID answer agreed.
  INCOMPLETE = 3,
};

// Folds one register result into a running verdict. MISMATCH is sticky and
// outranks INCOMPLETE: a proven disagreement is worse news than a missing read.
ProfileVerdict foldRegisterCheck(ProfileVerdict running, RegisterCheck check);

// Classifies one comparison. A negative `observed` is NO_ANSWER, never a
// value to compare.
RegisterCheck checkRegister(const ProfileRegister& expected, int32_t observed);

// Decodes the raw 16-bit PositionOffset word. The ST-3215-C018 stores it as
// int16 little-endian TWO'S COMPLEMENT - not sign-magnitude, which the
// provisioner's own tests distinguish explicitly. Getting this wrong turns a
// large negative offset into a large positive one.
int16_t decodePositionOffset(uint16_t raw);

const char* toString(RegisterCheck check);
const char* toString(ProfileVerdict verdict);

}  // namespace servo
}  // namespace matdog

#endif  // MATDOG_SERVO_SERVO_PROFILE_H
