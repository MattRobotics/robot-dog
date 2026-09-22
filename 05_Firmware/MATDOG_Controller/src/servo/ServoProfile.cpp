#include "ServoProfile.h"

namespace matdog {
namespace servo {

RegisterCheck checkRegister(const ProfileRegister& expected, int32_t observed) {
  // A failed read is not a value. It is never compared and never matches.
  if (observed < 0) return RegisterCheck::NO_ANSWER;
  return static_cast<uint32_t>(observed) == expected.expected ? RegisterCheck::MATCH
                                                              : RegisterCheck::MISMATCH;
}

ProfileVerdict foldRegisterCheck(ProfileVerdict running, RegisterCheck check) {
  // MISMATCH is sticky: once a register is proven wrong, nothing later can
  // improve the verdict.
  if (running == ProfileVerdict::MISMATCH) return running;
  switch (check) {
    case RegisterCheck::MISMATCH:
      return ProfileVerdict::MISMATCH;
    case RegisterCheck::NO_ANSWER:
    case RegisterCheck::NOT_READ:
      // Fail closed. An unread register is never assumed to hold its expected
      // value, so a partially-read unit can never report MATCH.
      return ProfileVerdict::INCOMPLETE;
    case RegisterCheck::MATCH:
      return running == ProfileVerdict::NOT_RUN ? ProfileVerdict::MATCH : running;
  }
  return ProfileVerdict::INCOMPLETE;
}

int16_t decodePositionOffset(uint16_t raw) {
  // Two's complement, not sign-magnitude. The bench provisioner asserts the
  // difference explicitly because the two decoders disagree on the same bytes.
  return static_cast<int16_t>(raw);
}

const char* toString(RegisterCheck check) {
  switch (check) {
    case RegisterCheck::NOT_READ:  return "NOT_READ";
    case RegisterCheck::NO_ANSWER: return "NO_ANSWER";
    case RegisterCheck::MATCH:     return "MATCH";
    case RegisterCheck::MISMATCH:  return "MISMATCH";
  }
  return "UNKNOWN";
}

const char* toString(ProfileVerdict verdict) {
  switch (verdict) {
    case ProfileVerdict::NOT_RUN:    return "NOT_RUN";
    case ProfileVerdict::MATCH:      return "MATCH";
    case ProfileVerdict::MISMATCH:   return "MISMATCH";
    case ProfileVerdict::INCOMPLETE: return "INCOMPLETE";
  }
  return "UNKNOWN";
}

}  // namespace servo
}  // namespace matdog
