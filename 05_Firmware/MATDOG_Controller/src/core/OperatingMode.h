#ifndef MATDOG_CORE_OPERATING_MODE_H
#define MATDOG_CORE_OPERATING_MODE_H

#include <Arduino.h>

namespace matdog {
namespace core {

// Session 2.1 hardening: explicit architectural boundary between
// potentially-blocking servo diagnostics and a future deterministic motion
// loop. See ServoBus.h for why "non-blocking" was the wrong word for the
// Session 2 scan — every SCServo::Ping() call still carries the library's
// own bounded per-call timeout, so a scan is "incremental with bounded
// per-ID blocking", not truly non-blocking. That is fine for a diagnostic
// tool used between motion cycles; it would starve a real-time motion loop.
//
// V0.1 has no motion execution at all, so this boundary currently gates
// nothing but diagnostics against nothing but itself. The point is that it
// already exists, so a future motion-loop integration cannot silently
// inherit @SERVO SCAN/@SERVO READ as callable from inside RUN.
//
// NOT the same concept as core::PowerState::RUN (PowerState.h) — the two
// are orthogonal and the shared name "RUN" is coincidental:
//   - PowerState is about the power-on lifecycle: BOOTING -> POWER_CHECK ->
//     RUN -> SHUTDOWN_REQUESTED -> ... . Its RUN means "the controller has
//     finished booting and is operating normally"; it says nothing about
//     which servo diagnostics are currently safe to run.
//   - OperatingMode is about whether potentially-blocking servo diagnostics
//     are safe to run right now. Its RUN means "a deterministic motion
//     loop may be active; do not block it with a diagnostic".
// A running controller is normally PowerState::RUN *and*
// OperatingMode::MAINTENANCE at the same time in V0.1 (see boot banner /
// @STATUS, which prints both) — that is expected, not a naming collision
// to "resolve".
enum class OperatingMode : uint8_t {
  MAINTENANCE = 0,  // servo scan/read diagnostics allowed; no motion allowed (none exists yet)
  RUN         = 1,  // future deterministic motion loop; blocking diagnostics refused
};

const char* toString(OperatingMode mode);

// Deliberately tiny: two values, no transition guards, no history. V0.1's
// default is MAINTENANCE — there is no motion loop yet to protect, and
// Session 2's already-hardware-validated diagnostic workflow (@SERVO SCAN
// usable right after boot) should not regress for no protective benefit.
//
// IMPORTANT for whoever adds the motion controller: that work must flip the
// default to RUN and require an explicit, reviewed transition into
// MAINTENANCE (with torque confirmed off) before servo scan/read diagnostics
// are reachable again. Do not carry today's MAINTENANCE default forward
// unexamined.
class OperatingModeManager {
 public:
  OperatingMode mode() const { return mode_; }
  void setMode(OperatingMode mode) { mode_ = mode; }

 private:
  OperatingMode mode_ = OperatingMode::MAINTENANCE;
};

}  // namespace core
}  // namespace matdog

#endif  // MATDOG_CORE_OPERATING_MODE_H
