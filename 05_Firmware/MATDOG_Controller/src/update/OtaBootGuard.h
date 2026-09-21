#ifndef MATDOG_UPDATE_OTA_BOOT_GUARD_H
#define MATDOG_UPDATE_OTA_BOOT_GUARD_H

#include <stdint.h>

#include "OtaPolicy.h"

// First-boot validation for an image that arrived over OTA. Pure: no
// <Arduino.h>, no ESP-IDF. The facts it judges are passed in, so
// scripts/tests/test_ota_policy.cpp exercises the shipped decision.
//
// WHY THIS IS A SEPARATE UNIT FROM OtaPolicy
// ------------------------------------------
// OtaPolicy is about an update being received. This is about the image that
// is ALREADY RUNNING. They are different lifecycles with different failure
// modes, and conflating them is how "mark the app valid" ends up being
// called by the code path that just finished a download.
//
// WHY IT MATTERS ON THIS BUILD SPECIFICALLY
// -----------------------------------------
// The real build has CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y (read from the
// generated sdkconfig, not assumed). With that set, the bootloader moves an
// ESP_OTA_IMG_PENDING_VERIFY entry to ESP_OTA_IMG_ABORTED on the NEXT boot
// and falls back to the other slot. So:
//
//   - never confirming is the SAFE default: a firmware that cannot run long
//     enough to confirm itself gets rolled back by the bootloader with no
//     code of ours involved;
//   - confirming too early throws that safety net away. Calling
//     esp_ota_mark_app_valid_cancel_rollback() at the top of setup() would
//     confirm an image that is about to panic on its first real loop.
//
// So the rule here is: confirmation is EARNED by running, and the criteria
// are finite, deterministic and software-only.

namespace matdog {
namespace update {

enum class OtaBootState : uint8_t {
  UNEVALUATED           = 0,  // update() has not run yet
  NOT_OTA_MANAGED       = 1,  // running slot has no otadata record (factory/undefined)
  CONFIRMED             = 2,  // already ESP_OTA_IMG_VALID; nothing to do
  PENDING_SELF_CHECK    = 3,  // first boot of a new image; criteria not all met yet
  SELF_CHECK_PASSED     = 4,  // criteria met and the app was marked valid
  SELF_CHECK_REFUSED    = 5,  // a criterion definitively failed; we did NOT confirm
  MARK_VALID_FAILED     = 6,  // criteria met but the backend call failed
  PREVIOUSLY_INVALIDATED = 7, // running image is INVALID/ABORTED - report, do not hide
};

// Why the guard is waiting, or why it refused. PENDING states name what is
// still missing; REFUSED states name what went wrong.
enum class OtaSelfCheckFault : uint8_t {
  NONE                     = 0,
  WAITING_CONTROLLER_INIT  = 1,
  WAITING_COMMAND_ROUTER   = 2,
  WAITING_STABLE_UPTIME    = 3,
  WAITING_LOOP_TICKS       = 4,
  IDENTITY_UNREADABLE      = 5,  // refusal: cannot say which firmware this is
  FATAL_RESET_REASON       = 6,  // refusal: this boot came from a panic/watchdog
};

// Every input is software-only and available with the robot unpowered. There
// is deliberately NO servo, DALY, IMU or motion condition here: peripheral
// presence is not evidence about the firmware, and making it one would roll
// back a perfectly good image because a cable was unplugged.
struct OtaSelfCheckInputs {
  bool controller_initialized = false;  // Controller::begin() ran to completion
  bool command_router_bound = false;    // the command surface is usable
  bool identity_readable = false;       // running partition + build id both readable
  bool fatal_reset_reason = false;      // this boot followed PANIC/WDT/BROWNOUT
  uint32_t uptime_ms = 0;
};

struct OtaSelfCheckConfig {
  // The image has to survive this long before it is allowed to confirm
  // itself. A boot-loop crash happens well inside it, so a firmware that
  // cannot run never reaches confirmation and the bootloader rolls it back.
  uint32_t min_uptime_ms = 0;
  // And it has to actually keep looping, not merely exist. Uptime alone
  // would be satisfied by a controller wedged in a single long call.
  uint32_t min_loop_ticks = 0;
};

class OtaBootGuard {
 public:
  void begin(OtaBackend* backend, const OtaSelfCheckConfig& config);

  // Bounded, called once per Controller tick. Does nothing at all once the
  // lifecycle has settled.
  void update(const OtaSelfCheckInputs& inputs);

  OtaBootState state() const { return state_; }
  OtaSelfCheckFault fault() const { return fault_; }
  OtaImgState runningImageState() const { return running_state_; }
  uint32_t loopTicks() const { return loop_ticks_; }
  uint32_t confirmedAtMs() const { return confirmed_at_ms_; }
  bool settled() const;

  // True while the running image could still be rolled back by the
  // bootloader if it never confirms. The operator-facing answer to
  // "is this firmware committed yet?".
  bool rollbackStillArmed() const { return state_ == OtaBootState::PENDING_SELF_CHECK; }

 private:
  OtaBackend* backend_ = nullptr;
  OtaSelfCheckConfig config_{};
  OtaBootState state_ = OtaBootState::UNEVALUATED;
  OtaSelfCheckFault fault_ = OtaSelfCheckFault::NONE;
  OtaImgState running_state_ = OtaImgState::UNDEFINED;
  uint32_t loop_ticks_ = 0;
  uint32_t confirmed_at_ms_ = 0;
};

// The pure verdict, exposed separately so the criteria can be tested without
// a backend at all.
enum class OtaSelfCheckVerdict : uint8_t { PENDING = 0, PASS = 1, REFUSE = 2 };

OtaSelfCheckVerdict evaluateSelfCheck(const OtaSelfCheckInputs& inputs,
                                      const OtaSelfCheckConfig& config,
                                      uint32_t loop_ticks,
                                      OtaSelfCheckFault* out_fault);

const char* toString(OtaBootState state);
const char* toString(OtaSelfCheckFault fault);

}  // namespace update
}  // namespace matdog

#endif  // MATDOG_UPDATE_OTA_BOOT_GUARD_H
