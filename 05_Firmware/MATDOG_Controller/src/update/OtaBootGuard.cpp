#include "OtaBootGuard.h"

namespace matdog {
namespace update {

OtaSelfCheckVerdict evaluateSelfCheck(const OtaSelfCheckInputs& inputs,
                                      const OtaSelfCheckConfig& config,
                                      uint32_t loop_ticks,
                                      OtaSelfCheckFault* out_fault) {
  OtaSelfCheckFault fault = OtaSelfCheckFault::NONE;
  OtaSelfCheckVerdict verdict = OtaSelfCheckVerdict::PASS;

  // Refusals are evaluated first and are terminal: they are facts that will
  // not become true by waiting longer.
  if (inputs.fatal_reset_reason) {
    fault = OtaSelfCheckFault::FATAL_RESET_REASON;
    verdict = OtaSelfCheckVerdict::REFUSE;
  } else if (inputs.controller_initialized && !inputs.identity_readable) {
    // Only meaningful once init finished; before that it is simply not known
    // yet, which is a PENDING condition handled below.
    fault = OtaSelfCheckFault::IDENTITY_UNREADABLE;
    verdict = OtaSelfCheckVerdict::REFUSE;
  } else if (!inputs.controller_initialized) {
    fault = OtaSelfCheckFault::WAITING_CONTROLLER_INIT;
    verdict = OtaSelfCheckVerdict::PENDING;
  } else if (!inputs.command_router_bound) {
    fault = OtaSelfCheckFault::WAITING_COMMAND_ROUTER;
    verdict = OtaSelfCheckVerdict::PENDING;
  } else if (inputs.uptime_ms < config.min_uptime_ms) {
    fault = OtaSelfCheckFault::WAITING_STABLE_UPTIME;
    verdict = OtaSelfCheckVerdict::PENDING;
  } else if (loop_ticks < config.min_loop_ticks) {
    fault = OtaSelfCheckFault::WAITING_LOOP_TICKS;
    verdict = OtaSelfCheckVerdict::PENDING;
  }

  if (out_fault != nullptr) *out_fault = fault;
  return verdict;
}

void OtaBootGuard::begin(OtaBackend* backend, const OtaSelfCheckConfig& config) {
  backend_ = backend;
  config_ = config;
  state_ = OtaBootState::UNEVALUATED;
  fault_ = OtaSelfCheckFault::NONE;
  running_state_ = OtaImgState::UNDEFINED;
  loop_ticks_ = 0;
  confirmed_at_ms_ = 0;
}

bool OtaBootGuard::settled() const {
  return state_ == OtaBootState::NOT_OTA_MANAGED ||
         state_ == OtaBootState::CONFIRMED ||
         state_ == OtaBootState::SELF_CHECK_PASSED ||
         state_ == OtaBootState::SELF_CHECK_REFUSED ||
         state_ == OtaBootState::MARK_VALID_FAILED ||
         state_ == OtaBootState::PREVIOUSLY_INVALIDATED;
}

void OtaBootGuard::update(const OtaSelfCheckInputs& inputs) {
  if (backend_ == nullptr) return;
  if (settled()) return;  // bounded: nothing runs once the lifecycle is over

  loop_ticks_++;

  if (state_ == OtaBootState::UNEVALUATED) {
    // Classify what the bootloader left us with, exactly once.
    running_state_ = backend_->imageState(backend_->runningPartition());
    switch (running_state_) {
      case OtaImgState::PENDING_VERIFY:
      case OtaImgState::NEW:
        // NEW is what esp_ota_set_boot_partition() writes; the bootloader
        // turns it into PENDING_VERIFY on the first boot. Seeing either from
        // the application means "this image has not confirmed itself yet".
        state_ = OtaBootState::PENDING_SELF_CHECK;
        break;
      case OtaImgState::VALID:
        state_ = OtaBootState::CONFIRMED;
        return;
      case OtaImgState::INVALID:
      case OtaImgState::ABORTED:
        state_ = OtaBootState::PREVIOUSLY_INVALIDATED;
        return;
      case OtaImgState::UNDEFINED:
      case OtaImgState::UNREADABLE:
        // No otadata record for this slot: a factory/first image, or a
        // partition table without otadata. Nothing to confirm, and nothing
        // that would be rolled back.
        state_ = OtaBootState::NOT_OTA_MANAGED;
        return;
    }
  }

  const OtaSelfCheckVerdict verdict =
      evaluateSelfCheck(inputs, config_, loop_ticks_, &fault_);

  switch (verdict) {
    case OtaSelfCheckVerdict::PENDING:
      // Stay PENDING_SELF_CHECK. The bootloader's rollback remains armed,
      // which is the correct state to be in while we are still unsure.
      return;

    case OtaSelfCheckVerdict::REFUSE:
      // Deliberately does NOT call the invalidate-and-reboot API. Refusing to
      // confirm is already sufficient: the bootloader marks this image
      // ABORTED on the next boot and falls back on its own. Triggering a
      // reboot from here would be an unannounced hardware action, and
      // rebooting a robot is not OTA-A's decision to make.
      // TO_IMPLEMENT / OTA-B: an explicit, authorized operator rollback.
      state_ = OtaBootState::SELF_CHECK_REFUSED;
      return;

    case OtaSelfCheckVerdict::PASS:
      if (backend_->markAppValid()) {
        state_ = OtaBootState::SELF_CHECK_PASSED;
        fault_ = OtaSelfCheckFault::NONE;
        confirmed_at_ms_ = inputs.uptime_ms;
      } else {
        state_ = OtaBootState::MARK_VALID_FAILED;
      }
      return;
  }
}

const char* toString(OtaBootState state) {
  switch (state) {
    case OtaBootState::UNEVALUATED:            return "UNEVALUATED";
    case OtaBootState::NOT_OTA_MANAGED:        return "NOT_OTA_MANAGED";
    case OtaBootState::CONFIRMED:              return "CONFIRMED";
    case OtaBootState::PENDING_SELF_CHECK:     return "PENDING_SELF_CHECK";
    case OtaBootState::SELF_CHECK_PASSED:      return "SELF_CHECK_PASSED";
    case OtaBootState::SELF_CHECK_REFUSED:     return "SELF_CHECK_REFUSED";
    case OtaBootState::MARK_VALID_FAILED:      return "MARK_VALID_FAILED";
    case OtaBootState::PREVIOUSLY_INVALIDATED: return "PREVIOUSLY_INVALIDATED";
  }
  return "UNKNOWN";
}

const char* toString(OtaSelfCheckFault fault) {
  switch (fault) {
    case OtaSelfCheckFault::NONE:                    return "NONE";
    case OtaSelfCheckFault::WAITING_CONTROLLER_INIT: return "WAITING_CONTROLLER_INIT";
    case OtaSelfCheckFault::WAITING_COMMAND_ROUTER:  return "WAITING_COMMAND_ROUTER";
    case OtaSelfCheckFault::WAITING_STABLE_UPTIME:   return "WAITING_STABLE_UPTIME";
    case OtaSelfCheckFault::WAITING_LOOP_TICKS:      return "WAITING_LOOP_TICKS";
    case OtaSelfCheckFault::IDENTITY_UNREADABLE:     return "IDENTITY_UNREADABLE";
    case OtaSelfCheckFault::FATAL_RESET_REASON:      return "FATAL_RESET_REASON";
  }
  return "UNKNOWN";
}

}  // namespace update
}  // namespace matdog
