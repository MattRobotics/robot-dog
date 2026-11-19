#ifndef MATDOG_STATUS_LED_STATUS_MANAGER_H
#define MATDOG_STATUS_LED_STATUS_MANAGER_H

#include "LedRing.h"
#include "LedStatusPolicy.h"

namespace matdog {
namespace status {

// The single periodic owner of LED presentation, above LedRing. Every tick
// it turns a fresh LedStatusInputs snapshot into exactly one
// LedRing::setFrame() call — never a duplicate hardware read, never a
// second decision core. scripts/static_audit.py's
// check_led_status_boundaries() fails the build if any other translation
// unit calls setFrame() or setSolid().
//
// The exceptions are @LED TEST and @LED SOC TEST diagnostics
// (CommandRouter -> LedRing::startTest()/startSocTest()/off()): while
// LedRing::testRunning() is true, update() keeps the policy snapshot current
// and steps aside from rendering, then resumes ownership on the next
// tick. @LED OFF, similarly, blanks the ring for one tick before this
// manager's next update() call restores the current status — a manual
// diagnostic blip, not a persistent override, because presentation has
// exactly one owner and must never silently keep two.
//
// While Motion is engaged (deferRender()), update() likewise keeps the
// policy snapshot current and submits no frame.
//
// This class never touches GPIO47, never
// calls pixels_.begin()/show(), and inherits the USB_ONLY anti-back-power
// guarantee because LedRing::setFrame() is a no-op when
// the rail is unpowered.
class LedStatusManager {
 public:
  void begin(LedRing* ring) { ring_ = ring; }

  void update(uint32_t now_ms, const LedStatusInputs& inputs);
  // Set by Controller before every update(): true while Motion is engaged.
  // The policy, its snapshot (battery warning/critical included) and the
  // transition history stay current, RAM only; the frame is not submitted,
  // because the WS2812 transmission can wait up to 20 ms for the RMT
  // (Ws2812DmaStrip::kTransmitTimeoutMs). The first update() after torque
  // OFF renders the then-current presentation.
  void deferRender(bool deferred) { render_deferred_ = deferred; }
  bool renderDeferred() const { return render_deferred_; }

  LedPresentationState state() const { return policy_.state(); }
  const LedStatusSnapshot& snapshot() const { return policy_.snapshot(); }

  // Diagnostics only (14 Oct cyan transient): every presentation change with
  // the inputs that selected it. A cyan flash WITHOUT a recorded
  // WIFI_CONNECTING transition points at the pixel transport, not policy.
  struct Transition {
    LedPresentationState from = LedPresentationState::BOOTING;
    LedPresentationState to = LedPresentationState::BOOTING;
    uint32_t at_ms = 0;
    bool wifi_connecting = false;
    bool firmware_update = false;
  };
  static constexpr uint8_t kTransitionHistory = 8;
  uint32_t transitionCount() const { return transition_count_; }
  // 0 = newest. Valid for i < min(count, kTransitionHistory).
  const Transition& transition(uint8_t i) const {
    return history_[(transition_count_ + kTransitionHistory - 1 - (i % kTransitionHistory)) % kTransitionHistory];
  }

 private:
  LedRing* ring_ = nullptr;
  LedStatusPolicy policy_;
  bool render_deferred_ = false;
  Transition history_[kTransitionHistory]{};
  uint32_t transition_count_ = 0;
  bool first_update_ = true;
};

}  // namespace status
}  // namespace matdog

#endif  // MATDOG_STATUS_LED_STATUS_MANAGER_H
