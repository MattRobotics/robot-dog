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
// This class never touches GPIO47, never
// calls pixels_.begin()/show(), and inherits the USB_ONLY anti-back-power
// guarantee because LedRing::setFrame() is a no-op when
// the rail is unpowered.
class LedStatusManager {
 public:
  void begin(LedRing* ring) { ring_ = ring; }

  void update(uint32_t now_ms, const LedStatusInputs& inputs);

  LedPresentationState state() const { return policy_.state(); }
  const LedStatusSnapshot& snapshot() const { return policy_.snapshot(); }

 private:
  LedRing* ring_ = nullptr;
  LedStatusPolicy policy_;
};

}  // namespace status
}  // namespace matdog

#endif  // MATDOG_STATUS_LED_STATUS_MANAGER_H
