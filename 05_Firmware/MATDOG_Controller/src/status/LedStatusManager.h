#ifndef MATDOG_STATUS_LED_STATUS_MANAGER_H
#define MATDOG_STATUS_LED_STATUS_MANAGER_H

#include "LedRing.h"
#include "LedStatusPolicy.h"

namespace matdog {
namespace status {

// The single periodic owner of LED presentation, above LedRing. Every tick
// it turns a fresh LedStatusInputs snapshot into exactly one
// LedRing::setSolid() call — never a duplicate hardware read, never a
// second decision core. scripts/static_audit.py's
// check_led_status_boundaries() fails the build if any other translation
// unit calls setSolid().
//
// The one exception is the existing @LED TEST diagnostic
// (CommandRouter -> LedRing::startTest()/off()), which is intentionally
// untouched by this manager: while LedRing::testRunning() is true, update()
// steps aside so the chase can finish, then resumes ownership on the next
// tick. @LED OFF, similarly, blanks the ring for one tick before this
// manager's next update() call restores the current status — a manual
// diagnostic blip, not a persistent override, because presentation has
// exactly one owner and must never silently keep two.
//
// Reuses LedRing exactly as it is: this class never touches GPIO47, never
// calls pixels_.begin()/show(), and inherits the USB_ONLY anti-back-power
// guarantee for free because LedRing::setSolid() is already a no-op when
// the rail is unpowered.
class LedStatusManager {
 public:
  void begin(LedRing* ring) { ring_ = ring; }

  void update(uint32_t now_ms, const LedStatusInputs& inputs);

  LedPresentationState state() const { return policy_.state(); }

 private:
  LedRing* ring_ = nullptr;
  LedStatusPolicy policy_;
};

}  // namespace status
}  // namespace matdog

#endif  // MATDOG_STATUS_LED_STATUS_MANAGER_H
