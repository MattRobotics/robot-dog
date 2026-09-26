#include "LedStatusManager.h"

namespace matdog {
namespace status {

void LedStatusManager::update(uint32_t now_ms, const LedStatusInputs& inputs) {
  if (ring_ == nullptr) return;
  // The manual @LED TEST diagnostic owns the ring until its lap finishes -
  // see the class comment in LedStatusManager.h.
  if (ring_->testRunning()) return;

  const LedEffect effect = policy_.update(inputs, now_ms, LedRing::kMaxBrightness);
  ring_->setSolid(effect.r, effect.g, effect.b, effect.brightness);
}

}  // namespace status
}  // namespace matdog
