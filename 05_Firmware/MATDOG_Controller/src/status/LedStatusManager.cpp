#include "LedStatusManager.h"

namespace matdog {
namespace status {

void LedStatusManager::update(uint32_t now_ms, const LedStatusInputs& inputs) {
  const LedPresentationState before = policy_.state();
  const LedFrame frame = policy_.update(inputs, now_ms, LedRing::kMaxBrightness);
  if (first_update_ || policy_.state() != before) {
    Transition& t = history_[transition_count_ % kTransitionHistory];
    t.from = before; t.to = policy_.state(); t.at_ms = now_ms;
    t.wifi_connecting = inputs.wifi_connecting; t.firmware_update = inputs.firmware_update_in_progress;
    ++transition_count_;
    first_update_ = false;
  }
  // Keep read-only status facts fresh while either manual diagnostic owns
  // the pixels. Normal presentation resumes as soon as it finishes.
  if (ring_ == nullptr || ring_->testRunning()) return;
  ring_->setFrame(frame);
}

}  // namespace status
}  // namespace matdog
