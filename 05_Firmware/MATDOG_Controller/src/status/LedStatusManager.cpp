#include "LedStatusManager.h"

namespace matdog {
namespace status {

void LedStatusManager::update(uint32_t now_ms, const LedStatusInputs& inputs) {
  const LedFrame frame = policy_.update(inputs, now_ms, LedRing::kMaxBrightness);
  // Keep read-only status facts fresh while either manual diagnostic owns
  // the pixels. Normal presentation resumes as soon as it finishes.
  if (ring_ == nullptr || ring_->testRunning()) return;
  ring_->setFrame(frame);
}

}  // namespace status
}  // namespace matdog
