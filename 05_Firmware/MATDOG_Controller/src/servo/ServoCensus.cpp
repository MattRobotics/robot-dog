#include "ServoCensus.h"

namespace matdog {
namespace servo {

bool ServoCensus::start() {
  if (bus_ == nullptr) return false;
  if (state_ == State::RUNNING) return false;

  // ServoBus owns the scan state machine and refuses if a scan (ours or a
  // raw @SERVO SCAN) is already RUNNING. We do not duplicate that check, we
  // defer to it — one scan owner, one refusal path.
  if (!bus_->startScan(kCanonicalScanLo, kCanonicalScanHi)) return false;

  state_ = State::RUNNING;
  return true;
}

void ServoCensus::update() {
  if (state_ != State::RUNNING) return;
  if (bus_ == nullptr) return;
  if (bus_->scanState() != ScanState::COMPLETE) return;

  const ScanResult& scan = bus_->lastScanResult();

  // ScanResult::found_count is the true number of responders; found_ids[]
  // only holds the first kMaxScanIds of them. Both are passed so the
  // classifier can detect and report that overflow rather than silently
  // classifying a short list as a complete picture.
  result_ = classifyObserved(scan.found_ids,
                             ServoBus::kMaxScanIds,
                             scan.found_count,
                             scan.lo,
                             scan.hi);

  state_ = State::COMPLETE;
}

}  // namespace servo
}  // namespace matdog
