#include "ActuatorEnvelope.h"
#include <cmath>
namespace matdog { namespace motion {
RequirementVerdict classifyRequirement(double required, const ActuatorLimit& limit, double margin) {
  if (!limit.set || !std::isfinite(limit.value) || limit.value <= 0 || limit.provenance == LimitProvenance::UNMEASURED ||
      !std::isfinite(required) || required < 0 || !std::isfinite(margin) || margin < 1.0)
    return RequirementVerdict::REQUIRES_MEASUREMENT;
  if (required * margin > limit.value) return RequirementVerdict::EXCEEDS_LIMIT;
  return limit.provenance == LimitProvenance::HARDWARE_LOADED_VERIFIED ? RequirementVerdict::VERIFIED : RequirementVerdict::PROVISIONALLY_SUPPORTED;
}
const char* toString(RequirementVerdict v) {
  switch (v) {
    case RequirementVerdict::REQUIRES_MEASUREMENT: return "REQUIRES_MEASUREMENT";
    case RequirementVerdict::EXCEEDS_LIMIT: return "EXCEEDS_LIMIT";
    case RequirementVerdict::PROVISIONALLY_SUPPORTED: return "PROVISIONALLY_SUPPORTED";
    default: return "VERIFIED";
  }
}
const char* toString(LimitProvenance p) {
  switch (p) {
    case LimitProvenance::UNMEASURED: return "UNMEASURED";
    case LimitProvenance::VENDOR_NOMINAL: return "VENDOR_NOMINAL";
    case LimitProvenance::BENCH_NO_LOAD: return "BENCH_NO_LOAD";
    default: return "HARDWARE_LOADED_VERIFIED";
  }
}
} }
