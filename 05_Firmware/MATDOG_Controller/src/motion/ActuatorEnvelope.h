#ifndef MATDOG_MOTION_ACTUATOR_ENVELOPE_H
#define MATDOG_MOTION_ACTUATOR_ENVELOPE_H
#include <stdint.h>
namespace matdog { namespace motion {
// G5-A classification of a semantic requirement against an actuator limit, with PROVENANCE. This is deliberately NOT an
// actuator safety layer: it holds no limits, commands nothing, and has no default value for any limit. The safety layer
// that will gate real commands is separate and later. A limit that was never measured is reported as such, never guessed.
enum class LimitProvenance : uint8_t {
  UNMEASURED,                 // no value exists
  VENDOR_NOMINAL,             // datasheet value at the vendor's test condition
  BENCH_NO_LOAD,              // measured on the bench without joint load (MATDOG QC campaign)
  HARDWARE_LOADED_VERIFIED    // measured on the assembled robot under load
};
struct ActuatorLimit { bool set = false; double value = 0; LimitProvenance provenance = LimitProvenance::UNMEASURED; };
enum class RequirementVerdict : uint8_t {
  REQUIRES_MEASUREMENT,   // no usable limit
  EXCEEDS_LIMIT,          // requirement above the limit that does exist
  PROVISIONALLY_SUPPORTED,// within a vendor-nominal or no-load-bench limit only
  VERIFIED                // within a loaded-hardware verified limit
};
// `margin` (>= 1) is the factor the requirement must stay below the limit by (0 means exactly the limit).
RequirementVerdict classifyRequirement(double required, const ActuatorLimit& limit, double margin);
const char* toString(RequirementVerdict v);
const char* toString(LimitProvenance p);
} }
#endif
