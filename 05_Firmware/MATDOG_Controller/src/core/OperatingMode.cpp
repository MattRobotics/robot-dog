#include "OperatingMode.h"

namespace matdog {
namespace core {

const char* toString(OperatingMode mode) {
  switch (mode) {
    case OperatingMode::MAINTENANCE: return "MAINTENANCE";
    case OperatingMode::RUN:         return "RUN";
  }
  return "UNKNOWN";
}

}  // namespace core
}  // namespace matdog
