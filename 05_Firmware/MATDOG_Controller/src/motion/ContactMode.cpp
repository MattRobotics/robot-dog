#include "ContactMode.h"
#include <cmath>
namespace matdog { namespace motion {
bool contactMode(const LegPhase& ph, const GaitParameters& p, ContactMode& mode) {
  if (!validGaitParameters(p) || !std::isfinite(ph.phase) || ph.phase < 0 || ph.phase >= 1) return false;
  if (ph.boundary) {
    if (ph.phase == 0 && ph.stance) { mode = ContactMode::TOUCHDOWN; return true; }
    if (ph.phase == p.duty && !ph.stance) { mode = ContactMode::LIFT_OFF; return true; }
    return false;
  }
  if (ph.stance != (ph.phase < p.duty)) return false;
  mode = ph.stance ? ContactMode::STANCE : ContactMode::SWING;
  return true;
}
} }
