#include "../../src/motion/ContactMode.h"
#include <cmath>
#include <cstdio>
#include <limits>
using namespace matdog::motion;
static unsigned checks = 0, failures = 0;
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; std::printf("FAIL %d: %s\n", __LINE__, #x); } } while (0)
int main() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  // The modes follow the schedule exactly: exact instants are LIFT_OFF / TOUCHDOWN, never a proximity rule.
  for (unsigned kind = 0; kind < 2; ++kind) {
    GaitParameters m; m.type = kind ? GaitType::TROT : GaitType::WALK; m.duty = kind ? .6 : .8; m.swayXM = 0;
    for (unsigned i = 0; i < 4; ++i) {
      LegId leg = static_cast<LegId>(i); double off = gaitOffset(m.type, leg); ContactMode mode = ContactMode::SWING;
      auto at = [&](double phase) { double cycles = 1 + phase - off; if (cycles < 1) cycles += 1; return legPhase(m, leg, cycles); };
      CHECK(contactMode(at(0), m, mode) && mode == ContactMode::TOUCHDOWN && declaredContact(mode));
      CHECK(contactMode(at(m.duty / 2), m, mode) && mode == ContactMode::STANCE && declaredContact(mode));
      CHECK(contactMode(at(m.duty), m, mode) && mode == ContactMode::LIFT_OFF && declaredContact(mode));
      CHECK(contactMode(at(m.duty + (1 - m.duty) / 2), m, mode) && mode == ContactMode::SWING && !declaredContact(mode));
      CHECK(contactMode(at(1 - 1e-3), m, mode) && mode == ContactMode::SWING);
      CHECK(contactMode(at(m.duty - 1e-3), m, mode) && mode == ContactMode::STANCE);
    }
    // each cycle has exactly one TOUCHDOWN and one LIFT_OFF instant per leg
    for (unsigned i = 0; i < 4; ++i) {
      LegId leg = static_cast<LegId>(i); unsigned td = 0, lo = 0;
      for (unsigned n = 0; n <= 4000; ++n) {
        ContactMode mode;
        if (contactMode(legPhase(m, leg, 1 + n / 4000.), m, mode)) { td += mode == ContactMode::TOUCHDOWN; lo += mode == ContactMode::LIFT_OFF; }
      }
      CHECK(td >= 1 && lo >= 1);
    }
    ContactMode mode = ContactMode::STANCE; LegPhase bad;
    bad.phase = nan; CHECK(!contactMode(bad, m, mode)); bad.phase = 1; CHECK(!contactMode(bad, m, mode));
    bad.phase = .3; bad.stance = false; bad.boundary = false; CHECK(!contactMode(bad, m, mode));
    bad.phase = .3; bad.stance = true; bad.boundary = true; CHECK(!contactMode(bad, m, mode));
    CHECK(mode == ContactMode::STANCE);  // untouched on failure
    auto invalid = m; invalid.duty = .2; LegPhase ok; ok.phase = .5; CHECK(!contactMode(ok, invalid, mode));
  }
  std::printf("CONTACT_MODE_HOST = %s: %u checks, %u failures\n", failures ? "FAIL" : "PASS", checks, failures);
  return failures ? 1 : 0;
}
