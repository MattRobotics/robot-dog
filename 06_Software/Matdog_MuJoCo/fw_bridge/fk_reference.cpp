// Firmware reference FK for the MuJoCo twin. Reads rows of 12 joint angles
// (URDF radians, slots LF RF RH LH x hip upper lower) from stdin and writes,
// per leg, the foot_link origin and the contact reference (lowest point of the
// foot cylinder for ground normal +Z in base_link) computed by the production
// src/motion/LegKinematics.cpp and FootContact.cpp, compiled unchanged.
#include "motion/FootContact.h"
#include "motion/LegKinematics.h"
#include <cstdio>
#include <initializer_list>

using namespace matdog::motion;

int main() {
  std::printf("row");
  for (const char* leg : {"lf", "rf", "rh", "lh"})
    for (const char* what : {"status", "origin_x", "origin_y", "origin_z", "contact_x", "contact_y", "contact_z", "center_x", "center_y", "center_z"})
      std::printf(",%s_%s", leg, what);
  std::printf("\n");
  double q[12];
  for (unsigned row = 0;; ++row) {
    for (double& v : q)
      if (std::scanf(" %lf ,", &v) != 1) return 0;
    std::printf("%u", row);
    for (unsigned l = 0; l < 4; ++l) {
      const LegJointAngles angles{q[3 * l], q[3 * l + 1], q[3 * l + 2]};
      const FkResult fk = forwardKinematics(static_cast<LegId>(l), angles);
      const ContactResult c = contactForwardKinematics(static_cast<LegId>(l), angles);
      std::printf(",%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g", int(c.status),
                  fk.footOriginM.x, fk.footOriginM.y, fk.footOriginM.z,
                  c.referenceM.x, c.referenceM.y, c.referenceM.z,
                  c.cylinderCenterM.x, c.cylinderCenterM.y, c.cylinderCenterM.z);
    }
    std::printf("\n");
  }
}
