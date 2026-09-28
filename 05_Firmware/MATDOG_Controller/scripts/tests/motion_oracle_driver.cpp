// Text-only host adapter for the existing Python oracle. No production I/O.
#include "../../src/motion/LegKinematics.h"
#include <iomanip>
#include <iostream>
using namespace matdog::motion;
int main() {
  unsigned leg;
  LegJointAngles q{};
  Vector3 target{};
  IkOptions options;
  std::cout << std::setprecision(17);
  while (std::cin >> leg >> q.hip >> q.upper >> q.lower
                  >> target.x >> target.y >> target.z
                  >> options.seed.hip >> options.seed.upper >> options.seed.lower) {
    if (leg > 3) return 2;
    const auto fk = forwardKinematics(static_cast<LegId>(leg), q);
    const auto ik = inverseKinematics(static_cast<LegId>(leg), target, options);
    std::cout << static_cast<unsigned>(fk.status) << ' '
              << fk.footOriginM.x << ' ' << fk.footOriginM.y << ' ' << fk.footOriginM.z;
    for (const auto& row : fk.baseFromFootRotation)
      for (double value : row) std::cout << ' ' << value;
    std::cout << ' ' << static_cast<unsigned>(ik.status) << ' '
              << ik.joints.hip << ' ' << ik.joints.upper << ' ' << ik.joints.lower
              << ' ' << ik.residualM << ' ' << static_cast<unsigned>(ik.solutions) << '\n';
  }
  return std::cin.eof() ? 0 : 2;
}
