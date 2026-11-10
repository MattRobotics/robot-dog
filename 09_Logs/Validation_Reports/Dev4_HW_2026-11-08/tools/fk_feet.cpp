// Read "t a0..a11" lines (rad, slot order LF RF RH LH x hip/upper/lower) and
// print t followed by the base_link contact reference of each leg (m).
#include "motion/FootContact.h"
#include <cstdio>
using namespace matdog::motion;
int main(){double t,a[12];
  while(std::scanf("%lf",&t)==1){for(double& x:a)if(std::scanf("%lf",&x)!=1)return 1;
    std::printf("%.0f",t);
    for(unsigned l=0;l<4;++l){const auto c=contactForwardKinematics(static_cast<LegId>(l),{a[3*l],a[3*l+1],a[3*l+2]});
      std::printf(" %d %.5f %.5f %.5f",int(c.status),c.referenceM.x,c.referenceM.y,c.referenceM.z);}
    std::printf("\n");}
}
