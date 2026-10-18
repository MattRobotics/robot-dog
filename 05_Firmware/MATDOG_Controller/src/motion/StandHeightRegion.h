#ifndef MATDOG_MOTION_STAND_HEIGHT_REGION_H
#define MATDOG_MOTION_STAND_HEIGHT_REGION_H
#include "StandTrajectory.h"
namespace matdog { namespace motion {
// Conservative target enclosure of canonical fixed-contact STAND 60..150 mm.
// Whole independent box +/- OperationalTolerance::kGeometryTubeRad is proved
// offline, not inferred from sampled IK. Every runtime target is checked here.
constexpr double kHeightTargetMinRad[12]={-.001,.409,-.956, -.001,.409,-.956, -.001,.897,-1.128, -.001,.897,-1.128};
constexpr double kHeightTargetMaxRad[12]={ .001,1.134,.351, .001,1.134,.351, .001,1.875,.002, .001,1.875,.002};
inline bool heightTargetInRegion(const JointTargetFrame& f){
 if(!f.valid)return false;
 for(unsigned i=0;i<12;++i){const auto& q=f.legs[i/3];const double x=i%3==0?q.hip:i%3==1?q.upper:q.lower;
  if(!(x>=kHeightTargetMinRad[i]&&x<=kHeightTargetMaxRad[i]))return false;}
 return true;
}
} }
#endif
