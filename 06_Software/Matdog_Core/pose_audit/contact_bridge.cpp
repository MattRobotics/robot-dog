// Offline research adapter to the unchanged G2 contact IK; no hardware linkage.
#include "FootContact.h"
extern "C" int solve_contact(unsigned leg,const double* target,const double* seed,double* output) {
  using namespace matdog::motion;
  if(leg>=4||!target||!seed||!output)return -1;
  ContactIkOptions options;options.kinematics.seed={seed[0],seed[1],seed[2]};
  const auto r=contactInverseKinematics(static_cast<LegId>(leg),{target[0],target[1],target[2]},options);
  if(r.status!=ContactStatus::OK)return static_cast<int>(r.status);
  output[0]=r.joints.hip;output[1]=r.joints.upper;output[2]=r.joints.lower;return 0;
}
