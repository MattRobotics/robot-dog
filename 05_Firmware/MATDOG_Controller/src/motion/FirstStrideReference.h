#ifndef MATDOG_FIRST_STRIDE_REFERENCE_H
#define MATDOG_FIRST_STRIDE_REFERENCE_H
#include "Gait.h"
namespace matdog { namespace motion {
LocomotionFrame compiledFirstStrideReference(const CartesianSample&,const LocomotionFrame&,double native_clock_s,double time_scale);
const char* compiledFirstStrideDigest();
double compiledFirstStrideTubeRad();
} }
#endif
