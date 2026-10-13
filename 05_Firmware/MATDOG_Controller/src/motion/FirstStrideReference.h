#ifndef MATDOG_FIRST_STRIDE_REFERENCE_H
#define MATDOG_FIRST_STRIDE_REFERENCE_H
#include "Gait.h"
#include "MotionFeedback.h"
namespace matdog { namespace motion {
LocomotionFrame compiledFirstStrideReference(const CartesianSample&,const LocomotionFrame&,double native_clock_s,double time_scale);
const char* compiledFirstStrideDigest();
double compiledFirstStrideTubeRad();
bool compiledFirstStrideFitsLimits(const FeedbackPolicy&,double tube_rad,unsigned* failed_joint=nullptr);
} }
#endif
