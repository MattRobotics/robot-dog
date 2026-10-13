#include "FirstStrideReference.h"
#include "FirstStrideData.h"
#include <cmath>
namespace matdog { namespace motion {
const char* compiledFirstStrideDigest(){return first_stride_data::kDataSha256;}
double compiledFirstStrideTubeRad(){return .006-first_stride_data::kQuantizationError;}
bool compiledFirstStrideFitsLimits(const FeedbackPolicy& p,double tube,unsigned* failed){
 if(!std::isfinite(tube)||tube<=0||tube>compiledFirstStrideTubeRad())return false;
 for(unsigned i=0;i<12;++i){const auto& b=p.jointLimits[i/3][i%3];
  if(!std::isfinite(b.lower)||!std::isfinite(b.upper)||b.lower>=b.upper||first_stride_data::kMinimum[i]-tube<b.lower||first_stride_data::kMaximum[i]+tube>b.upper){if(failed)*failed=i;return false;}}
 return true;
}
LocomotionFrame compiledFirstStrideReference(const CartesianSample& cart,const LocomotionFrame& previous,double clock,double scale){
 LocomotionFrame f;if(!cart.valid||!std::isfinite(clock)||clock<0||clock>106+1e-7||!std::isfinite(scale)||scale<9||scale>100)return f;
 const auto index=static_cast<unsigned>(std::llround(clock/first_stride_data::kNativeStep));
 if(index>=first_stride_data::kSamples||std::abs(clock-index*first_stride_data::kNativeStep)>1e-7)return f;
 const auto& row=first_stride_data::kSamplesData[index];f.cartesian=cart;const double vs=9/scale,as=vs*vs;
 for(unsigned i=0;i<4;++i){f.target.legs[i]={row.q[i*3],row.q[i*3+1],row.q[i*3+2]};
  f.velocityRadS[i]={row.velocity[i*3]*first_stride_data::kVelocityUnit*vs,row.velocity[i*3+1]*first_stride_data::kVelocityUnit*vs,row.velocity[i*3+2]*first_stride_data::kVelocityUnit*vs};
  f.accelerationRadS2[i]={row.acceleration[i*3]*first_stride_data::kAccelerationUnit*as,row.acceleration[i*3+1]*first_stride_data::kAccelerationUnit*as,row.acceleration[i*3+2]*first_stride_data::kAccelerationUnit*as};f.branches[i]=previous.branches[i];}
 f.target.valid=true;f.status=GaitStatus::OK;return f;
}
} }
