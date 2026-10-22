#include "GaitTiming.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
bool validDuration(double t) {
  if(!std::isfinite(t)||t<=0)return false;
  const double inverse=1.0/t;
  return std::isfinite(inverse*inverse)&&inverse*inverse>0;
}
}
TimeLawSample quinticTimeLaw(double u,double t) {
  TimeLawSample result;
  if(!std::isfinite(u)||u<0||u>1||!validDuration(t))return result;
  result.valid=true;
  // Complement evaluation near the end avoids progress >1 from cancellation.
  const double v=u<=0.5?u:1-u;
  const double s=v*v*v*(10+v*(-15+6*v));
  result.progress=u<=0.5?s:1-s;
  const double inverse=1.0/t;
  result.velocity=30*u*u*(1-u)*(1-u)*inverse;
  result.acceleration=60*u*(1-u)*(1-2*u)*inverse*inverse;
  if(!std::isfinite(result.velocity)||!std::isfinite(result.acceleration))return {};
  return result;
}
} }
