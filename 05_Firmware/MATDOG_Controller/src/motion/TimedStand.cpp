#include "TimedStand.h"
#include <cmath>
#include <limits>
namespace matdog { namespace motion {
namespace {
bool validDuration(double t) {
  if(!std::isfinite(t)||t<=0)return false;
  const double inverse=1.0/t;
  return std::isfinite(inverse*inverse)&&inverse*inverse>0;
}
bool solve3(const double j[3][3], const double b[3], double x[3]) {
  double a[3][4];
  for(unsigned r=0;r<3;++r) {for(unsigned c=0;c<3;++c)a[r][c]=j[r][c];a[r][3]=b[r];}
  for(unsigned c=0;c<3;++c) {
    unsigned pivot=c;
    for(unsigned r=c+1;r<3;++r)if(std::abs(a[r][c])>std::abs(a[pivot][c]))pivot=r;
    if(!std::isfinite(a[pivot][c])||std::abs(a[pivot][c])<1e-10)return false;
    for(unsigned k=0;k<4;++k) {double tmp=a[c][k];a[c][k]=a[pivot][k];a[pivot][k]=tmp;}
    const double divisor=a[c][c];for(unsigned k=c;k<4;++k)a[c][k]/=divisor;
    for(unsigned r=0;r<3;++r)if(r!=c) {
      const double factor=a[r][c];for(unsigned k=c;k<4;++k)a[r][k]-=factor*a[c][k];
    }
  }
  for(unsigned i=0;i<3;++i) {x[i]=a[i][3];if(!std::isfinite(x[i]))return false;}
  return true;
}
// Differentiate the exact G2 contact map, with +Z normal and cos(hip)>0.
bool pathDerivatives(LegId leg, LegJointAngles q, double heightSpan, double qs[3], double qss[3]) {
  const auto* m=legModel(leg);
  if(!m)return false;
  const double a=-m->lowerOrigin.z, f=m->footOrigin.x;
  const double r=footContactModel().radiusM, g=m->footOrigin.z+r;
  const double d=m->upperOrigin.y+m->footOrigin.y;
  const double ch=std::cos(q.hip), sh=std::sin(q.hip), su=std::sin(q.upper), cu=std::cos(q.upper);
  const double st=std::sin(q.upper+q.lower), ct=std::cos(q.upper+q.lower);
  const double v=-a*cu-f*st+g*ct-r;
  const double xu=-a*cu-f*st+g*ct, xl=-f*st+g*ct;
  const double vu=a*su-f*ct-g*st, vl=-f*ct-g*st;
  const double j[3][3]={{0,xu,xl},{-d*sh-v*ch,-vu*sh,-vl*sh},{d*ch-v*sh,vu*ch,vl*ch}};
  const double pathVelocity[3]={0,0,-heightSpan};
  if(!solve3(j,pathVelocity,qs))return false;
  const double h=qs[0],u=qs[1],t=qs[1]+qs[2];
  const double vp=vu*qs[1]+vl*qs[2];
  const double xpp=a*su*u*u-(f*ct+g*st)*t*t;
  const double vpp=a*cu*u*u+(f*st-g*ct)*t*t;
  const double bias[3]={-xpp,
      -(-d*ch*h*h+v*sh*h*h-2*vp*ch*h-vpp*sh),
      -(-d*sh*h*h-v*ch*h*h-2*vp*sh*h+vpp*ch)};
  return solve3(j,bias,qss);
}
}
bool validTiming(const TimingSpec& s) {
  // Bounded resource count, not an execution-rate limit.
  return validDuration(s.durationS)&&s.intervals>=2&&s.intervals<=1000000;
}
TimingSpec timingFromPeriod(double period,uint32_t intervals) {
  TimingSpec result{period*intervals,intervals};
  return std::isfinite(period)&&period>0&&validTiming(result)?result:TimingSpec{};
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
TimedStandSample evaluateTimedStand(const StandDefinition& d,double u,double duration,const LegJointAngles seeds[4]) {
  TimedStandSample result;
  const auto law=quinticTimeLaw(u,duration);
  if(!law.valid) {result.status=TimedStatus::INVALID_TIMING;return result;}
  const auto geometry=sampleStandPath(d,law.progress,seeds);
  if(!geometry.target.valid) {result.status=TimedStatus::PATH_FAILURE;return result;}
  for(unsigned i=0;i<4;++i) {
    double qs[3],qss[3];
    if(!pathDerivatives(static_cast<LegId>(i),geometry.target.legs[i],d.standBodyHeightM-d.lowBodyHeightM,qs,qss)) {
      result={};result.status=TimedStatus::SINGULAR_DERIVATIVE;return result;
    }
    double velocity[3],acceleration[3];
    for(unsigned j=0;j<3;++j) {
      velocity[j]=qs[j]*law.velocity;
      acceleration[j]=qss[j]*law.velocity*law.velocity+qs[j]*law.acceleration;
      if(!std::isfinite(velocity[j])||!std::isfinite(acceleration[j])) {
        result={};result.status=TimedStatus::SINGULAR_DERIVATIVE;return result;
      }
    }
    result.velocityRadS[i]={velocity[0],velocity[1],velocity[2]};
    result.accelerationRadS2[i]={acceleration[0],acceleration[1],acceleration[2]};
  }
  result.geometry=geometry;
  result.body.translationM.z=geometry.bodyHeightM;
  result.timeS=u*duration;result.normalizedTime=u;result.progress=law.progress;
  result.phase=u==0?TrajectoryPhase::ACQUISITION_HOLD:TrajectoryPhase::CONTACT_LOCKED_RISE;
  result.status=TimedStatus::OK;return result;
}
TimedStatus TimedStand::initialize(const StandDefinition& d,const TimingSpec& timing) {
  ready_=false;previous_={};metrics_={};
  if(!validTiming(timing))return TimedStatus::INVALID_TIMING;
  const auto stand=generateStandTarget(d);
  if(!stand.target.valid)return TimedStatus::PATH_FAILURE;
  const auto low=evaluateTimedStand(d,0,timing.durationS,stand.target.legs);
  if(low.status!=TimedStatus::OK)return low.status;
  definition_=d;timing_=timing;previous_=low.geometry;
  metrics_.durationS=timing.durationS;
  metrics_.minJointLimitMarginRad=std::numeric_limits<double>::infinity();
  ready_=true;return TimedStatus::OK;
}
TimedStandSample TimedStand::next() {
  TimedStandSample result;
  if(!ready_)return result;
  if(metrics_.emitted>timing_.intervals) {result.status=TimedStatus::COMPLETE;return result;}
  const auto index=metrics_.emitted;
  result=evaluateTimedStand(definition_,static_cast<double>(index)/timing_.intervals,timing_.durationS,previous_.target.legs);
  if(result.status!=TimedStatus::OK) {ready_=false;return result;}
  for(unsigned i=0;i<4;++i) {
    const auto a=result.geometry.branches[i], b=previous_.branches[i];
    if(a.hip!=b.hip||a.elbow!=b.elbow) {
      ++metrics_.branchChanges[i];ready_=false;result={};result.status=TimedStatus::BRANCH_CHANGE;return result;
    }
  }
  result.geometry.target.sequence=index;
  for(unsigned i=0;i<4;++i) {
    const auto q=result.geometry.target.legs[i], old=previous_.target.legs[i];
    result.geometry.jointDeltaRad[i]=index==0?LegJointAngles{}:LegJointAngles{q.hip-old.hip,q.upper-old.upper,q.lower-old.lower};
    const auto contact=result.geometry.contactsWorldM[i];
    if(index==0)firstContacts_[i]=contact;
    const auto anchor=firstContacts_[i];
    result.geometry.maxContactDriftM=std::fmax(result.geometry.maxContactDriftM,
      std::hypot(std::hypot(contact.x-anchor.x,contact.y-anchor.y),contact.z-anchor.z));
    const auto v=result.velocityRadS[i],a=result.accelerationRadS2[i];
    auto& pv=metrics_.peakVelocityRadS[i];auto& pa=metrics_.peakAccelerationRadS2[i];
    pv={std::fmax(pv.hip,std::abs(v.hip)),std::fmax(pv.upper,std::abs(v.upper)),std::fmax(pv.lower,std::abs(v.lower))};
    pa={std::fmax(pa.hip,std::abs(a.hip)),std::fmax(pa.upper,std::abs(a.upper)),std::fmax(pa.lower,std::abs(a.lower))};
    const auto m=result.geometry.jointLimitMarginRad[i];
    metrics_.minJointLimitMarginRad=std::fmin(metrics_.minJointLimitMarginRad,std::fmin(m.hip,std::fmin(m.upper,m.lower)));
  }
  metrics_.maxContactResidualM=std::fmax(metrics_.maxContactResidualM,result.geometry.maxContactResidualM);
  metrics_.maxContactDriftM=std::fmax(metrics_.maxContactDriftM,result.geometry.maxContactDriftM);
  previous_=result.geometry;++metrics_.emitted;return result;
}
} }
