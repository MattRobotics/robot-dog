#include "FixtureEntryPlanner.h"
#include "TimedStand.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
double& at(JointTargetFrame& f,unsigned i){auto& q=f.legs[i/3];return i%3==0?q.hip:i%3==1?q.upper:q.lower;}
double at(const JointTargetFrame& f,unsigned i){const auto& q=f.legs[i/3];return i%3==0?q.hip:i%3==1?q.upper:q.lower;}
double rate(const LegJointAngles& q,unsigned joint){return joint==0?q.hip:joint==1?q.upper:q.lower;}
bool moves(unsigned stage,unsigned i){return stage==0?i%3==2:i%3!=2;}
bool finiteFrame(const JointTargetFrame& f){if(!f.valid)return false;for(unsigned i=0;i<12;++i)if(!std::isfinite(at(f,i)))return false;return true;}
}
const char* entryStatusName(EntryStatus s){
  const char* n[]={"OK","INVALID_INPUT","LIMIT","CERTIFICATE_REQUIRED","START_OUTSIDE_CERTIFIED_REGION","GOAL_MISMATCH","RATE_POLICY"};
  const unsigned i=static_cast<unsigned>(s);return i<sizeof(n)/sizeof(n[0])?n[i]:"UNKNOWN";
}
FixtureEntryPlan planFixtureEntry(const JointTargetFrame& start,const JointTargetFrame& goal,const FeedbackPolicy& p,
                                  const FixtureEntryCertificate& c,double minStage){
  FixtureEntryPlan plan;plan.start=start;plan.goal=goal;
  if(!finiteFrame(start)||!finiteFrame(goal)||!std::isfinite(minStage)||minStage<=0||minStage>120||
     !std::isfinite(p.samplePeriodS)||p.samplePeriodS<=0||!std::isfinite(p.maxJointStepRad)||p.maxJointStepRad<=0||
     !std::isfinite(p.maxTrackingErrorRad)||p.maxTrackingErrorRad<=0)return plan;
  if(!c.present||!finiteFrame(c.start)||!finiteFrame(c.goal)||!std::isfinite(c.tubeRad)||c.tubeRad<=p.maxTrackingErrorRad||c.tubeRad>.02){
    plan.status=EntryStatus::CERTIFICATE_REQUIRED;return plan;}
  for(unsigned i=0;i<12;++i)if(std::abs(at(goal,i)-at(c.goal,i))>1e-9){plan.failedJoint=i;plan.status=EntryStatus::GOAL_MISMATCH;return plan;}
  // The certificate covers straight staged segments from its start inside a
  // joint tube. A different start shifts every point of the path by at most
  // its distance from the certified set, and tracking adds its tolerance.
  // For LOWER the certified set is the whole stage-0 interval: a start
  // already moved along that interval (inward prime) is on the certified path.
  for(unsigned i=0;i<12;++i){const double q=at(start,i),a=at(c.start,i),b=at(c.goal,i);
    const double d=i%3==2?(q<std::fmin(a,b)?std::fmin(a,b)-q:q>std::fmax(a,b)?q-std::fmax(a,b):0.):std::abs(q-a);
    if(d>plan.startDeviationRad){plan.startDeviationRad=d;if(d+p.maxTrackingErrorRad>c.tubeRad)plan.failedJoint=i;}}
  if(plan.failedJoint<12){plan.status=EntryStatus::START_OUTSIDE_CERTIFIED_REGION;return plan;}
  plan.middle=start;for(unsigned i=2;i<12;i+=3)at(plan.middle,i)=at(goal,i);
  const JointTargetFrame* frames[]={&plan.start,&plan.middle,&plan.goal};
  for(const auto* f:frames)for(unsigned i=0;i<12;++i){const auto& l=p.jointLimits[i/3][i%3];
    if(at(*f,i)<l.lower||at(*f,i)>l.upper){plan.failedJoint=i;plan.status=EntryStatus::LIMIT;return plan;}}
  for(unsigned s=0;s<kFixtureEntryStages;++s){double T=minStage;
    for(unsigned i=0;i<12;++i){if(!moves(s,i))continue;
      const double delta=std::abs(at(*frames[s+1],i)-at(*frames[s],i));
      const double v=rate(p.maxVelocityRadS[i/3],i%3),a=rate(p.maxAccelerationRadS2[i/3],i%3);
      if(!std::isfinite(v)||v<=0||!std::isfinite(a)||a<=0){plan.failedJoint=i;plan.status=EntryStatus::RATE_POLICY;return plan;}
      // Quintic peaks: velocity 1.875 d/T, acceleration 10/sqrt(3) d/T^2,
      // largest per-sample step 1.875 d dt/T.
      T=std::fmax(T,1.875*delta/v);T=std::fmax(T,std::sqrt(5.773502691896258*delta/a));
      T=std::fmax(T,1.875*delta*p.samplePeriodS/p.maxJointStepRad);}
    // Whole samples; tiny upward margin keeps the closed-form peaks inside.
    plan.durationS[s]=std::ceil(T*(1+1e-9)/p.samplePeriodS)*p.samplePeriodS;}
  plan.status=EntryStatus::OK;return plan;
}
LocomotionFrame fixtureEntrySample(const FixtureEntryPlan& plan,unsigned stage,double u,const BodyPose& body){
  LocomotionFrame f;
  if(plan.status!=EntryStatus::OK||stage>=kFixtureEntryStages||!std::isfinite(u)||u<0||u>1)return f;
  const auto law=quinticTimeLaw(u,plan.durationS[stage]);if(!law.valid)return f;
  const JointTargetFrame& a=stage==0?plan.start:plan.middle;const JointTargetFrame& b=stage==0?plan.middle:plan.goal;
  f.target=a;f.target.sequence=0;
  for(unsigned i=0;i<12;++i){const double d=at(b,i)-at(a,i);
    at(f.target,i)=at(a,i)+d*law.progress;
    auto& v=f.velocityRadS[i/3];auto& acc=f.accelerationRadS2[i/3];
    double& vi=i%3==0?v.hip:i%3==1?v.upper:v.lower;double& ai=i%3==0?acc.hip:i%3==1?acc.upper:acc.lower;
    vi=d*law.velocity;ai=d*law.acceleration;}
  f.cartesian.body=body;f.cartesian.valid=true;
  // Display-only feet; the fixture requires no foot support.
  for(unsigned leg=0;leg<4;++leg){const auto fk=contactForwardKinematics(static_cast<LegId>(leg),f.target.legs[leg]);
    if(fk.status==ContactStatus::OK)pointToWorld(body,fk.referenceM,f.cartesian.feet[leg]);}
  f.target.valid=true;f.status=GaitStatus::OK;return f;
}
} }
