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
  const char* n[]={"OK","INVALID_INPUT","LIMIT","CERTIFICATE_REQUIRED","START_OUTSIDE_CERTIFIED_REGION","GOAL_MISMATCH","RATE_POLICY",
                   "PATH_START_OUTSIDE_LIMITS","PATH_START_IN_KEEPOUT","PATH_LIMIT","PATH_KEEPOUT"};
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
  // The certificate covers independent LOWER ranges during DISENGAGE and the
  // straight ALIGN stage inside a joint tube. A different start shifts the
  // HIP/UPPER part of every point by at most
  // its distance from the certified set, and tracking adds its tolerance.
  // Each LOWER may start anywhere within its own certified stage-0 interval
  // after inward PRIME, independently of the other three LOWER joints.
  double deviation[12]{};
  for(unsigned i=0;i<12;++i){const double q=at(start,i),a=at(c.start,i),b=at(c.goal,i);
    const double d=i%3==2?(q<std::fmin(a,b)?std::fmin(a,b)-q:q>std::fmax(a,b)?q-std::fmax(a,b):0.):std::abs(q-a);
    deviation[i]=d;
    if(d>plan.startDeviationRad)plan.startDeviationRad=d;
    // Reserve roundoff and positive space for the sequential write box.
    if(d+p.maxTrackingErrorRad+1e-6>=c.tubeRad)plan.failedJoint=i;}
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
      T=std::fmax(T,1.875*delta*p.samplePeriodS/p.maxJointStepRad);
      // Executor feedback can occupy [previous,target] +/- tracking while
      // the twelve writes complete. The midpoint lies on this straight stage;
      // half the largest sample step must fit the certificate's remaining tube.
      const double budget=c.tubeRad-p.maxTrackingErrorRad-deviation[i]-1e-6;
      T=std::fmax(T,1.875*delta*p.samplePeriodS/(2*budget));}
    // Whole samples; tiny upward margin keeps the closed-form peaks inside.
    plan.durationS[s]=std::ceil(T*(1+1e-9)/p.samplePeriodS)*p.samplePeriodS;
    if(!std::isfinite(plan.durationS[s])||plan.durationS[s]>p.segmentTimeoutS){plan.status=EntryStatus::RATE_POLICY;return plan;}}
  plan.status=EntryStatus::OK;return plan;
}
FixtureEntryPlan planMeasuredEntry(const JointTargetFrame& start,const JointTargetFrame& goal,const FeedbackPolicy& p,
                                   const MeasuredEntryConfig& c,double minStage){
  FixtureEntryPlan plan;plan.start=start;plan.goal=goal;
  if(!finiteFrame(start)||!finiteFrame(goal)||!std::isfinite(minStage)||minStage<=0||minStage>120||
     !std::isfinite(p.samplePeriodS)||p.samplePeriodS<=0||!std::isfinite(p.maxJointStepRad)||p.maxJointStepRad<=0||
     !std::isfinite(p.maxTrackingErrorRad)||p.maxTrackingErrorRad<=0||!std::isfinite(c.pathMarginRad)||c.pathMarginRad<=0||
     (c.keepoutCount&&!c.keepouts)||c.keepoutCount>16)return plan;
  plan.middle=start;for(unsigned i=2;i<12;i+=3)at(plan.middle,i)=at(goal,i);
  // The specific swept path, sampled, against limits and keep-outs.
  double s[12],g[12];for(unsigned i=0;i<12;++i){s[i]=at(start,i);g[i]=at(goal,i);}
  // Stack budget (2026-10-16 boot-loop root cause): ~8 KB of samples never live
  // on the 8 KB Controller loop stack. Static storage; the planner runs only on
  // the Controller thread (and single-threaded host tests) - not reentrant.
  static double path[startpose::kEntryPathSamples][12];startpose::buildTwoStageEntry(s,g,path);
  startpose::JointLimits12 limits;for(unsigned i=0;i<12;++i){limits.lower[i]=p.jointLimits[i/3][i%3].lower;limits.upper[i]=p.jointLimits[i/3][i%3].upper;}
  startpose::TrackingTolerance tracking;tracking.rad=p.maxTrackingErrorRad;tracking.settle_s=p.segmentTimeoutS>0?p.segmentTimeoutS:1;
  startpose::PathClearance margin;margin.margin_rad=c.pathMarginRad;
  const auto v=startpose::checkPath(path,startpose::kEntryPathSamples,limits,c.keepouts,c.keepoutCount,tracking,margin);
  plan.failedSample=v.sample;plan.failedKeepout=v.keepout;plan.clearanceRad=v.clearance_rad;plan.requiredClearanceRad=v.required_rad;
  if(!v.admissible){
    plan.failedJoint=v.joint<12?v.joint:12;
    plan.status=v.reason==startpose::PathReason::START_OUTSIDE_LIMITS?EntryStatus::PATH_START_OUTSIDE_LIMITS:
                v.reason==startpose::PathReason::START_IN_KEEPOUT?EntryStatus::PATH_START_IN_KEEPOUT:
                v.reason==startpose::PathReason::KEEPOUT?EntryStatus::PATH_KEEPOUT:
                v.reason==startpose::PathReason::LIMIT?EntryStatus::PATH_LIMIT:EntryStatus::INVALID_INPUT;
    return plan;
  }
  const JointTargetFrame* frames[]={&plan.start,&plan.middle,&plan.goal};
  for(unsigned st=0;st<kFixtureEntryStages;++st){double T=minStage,vP=1e18,aP=1e18;bool any=false;
    for(unsigned i=0;i<12;++i){if(!moves(st,i))continue;
      const double delta=std::abs(at(*frames[st+1],i)-at(*frames[st],i));
      const double vmax=rate(p.maxVelocityRadS[i/3],i%3),amax=rate(p.maxAccelerationRadS2[i/3],i%3);
      if(!std::isfinite(vmax)||vmax<=0||!std::isfinite(amax)||amax<=0){plan.failedJoint=i;plan.status=EntryStatus::RATE_POLICY;return plan;}
      // Quintic peaks: velocity 1.875 d/T, acceleration 10/sqrt(3) d/T^2, step 1.875 d dt/T.
      T=std::fmax(T,1.875*delta/vmax);T=std::fmax(T,std::sqrt(5.773502691896258*delta/amax));
      T=std::fmax(T,1.875*delta*p.samplePeriodS/p.maxJointStepRad);
      // Common law: every joint moves delta_i * s(t), s in [0,1]; the binding joint sets the progress limits
      // (velocity incl. the per-sample step cap, acceleration; jerk from a fixed ramp).
      if(delta>1e-9){
        const double vJ=std::fmin(vmax,p.maxJointStepRad/p.samplePeriodS);
        vP=std::fmin(vP,vJ/delta);aP=std::fmin(aP,amax/delta);any=true;}}
    plan.lawValid[st]=false;
    if(any){ScurveProfile profile;
      if(profile.plan(1.0,{vP,aP,aP/kLawAccelRampS})){plan.law[st]=profile;plan.lawValid[st]=true;T=std::fmax(minStage,profile.durationS());}}
    plan.durationS[st]=std::ceil(T*(1+1e-9)/p.samplePeriodS)*p.samplePeriodS;
    if(!std::isfinite(plan.durationS[st])||plan.durationS[st]>p.segmentTimeoutS){plan.status=EntryStatus::RATE_POLICY;return plan;}}
  plan.status=EntryStatus::OK;return plan;
}
LocomotionFrame fixtureEntrySample(const FixtureEntryPlan& plan,unsigned stage,double u,const BodyPose& body){
  LocomotionFrame f;
  if(plan.status!=EntryStatus::OK||stage>=kFixtureEntryStages||!std::isfinite(u)||u<0||u>1)return f;
  TimeLawSample law;
  if(plan.lawValid[stage]){  // dilated common S-curve
    const double k=plan.law[stage].durationS()/plan.durationS[stage];const auto q=plan.law[stage].at(u*plan.durationS[stage]*k);
    law.valid=true;law.progress=q.s<0?0:q.s>1?1:q.s;law.velocity=q.v*k;law.acceleration=q.a*k*k;
  }else law=quinticTimeLaw(u,plan.durationS[stage]);
  if(!law.valid)return f;
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
