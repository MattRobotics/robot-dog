#include "../../src/motion/StandTransition.h"
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>
using namespace matdog::motion;
static unsigned checks=0,failures=0;
#define CHECK(x) do {++checks;if(!(x)){++failures;std::printf("FAIL %d: %s\n",__LINE__,#x);}}while(0)
static double value(LegJointAngles q,unsigned i){return i==0?q.hip:(i==1?q.upper:q.lower);}
static bool near(double a,double b,double tolerance=1e-10){return std::abs(a-b)<=tolerance;}
static StartupObservation readyObservation() {
  StartupObservation o;
  const auto& d=canonicalStandDefinition();const auto stand=generateStandTarget(d);
  o.joints=sampleStandPath(d,0,stand.target.legs).target;
  o.body.translationM.z=d.lowBodyHeightM;o.condition=StartupCondition::VERIFIED_LOW_STANCE;
  o.fresh=o.stationary=o.fourFootSupportConfirmed=o.flatGroundConfirmed=o.collisionClearanceReviewed=o.supportAndLoadReviewed=true;
  return o;
}
static StandTransition at(MotionState state) {
  StandTransition t;
  if(state==MotionState::OFF)return t;
  t.apply(MotionEvent::ENABLE);if(state==MotionState::IDLE)return t;
  t.begin(readyObservation(),{5,2});if(state==MotionState::STAND_TRANSITION)return t;
  for(unsigned i=0;i<3;++i)t.next();
  if(state==MotionState::STAND)return t;
  t.apply(MotionEvent::STOP);return t;
}
int main() {
  const auto& d=canonicalStandDefinition();const auto o=readyObservation();
  const double nan=std::numeric_limits<double>::quiet_NaN();
  CHECK(evaluateStartup(o).status==AcquisitionStatus::READY);
  for(unsigned condition=0;condition<4;++condition) {
    auto input=o;input.condition=static_cast<StartupCondition>(condition);
    const auto result=evaluateStartup(input);
    const AcquisitionStatus expected[]={AcquisitionStatus::UNKNOWN_POSE,AcquisitionStatus::SUSPENDED_PATH_UNPROVEN,
      AcquisitionStatus::FLOOR_ACQUISITION_UNPROVEN,AcquisitionStatus::READY};
    CHECK(result.status==expected[condition]);CHECK(result.lowStance.target.valid==(condition==3));
  }
  for(unsigned flag=0;flag<6;++flag) {
    auto input=o;
    bool* fields[]={&input.fresh,&input.stationary,&input.fourFootSupportConfirmed,&input.flatGroundConfirmed,
                    &input.collisionClearanceReviewed,&input.supportAndLoadReviewed};
    *fields[flag]=false;
    CHECK(evaluateStartup(input).status==AcquisitionStatus::MISSING_EVIDENCE);
  }
  auto bad=o;bad.joints.valid=false;CHECK(evaluateStartup(bad).status==AcquisitionStatus::INVALID_OBSERVATION);
  bad=o;bad.body.translationM.z=0.15;CHECK(evaluateStartup(bad).status==AcquisitionStatus::BODY_POSE_MISMATCH);
  bad=o;bad.joints.legs[2].hip=nan;CHECK(evaluateStartup(bad).status==AcquisitionStatus::INVALID_OBSERVATION);
  bad=o;bad.joints.legs[1].upper=9;CHECK(evaluateStartup(bad).status==AcquisitionStatus::JOINT_LIMIT);
  bad=o;for(auto& q:bad.joints.legs)q={};
  CHECK(evaluateStartup(bad).status==AcquisitionStatus::LOW_STANCE_MISMATCH);
  const auto front=contactForwardKinematics(LegId::LF,{}),rear=contactForwardKinematics(LegId::LH,{});
  CHECK(near(front.referenceM.z-rear.referenceM.z,0.02));
  // No body height can close both zero-pose contact planes for a parallel body.
  CHECK(!near(0.1134+front.referenceM.z,0)&&near(0.1134+rear.referenceM.z,0));
  BodyPose pose;pose.translationM={0.3,-0.2,0.1};const double angle=0.3;
  pose.rotation[0][0]=pose.rotation[1][1]=std::cos(angle);
  pose.rotation[0][1]=-std::sin(angle);pose.rotation[1][0]=std::sin(angle);
  Vector3 world{},base{};
  CHECK(validBodyPose(pose)&&supportsFlatContactIk(pose));
  CHECK(pointToWorld(pose,{0.1,0.2,-0.3},world)&&pointToBase(pose,world,base));
  CHECK(near(base.x,0.1)&&near(base.y,0.2)&&near(base.z,-0.3));
  BodyPose aliased=pose;
  CHECK(pointToWorld(aliased,{0.1,0.2,-0.3},aliased.translationM));
  CHECK(near(aliased.translationM.x,world.x)&&near(aliased.translationM.y,world.y)&&near(aliased.translationM.z,world.z));
  auto contact=contactForwardKinematics(LegId::LF,o.joints.legs[0]);
  CHECK(pointToWorld(pose,contact.referenceM,world));
  ContactIkOptions options;options.kinematics.seed=o.joints.legs[0];
  CHECK(worldContactInverseKinematics(LegId::LF,world,pose,options).status==WorldContactIkResult::Status::OK);
  pose={};pose.rotation[1][1]=pose.rotation[2][2]=std::cos(angle);
  pose.rotation[1][2]=-std::sin(angle);pose.rotation[2][1]=std::sin(angle);
  CHECK(validBodyPose(pose)&&!supportsFlatContactIk(pose));
  CHECK(worldContactInverseKinematics(LegId::LF,world,pose).status==WorldContactIkResult::Status::UNSUPPORTED_ORIENTATION);
  CHECK(pointToWorld(pose,{1,2,3},world)&&pointToBase(pose,world,base));
  CHECK(near(base.x,1)&&near(base.y,2)&&near(base.z,3));
  pose={};pose.rotation[0][0]=-1;CHECK(!validBodyPose(pose));
  pose={};pose.translationM.x=nan;CHECK(!pointToWorld(pose,{},world));
  for(double duration:{0.,-1.,nan,1e-300,1e300})CHECK(!validTiming({duration,50}));
  CHECK(!validTiming({5,1})&&!validTiming({5,1000001}));
  CHECK(near(timingFromPeriod(0.1,50).durationS,5));
  CHECK(!validTiming(timingFromPeriod(nan,50)));
  CHECK(!quinticTimeLaw(-0.1,5).valid&&!quinticTimeLaw(1.1,5).valid);
  CHECK(!sampleStandPath(d,-0.1,o.joints.legs).target.valid);
  for(double u:{0.,1.}) {auto law=quinticTimeLaw(u,5);CHECK(law.valid&&law.progress==u&&law.velocity==0&&law.acceleration==0);}
  double previous=-1;
  for(unsigned i=0;i<=1000;++i) {auto law=quinticTimeLaw(i/1000.,5);CHECK(law.valid&&law.progress>=previous);previous=law.progress;}
  TimedStand two,five,ten,repeat;
  CHECK(two.initialize(d,{2,100})==TimedStatus::OK);CHECK(five.initialize(d,{5,100})==TimedStatus::OK);
  CHECK(ten.initialize(d,{10,100})==TimedStatus::OK);CHECK(repeat.initialize(d,{5,100})==TimedStatus::OK);
  double lastTime=-1;
  for(unsigned i=0;i<=100;++i) {
    const auto a=two.next(),b=five.next(),c=ten.next(),r=repeat.next();
    CHECK(a.status==TimedStatus::OK&&b.status==TimedStatus::OK&&c.status==TimedStatus::OK);
    CHECK(b.timeS>lastTime);lastTime=b.timeS;
    CHECK(b.geometry.target.sequence==i&&b.progress==r.progress&&b.timeS==r.timeS);
    CHECK(b.geometry.maxContactResidualM<1e-9);
    CHECK(b.phase==(i==0?TrajectoryPhase::ACQUISITION_HOLD:TrajectoryPhase::CONTACT_LOCKED_RISE));
    for(unsigned leg=0;leg<4;++leg)for(unsigned j=0;j<3;++j) {
      CHECK(value(a.geometry.target.legs[leg],j)==value(b.geometry.target.legs[leg],j));
      CHECK(value(b.geometry.target.legs[leg],j)==value(c.geometry.target.legs[leg],j));
      CHECK(value(b.velocityRadS[leg],j)==value(r.velocityRadS[leg],j));
      CHECK(value(b.accelerationRadS2[leg],j)==value(r.accelerationRadS2[leg],j));
      CHECK(near(value(a.velocityRadS[leg],j)*2,value(b.velocityRadS[leg],j)*5));
      CHECK(near(value(b.velocityRadS[leg],j)*5,value(c.velocityRadS[leg],j)*10));
      CHECK(near(value(a.accelerationRadS2[leg],j)*4,value(b.accelerationRadS2[leg],j)*25));
      CHECK(near(value(b.accelerationRadS2[leg],j)*25,value(c.accelerationRadS2[leg],j)*100));
      if(i==0||i==100)CHECK(value(b.velocityRadS[leg],j)==0&&value(b.accelerationRadS2[leg],j)==0);
    }
    if(i==0||i==100) {
      auto exact=sampleStandPath(d,i==0?0:1,o.joints.legs);
      for(unsigned leg=0;leg<4;++leg)for(unsigned j=0;j<3;++j)
        CHECK(near(value(exact.target.legs[leg],j),value(b.geometry.target.legs[leg],j),1e-12));
    }
  }
  CHECK(five.next().status==TimedStatus::COMPLETE);
  for(unsigned i=0;i<4;++i)CHECK(five.metrics().branchChanges[i]==0);
  // All original G2 geometric samples are exactly the same path under timing.
  ContactLockedStand geometric;CHECK(geometric.initialize(d)==StandStatus::OK);
  for(unsigned i=0;i<51;++i) {
    auto a=geometric.next(),b=sampleStandPath(d,i/50.,o.joints.legs);
    for(unsigned leg=0;leg<4;++leg)for(unsigned j=0;j<3;++j)
      CHECK(near(value(a.target.legs[leg],j),value(b.target.legs[leg],j),1e-12));
  }
  // Full public state/event matrix. Entry/completion are now private verified operations.
  const int table[5][8]={{1,0,-1,-1,-1,-1,0,-1},{-1,0,-1,-1,-1,-1,0,-1},
                        {-1,0,-1,-1,4,-1,0,-1},{-1,0,-1,-1,4,-1,0,-1},
                        {-1,0,-1,-1,-1,1,0,-1}};
  for(unsigned s=0;s<5;++s)for(unsigned e=0;e<8;++e) {
    auto t=at(static_cast<MotionState>(s));CHECK(t.state()==static_cast<MotionState>(s));
    CHECK(t.apply(static_cast<MotionEvent>(e))==(table[s][e]>=0));
    CHECK(t.state()==static_cast<MotionState>(table[s][e]<0?s:table[s][e]));
    if(e==1||e==4||e==6)CHECK(!t.next().geometry.target.valid);
  }
  StandTransition transition;CHECK(!transition.begin(o,{5,50}));
  CHECK(transition.apply(MotionEvent::ENABLE));CHECK(!transition.begin(bad,{5,50}));
  CHECK(transition.state()==MotionState::IDLE);
  CHECK(!transition.begin(o,{0,50}));CHECK(!transition.apply(MotionEvent::STAND_COMPLETE));
  CHECK(transition.begin(o,{5,50}));CHECK(!transition.begin(o,{5,50}));
  for(unsigned i=0;i<51;++i) {
    CHECK(!transition.apply(MotionEvent::STAND_COMPLETE));
    CHECK(transition.next().geometry.target.valid);
    CHECK(transition.state()==(i<50?MotionState::STAND_TRANSITION:MotionState::STAND));
  }
  CHECK(!transition.next().geometry.target.valid);
  auto stopped=at(MotionState::STAND_TRANSITION);
  stopped.next();CHECK(stopped.apply(MotionEvent::STOP));CHECK(!stopped.next().geometry.target.valid);
  CHECK(stopped.apply(MotionEvent::STOP_COMPLETE));CHECK(!stopped.apply(MotionEvent::BEGIN_STAND));
  CHECK(stopped.begin(o,{5,2}));CHECK(stopped.next().timeS==0);
  // Numerically impossible interior rates must fail, never advance to STAND.
  StandTransition overflow;overflow.apply(MotionEvent::ENABLE);
  if(overflow.begin(o,{1e-154,4})) {
    CHECK(overflow.next().status==TimedStatus::OK);
    CHECK(!overflow.next().geometry.target.valid);CHECK(overflow.state()==MotionState::OFF);
  } else CHECK(overflow.state()==MotionState::IDLE);
  std::printf("STARTUP_TIMED_STAND: %u checks, %u failures; 40 state/event pairs\n",checks,failures);
  return failures?1:0;
}
