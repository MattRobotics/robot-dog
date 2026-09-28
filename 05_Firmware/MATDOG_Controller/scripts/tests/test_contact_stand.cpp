#include "../../src/motion/FootContact.h"
#include "../../src/motion/StandTrajectory.h"
#include "../../src/motion/MotionState.h"
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>
using namespace matdog::motion;
static unsigned checks=0, failures=0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; std::printf("FAIL %d: %s\n",__LINE__,#c); } } while(0)
static double distance(Vector3 a,Vector3 b) { return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z); }
static MotionStateMachine at(MotionState state) {
  MotionStateMachine m;
  if (state == MotionState::OFF) return m;
  m.apply(MotionEvent::ENABLE);
  if (state == MotionState::IDLE) return m;
  m.apply(MotionEvent::BEGIN_STAND);
  if (state == MotionState::STAND_TRANSITION) return m;
  m.apply(MotionEvent::STAND_COMPLETE);
  if (state == MotionState::STAND) return m;
  m.apply(MotionEvent::STOP);
  return m;
}
int main() {
  const double nan=std::numeric_limits<double>::quiet_NaN();
  for (unsigned i=0;i<4;++i) {
    const auto leg=static_cast<LegId>(i);
    auto fk=contactForwardKinematics(leg,{});
    CHECK(fk.status==ContactStatus::OK);
    CHECK(fk.support==SupportMode::NOMINAL_STRIP);
    CHECK(distance(fk.referenceM,forwardKinematics(leg,{}).footOriginM)<1e-12);
    CHECK(std::abs(distance(fk.stripEndAM,fk.stripEndBM)-footContactModel().supportWidthM)<1e-12);
    for (double h : {0.0,0.01,0.2,-0.2}) {
      LegJointAngles q{h,0.6,-0.3};
      fk=contactForwardKinematics(leg,q);
      CHECK(fk.status==ContactStatus::OK);
      CHECK(distance(fk.referenceM,forwardKinematics(leg,q).footOriginM)>1e-4);
      ContactIkOptions options; options.kinematics.seed=q; options.requireNominalStrip=false;
      const auto ik=contactInverseKinematics(leg,fk.referenceM,options);
      CHECK(ik.status==ContactStatus::OK);
      CHECK(ik.residualM<1e-9);
      CHECK(std::abs(ik.joints.hip-q.hip)<1e-8);
      CHECK(std::abs(ik.joints.upper-q.upper)<1e-8);
      CHECK(std::abs(ik.joints.lower-q.lower)<1e-8);
      options.requireNominalStrip=true;
      CHECK(contactInverseKinematics(leg,fk.referenceM,options).status==
            (std::abs(h)<0.02?ContactStatus::OK:ContactStatus::CONTACT_MODE));
    }
    CHECK(contactForwardKinematics(leg,{nan,0,0}).status==ContactStatus::INVALID_INPUT);
    CHECK(contactForwardKinematics(leg,{0,0,0},{nan,0,1}).status==ContactStatus::INVALID_INPUT);
    CHECK(contactForwardKinematics(leg,{},{0,0,0}).status==ContactStatus::INVALID_INPUT);
    CHECK(contactForwardKinematics(leg,{},{0,1,0}).status==ContactStatus::DEGENERATE);
    CHECK(contactForwardKinematics(leg,{},{0,0,2}).status==ContactStatus::OK);
    CHECK(contactForwardKinematics(leg,{2,0,0}).status==ContactStatus::JOINT_LIMIT);
    CHECK(contactInverseKinematics(leg,{nan,0,0}).status==ContactStatus::INVALID_INPUT);
    CHECK(contactInverseKinematics(leg,{1e308,1e308,1e308}).status==ContactStatus::UNREACHABLE_GEOMETRY);
    const auto* m=legModel(leg);
    CHECK(contactInverseKinematics(leg,m->hipOrigin).status==ContactStatus::UNREACHABLE_GEOMETRY);
    const double a=-m->lowerOrigin.z;
    const double b=std::hypot(m->footOrigin.x,m->footOrigin.z+footContactModel().radiusM);
    const Vector3 extended={m->hipOrigin.x,m->hipOrigin.y+m->upperOrigin.y+m->footOrigin.y,
                           m->hipOrigin.z-a-b-footContactModel().radiusM};
    CHECK(contactInverseKinematics(leg,extended).status==ContactStatus::UNREACHABLE_LIMITS);
    ContactIkOptions bad;bad.kinematics.seed={2,0,0};
    CHECK(contactInverseKinematics(leg,fk.referenceM,bad).status==ContactStatus::JOINT_LIMIT);
    bad={};bad.kinematics.toleranceM=nan;
    CHECK(contactInverseKinematics(leg,fk.referenceM,bad).status==ContactStatus::INVALID_INPUT);
  }
  CHECK(contactForwardKinematics(static_cast<LegId>(255),{}).status==ContactStatus::INVALID_INPUT);
  CHECK(contactInverseKinematics(static_cast<LegId>(255),{}).status==ContactStatus::INVALID_INPUT);
  const auto d=canonicalStandDefinition();
  const auto stand=generateStandTarget(d);
  CHECK(stand.status==StandStatus::OK && stand.target.valid);
  CHECK(stand.maxContactResidualM<1e-9);
  ContactLockedStand a,b;
  CHECK(a.next().status==StandStatus::NOT_READY);
  CHECK(a.initialize(d)==StandStatus::OK);
  CHECK(b.initialize(d)==StandStatus::OK);
  MotionStateMachine lifecycle;
  CHECK(lifecycle.apply(MotionEvent::ENABLE));
  CHECK(lifecycle.apply(MotionEvent::BEGIN_STAND));
  for (unsigned index=0;index<51;++index) {
    const auto x=a.next(),y=b.next();
    CHECK(x.status==StandStatus::OK && x.target.valid);
    CHECK(x.target.sequence==index);
    CHECK(x.bodyHeightM==y.bodyHeightM);
    CHECK(std::abs(x.bodyHeightM-(0.1+index*0.001))<1e-12);
    CHECK(x.maxContactResidualM<1e-9 && x.maxContactDriftM<1e-9);
    for (unsigned leg=0;leg<4;++leg) {
      const auto q=x.target.legs[leg],r=y.target.legs[leg];
      CHECK(q.hip==r.hip && q.upper==r.upper && q.lower==r.lower);
      CHECK(x.branches[leg].hip==-1 && x.branches[leg].elbow==-1);
      CHECK(x.jointLimitMarginRad[leg].hip>=0 && x.jointLimitMarginRad[leg].upper>=0 && x.jointLimitMarginRad[leg].lower>=0);
      CHECK(distance(x.contactsWorldM[leg],d.contactsWorldM[leg])<1e-9);
      // Geometric continuity regression threshold for this 1-mm height grid;
      // the archived maximum is 0.022529 rad (checked independently in Python).
      // This is NOT a hardware velocity/rate limit.
      CHECK(std::abs(x.jointDeltaRad[leg].hip)<0.025);
      CHECK(std::abs(x.jointDeltaRad[leg].upper)<0.025);
      CHECK(std::abs(x.jointDeltaRad[leg].lower)<0.025);
    }
  }
  CHECK(a.next().status==StandStatus::COMPLETE && !a.next().target.valid);
  CHECK(lifecycle.apply(MotionEvent::STAND_COMPLETE));
  CHECK(lifecycle.state()==MotionState::STAND);
  CHECK(a.metrics().emittedSamples==51);
  CHECK(a.metrics().maxJointDeltaRad<0.025);
  CHECK(a.metrics().minJointLimitMarginRad>0);
  for(unsigned i=0;i<4;++i) CHECK(a.metrics().branchChanges[i]==0);
  CHECK(lifecycle.apply(MotionEvent::STOP));a.stop();
  CHECK(a.next().status==StandStatus::NOT_READY);
  CHECK(lifecycle.apply(MotionEvent::STOP_COMPLETE));
  CHECK(lifecycle.state()==MotionState::IDLE);
  CHECK(a.initialize(d)==StandStatus::OK);
  CHECK(a.next().target.sequence==0);
  a.stop(); CHECK(!a.next().target.valid);
  // Invalid configuration and late-leg failure must never expose partial frames.
  for(unsigned mutation=0;mutation<6;++mutation) {
    auto bad=d;
    if(mutation==0) bad.samples=1;
    if(mutation==1) bad.lowBodyHeightM=bad.standBodyHeightM;
    if(mutation==2) bad.standBodyHeightM=nan;
    if(mutation==3) bad.contactsWorldM[3].z=0.001;
    if(mutation==4) bad.standSeed[3].lower=nan;
    if(mutation==5) bad.contactsWorldM[3].x=10;
    const auto failure=generateStandTarget(bad);
    CHECK(!failure.target.valid);
    CHECK(failure.target.legs[0].hip==0 && failure.target.legs[0].upper==0 && failure.target.legs[0].lower==0);
    CHECK(a.initialize(bad)!=StandStatus::OK);
    CHECK(!a.next().target.valid);
  }
  // Exhaustive state/event matrix, including invalid enum; rejects leave state intact.
  for(unsigned s=0;s<5;++s) for(unsigned e=0;e<8;++e) {
    const auto state=static_cast<MotionState>(s);
    const auto event=static_cast<MotionEvent>(e);
    auto machine=at(state);
    const bool accepted=machine.apply(event);
    const int table[5][8]={{1,0,-1,-1,-1,-1,0,-1},{-1,0,2,-1,-1,-1,0,-1},
                          {-1,0,-1,3,4,-1,0,-1},{-1,0,-1,-1,4,-1,0,-1},
                          {-1,0,-1,-1,-1,1,0,-1}};
    CHECK(accepted==(table[s][e]>=0));
    CHECK(machine.state()==(table[s][e]<0?state:static_cast<MotionState>(table[s][e])));
  }
  std::printf("CONTACT_STAND: %u checks, %u failures (40 state/event pairs)\n",checks,failures);
  return failures?1:0;
}
