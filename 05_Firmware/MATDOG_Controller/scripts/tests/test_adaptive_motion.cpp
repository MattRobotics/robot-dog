#include "motion/AdaptiveStand.h"
#include "motion/FeedbackWalk.h"
#include "motion/PoseSupport.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
using namespace matdog::motion;
static unsigned checks=0;
#define CHECK(x) do {++checks;if(!(x)){std::fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);std::abort();}}while(0)
FeedbackPolicy simulationPolicy(){
  FeedbackPolicy p;p.mode=FeedbackMode::SIMULATION;p.envelopesApproved=true;
  p.maxJointStepRad=.10;p.maxTrackingErrorRad=.02;p.observationMaxAgeS=.15;p.assessmentMaxAgeS=.15;
  p.segmentTimeoutS=.5;p.heightToleranceM=.001;p.contactToleranceM=.001;p.samplePeriodS=.04;
  for(unsigned i=0;i<4;++i){const auto m=legModel(static_cast<LegId>(i));for(unsigned j=0;j<3;++j)p.jointLimits[i][j]=m->limits[j];
    p.maxVelocityRadS[i]={2,2,2};p.maxAccelerationRadS2[i]={12,12,12};}
  return p;
}
AdaptiveObservation observation(const BodyPose& b,const JointTargetFrame& q,double now,uint8_t feet,bool belly){
  AdaptiveObservation o;o.body=b;o.actual=q;o.stampS=now;o.confirmedFeet=feet;o.bellySupported=belly;
  o.heightSource=o.supportSource=EvidenceSource::SYNTHETIC;o.flatPlaneConfirmed=true;o.stationary=true;return o;
}
AdaptiveObservation supported(double height,double now){
  GaitParameters p;CartesianSample c=gaitHold(p,0,height,0);auto f=solveGait(p,c);
  CHECK(f.target.valid);return observation(c.body,f.target,now,15,false);
}
AcquisitionRoute simulationRoute(AdaptiveObservation& o,double now){
  // Canonical deep-folded REST_GROUND: flat belly plane height derived from
  // the actual base collision mesh. Foot contact references begin in free space.
  const auto* rest=researchedPose(0);CHECK(rest!=nullptr);
  o=observation(rest->body,rest->joints,now,0,true);
  o.externalSupportConfirmed=true; // explicit synthetic fixture for the transfer candidate
  const auto observed=observedSolution(o);CHECK(observed.target.valid);
  AcquisitionRoute r;CHECK(restGroundAcquisitionCandidate(r));CHECK(r.count<=AcquisitionRoute::kMaxWaypoints);
  CHECK(r.waypoints[2].requiredFeet==0&&r.waypoints[2].externalSupportRequired);
  CHECK(r.waypoints[1].feetWorldM[2].z>0&&r.waypoints[1].feetWorldM[3].z>0);return r;
}
SegmentAssessment syntheticAssessment(const FeedbackFrame& f,double now){
  SegmentAssessment a;a.sequence=f.target.sequence;a.stampS=now;a.source=EvidenceSource::SYNTHETIC;
  a.geometryIdentityVerified=a.sweptCollisionClear=a.supportCompatible=a.continuousBoundsConfirmed=true;return a;
}
void follow(AdaptiveObservation& o,const FeedbackFrame& f,double now){
  o.actual=f.target;o.body=f.body;o.stampS=now;o.acknowledged=true;o.acknowledgedSequence=f.target.sequence;
  o.bellySupported=f.bellySupportRequired;o.confirmedFeet=0;
  // Synthetic fixture persists through deployment/transfer, then releases once
  // the 65 mm four-foot handoff is reached; this is not a measured load proof.
  o.externalSupportConfirmed=(o.externalSupportConfirmed&&f.body.translationM.z<.065)||f.externalSupportRequired;
  for(unsigned i=0;i<4;++i)if(std::abs(f.feetWorldM[i].z)<1e-8)o.confirmedFeet|=1<<i;
  CHECK(observedSolution(o).target.valid); // independent posterior FK -> G2 IK reconstruction
}
std::ofstream trace;
void emit(const AdaptiveObservation& o,const FeedbackFrame& f,const char* phase,double now){
  if(!trace)return;
  trace<<"{\"version\":1,\"now_ms\":"<<static_cast<unsigned long>(now*1000)<<",\"executor_state\":\"SIMULATION\",\"stand_state\":\""<<phase<<"\",\"telemetry\":{\"actual_rad\":[";
  for(unsigned i=0;i<4;++i){auto q=o.actual.legs[i];if(i)trace<<",";trace<<q.hip<<","<<q.upper<<","<<q.lower;}
  trace<<"],\"target_rad\":[";
  for(unsigned i=0;i<4;++i){auto q=f.target.valid?f.target.legs[i]:o.actual.legs[i];if(i)trace<<",";trace<<q.hip<<","<<q.upper<<","<<q.lower;}
  const double yaw=std::atan2(o.body.rotation[1][0],o.body.rotation[0][0]);
  trace<<"],\"quaternion\":["<<std::cos(yaw/2)<<",0,0,"<<std::sin(yaw/2)<<"],\"base_height_m\":"<<o.body.translationM.z
       <<",\"base_position_m\":["<<o.body.translationM.x<<","<<o.body.translationM.y<<","<<o.body.translationM.z<<"],\"feet\":{";
  const char* names[]={"LF","RF","RH","LH"};
  auto actual=observedSolution(o);
  for(unsigned i=0;i<4;++i){if(i)trace<<",";auto foot=actual.cartesian.feet[i];trace<<"\""<<names[i]<<"\":{\"position_m\":["<<foot.x<<","<<foot.y<<","<<foot.z<<"],\"state\":\""<<((o.confirmedFeet&(1<<i))?"STANCE":"SWING")<<"\"}";}
  const char* evidence=o.externalSupportConfirmed?"SYNTHETIC_NOT_HARDWARE_QUALIFIED;EXTERNAL_FIXTURE_SUPPORT":"SYNTHETIC_NOT_HARDWARE_QUALIFIED";
  trace<<"},\"external_support_confirmed\":"<<(o.externalSupportConfirmed?"true":"false")<<",\"tracking_error_rad\":0,\"fault\":null,\"stamp_ms\":"<<static_cast<unsigned long>(o.stampS*1000)<<"},\"reason\":\""<<evidence<<"\",\"evidence\":\""<<evidence<<"\"}\n";
}
unsigned completeStand(AdaptiveStand& s,AdaptiveObservation& o,const FeedbackPolicy& p,double& now,const char* label){
  unsigned frames=0;for(unsigned guard=0;guard<10000&&s.state()!=AdaptiveStandState::STAND;++guard){
    o.stampS=now;auto f=s.update(o,now);
    if(s.state()==AdaptiveStandState::REJECTED){std::fprintf(stderr,"%s rejected: %s at t %.3f\n",label,feedbackReasonName(s.reason()),now);CHECK(false);}
    if(f.target.valid){CHECK(!f.executable);CHECK(s.assess(syntheticAssessment(f,now),now));
      auto approved=s.update(o,now);CHECK(approved.target.valid&&approved.executable);CHECK(!approved.physicalReadiness);
      emit(o,approved,adaptiveStandStateName(s.state()),now);now+=p.samplePeriodS;follow(o,approved,now);++frames;
    }else{emit(o,f,adaptiveStandStateName(s.state()),now);now+=p.samplePeriodS;}}
  CHECK(s.state()==AdaptiveStandState::STAND);CHECK(std::abs(o.body.translationM.z-.150)<1e-8);return frames;
}
void rejectionTests(){
  const auto p=simulationPolicy();CHECK(validFeedbackPolicy(p));CHECK(!validFeedbackPolicy(FeedbackPolicy{}));
  auto o=supported(.125,0);AdaptiveStand s;CHECK(s.request({},p,0));o.heightSource=EvidenceSource::UNKNOWN;
  CHECK(s.update(o,0).reason==FeedbackReason::UNOBSERVABLE_HEIGHT);CHECK(s.state()==AdaptiveStandState::REJECTED);
  o=supported(.125,0);CHECK(s.request({},p,0));o.supportSource=EvidenceSource::UNKNOWN;CHECK(s.update(o,0).reason==FeedbackReason::UNOBSERVABLE_SUPPORT);
  o=supported(.125,0);CHECK(s.request({},p,0));CHECK(s.update(o,.16).reason==FeedbackReason::STALE_OBSERVATION);
  o=supported(.125,0);CHECK(s.request({},p,0));s.update(o,0);auto f=s.update(o,0);CHECK(f.target.valid&&!f.executable);
  o.stampS=.04;auto same=s.update(o,.04);CHECK(same.target.sequence==f.target.sequence&&!same.executable);
  auto bad=syntheticAssessment(f,.04);bad.sweptCollisionClear=false;CHECK(!s.assess(bad,.04));CHECK(s.reason()==FeedbackReason::COLLISION);
  CHECK(s.request({},p,0));o.stampS=0;s.update(o,0);f=s.update(o,0);auto a=syntheticAssessment(f,0);a.sequence++;CHECK(!s.assess(a,0));CHECK(s.reason()==FeedbackReason::ASSESSMENT_STALE);
  CHECK(s.request({},p,0));s.update(o,0);f=s.update(o,0);a=syntheticAssessment(f,0);a.continuousBoundsConfirmed=false;CHECK(!s.assess(a,0));CHECK(s.reason()==FeedbackReason::CONTINUOUS_BOUNDS_MISSING);
  CHECK(s.request({},p,0));s.update(o,0);f=s.update(o,0);CHECK(s.assess(syntheticAssessment(f,0),0));o.actual.legs[0].upper+=.04;o.acknowledged=true;o.acknowledgedSequence=f.target.sequence;
  o.stampS=.04;CHECK(s.update(o,.04).reason==FeedbackReason::TRACKING_ERROR);
  o=supported(.125,0);CHECK(s.request({},p,0));s.update(o,0);f=s.update(o,0);o.stampS=.51;CHECK(s.update(o,.51).reason==FeedbackReason::SEGMENT_TIMEOUT);
  auto limits=p;limits.jointLimits[0][1]={-.2,.2};CHECK(s.request({},limits,0));o=supported(.125,0);CHECK(s.update(o,0).reason==FeedbackReason::JOINT_LIMIT);
  CHECK(s.request({},p,0));o=supported(.125,0);o.inhibited=true;CHECK(s.update(o,0).reason==FeedbackReason::INHIBITED);
  auto physical=p;physical.mode=FeedbackMode::PHYSICAL_REVIEW;physical.loadedRatesVerified=true;CHECK(s.request({},physical,0));o=supported(.125,0);CHECK(s.update(o,0).reason==FeedbackReason::UNOBSERVABLE_HEIGHT);
  auto rest=researchedPose(0);o=observation(rest->body,rest->joints,0,0,true);CHECK(s.request({},p,0));s.update(o,0);CHECK(s.update(o,0).reason==FeedbackReason::ROUTE_REQUIRED);
  CHECK(!s.request(AdaptiveStandRequest{.151,8,1,nullptr},p,0));
  // Each internal segment gate must independently deny generation.
  auto restrictive=p;restrictive.maxJointStepRad=1e-10;o=supported(.117,0);
  CHECK(s.request({},restrictive,0));s.update(o,0);CHECK(s.update(o,0).reason==FeedbackReason::CONTINUITY);
  restrictive=p;for(auto& v:restrictive.maxVelocityRadS)v={1e-8,1e-8,1e-8};
  CHECK(s.request({},restrictive,0));s.update(o,0);CHECK(s.update(o,0).reason==FeedbackReason::VELOCITY_LIMIT);
  restrictive=p;for(auto& v:restrictive.maxAccelerationRadS2)v={1e-8,1e-8,1e-8};
  CHECK(s.request({},restrictive,0));s.update(o,0);CHECK(s.update(o,0).reason==FeedbackReason::ACCELERATION_LIMIT);
  CHECK(s.request({},p,0));s.update(o,0);f=s.update(o,0);CHECK(s.assess(syntheticAssessment(f,0),0));
  follow(o,f,0);auto tooEarly=s.update(o,0);CHECK(tooEarly.target.sequence==f.target.sequence&&tooEarly.reason==FeedbackReason::FEEDBACK_PENDING);
  o.stampS=.2;CHECK(s.update(o,.2).reason==FeedbackReason::ASSESSMENT_STALE);
  o=supported(.117,0);CHECK(s.request({},p,0));s.update(o,0);f=s.update(o,0);CHECK(s.assess(syntheticAssessment(f,0),0));
  follow(o,f,.04);o.body.translationM.x+=.02;CHECK(s.update(o,.04).reason==FeedbackReason::TRACKING_ERROR);
  o=supported(.117,0);o.heightSource=o.supportSource=EvidenceSource::EXTERNAL_MEASURED;
  CHECK(s.request({},physical,0));s.update(o,0);f=s.update(o,0);CHECK(f.target.valid);CHECK(!s.assess(syntheticAssessment(f,0),0));CHECK(s.reason()==FeedbackReason::ASSESSMENT_REQUIRED);
  o=supported(.117,0);CHECK(s.request({},p,0));s.update(o,0);s.stop();CHECK(!s.update(o,0).target.valid&&s.state()==AdaptiveStandState::STOPPED);
  o=supported(.117,0);CHECK(s.request({},p,0));s.update(o,0);f=s.update(o,0);follow(o,f,.04);
  f.externalSupportRequired=true;CHECK(checkTracking(o,f,p)==FeedbackReason::SUPPORT_INVALID);
  o.externalSupportConfirmed=true;CHECK(checkTracking(o,f,p)==FeedbackReason::OK);
  // The fixture-supported candidate cannot emit its first lift target without
  // explicitly observed external support; belly contact alone is insufficient.
  double clock=0;auto route=simulationRoute(o,clock);o.externalSupportConfirmed=false;
  AdaptiveStandRequest request;request.acquisition=&route;CHECK(s.request(request,p,clock));unsigned deployed=0;
  for(unsigned guard=0;guard<110&&s.state()!=AdaptiveStandState::REJECTED;++guard){
    o.stampS=clock;auto frame=s.update(o,clock);
    if(frame.target.valid){CHECK(!frame.externalSupportRequired);CHECK(s.assess(syntheticAssessment(frame,clock),clock));
      clock+=p.samplePeriodS;follow(o,frame,clock);++deployed;}
    else clock+=p.samplePeriodS;
  }
  CHECK(deployed==0&&s.state()==AdaptiveStandState::REJECTED&&s.reason()==FeedbackReason::SUPPORT_INVALID);
  // Fixture revocation also rejects a pending transfer before its acknowledgement.
  clock=0;route=simulationRoute(o,clock);request.acquisition=&route;CHECK(s.request(request,p,clock));bool revoked=false;
  for(unsigned guard=0;guard<110;++guard){
    o.stampS=clock;auto frame=s.update(o,clock);
    if(frame.target.valid&&frame.externalSupportRequired){o.externalSupportConfirmed=false;o.acknowledged=false;
      CHECK(s.update(o,clock).reason==FeedbackReason::SUPPORT_INVALID);revoked=true;break;}
    if(frame.target.valid){CHECK(s.assess(syntheticAssessment(frame,clock),clock));clock+=p.samplePeriodS;follow(o,frame,clock);}
    else clock+=p.samplePeriodS;
  }
  CHECK(revoked&&s.state()==AdaptiveStandState::REJECTED);
  // VERIFY_STAND cannot wait indefinitely for a stationary support observation.
  clock=0;o=supported(.117,clock);CHECK(s.request({},p,clock));
  for(unsigned guard=0;guard<210&&s.state()!=AdaptiveStandState::VERIFY_STAND;++guard){
    o.stampS=clock;auto frame=s.update(o,clock);
    if(frame.target.valid){CHECK(s.assess(syntheticAssessment(frame,clock),clock));clock+=p.samplePeriodS;follow(o,frame,clock);
      if(std::abs(frame.body.translationM.z-.150)<1e-10)o.stationary=false;}
    else clock+=p.samplePeriodS;
  }
  CHECK(s.state()==AdaptiveStandState::VERIFY_STAND&&s.reason()==FeedbackReason::VERIFY_PENDING);
  clock+=p.segmentTimeoutS+.01;o.stampS=clock;CHECK(s.update(o,clock).reason==FeedbackReason::SEGMENT_TIMEOUT);
  // After completion, support/stationarity/height loss revokes STAND immediately.
  for(unsigned lost=0;lost<3;++lost){
    o=supported(.150,0);CHECK(s.request({},p,0));s.update(o,0);s.update(o,0);CHECK(s.state()==AdaptiveStandState::STAND);
    if(lost==0)o.confirmedFeet=14;else if(lost==1)o.stationary=false;else o.body.translationM.z+=.002;
    o.stampS=.04;auto frame=s.update(o,.04);CHECK(!frame.target.valid&&s.state()==AdaptiveStandState::REJECTED);
    CHECK(frame.reason==(lost==0?FeedbackReason::SUPPORT_INVALID:FeedbackReason::TRACKING_ERROR));CHECK(!s.physicalReadiness());
  }
  // Losing an already held foot rejects immediately, even before target ACK.
  o=supported(.117,0);CHECK(s.request({},p,0));s.update(o,0);f=s.update(o,0);
  CHECK(f.target.valid);o.confirmedFeet=14;o.acknowledged=false;
  CHECK(s.update(o,0).reason==FeedbackReason::SUPPORT_INVALID&&s.state()==AdaptiveStandState::REJECTED);
  o=supported(.150,0);CHECK(s.request({},p,0));s.update(o,0);s.update(o,0);
  WalkSpecification ws;ws.periodS=6;ws.rampDurationS=3;ws.liftM=.005;ws.duty=.8;
  ws.maxVxMps=ws.maxVyMps=.005;ws.maxYawRateRadS=.01;ws.maxCommandTtlS=.5;
  FeedbackWalk walk;PlanarVelocityCommand cmd;cmd.vxMps=.001;cmd.ttlS=.5;cmd.sequence=1;
  CHECK(walk.start(s,o,cmd,ws,p,0));f=walk.update(o,0);CHECK(f.target.valid);
  o.confirmedFeet=14;o.acknowledged=false;CHECK(walk.update(o,0).reason==FeedbackReason::SUPPORT_INVALID);
  CHECK(walk.state()==FeedbackWalkState::REJECTED);
  o=supported(.150,0);o.bellySupported=true;FeedbackWalk bellyWalk;
  CHECK(!bellyWalk.start(s,o,cmd,ws,p,0)&&bellyWalk.reason()==FeedbackReason::SUPPORT_INVALID);
  o.bellySupported=false;CHECK(bellyWalk.start(s,o,cmd,ws,p,0));
  f=bellyWalk.update(o,0);CHECK(f.target.valid);o.bellySupported=true;o.acknowledged=false;
  CHECK(bellyWalk.update(o,0).reason==FeedbackReason::SUPPORT_INVALID);
  // A foot that is only a future touchdown must not be demanded before ACK.
  o=supported(.150,0);auto previous=observedSolution(o);previous.cartesian.phases[0].stance=false;
  f=feedbackFrame(previous,1);f.requiredFeet=15;o.confirmedFeet=14;
  CHECK(checkHeldSupport(o,previous,f)==FeedbackReason::OK);
  previous.cartesian.phases[0].stance=true;CHECK(checkHeldSupport(o,previous,f)==FeedbackReason::SUPPORT_INVALID);

}
void partialSupportTests(){
  const auto policy=simulationPolicy();
  for(unsigned mask=1;mask<15;++mask){
    auto o=supported(.117,0);o.confirmedFeet=static_cast<uint8_t>(mask);
    AdaptiveStand stand;CHECK(stand.request({},policy,0));stand.update(o,0);
    auto noRoute=stand.update(o,0);CHECK(!noRoute.target.valid&&stand.reason()==FeedbackReason::ROUTE_REQUIRED);
    CHECK(stand.poseClass()==ObservedPoseClass::PARTIAL_SUPPORT);
    auto solved=observedSolution(o);CHECK(solved.target.valid);
    AcquisitionRoute route;route.geometryIdentityVerified=true;route.source=EvidenceSource::SYNTHETIC;route.count=2;
    route.waypoints[0].body=o.body;route.waypoints[0].requiredFeet=o.confirmedFeet;
    for(unsigned leg=0;leg<4;++leg)route.waypoints[0].feetWorldM[leg]=solved.cartesian.feet[leg];
    route.waypoints[1]=route.waypoints[0];route.waypoints[1].requiredFeet=15;route.waypoints[1].durationS=1;
    AdaptiveStandRequest request;request.acquisition=&route;
    CHECK(stand.request(request,policy,0));stand.update(o,0);auto target=stand.update(o,0);
    CHECK(target.target.valid&&stand.poseClass()==ObservedPoseClass::PARTIAL_SUPPORT);
    CHECK(!target.executable&&!target.physicalReadiness);
    // Contact acquisition is not manufactured from the unchanged XYZ target.
    CHECK(stand.assess(syntheticAssessment(target,0),0));
    o.acknowledged=true;o.acknowledgedSequence=target.target.sequence;o.stampS=.04;
    CHECK(stand.update(o,.04).reason==FeedbackReason::SUPPORT_INVALID);
    o=supported(.117,0);o.confirmedFeet=static_cast<uint8_t>(mask);
    route.waypoints[0].requiredFeet=static_cast<uint8_t>(mask^1u);
    CHECK(stand.request(request,policy,0));stand.update(o,0);auto mismatch=stand.update(o,0);
    CHECK(!mismatch.target.valid&&stand.reason()==FeedbackReason::ROUTE_MISMATCH);
  }
  // Four confirmed foot contacts still cannot bypass ventral load transfer.
  auto o=supported(.117,0);o.bellySupported=true;AdaptiveStand stand;
  CHECK(stand.request({},policy,0));stand.update(o,0);auto refused=stand.update(o,0);
  CHECK(!refused.target.valid&&stand.reason()==FeedbackReason::ROUTE_REQUIRED);
  CHECK(stand.poseClass()==ObservedPoseClass::BELLY_FOLDED);
  o=supported(.150,0);CHECK(stand.request({},policy,0));stand.update(o,0);stand.update(o,0);
  CHECK(stand.state()==AdaptiveStandState::STAND);o.bellySupported=true;o.stampS=.04;
  CHECK(stand.update(o,.04).reason==FeedbackReason::SUPPORT_INVALID);
  // C3 must release belly support before C4, even after all foot ACKs arrive.
  o=supported(.117,0);o.confirmedFeet=14;auto solved=observedSolution(o);
  AcquisitionRoute route;route.geometryIdentityVerified=true;route.source=EvidenceSource::SYNTHETIC;route.count=2;
  route.waypoints[0].body=o.body;route.waypoints[0].requiredFeet=14;
  for(unsigned leg=0;leg<4;++leg)route.waypoints[0].feetWorldM[leg]=solved.cartesian.feet[leg];
  route.waypoints[1]=route.waypoints[0];route.waypoints[1].requiredFeet=15;route.waypoints[1].durationS=.08;
  AdaptiveStandRequest request;request.acquisition=&route;CHECK(stand.request(request,policy,0));
  stand.update(o,0);auto target=stand.update(o,0);CHECK(target.target.valid);
  CHECK(stand.assess(syntheticAssessment(target,0),0));follow(o,target,.04);
  target=stand.update(o,.04);CHECK(target.target.valid);
  CHECK(stand.assess(syntheticAssessment(target,.04),.04));follow(o,target,.08);o.bellySupported=true;
  CHECK(stand.update(o,.08).reason==FeedbackReason::SUPPORT_INVALID);
}
int main(int argc,char** argv){
  partialSupportTests();
  if(argc>1){trace.open(argv[1]);trace<<std::setprecision(17);}
  rejectionTests();const auto p=simulationPolicy();double now=0;AdaptiveObservation o;
  auto route=simulationRoute(o,now);AdaptiveStand s;AdaptiveStandRequest r;r.acquisition=&route;
  CHECK(s.request(r,p,now));auto n=completeStand(s,o,p,now,"legacy fixture acquisition");std::printf("Legacy synthetic fixture route (not geometry-approved) -> STAND 150mm: %u feedback frames\n",n);
  CHECK(!s.physicalReadiness());auto nominal=s.update(o,now);CHECK(!nominal.target.valid);CHECK(s.state()==AdaptiveStandState::STAND);
  AdaptiveStand confirmation;CHECK(confirmation.request({},p,now));confirmation.update(o,now);auto already=confirmation.update(o,now);CHECK(!already.target.valid&&confirmation.state()==AdaptiveStandState::STAND);
  FeedbackWalk walk;WalkSpecification spec;spec.periodS=6;spec.rampDurationS=3;spec.liftM=.005;spec.duty=.8;
  spec.maxVxMps=.004;spec.maxVyMps=.003;spec.maxYawRateRadS=.03;spec.maxCommandTtlS=.30;spec.firstTargetSequence=10000;
  PlanarVelocityCommand cmd;cmd.vxMps=.001;cmd.vyMps=0;cmd.yawRateRadS=0;cmd.stampS=now;cmd.ttlS=.25;cmd.sequence=1;
  CHECK(walk.start(s,o,cmd,spec,p,now));unsigned walkFrames=0;bool requested=false;
  for(unsigned guard=0;guard<4000&&walk.state()!=FeedbackWalkState::STAND;++guard){
    cmd.stampS=now;cmd.sequence++;if(!requested)CHECK(walk.command(cmd,now));o.stampS=now;auto f=walk.update(o,now);
    if(walk.state()==FeedbackWalkState::REJECTED){std::fprintf(stderr,"walk rejected %s at cycle%.4f\n",feedbackReasonName(walk.reason()),walk.cycles());CHECK(false);}
    if(f.target.valid){CHECK(walk.assess(syntheticAssessment(f,now),now));auto approved=walk.update(o,now);CHECK(approved.executable);
      emit(o,approved,requested?"STOPPING":"WALK",now);now+=p.samplePeriodS;follow(o,approved,now);++walkFrames;
    } else now+=p.samplePeriodS;
    if(!requested&&walk.cycles()>.55){walk.requestStop();requested=true;}}
  CHECK(walk.state()==FeedbackWalkState::STAND);CHECK(o.confirmedFeet==15);CHECK(requested);std::printf("WALK forward -> coordinated STOP: %u feedback frames\n",walkFrames);
  // The stopped footprint can restart G4 at its new world transform.
  cmd.sequence++;cmd.stampS=now;FeedbackWalk restart;CHECK(restart.start(s,o,cmd,spec,p,now));
  auto replay=cmd;CHECK(!restart.command(replay,now));CHECK(restart.reason()==FeedbackReason::SEQUENCE_REPLAY);
  auto changed=cmd;changed.sequence++;changed.vxMps=.002;CHECK(restart.command(changed,now));CHECK(restart.state()==FeedbackWalkState::STOPPING);
  changed.vyMps=.0002;changed.yawRateRadS=.002;changed.sequence++;CHECK(restart.command(changed,now));
  unsigned transitionFrames=0;bool transitioned=false;
  for(unsigned guard=0;guard<1500&&!transitioned;++guard){
    changed.stampS=now;changed.sequence++;CHECK(restart.command(changed,now));o.stampS=now;
    auto f=restart.update(o,now);CHECK(restart.state()!=FeedbackWalkState::REJECTED);
    if(f.target.valid){CHECK(restart.assess(syntheticAssessment(f,now),now));auto accepted=restart.update(o,now);CHECK(accepted.executable);
      now+=p.samplePeriodS;follow(o,accepted,now);++transitionFrames;
    }else now+=p.samplePeriodS;
    transitioned=restart.state()==FeedbackWalkState::STARTING;
  }
  CHECK(transitioned&&transitionFrames>0);restart.requestStop();CHECK(restart.state()==FeedbackWalkState::STOPPING);
  // STOP clears the queued velocity even if a parameter transition was pending.
  changed.sequence++;changed.stampS=now;changed.vxMps=.001;CHECK(restart.command(changed,now));restart.requestStop();
  for(unsigned guard=0;guard<1500&&restart.state()!=FeedbackWalkState::STAND;++guard){
    o.stampS=now;auto f=restart.update(o,now);CHECK(restart.state()!=FeedbackWalkState::REJECTED);
    if(f.target.valid){CHECK(restart.assess(syntheticAssessment(f,now),now));auto accepted=restart.update(o,now);CHECK(accepted.executable);
      now+=p.samplePeriodS;follow(o,accepted,now);
    }else now+=p.samplePeriodS;
  }
  CHECK(restart.state()==FeedbackWalkState::STAND);
  auto timeoutWalk=FeedbackWalk{};cmd.sequence++;cmd.stampS=now;CHECK(timeoutWalk.start(s,o,cmd,spec,p,now));o.stampS=now+.26;timeoutWalk.update(o,now+.26);CHECK(timeoutWalk.state()==FeedbackWalkState::STOPPING);
  // API receives SI velocity, including lateral/diagonal/yaw. Qualification
  // of a physical finite-cylinder contact remains a separate geometry gate.
  cmd.sequence++;cmd.stampS=now;o.stampS=now;cmd.vyMps=.0002;cmd.yawRateRadS=.002;
  FeedbackWalk planar;CHECK(planar.start(s,o,cmd,spec,p,now));
  auto planarFrame=planar.update(o,now);CHECK(planarFrame.target.valid&&!planarFrame.executable);
  auto excessive=cmd;excessive.sequence++;excessive.vxMps=spec.maxVxMps+.001;CHECK(!planar.command(excessive,now));
  auto release=cmd;release.sequence++;release.vxMps=release.vyMps=release.yawRateRadS=0;CHECK(planar.command(release,now));CHECK(planar.state()==FeedbackWalkState::STOPPING);
  auto physicalPolicy=p;physicalPolicy.mode=FeedbackMode::PHYSICAL_REVIEW;physicalPolicy.loadedRatesVerified=true;
  auto physicalObservation=supported(.150,now);physicalObservation.heightSource=physicalObservation.supportSource=EvidenceSource::EXTERNAL_MEASURED;
  AdaptiveStand physicalStand;CHECK(physicalStand.request({},physicalPolicy,now));physicalStand.update(physicalObservation,now);physicalStand.update(physicalObservation,now);CHECK(physicalStand.state()==AdaptiveStandState::STAND);
  cmd.sequence++;cmd.stampS=now;cmd.vyMps=cmd.yawRateRadS=0;
  FeedbackWalk unqualifiedDynamic;CHECK(unqualifiedDynamic.start(physicalStand,physicalObservation,cmd,spec,physicalPolicy,now));
  auto physicalCandidate=unqualifiedDynamic.update(physicalObservation,now);CHECK(physicalCandidate.target.valid);
  auto noDynamic=syntheticAssessment(physicalCandidate,now);noDynamic.source=EvidenceSource::EXTERNAL_MEASURED;noDynamic.loadedTrackingQualified=true;
  CHECK(!unqualifiedDynamic.assess(noDynamic,now));CHECK(unqualifiedDynamic.reason()==FeedbackReason::SUPPORT_INVALID);
  auto intermediate=supported(.117,now);AdaptiveStand resumed;CHECK(resumed.request({},p,now));n=completeStand(resumed,intermediate,p,now,"intermediate restart");std::printf("intermediate 117mm -> STAND 150mm: %u feedback frames\n",n);
  auto higher=supported(.153,now);AdaptiveStand adjust;CHECK(adjust.request({},p,now));n=completeStand(adjust,higher,p,now,"higher stand adjustment");CHECK(n>0);
  std::printf("ADAPTIVE_MOTION_PASS checks=%u hardware_access=false physical_readiness=false dynamic_stability=false\n",checks);
}
