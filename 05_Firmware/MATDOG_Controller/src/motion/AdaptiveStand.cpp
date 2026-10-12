#include "AdaptiveStand.h"
#include "TimedStand.h"
#include "PoseSupport.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
Vector3 mix(Vector3 a,Vector3 b,double s){return {a.x+s*(b.x-a.x),a.y+s*(b.y-a.y),a.z+s*(b.z-a.z)};}
Vector3 rate(Vector3 a,Vector3 b,double s){return {(b.x-a.x)*s,(b.y-a.y)*s,(b.z-a.z)*s};}
double distance(Vector3 a,Vector3 b){return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z);}
bool sameRotation(const BodyPose& a,const BodyPose& b){for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)if(std::abs(a.rotation[i][j]-b.rotation[i][j])>1e-10)return false;return true;}
}
const char* adaptiveStandStateName(AdaptiveStandState s){const char* n[]={"IDLE","OBSERVE","CLASSIFY","ACQUIRE_SUPPORT","RISE","VERIFY_STAND","STAND","REJECTED","STOPPED"};unsigned i=static_cast<unsigned>(s);return i<sizeof(n)/sizeof(n[0])?n[i]:"UNKNOWN";}
bool restGroundAcquisitionCandidate(AcquisitionRoute& route){
  const auto* rest=researchedPose(0);if(!rest||!rest->joints.valid)return false;
  route.count=0;route.geometryIdentityVerified=true;route.source=EvidenceSource::SYNTHETIC;
  AcquisitionWaypoint w;w.body=rest->body;w.bellySupportRequired=true;
  for(unsigned i=0;i<4;++i){const auto fk=contactForwardKinematics(static_cast<LegId>(i),rest->joints.legs[i]);
    if(fk.status!=ContactStatus::OK||!pointToWorld(w.body,fk.referenceM,w.feetWorldM[i]))return false;}
  route.waypoints[route.count++]=w;
  // The ground-only deployment conflicts with current collision meshes. This
  // candidate explicitly requires a fixture to lift the rigid REST_GROUND pose,
  // then holds external support while the feet acquire their canonical footprint.
  w.body.translationM.z+=.065;w.bellySupportRequired=false;w.externalSupportRequired=true;
  for(auto& foot:w.feetWorldM)foot.z+=.065;
  w.durationS=8;route.waypoints[route.count++]=w;
  // An offline Cartesian unfolding waypoint prevents the direct chord from
  // crossing the upper-joint envelope. Every intervening sample still uses G2.
  for(unsigned i=0;i<4;++i){auto unfolded=rest->joints.legs[i];unfolded.upper=i<2?1.6:1.8;
    const auto fk=contactForwardKinematics(static_cast<LegId>(i),unfolded);
    if(fk.status!=ContactStatus::OK||!pointToWorld(w.body,fk.referenceM,w.feetWorldM[i]))return false;}
  w.durationS=20;route.waypoints[route.count++]=w;
  // Descend first to the rear footprint at 65 mm. The direct forward/downward
  // chord from folded rear legs to canonical feet crosses the upper envelope.
  for(unsigned i=0;i<4;++i)w.feetWorldM[i]={i<2?-.05:-.2875,canonicalStandDefinition().contactsWorldM[i].y,0};
  w.durationS=10;route.waypoints[route.count++]=w;
  for(unsigned i=0;i<4;++i)w.feetWorldM[i]=canonicalStandDefinition().contactsWorldM[i];
  w.durationS=10;route.waypoints[route.count++]=w;
  w.externalSupportRequired=false;
  w.requiredFeet=15;w.durationS=1;route.waypoints[route.count++]=w;return true;
}
bool AdaptiveStand::request(const AdaptiveStandRequest& r,const FeedbackPolicy& p,double now){
  state_=AdaptiveStandState::IDLE;poseClass_=ObservedPoseClass::UNKNOWN;reason_=FeedbackReason::NOT_REQUESTED;
  previous_={};segmentStart_={};pending_=FeedbackFrame{};assessment_={};pendingValid_=assessed_=physicalReady_=false;
  route_.count=0;routeIndex_=0;segmentIndex_=segmentIntervals_=0;
  policy_=p;request_=r;lastNow_=now;verifySince_=now;sequence_=r.firstSequence;
  if(!validFeedbackPolicy(p)){reject(FeedbackReason::INVALID_POLICY);return false;}
  if(!std::isfinite(now)||!std::isfinite(r.targetHeightM)||r.targetHeightM<=0||r.targetHeightM>.150||
    !std::isfinite(r.riseDurationS)||r.riseDurationS<=0||r.firstSequence==0){reject(FeedbackReason::INVALID_COMMAND);return false;}
  if(r.acquisition){route_=*r.acquisition;if(route_.count<2||route_.count>AcquisitionRoute::kMaxWaypoints){reject(FeedbackReason::ROUTE_MISMATCH);return false;}}
  request_.acquisition=nullptr;state_=AdaptiveStandState::OBSERVE;reason_=FeedbackReason::OK;return true;
}
FeedbackFrame AdaptiveStand::reject(FeedbackReason r){state_=AdaptiveStandState::REJECTED;reason_=r;pendingValid_=false;physicalReady_=false;FeedbackFrame f;f.reason=r;return f;}
void AdaptiveStand::stop(){state_=AdaptiveStandState::STOPPED;reason_=FeedbackReason::STOPPED;pendingValid_=false;physicalReady_=false;}
bool AdaptiveStand::assess(const SegmentAssessment& a,double now){
  if(!pendingValid_)return false;
  auto r=checkAssessment(a,pending_.target.sequence,policy_,now);
  if(r!=FeedbackReason::OK){reject(r);return false;}
  assessment_=a;assessed_=true;pending_.executable=true;
  pending_.physicalReadiness=policy_.mode==FeedbackMode::PHYSICAL_REVIEW&&a.loadedTrackingQualified;
  return true;
}
bool AdaptiveStand::beginSegment(const AcquisitionWaypoint& e){
  if(!supportsFlatContactIk(e.body)||!sameRotation(e.body,previous_.cartesian.body)||
    !std::isfinite(e.durationS)||e.durationS<=0||e.requiredFeet>15||
    (!e.bellySupportRequired&&!e.externalSupportRequired&&e.requiredFeet==0))return false;
  const double n=std::ceil(e.durationS/policy_.samplePeriodS);if(n<2||n>1000000)return false;
  endpoint_=e;segmentStart_=previous_;segmentIndex_=0;segmentIntervals_=static_cast<uint32_t>(n);return true;
}
bool AdaptiveStand::beginRise(){
  AcquisitionWaypoint e;e.body=previous_.cartesian.body;e.body.translationM.z=request_.targetHeightM;
  for(unsigned i=0;i<4;++i)e.feetWorldM[i]=previous_.cartesian.feet[i];
  e.requiredFeet=15;e.durationS=request_.riseDurationS;state_=AdaptiveStandState::RISE;return beginSegment(e);
}
FeedbackFrame AdaptiveStand::generate(double now){
  const double u=static_cast<double>(segmentIndex_+1)/segmentIntervals_;
  const auto law=quinticTimeLaw(u,endpoint_.durationS);if(!law.valid)return reject(FeedbackReason::INVALID_COMMAND);
  CartesianSample c;c.body=endpoint_.body;c.body.translationM=mix(segmentStart_.cartesian.body.translationM,endpoint_.body.translationM,law.progress);
  c.bodyVelocity=rate(segmentStart_.cartesian.body.translationM,endpoint_.body.translationM,law.velocity);
  c.bodyAcceleration=rate(segmentStart_.cartesian.body.translationM,endpoint_.body.translationM,law.acceleration);
  for(unsigned i=0;i<4;++i){auto a=segmentStart_.cartesian.feet[i],b=endpoint_.feetWorldM[i];
    c.feet[i]=mix(a,b,law.progress);c.velocity[i]=rate(a,b,law.velocity);c.acceleration[i]=rate(a,b,law.acceleration);
    c.phases[i].stance=(endpoint_.requiredFeet&(1<<i))!=0;}
  c.valid=true;GaitParameters p;p.heightM=request_.targetHeightM;
  auto f=solveGait(p,c,&previous_);
  if(!f.target.valid)return reject(f.status==GaitStatus::BRANCH_CHANGE?FeedbackReason::BRANCH_CHANGE:FeedbackReason::IK_FAILURE);
  // Reuse the C4-C fixed-contact rise component at any positive observed height.
  // Yaw/translation are factored out; descending adjustments reverse its clock.
  if(state_==AdaptiveStandState::RISE&&segmentStart_.cartesian.body.translationM.z>0&&
     segmentStart_.cartesian.body.translationM.z!=endpoint_.body.translationM.z){
    StandDefinition d=canonicalStandDefinition();BodyPose origin=segmentStart_.cartesian.body;origin.translationM.z=0;
    for(unsigned i=0;i<4;++i){Vector3 local;if(!pointToBase(origin,c.feet[i],local))return reject(FeedbackReason::IK_FAILURE);
      d.contactsWorldM[i]=local;d.contactsWorldM[i].z=0;d.standSeed[i]=segmentStart_.target.legs[i];}
    const bool ascending=endpoint_.body.translationM.z>segmentStart_.cartesian.body.translationM.z;
    d.lowBodyHeightM=ascending?segmentStart_.cartesian.body.translationM.z:endpoint_.body.translationM.z;
    d.standBodyHeightM=ascending?endpoint_.body.translationM.z:segmentStart_.cartesian.body.translationM.z;
    const auto timed=evaluateTimedStand(d,ascending?u:1-u,endpoint_.durationS,previous_.target.legs);
    if(timed.status!=TimedStatus::OK)return reject(FeedbackReason::IK_FAILURE);
    f.target=timed.geometry.target;
    for(unsigned i=0;i<4;++i){f.velocityRadS[i]=timed.velocityRadS[i];f.accelerationRadS2[i]=timed.accelerationRadS2[i];
      if(!ascending){auto& v=f.velocityRadS[i];v={-v.hip,-v.upper,-v.lower};}}
  }
  auto r=checkSegment(f,previous_.target,policy_);
  if(r!=FeedbackReason::OK)return reject(r);
  if(sequence_==0||sequence_==UINT32_MAX)return reject(FeedbackReason::SEQUENCE_REPLAY);
  pending_=feedbackFrame(f,sequence_++);pending_.bellySupportRequired=endpoint_.bellySupportRequired;
  pending_.externalSupportRequired=endpoint_.externalSupportRequired;pending_.timeS=now;
  // The solve is retained only after matching operational feedback acknowledges it.
  pendingValid_=true;assessed_=false;pendingSince_=now;reason_=FeedbackReason::ASSESSMENT_REQUIRED;return pending_;
}
FeedbackFrame AdaptiveStand::update(const AdaptiveObservation& o,double now){
  if(state_==AdaptiveStandState::IDLE||state_==AdaptiveStandState::REJECTED||state_==AdaptiveStandState::STOPPED){FeedbackFrame f;f.reason=reason_;return f;}
  if(now<lastNow_)return reject(FeedbackReason::STALE_OBSERVATION);
  lastNow_=now;
  auto r=checkObservation(o,policy_,now);if(r!=FeedbackReason::OK)return reject(r);
  if(pendingValid_){
    r=checkHeldSupport(o,previous_,pending_);if(r!=FeedbackReason::OK)return reject(r);
    if(now-pendingSince_>policy_.segmentTimeoutS)return reject(FeedbackReason::SEGMENT_TIMEOUT);
    if(!assessed_){pending_.reason=FeedbackReason::ASSESSMENT_REQUIRED;return pending_;}
    r=checkAssessment(assessment_,pending_.target.sequence,policy_,now);if(r!=FeedbackReason::OK)return reject(r);
    r=checkTracking(o,pending_,policy_);
    if(r==FeedbackReason::FEEDBACK_PENDING){pending_.reason=r;return pending_;}
    if(r!=FeedbackReason::OK)return reject(r);
    if(now-pendingSince_+1e-9<endpoint_.durationS/segmentIntervals_){pending_.reason=FeedbackReason::FEEDBACK_PENDING;return pending_;}
    previous_=observedSolution(o);if(!previous_.target.valid)return reject(FeedbackReason::IK_FAILURE);
    pendingValid_=false;physicalReady_=pending_.physicalReadiness;++segmentIndex_;
    if(segmentIndex_==segmentIntervals_){
      if(state_==AdaptiveStandState::ACQUIRE_SUPPORT){++routeIndex_;
        if(routeIndex_<route_.count){if(!beginSegment(route_.waypoints[routeIndex_]))return reject(FeedbackReason::ROUTE_MISMATCH);}
        else {if(o.confirmedFeet!=15||o.bellySupported)return reject(FeedbackReason::SUPPORT_INVALID);if(!beginRise())return reject(FeedbackReason::ROUTE_MISMATCH);}}
      else {state_=AdaptiveStandState::VERIFY_STAND;verifySince_=now;}
    }
  }
  if(state_==AdaptiveStandState::OBSERVE){state_=AdaptiveStandState::CLASSIFY;FeedbackFrame f;f.reason=FeedbackReason::OK;return f;}
  if(state_==AdaptiveStandState::CLASSIFY){
    if(!o.stationary)return reject(FeedbackReason::UNRECOVERABLE_POSE);
    previous_=observedSolution(o);if(!previous_.target.valid)return reject(FeedbackReason::IK_FAILURE);
    if(o.confirmedFeet==15&&!o.bellySupported){
      for(unsigned i=0;i<4;++i)if(std::abs(previous_.cartesian.feet[i].z)>policy_.contactToleranceM)return reject(FeedbackReason::SUPPORT_INVALID);
      if(std::abs(o.body.translationM.z-request_.targetHeightM)<=policy_.heightToleranceM){poseClass_=ObservedPoseClass::NOMINAL_STAND;state_=AdaptiveStandState::VERIFY_STAND;verifySince_=now;}
      else {poseClass_=o.body.translationM.z<.100?ObservedPoseClass::SUPPORTED_INTERMEDIATE:ObservedPoseClass::SUPPORTED_STAND;
        if(!beginRise())return reject(FeedbackReason::UNRECOVERABLE_POSE);}
    } else if(o.bellySupported||o.confirmedFeet!=0){
      poseClass_=o.bellySupported?ObservedPoseClass::BELLY_FOLDED:ObservedPoseClass::PARTIAL_SUPPORT;
      if(route_.count<2)return reject(FeedbackReason::ROUTE_REQUIRED);
      if(!route_.geometryIdentityVerified||route_.source==EvidenceSource::UNKNOWN||
        (policy_.mode==FeedbackMode::PHYSICAL_REVIEW&&route_.source!=EvidenceSource::EXTERNAL_MEASURED))return reject(FeedbackReason::ROUTE_REQUIRED);
      const auto& first=route_.waypoints[0];const auto& last=route_.waypoints[route_.count-1];
      // A partial-support route must describe the actually observed initial
      // contact mask. Missing contacts are acquired with independent feedback;
      // proximity to the support plane does not make them load-bearing.
      if(first.requiredFeet!=o.confirmedFeet||first.bellySupportRequired!=o.bellySupported||
         (first.externalSupportRequired&&!o.externalSupportConfirmed))return reject(FeedbackReason::ROUTE_MISMATCH);
      if(last.requiredFeet!=15||last.bellySupportRequired||distance(first.body.translationM,o.body.translationM)>policy_.heightToleranceM||!sameRotation(first.body,o.body))return reject(FeedbackReason::ROUTE_MISMATCH);
      for(unsigned i=0;i<4;++i)if(distance(first.feetWorldM[i],previous_.cartesian.feet[i])>policy_.contactToleranceM)return reject(FeedbackReason::ROUTE_MISMATCH);
      state_=AdaptiveStandState::ACQUIRE_SUPPORT;routeIndex_=1;if(!beginSegment(route_.waypoints[1]))return reject(FeedbackReason::ROUTE_MISMATCH);
    } else {poseClass_=ObservedPoseClass::UNRECOVERABLE;return reject(FeedbackReason::UNRECOVERABLE_POSE);}
  }
  if(state_==AdaptiveStandState::VERIFY_STAND||state_==AdaptiveStandState::STAND){
    FeedbackFrame f;
    if(o.confirmedFeet!=15||o.bellySupported||!o.stationary||std::abs(o.body.translationM.z-request_.targetHeightM)>policy_.heightToleranceM){
      if(state_==AdaptiveStandState::STAND)return reject(o.confirmedFeet!=15||o.bellySupported?FeedbackReason::SUPPORT_INVALID:FeedbackReason::TRACKING_ERROR);
      if(now-verifySince_>policy_.segmentTimeoutS)return reject(FeedbackReason::SEGMENT_TIMEOUT);
      f.reason=FeedbackReason::VERIFY_PENDING;reason_=f.reason;return f;
    }
    for(unsigned i=0;i<4;++i){auto fk=contactForwardKinematics(static_cast<LegId>(i),o.actual.legs[i]);Vector3 world;
      if(fk.status!=ContactStatus::OK||fk.support!=SupportMode::NOMINAL_STRIP||!pointToWorld(o.body,fk.referenceM,world)||std::abs(world.z)>policy_.contactToleranceM)return reject(FeedbackReason::SUPPORT_INVALID);}
    state_=AdaptiveStandState::STAND;reason_=FeedbackReason::OK;f.reason=reason_;f.body=o.body;
    // Nominal confirmation emits NO actuator frame. Existing pose is held by its owner.
    f.physicalReadiness=physicalReady_;return f;
  }
  if(endpoint_.externalSupportRequired&&!o.externalSupportConfirmed)return reject(FeedbackReason::SUPPORT_INVALID);
  return generate(now);
}
} }
