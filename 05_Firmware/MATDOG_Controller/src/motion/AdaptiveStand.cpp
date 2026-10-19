#include "AdaptiveStand.h"
#include "TimedStand.h"
#include "PoseSupport.h"
#include "StandHeightRegion.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
Vector3 mix(Vector3 a,Vector3 b,double s){return {a.x+s*(b.x-a.x),a.y+s*(b.y-a.y),a.z+s*(b.z-a.z)};}
Vector3 rate(Vector3 a,Vector3 b,double s){return {(b.x-a.x)*s,(b.y-a.y)*s,(b.z-a.z)*s};}
double distance(Vector3 a,Vector3 b){return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z);}
bool sameRotation(const BodyPose& a,const BodyPose& b){for(unsigned i=0;i<3;++i)for(unsigned j=0;j<3;++j)if(std::abs(a.rotation[i][j]-b.rotation[i][j])>1e-10)return false;return true;}
}
const char* adaptiveStandStateName(AdaptiveStandState s){const char* n[]={"IDLE","OBSERVE","CLASSIFY","ACQUIRE_SUPPORT","RISE","VERIFY_STAND","STAND","REJECTED","STOPPED","PRE_ALIGN","REAR_DEPLOY","FRONT_DEPLOY","LOAD_TRANSFER","RISE","ENTRY_DISENGAGE","ENTRY_ALIGN"};unsigned i=static_cast<unsigned>(s);return i<sizeof(n)/sizeof(n[0])?n[i]:"UNKNOWN";}
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
bool AdaptiveStand::prepareTimedRoute(const AcquisitionRoute& r,const FeedbackPolicy& p){
  if(!r.pairedProne||!r.phaseTimed)return true;
  const double v=.999*r.phaseVelocityRadS,a=.999*r.phaseAccelerationRadS2/2;
  if(timed_.valid&&timedVariant_==p.fixtureLimitVariant&&timed_.velocityRadS==v&&
     timed_.accelerationRadS2==a&&timed_.frameS==p.samplePeriodS)return true;
  timedVariant_=p.fixtureLimitVariant;
  return buildPhaseTimedRoute(timedVariant_,v,a,p.samplePeriodS,&timed_);
}
bool AdaptiveStand::request(const AdaptiveStandRequest& r,const FeedbackPolicy& p,double now){
  state_=AdaptiveStandState::IDLE;poseClass_=ObservedPoseClass::UNKNOWN;reason_=FeedbackReason::NOT_REQUESTED;
  previous_={};segmentStart_={};pending_=FeedbackFrame{};assessment_={};pendingValid_=assessed_=submitted_=physicalReady_=false;
  entry_progress_=0;approvedSegment_=entry_knot_=UINT16_MAX;for(auto& q:entry_offset_)q={};
  entry_=FixtureEntryPlan{};prone_=FixturePoseClassification{};entryActive_=false;entryStage_=0;entryApproved_=UINT8_MAX;approvedThrough_=UINT16_MAX;
  route_.count=0;route_.pairedProne=false;pairedSegment_=0;routeIndex_=0;segmentIndex_=segmentIntervals_=0;
  policy_=p;request_=r;lastNow_=now;verifySince_=now;sequence_=r.firstSequence;
  if(!validFeedbackPolicy(p)){reject(FeedbackReason::INVALID_POLICY);return false;}
  if(!std::isfinite(now)||!std::isfinite(r.targetHeightM)||r.targetHeightM<=0||r.targetHeightM>.150||
    !std::isfinite(r.riseDurationS)||r.riseDurationS<=0||r.firstSequence==0){reject(FeedbackReason::INVALID_COMMAND);return false;}
  if(r.acquisition){route_=*r.acquisition;if(!route_.pairedProne&&(route_.count<2||route_.count>AcquisitionRoute::kMaxWaypoints)){reject(FeedbackReason::ROUTE_MISMATCH);return false;}}
  if(r.entry){
    // A staged entry only exists for the suspended fixture-limit route.
    if(!r.acquisition||!route_.pairedProne||p.mode!=FeedbackMode::FIXTURE_REVIEW||!p.fixtureLimitVariant||
       r.entry->status!=EntryStatus::OK||!std::isfinite(r.proneMaxDeviationRad)||!std::isfinite(r.proneMaxRmsRad)||
       r.proneMaxDeviationRad<=0||r.proneMaxRmsRad<=0){reject(FeedbackReason::ROUTE_REQUIRED);return false;}
    entry_=*r.entry;entryActive_=true;
  }
  timedActive_=false;timedPhase_=0;adjust_=false;
  if(route_.pairedProne&&route_.phaseTimed){
    // Only behind a staged entry (the route then starts exactly at knot 0).
    // 0.999: the law's peaks sit strictly inside the checkSegment limits (an
    // exactly-equal analytic peak can exceed them by rounding).
    if(!r.entry||!prepareTimedRoute(route_,p)||
       route_.phaseVelocityRadS>p.maxVelocityRadS[0].upper+1e-12){reject(FeedbackReason::INVALID_POLICY);return false;}
    timedActive_=true;
  }
  request_.acquisition=nullptr;request_.entry=nullptr;state_=AdaptiveStandState::OBSERVE;reason_=FeedbackReason::OK;return true;
}
bool AdaptiveStand::requestHeightAdjust(double h,uint32_t firstSequence,double now){
  if(state_!=AdaptiveStandState::STAND||policy_.mode!=FeedbackMode::FIXTURE_REVIEW||!previous_.target.valid||!previous_.cartesian.valid||
     !std::isfinite(h)||h<=0||h>.150||firstSequence==0||!std::isfinite(now)||std::abs(previous_.cartesian.body.translationM.z-h)<1e-9)return false;
  request_.targetHeightM=h;request_.firstSequence=firstSequence;request_.acquisition=nullptr;request_.entry=nullptr;
  // V1's smaller frame box uses .19 rad/s. Twelve seconds per 50 mm keeps
  // the existing fixed-contact quintic inside that rate envelope.
  request_.riseDurationS=std::max(12.0,12.0*std::abs(previous_.cartesian.body.translationM.z-h)/.050);
  route_.count=0;route_.pairedProne=false;timedActive_=false;entryActive_=false;adjust_=true;
  pendingValid_=assessed_=submitted_=physicalReady_=false;sequence_=firstSequence;lastNow_=now;verifySince_=now;
  state_=AdaptiveStandState::OBSERVE;reason_=FeedbackReason::OK;return true;
}
FeedbackFrame AdaptiveStand::reject(FeedbackReason r){state_=AdaptiveStandState::REJECTED;reason_=r;pendingValid_=false;physicalReady_=false;FeedbackFrame f;f.reason=r;return f;}
void AdaptiveStand::stop(){state_=AdaptiveStandState::STOPPED;reason_=FeedbackReason::STOPPED;pendingValid_=false;physicalReady_=false;}
bool AdaptiveStand::assess(const SegmentAssessment& a,double now){
  if(!pendingValid_)return false;
  auto r=checkAssessment(a,pending_.target.sequence,policy_,now);
  if(r!=FeedbackReason::OK){reject(r);return false;}
  assessment_=a;assessed_=true;pending_.executable=true;
  pending_.physicalReadiness=policy_.mode==FeedbackMode::FIXTURE_REVIEW?a.fixtureTrialEnvelopeReviewed:policy_.mode==FeedbackMode::PHYSICAL_REVIEW&&a.loadedTrackingQualified;
  return true;
}
bool AdaptiveStand::beginSegment(const AcquisitionWaypoint& e){
  if(!supportsFlatContactIk(e.body)||!sameRotation(e.body,previous_.cartesian.body)||
    !std::isfinite(e.durationS)||e.durationS<=0||e.requiredFeet>15||
    (!e.bellySupportRequired&&!e.externalSupportRequired&&e.requiredFeet==0))return false;
  const double n=std::ceil(e.durationS/policy_.samplePeriodS);if(n<2||n>1000000)return false;
  endpoint_=e;segmentStart_=previous_;segmentIndex_=0;segmentIntervals_=static_cast<uint32_t>(n);referenceAtS_=lastNow_;referenceTimeS_=0;return true;
}
bool AdaptiveStand::beginRise(){
  AcquisitionWaypoint e;e.body=previous_.cartesian.body;e.body.translationM.z=request_.targetHeightM;
  for(unsigned i=0;i<4;++i)e.feetWorldM[i]=previous_.cartesian.feet[i];
  e.requiredFeet=15;e.durationS=request_.riseDurationS;state_=AdaptiveStandState::RISE;return beginSegment(e);
}
bool AdaptiveStand::beginEntryStage(){
  if(entryStage_>=kFixtureEntryStages)return false;
  state_=entryStage_==0?AdaptiveStandState::ENTRY_DISENGAGE:AdaptiveStandState::ENTRY_ALIGN;
  endpoint_.durationS=entry_.durationS[entryStage_];
  const double n=std::ceil(endpoint_.durationS/policy_.samplePeriodS-1e-9);
  if(!std::isfinite(n)||n<2||n>1000000)return false;
  segmentIntervals_=static_cast<uint32_t>(n);segmentIndex_=0;referenceAtS_=lastNow_;referenceTimeS_=0;return true;
}
bool AdaptiveStand::beginTimedPhase(){
  if(timedPhase_>=5)return false;
  const AdaptiveStandState states[]={AdaptiveStandState::PRE_ALIGN,AdaptiveStandState::REAR_DEPLOY,
    AdaptiveStandState::FRONT_DEPLOY,AdaptiveStandState::LOAD_TRANSFER,AdaptiveStandState::PAIRED_RISE};
  state_=states[timedPhase_];pairedSegment_=static_cast<uint16_t>(timed_.first[timedPhase_]);
  endpoint_.durationS=timed_.durationS[timedPhase_];
  const double n=std::ceil(endpoint_.durationS/policy_.samplePeriodS-1e-9);
  if(!std::isfinite(n)||n<1||n>1000000)return false;
  segmentIntervals_=static_cast<uint32_t>(n);segmentIndex_=0;referenceAtS_=lastNow_;referenceTimeS_=0;return true;
}
bool AdaptiveStand::beginPairedSegment(){
  if(timedActive_)return beginTimedPhase();
  if(pairedSegment_>=kPairedProneSegments)return false;
  const AdaptiveStandState states[]={AdaptiveStandState::PRE_ALIGN,AdaptiveStandState::REAR_DEPLOY,
    AdaptiveStandState::FRONT_DEPLOY,AdaptiveStandState::LOAD_TRANSFER,AdaptiveStandState::PAIRED_RISE};
  state_=states[static_cast<unsigned>(pairedPronePhase(pairedSegment_))];
  endpoint_.durationS=pairedProneDuration(pairedSegment_)*route_.timeScale;
  if(!std::isfinite(route_.timeScale)||route_.timeScale<1)return false;
  const double n=std::ceil(endpoint_.durationS/policy_.samplePeriodS);
  if(n<1||n>1000000)return false;
  segmentIntervals_=static_cast<uint32_t>(n);segmentIndex_=0;referenceAtS_=lastNow_;referenceTimeS_=0;return true;
}
FeedbackFrame AdaptiveStand::generate(double now){
  // Advance by elapsed monotonic time, bounded by the approved step/rate
  // budget on a late release. Dropped time slows this SAME path; no catch-up
  // burst or future jump is emitted. The executor checks actual TX intervals.
  double vmax=0;
  for(const auto& v:policy_.maxVelocityRadS)vmax=std::fmax(vmax,std::fmax(std::abs(v.hip),std::fmax(std::abs(v.upper),std::abs(v.lower))));
  const double advance=std::fmin(std::fmax(0.,now-referenceAtS_),policy_.maxJointStepRad/vmax);
  pendingTimeS_=timeDriven_?std::fmin(endpoint_.durationS,referenceTimeS_+advance):
    endpoint_.durationS*(segmentIndex_+1)/segmentIntervals_;
  const double u=pendingTimeS_/endpoint_.durationS;
  if(entryPending()){
    auto f=fixtureEntrySample(entry_,entryStage_,u,previous_.cartesian.body);
    if(!f.target.valid)return reject(FeedbackReason::INVALID_COMMAND);
    auto r=checkSegment(f,previous_.target,policy_);if(r!=FeedbackReason::OK)return reject(r);
    if(sequence_==0||sequence_==UINT32_MAX)return reject(FeedbackReason::SEQUENCE_REPLAY);
    pendingSolution_=f;pending_=feedbackFrame(f,sequence_++);pending_.timeS=now;
    pending_.requiredFeet=0;pending_.bellySupportRequired=false;pending_.externalSupportRequired=true;
    pendingValid_=true;assessed_=submitted_=false;pendingSince_=now;return pending_;
  }
  if(route_.pairedProne&&timedActive_){
    unsigned segment=0;
    auto f=phaseTimedSample(timed_,timedPhase_,pendingTimeS_,policy_.fixtureLimitVariant,&segment);
    if(!f.target.valid)return reject(FeedbackReason::IK_FAILURE);
    pairedSegment_=static_cast<uint16_t>(segment);
    auto r=checkSegment(f,previous_.target,policy_);if(r!=FeedbackReason::OK)return reject(r);
    if(sequence_==0||sequence_==UINT32_MAX)return reject(FeedbackReason::SEQUENCE_REPLAY);
    pendingSolution_=f;pending_=feedbackFrame(f,sequence_++);pending_.timeS=now;
    pending_.requiredFeet=0;pending_.bellySupportRequired=false;pending_.externalSupportRequired=true;
    pendingValid_=true;assessed_=submitted_=false;pendingSince_=now;return pending_;
  }
  if(route_.pairedProne){
    auto f=pairedProneSample(pairedSegment_,u,policy_.fixtureLimitVariant,pairedSegment_==entry_knot_?entry_progress_:0);
    for(unsigned i=0;i<4;++i){auto& v=f.velocityRadS[i];v={v.hip/route_.timeScale,v.upper/route_.timeScale,v.lower/route_.timeScale};auto& a=f.accelerationRadS2[i];const double s=route_.timeScale*route_.timeScale;a={a.hip/s,a.upper/s,a.lower/s};}
    if(policy_.mode==FeedbackMode::FIXTURE_REVIEW&&pairedSegment_==entry_knot_){
      const auto law=quinticTimeLaw(u,endpoint_.durationS);
      for(unsigned i=0;i<4;++i){const auto d=entry_offset_[i];auto& q=f.target.legs[i];auto& v=f.velocityRadS[i];auto& a=f.accelerationRadS2[i];
        q={q.hip+d.hip*(1-law.progress),q.upper+d.upper*(1-law.progress),q.lower+d.lower*(1-law.progress)};
        v={v.hip-d.hip*law.velocity,v.upper-d.upper*law.velocity,v.lower-d.lower*law.velocity};
        a={a.hip-d.hip*law.acceleration,a.upper-d.upper*law.acceleration,a.lower-d.lower*law.acceleration};}
    }
    auto r=checkSegment(f,previous_.target,policy_);if(r!=FeedbackReason::OK)return reject(r);
    if(sequence_==0||sequence_==UINT32_MAX)return reject(FeedbackReason::SEQUENCE_REPLAY);
    pendingSolution_=f;pending_=feedbackFrame(f,sequence_++);pending_.timeS=now;
    pending_.bellySupportRequired=static_cast<unsigned>(pairedPronePhase(pairedSegment_))<3;
    if(policy_.mode==FeedbackMode::FIXTURE_REVIEW){pending_.requiredFeet=0;pending_.bellySupportRequired=false;pending_.externalSupportRequired=true;}
    pendingValid_=true;assessed_=submitted_=false;pendingSince_=now;return pending_;
  }
  const auto law=quinticTimeLaw(u,endpoint_.durationS);if(!law.valid)return reject(FeedbackReason::INVALID_COMMAND);
  CartesianSample c;c.body=endpoint_.body;c.body.translationM=mix(segmentStart_.cartesian.body.translationM,endpoint_.body.translationM,law.progress);
  c.bodyVelocity=rate(segmentStart_.cartesian.body.translationM,endpoint_.body.translationM,law.velocity);
  c.bodyAcceleration=rate(segmentStart_.cartesian.body.translationM,endpoint_.body.translationM,law.acceleration);
  for(unsigned i=0;i<4;++i){auto a=segmentStart_.cartesian.feet[i],b=endpoint_.feetWorldM[i];
    c.feet[i]=mix(a,b,law.progress);c.velocity[i]=rate(a,b,law.velocity);c.acceleration[i]=rate(a,b,law.acceleration);
    c.phases[i].stance=(endpoint_.requiredFeet&(1<<i))!=0;}
  c.valid=true;GaitParameters p;p.heightM=request_.targetHeightM;
  // Height adjust: the retained entry pose is a tabulated solution, not a solveGait branch; the
  // timed stand below is seeded with it and checkSegment bounds every step from it.
  auto f=solveGait(p,c,adjust_?nullptr:&previous_);
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
  if(adjust_&&!heightTargetInRegion(f.target))return reject(FeedbackReason::OUTSIDE_CERTIFIED_REGION);
  auto r=checkSegment(f,previous_.target,policy_);
  if(r!=FeedbackReason::OK)return reject(r);
  if(sequence_==0||sequence_==UINT32_MAX)return reject(FeedbackReason::SEQUENCE_REPLAY);
  pending_=feedbackFrame(f,sequence_++);pending_.bellySupportRequired=endpoint_.bellySupportRequired;
  pending_.externalSupportRequired=endpoint_.externalSupportRequired;pending_.timeS=now;
  if(adjust_){pendingSolution_=f;pending_.requiredFeet=0;pending_.bellySupportRequired=false;pending_.externalSupportRequired=true;}
  // The solve is retained only after matching operational feedback acknowledges it.
  pendingValid_=true;assessed_=submitted_=false;pendingSince_=now;reason_=FeedbackReason::ASSESSMENT_REQUIRED;return pending_;
}
FeedbackFrame AdaptiveStand::update(const AdaptiveObservation& o,double now,bool timedReference){
  if(state_==AdaptiveStandState::IDLE||state_==AdaptiveStandState::REJECTED||state_==AdaptiveStandState::STOPPED){FeedbackFrame f;f.reason=reason_;return f;}
  if(now<lastNow_)return reject(FeedbackReason::STALE_OBSERVATION);
  lastNow_=now;timeDriven_=timedReference;
  auto r=checkObservation(o,policy_,now);if(r!=FeedbackReason::OK)return reject(r);
  if(pendingValid_){
    r=checkHeldSupport(o,previous_,pending_);if(r!=FeedbackReason::OK)return reject(r);
    if(now-pendingSince_>policy_.segmentTimeoutS)return reject(FeedbackReason::SEGMENT_TIMEOUT);
    if(!assessed_){pending_.reason=FeedbackReason::ASSESSMENT_REQUIRED;return pending_;}
    r=checkAssessment(assessment_,pending_.target.sequence,policy_,now,submitted_);if(r!=FeedbackReason::OK)return reject(r);
    if(timeDriven_&&!submitted_)return pending_;
    // TX acceptance only for intermediate references. Structural endpoints
    // consume the executor's measured, post-TX HOLD settlement.
    // For the certified fixture stream, arrival/quiet/post-TX are already
    // checked atomically by HOLD before completed_sequence is published.
    // Re-running the finite segment's age gate would split that checkpoint.
    const bool streamed=timeDriven_&&policy_.mode==FeedbackMode::FIXTURE_REVIEW&&policy_.routeTubeReviewed&&(route_.pairedProne||adjust_);
    r=streamed?(o.acknowledged&&o.acknowledgedSequence==pending_.target.sequence?FeedbackReason::OK:FeedbackReason::FEEDBACK_PENDING):checkTracking(o,pending_,policy_);
    if(r==FeedbackReason::FEEDBACK_PENDING){pending_.reason=r;return pending_;}
    if(r!=FeedbackReason::OK)return reject(r);
    if(!timeDriven_&&now-pendingSince_+1e-9<endpoint_.durationS/segmentIntervals_){pending_.reason=FeedbackReason::FEEDBACK_PENDING;return pending_;}
    const bool endpoint=pendingFinal();
    referenceTimeS_=pendingTimeS_;referenceAtS_=pendingSince_;
    previous_=route_.pairedProne||adjust_?pendingSolution_:observedSolution(o);if(!previous_.target.valid)return reject(FeedbackReason::IK_FAILURE);
    pendingValid_=false;physicalReady_=pending_.physicalReadiness;++segmentIndex_;
    if(endpoint){
      if(entryPending()){
        // The last entry sample IS the route head (fixture variant knot 0).
        if(++entryStage_<kFixtureEntryStages){if(!beginEntryStage())return reject(FeedbackReason::ROUTE_MISMATCH);}
        else {pairedSegment_=0;entry_knot_=UINT16_MAX;entry_progress_=0;timedPhase_=0;if(!beginPairedSegment())return reject(FeedbackReason::ROUTE_MISMATCH);}
      } else if(route_.pairedProne&&timedActive_){
        if(++timedPhase_<5){if(!beginTimedPhase())return reject(FeedbackReason::ROUTE_MISMATCH);}
        else {pairedSegment_=kPairedProneSegments-1;state_=AdaptiveStandState::VERIFY_STAND;verifySince_=now;}
      } else if(route_.pairedProne){
        ++pairedSegment_;
        if(pairedSegment_<kPairedProneSegments){if(!beginPairedSegment())return reject(FeedbackReason::ROUTE_MISMATCH);}
        else {state_=AdaptiveStandState::VERIFY_STAND;verifySince_=now;}
      } else if(state_==AdaptiveStandState::ACQUIRE_SUPPORT){++routeIndex_;
        if(routeIndex_<route_.count){if(!beginSegment(route_.waypoints[routeIndex_]))return reject(FeedbackReason::ROUTE_MISMATCH);}
        else {if(o.confirmedFeet!=15||o.bellySupported)return reject(FeedbackReason::SUPPORT_INVALID);if(!beginRise())return reject(FeedbackReason::ROUTE_MISMATCH);}}
      else {state_=AdaptiveStandState::VERIFY_STAND;verifySince_=now;}
    }
  }
  if(state_==AdaptiveStandState::OBSERVE){state_=AdaptiveStandState::CLASSIFY;FeedbackFrame f;f.reason=FeedbackReason::OK;return f;}
  if(state_==AdaptiveStandState::CLASSIFY){
    if(!o.stationary)return reject(FeedbackReason::UNRECOVERABLE_POSE);
    if(adjust_){
      // A held, verified fixture stand: the joints must still be the retained
      // solution (the declared fixture body pose is not a height measurement).
      if(!o.externalSupportConfirmed)return reject(FeedbackReason::SUPPORT_INVALID);
      for(unsigned i=0;i<4;++i){const auto& a=o.actual.legs[i];const auto& t=previous_.target.legs[i];
        if(std::abs(a.hip-t.hip)>policy_.maxTrackingErrorRad||std::abs(a.upper-t.upper)>policy_.maxTrackingErrorRad||
           std::abs(a.lower-t.lower)>policy_.maxTrackingErrorRad)return reject(FeedbackReason::ROUTE_MISMATCH);}
      poseClass_=ObservedPoseClass::SUPPORTED_STAND;
      if(!beginRise())return reject(FeedbackReason::IK_FAILURE);
    } else if(entryActive_){
      // Recognition is a broad multi-joint region around the route head, NOT
      // the certified route tube; the staged entry is what reaches the tube.
      const auto head=pairedProneSample(0,0,true);if(!head.target.valid)return reject(FeedbackReason::ROUTE_REQUIRED);
      double actual[12]{},nominal[12]{};
      for(unsigned i=0;i<12;++i){const auto& a=o.actual.legs[i/3];const auto& n=head.target.legs[i/3];
        actual[i]=i%3==0?a.hip:i%3==1?a.upper:a.lower;nominal[i]=i%3==0?n.hip:i%3==1?n.upper:n.lower;}
      prone_=classifyFixtureProne(actual,nominal,o.externalSupportConfirmed,o.stationary,request_.proneMaxDeviationRad,request_.proneMaxRmsRad);
      if(prone_.kind!=FixturePoseKind::PRONE_COMPATIBLE){poseClass_=ObservedPoseClass::UNKNOWN;return reject(FeedbackReason::UNRECOVERABLE_POSE);}
      // The plan was built from the armed pose; the observation must still be it.
      for(unsigned i=0;i<12;++i){const auto& s=entry_.start.legs[i/3];const double q=i%3==0?s.hip:i%3==1?s.upper:s.lower;
        if(std::abs(q-actual[i])>policy_.maxTrackingErrorRad)return reject(FeedbackReason::ROUTE_MISMATCH);}
      for(unsigned i=0;i<12;++i){const auto& g=entry_.goal.legs[i/3];const double q=i%3==0?g.hip:i%3==1?g.upper:g.lower;
        if(std::abs(q-nominal[i])>1e-12)return reject(FeedbackReason::ROUTE_MISMATCH);}
      previous_=LocomotionFrame{};previous_.target=entry_.start;previous_.target.valid=true;
      previous_.cartesian.body=head.cartesian.body;previous_.cartesian.valid=true;previous_.status=GaitStatus::OK;
      poseClass_=ObservedPoseClass::BELLY_FOLDED;entryStage_=0;
      if(!beginEntryStage())return reject(FeedbackReason::ROUTE_MISMATCH);
    } else {
    previous_=observedSolution(o);if(!previous_.target.valid)return reject(FeedbackReason::IK_FAILURE);
    if(route_.pairedProne){
      if(request_.targetHeightM!=.150||!route_.geometryIdentityVerified||route_.source==EvidenceSource::UNKNOWN||
        (policy_.mode==FeedbackMode::PHYSICAL_REVIEW&&route_.source!=EvidenceSource::EXTERNAL_MEASURED&&route_.source!=EvidenceSource::OFFLINE_CERTIFIED))return reject(FeedbackReason::ROUTE_REQUIRED);
      const auto cursor=pairedProneCursor(o,policy_);
      const int knot=policy_.mode==FeedbackMode::FIXTURE_REVIEW&&policy_.routeTubeReviewed?cursor.segment:pairedProneMatch(o,policy_);if(knot<0)return reject(FeedbackReason::ROUTE_MISMATCH);
      entry_knot_=knot;entry_progress_=policy_.mode==FeedbackMode::FIXTURE_REVIEW?cursor.progress:0;
      const auto nominal=pairedProneSample(knot==static_cast<int>(kPairedProneSegments)?knot-1:knot,knot==static_cast<int>(kPairedProneSegments)?1:0,policy_.fixtureLimitVariant,entry_progress_);
      for(unsigned i=0;i<4;++i){const auto x=o.actual.legs[i],y=nominal.target.legs[i];entry_offset_[i]={x.hip-y.hip,x.upper-y.upper,x.lower-y.lower};}
      poseClass_=knot==0?ObservedPoseClass::BELLY_FOLDED:ObservedPoseClass::PARTIAL_SUPPORT;
      if(knot==static_cast<int>(kPairedProneSegments)){state_=AdaptiveStandState::VERIFY_STAND;verifySince_=now;}
      else {pairedSegment_=static_cast<uint16_t>(knot);if(!beginPairedSegment())return reject(FeedbackReason::ROUTE_MISMATCH);}
    } else if(o.confirmedFeet==15&&!o.bellySupported){
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
  }
  if(state_==AdaptiveStandState::VERIFY_STAND||state_==AdaptiveStandState::STAND){
    FeedbackFrame f;
    if((policy_.mode==FeedbackMode::FIXTURE_REVIEW?!o.externalSupportConfirmed:(o.confirmedFeet!=15||o.bellySupported))||!o.stationary||(policy_.mode!=FeedbackMode::FIXTURE_REVIEW&&std::abs(o.body.translationM.z-request_.targetHeightM)>policy_.heightToleranceM)){
      if(state_==AdaptiveStandState::STAND)return reject(o.confirmedFeet!=15||o.bellySupported?FeedbackReason::SUPPORT_INVALID:FeedbackReason::TRACKING_ERROR);
      if(now-verifySince_>policy_.segmentTimeoutS)return reject(FeedbackReason::SEGMENT_TIMEOUT);
      f.reason=FeedbackReason::VERIFY_PENDING;reason_=f.reason;return f;
    }
    for(unsigned i=0;i<4&&policy_.mode!=FeedbackMode::FIXTURE_REVIEW;++i){auto fk=contactForwardKinematics(static_cast<LegId>(i),o.actual.legs[i]);Vector3 world;
      if(fk.status!=ContactStatus::OK||fk.support!=SupportMode::NOMINAL_STRIP||!pointToWorld(o.body,fk.referenceM,world)||std::abs(world.z)>policy_.contactToleranceM)return reject(FeedbackReason::SUPPORT_INVALID);}
    state_=AdaptiveStandState::STAND;reason_=FeedbackReason::OK;f.reason=reason_;f.body=o.body;
    // Nominal confirmation emits NO actuator frame. Existing pose is held by its owner.
    f.physicalReadiness=physicalReady_;return f;
  }
  if(endpoint_.externalSupportRequired&&!o.externalSupportConfirmed)return reject(FeedbackReason::SUPPORT_INVALID);
  if(entryPending()&&route_.supervisedSegments&&entryApproved_!=entryStage_){FeedbackFrame f;f.reason=FeedbackReason::ASSESSMENT_REQUIRED;return f;}
  if(!entryPending()&&route_.pairedProne&&route_.supervisedSegments&&approvedSegment_!=pairedSegment_&&
     !(approvedThrough_!=UINT16_MAX&&pairedSegment_<=approvedThrough_)){FeedbackFrame f;f.reason=FeedbackReason::ASSESSMENT_REQUIRED;return f;}
  return generate(now);
}
bool standHeightReachable(double h,const FeedbackPolicy& p){
  if(!std::isfinite(h)||h<=0||h>.150)return false;
  if(std::abs(h-.150)<1e-12)return true;                 // the certified entry pose itself
  StandDefinition d=canonicalStandDefinition();d.lowBodyHeightM=h;d.standBodyHeightM=.150;
  LegJointAngles seeds[4];for(unsigned i=0;i<4;++i)seeds[i]=d.standSeed[i];
  for(int k=20;k>=0;--k){                                // whole descent, stand -> h, chained seeds
    const auto s=sampleStandPath(d,k/20.0,seeds);
    if(s.status!=StandStatus::OK||!heightTargetInRegion(s.target))return false;
    for(unsigned i=0;i<4;++i){seeds[i]=s.target.legs[i];const auto& q=s.target.legs[i];
      const double v[3]={q.hip,q.upper,q.lower};
      for(unsigned j=0;j<3;++j)if(!(v[j]>=p.jointLimits[i][j].lower&&v[j]<=p.jointLimits[i][j].upper))return false;}
  }
  return true;
}
} }
