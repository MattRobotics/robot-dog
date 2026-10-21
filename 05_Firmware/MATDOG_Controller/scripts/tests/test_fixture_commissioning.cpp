// Fixture-commissioning composition: production MotionSession, Coordinator,
// AdaptiveStand, entry planner, Executor, mapping and Safe Actuator policy.
// The start pose is the MEASURED 2026-10-13 prone vector (degrees, rounded to
// 0.001 deg from the encoder log). Limits, transforms and the entry
// certificate are SYNTHETIC TEST DATA built here; no hardware is involved.
#define main prior_motion_boundary_main
#include "test_motion_service_executor.cpp"
#undef main
#include "../../src/execution/CommissioningPolicy.h"
#include "../../src/motion/FixtureEntryPlanner.h"
#include "../../src/servo/ServoPopulation.h"
#include <array>
#include <algorithm>

namespace {
constexpr double kDeg=3.14159265358979323846/180;
// LF, RF, RH, LH x HIP, UPPER, LOWER - encoder-derived after RESTORE gen-1.
const double kMeasuredDeg[12]={-2.285,80.156,-91.758, 2.373,79.541,-90.615, -2.549,82.529,-91.406, 2.285,82.354,-91.406};
// How many ticks each folded LOWER rests outside its hard limit (rest is 2..4
// ticks inside the recorded contact, the limit is contact + 5 ticks inset).
const unsigned kLowerOutside[4]={2,1,2,3};

JointIdentity slotIdentity(unsigned i){
 const auto* unit=matdog::servo::legServoAt((i/3)*3+2-i%3);JointIdentity id;id.leg=static_cast<Leg>(i/3);id.joint=static_cast<JointKind>(i%3);
 std::strncpy(id.physical_unit,unit->physical_unit,sizeof(id.physical_unit)-1);return id;
}
uint16_t tickOf(Rig& r,unsigned i,double q){
 const auto* t=r.policy.transforms().find(slotIdentity(i),r.geometry.provenanceTag());uint16_t tick=0;
 CHECK(t&&resolveUrdfQToRaw(r.geometry,geometry_data::kProvenance,*t,static_cast<MicroRad>(std::llround(q*1e6)),&tick)==TargetResolveStatus::OK);return tick;
}
// The producer's own conversion (OperationalFeedback): RAW -> radians through the canonical resolver.
double qOf(Rig& r,unsigned i,uint16_t tick){
 const auto* t=r.policy.transforms().find(slotIdentity(i),r.geometry.provenanceTag());double q=0;
 CHECK(t&&resolveRawToRadians(r.geometry,geometry_data::kProvenance,*t,tick,&q)==TargetResolveStatus::OK);return q;
}
// URDF interval for every joint; each LOWER's folded-side edge sits just
// inside the measured rest, as the gen-1 contact inset does on the robot.
void commissioningLimits(Rig& r,double limits[12][2],double measured[12]){
 r.policy.limits().clear();
 for(unsigned i=0;i<12;++i){
  const auto id=slotIdentity(i);const auto* g=r.geometry.findJoint(id);CHECK(g!=nullptr);
  uint16_t a=tickOf(r,i,g->urdf_lower*1e-6),b=tickOf(r,i,g->urdf_upper*1e-6);
  uint16_t lo=std::min(a,b)+1,hi=std::max(a,b)-1;
  measured[i]=kMeasuredDeg[i]*kDeg;
  const uint16_t rest=tickOf(r,i,measured[i]);measured[i]=qOf(r,i,rest); // encoder quantization
  if(i%3==2){const uint16_t lower_end=tickOf(r,i,g->urdf_lower*1e-6);
   // Folded side = the tick end that maps to the URDF lower (negative) limit.
   if(lower_end<rest){lo=rest+kLowerOutside[i/3];}else{hi=rest-kLowerOutside[i/3];}}
  JointLimit l;l.identity=id;l.geometry=r.geometry.provenanceTag();l.origin=CalibrationOrigin::LIVE_SESSION;l.state=EvidenceState::PROMOTED;l.present=true;l.min_tick=lo;l.max_tick=hi;
  CHECK(r.policy.limits().admit(l));
  const double qa=qOf(r,i,lo),qb=qOf(r,i,hi);limits[i][0]=std::min(qa,qb);limits[i][1]=std::max(qa,qb);
 }
}
void observe(motion::AdaptiveObservation& o,const double q[12],uint32_t now){
 const auto head=motion::pairedProneSample(0,0,true);
 o.body=head.cartesian.body;o.stampS=now/1000.;o.stationary=true;o.flatPlaneConfirmed=true;o.externalSupportConfirmed=true;
 o.heightSource=o.supportSource=motion::EvidenceSource::EXTERNAL_MEASURED; // simulated fixture declaration
 o.actual.valid=true;for(unsigned i=0;i<4;++i)o.actual.legs[i]={q[i*3],q[i*3+1],q[i*3+2]};
}
double slot(const motion::JointTargetFrame& f,unsigned i){const auto& q=f.legs[i/3];return i%3==0?q.hip:i%3==1?q.upper:q.lower;}

struct Bench {
 Rig r;double limits[12][2]{},measured[12]{};execution::CommissioningParameters params;execution::ReviewedMotionRuntime runtime;
 MotionSession session;MotionCoordinator coordinator;MotionService service;LocalTrialEvidence evidence;uint32_t now=100;
 Bench(double tracking=.006){
  params.tracking_tolerance_rad=tracking;
  CHECK(r.arbiter.release(r.facts.lease)==AuthorityResult::RELEASED);r.facts.lease={};
  commissioningLimits(r,limits,measured);
  CHECK(execution::buildCommissioningRuntime(params,limits,1,r.policy.epoch(),&runtime));
  r.envelope=runtime.execution;
  for(unsigned i=0;i<12;++i){r.feedback.actual_rad[i]=measured[i];r.feedback.torque_on[i]=false;r.feedback.read_ok[i]=true;r.feedback.sampled_at_ms[i]=now;}
  session.begin(&r.policy,&r.runtime,&r.executor,&r.arbiter,&r.geometry,&geometry_data::kProvenance);
  evidence.scope=TrialScope::FIXTURE_COMMISSIONING;evidence.generation=1;evidence.expires_at_ms=100000000;evidence.prime_clamp_max_ticks=params.prime_clamp_ticks;
  evidence.local_operator_present=evidence.physical_power_cut_verified=evidence.fixture_support_verified=evidence.installation_verified=evidence.servo_output_ceiling_verified=evidence.imu_fresh=true;
  r.facts.permit_expires_at_ms=100000000;
 }
 void fresh(){++now;for(unsigned i=0;i<12;++i)r.feedback.sampled_at_ms[i]=now;}
 // PRIME -> ENERGIZE -> settle -> ARM, exactly as the Controller ticks it.
 bool arm(){
  if(!session.request(evidence,r.facts,r.envelope,r.feedback,true,now))return false;
  for(unsigned k=0;k<24;++k){fresh();session.update(evidence,r.facts,r.feedback,now);}
  for(auto& on:r.feedback.torque_on)on=true;
  fresh();session.update(evidence,r.facts,r.feedback,now); // not settled yet: waits
  CHECK(session.state()==MotionSessionState::ARMING&&!r.executor.active());
  for(unsigned i=0;i<12;++i)r.feedback.actual_rad[i]=session.primedRadians()[i]; // servo reached the primed tick
  fresh();session.update(evidence,r.facts,r.feedback,now);
  if(session.state()!=MotionSessionState::ARMED)return false;
  r.facts.lease=session.lease();return true;
 }
 motion::FixtureEntryCertificate certificate(double tube=.01){
  motion::FixtureEntryCertificate c;c.present=true;c.tubeRad=tube;c.goal=motion::pairedProneSample(0,0,true).target;c.start.valid=true;
  for(unsigned i=0;i<12;++i){auto& q=c.start.legs[i/3];(i%3==0?q.hip:i%3==1?q.upper:q.lower)=measured[i];}
  std::strcpy(c.sha256,"0000000000000000000000000000000000000000000000000000000000000001");return c;
 }
 bool configure(const motion::FixtureEntryCertificate& c){
  coordinator.begin(&r.executor,&r.policy);coordinator.bindSession(&session);
  motion::AcquisitionRoute route;route.pairedProne=true;route.supervisedSegments=true;route.timeScale=runtime.stand_time_scale;
  route.geometryIdentityVerified=true;route.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
  if(!coordinator.configure(runtime.feedback,runtime.walk,&route))return false;
  CommissioningStandConfig cc;cc.enabled=true;cc.certificate=c;cc.minStageDurationS=params.entry_min_stage_s;
  cc.proneMaxDeviationRad=params.prone_max_deviation_rad;cc.proneMaxRmsRad=params.prone_max_rms_rad;
  if(!coordinator.configureCommissioning(cc))return false;
  service.begin(&r.executor,&coordinator);auto rd=ready();rd.local_operator_owns_session=true;rd.walk_qualified=false;service.refresh(rd,now);return true;
 }
 motion::MotionAck stand(){auto c=command(motion::Intent::STAND,1,motion::ClientOwner::LOCAL);c.target_height_m=.150;c.issued_at_ms=now;c.ttl_ms=1000;return service.command(c,now);}
};

void primeClampAndRefusals(){
 {Bench b;b.evidence.prime_clamp_max_ticks=0;CHECK(!b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));
  REASON(b.session.reason(),"PRIME_REQUIRES_OFF_AND_LIMITS");CHECK(b.r.backend.writes.empty()&&b.r.arbiter.current()==ActuatorAuthority::NONE);}
 {Bench b;b.evidence.prime_clamp_max_ticks=2;CHECK(!b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now)); // LH needs 4
  CHECK(b.r.backend.writes.empty());}
 // A HIP or UPPER outside its limit is never re-aligned: abnormal pose, refused.
 {Bench b;b.r.policy.limits().clear();double lim[12][2],m[12];commissioningLimits(b.r,lim,m);
  const auto id=slotIdentity(1);auto l=*b.r.policy.limits().find(id,b.r.geometry.provenanceTag());
  const uint16_t rest=tickOf(b.r,1,b.measured[1]);if(rest>l.min_tick+2&&rest<l.max_tick-2){if(l.min_tick<rest){l.min_tick=rest+2;}CHECK(b.r.policy.limits().admit(l));}
  CHECK(!b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));REASON(b.session.reason(),"PRIME_REQUIRES_OFF_AND_LIMITS");CHECK(b.r.backend.writes.empty());}
 // A LOWER outside its UNFOLDED end is never re-aligned either.
 {Bench b;const auto id=slotIdentity(5);auto l=*b.r.policy.limits().find(id,b.r.geometry.provenanceTag());
  const uint16_t rest=tickOf(b.r,5,b.measured[5]);const uint16_t lower_end=tickOf(b.r,5,b.r.geometry.findJoint(id)->urdf_lower*1e-6);
  if(lower_end<rest){l.min_tick=rest-200;l.max_tick=rest-2;}else{l.min_tick=rest+2;l.max_tick=rest+200;}CHECK(b.r.policy.limits().admit(l));
  CHECK(!b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));CHECK(b.r.backend.writes.empty());}
 {Bench b;b.evidence.scope=TrialScope::SUPERVISED_FIXTURE_STAND;CHECK(!b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));}
 {Bench b;b.r.feedback.torque_on[4]=true;CHECK(!b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));}
 {Bench b;b.evidence.fixture_support_verified=false;CHECK(!b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));REASON(b.session.reason(),"FIXTURE_SUPPORT_REQUIRED");}
 {Bench b;b.now=5000;b.fresh();b.r.feedback.sampled_at_ms[7]=100;CHECK(!b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));REASON(b.session.reason(),"STALE_FEEDBACK");}
 // Exact readback prime for HIP/UPPER, inward clamp only for the 4 LOWER.
 {Bench b;CHECK(b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));
  CHECK(b.session.primeClampMask()==((1u<<2)|(1u<<5)|(1u<<8)|(1u<<11)));
  for(unsigned i=0;i<12;++i){
   if(i%3==2){CHECK(b.session.primeClampTicks(i)==kLowerOutside[i/3]+kPrimeInwardMarginTicks);
    // The clamp moves AWAY from the folded stop: LOWER angle increases.
    CHECK(b.session.primedRadians()[i]>b.measured[i]);CHECK(b.session.primedRadians()[i]-b.measured[i]<(b.params.prime_clamp_ticks+.5)*2*3.14159265358979/4096);}
   else CHECK(b.session.primedRadians()[i]==b.measured[i]);}
  for(unsigned k=0;k<12;++k){b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);}
  CHECK(b.r.backend.writes.size()==12);
  for(unsigned i=0;i<12;++i)CHECK(b.r.policy.limits().find(slotIdentity(i),b.r.geometry.provenanceTag())->contains(b.r.backend.writes[i].tick));
  // Drift away from the rest->primed segment revokes before any torque.
  b.r.feedback.actual_rad[2]=b.measured[2]-.02;b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);
  CHECK(b.session.safeOffRequired());REASON(b.session.reason(),"PRIME_DRIFT");}
 // A joint that never settles at its primed tick times out instead of arming.
 {Bench b;CHECK(b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));
  for(unsigned k=0;k<24;++k){b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);}
  for(auto& on:b.r.feedback.torque_on)on=true;
  for(unsigned k=0;k<40&&!b.session.safeOffRequired();++k){b.now+=50;for(unsigned i=0;i<12;++i)b.r.feedback.sampled_at_ms[i]=b.now;b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);}
  CHECK(!b.r.executor.active());CHECK(b.session.safeOffRequired());}
}

void classifierOnMeasuredPose(){
 double q[12];for(unsigned i=0;i<12;++i)q[i]=kMeasuredDeg[i]*kDeg;
 std::array<double,12> a{},n{};const auto head=motion::pairedProneSample(0,0,true);
 for(unsigned i=0;i<12;++i){a[i]=q[i];n[i]=slot(head.target,i);}
 const auto c=motion::classifyFixtureProne(a.data(),n.data(),true,true);
 CHECK(c.kind==motion::FixturePoseKind::PRONE_COMPATIBLE);CHECK(c.maxErrorJointIndex==10);CHECK(c.maxNominalErrorRad>.006);
 auto bad=a;bad[7]=-bad[7];CHECK(motion::classifyFixtureProne(bad.data(),n.data(),true,true).reason==motion::FixturePoseReason::UPPER_BAND);
 bad=a;bad[0]=.5;const auto hip=motion::classifyFixtureProne(bad.data(),n.data(),true,true);CHECK(hip.reason==motion::FixturePoseReason::HIP_BAND&&hip.failedJointIndex==0);
 CHECK(motion::classifyFixtureProne(a.data(),n.data(),false,true).reason==motion::FixturePoseReason::NOT_FIXTURE);
 // The certified 6 mrad route cursor still rejects this pose; recognition is separate.
 motion::AdaptiveObservation o;observe(o,q,100);auto p=execution::CommissioningParameters{};double lim[12][2];
 for(unsigned i=0;i<12;++i){const auto* m=motion::legModel(static_cast<motion::LegId>(i/3));lim[i][0]=m->limits[i%3].lower;lim[i][1]=m->limits[i%3].upper;}
 execution::ReviewedMotionRuntime rt;CHECK(execution::buildCommissioningRuntime(p,lim,1,1,&rt));
 CHECK(motion::pairedProneCursor(o,rt.feedback).segment<0);
}

void entryPlanner(){
 Bench b;const auto c=b.certificate();motion::JointTargetFrame start=c.start;const auto head=c.goal;
 auto plan=motion::planFixtureEntry(start,head,b.runtime.feedback,c,b.params.entry_min_stage_s);
 // The measured LOWER rest is outside the hard limit: the plan refuses it.
 CHECK(plan.status==motion::EntryStatus::LIMIT&&plan.failedJoint%3==2);
 // From the primed (clamped) pose it is inside the certified tube and limits.
 for(unsigned i=2;i<12;i+=3){auto& q=start.legs[i/3];q.lower+=(kLowerOutside[i/3]+kPrimeInwardMarginTicks)*2*3.14159265358979/4096;}
 plan=motion::planFixtureEntry(start,head,b.runtime.feedback,c,b.params.entry_min_stage_s);
 CHECK(plan.status==motion::EntryStatus::OK);CHECK(plan.durationS[0]>=b.params.entry_min_stage_s&&plan.durationS[1]>=b.params.entry_min_stage_s);
 // Stage 0 moves only LOWER, away from the stop; stage 1 never moves LOWER.
 for(unsigned k=0;k<=20;++k){const double u=k/20.;
  const auto s0=motion::fixtureEntrySample(plan,0,u,head.legs[0].hip==0?motion::BodyPose{}:motion::pairedProneSample(0,0,true).cartesian.body);
  const auto s1=motion::fixtureEntrySample(plan,1,u,motion::pairedProneSample(0,0,true).cartesian.body);
  CHECK(s0.target.valid&&s1.target.valid);
  for(unsigned i=0;i<12;++i){if(i%3!=2)CHECK(slot(s0.target,i)==slot(start,i));else CHECK(slot(s0.target,i)>=slot(start,i)-1e-12);
   if(i%3==2)CHECK(std::abs(slot(s1.target,i)-slot(head,i))<1e-12);}}
 const auto end=motion::fixtureEntrySample(plan,1,1,motion::BodyPose{});for(unsigned i=0;i<12;++i)CHECK(std::abs(slot(end.target,i)-slot(head,i))<1e-12);
 // Certificate gates.
 auto none=c;none.present=false;CHECK(motion::planFixtureEntry(start,head,b.runtime.feedback,none,4).status==motion::EntryStatus::CERTIFICATE_REQUIRED);
 auto tight=c;tight.tubeRad=b.params.tracking_tolerance_rad;CHECK(motion::planFixtureEntry(start,head,b.runtime.feedback,tight,4).status==motion::EntryStatus::CERTIFICATE_REQUIRED);
 auto back=start;back.legs[1].lower=c.start.legs[1].lower-.006;CHECK(motion::planFixtureEntry(back,head,b.runtime.feedback,c,4).status!=motion::EntryStatus::OK);
 auto moved=start;moved.legs[3].upper+=.01;CHECK(motion::planFixtureEntry(moved,head,b.runtime.feedback,c,4).status==motion::EntryStatus::START_OUTSIDE_CERTIFIED_REGION);
 auto other=head;other.legs[0].hip+=.001;CHECK(motion::planFixtureEntry(start,other,b.runtime.feedback,c,4).status==motion::EntryStatus::GOAL_MISMATCH);
 auto nan=start;nan.legs[2].lower=NAN;CHECK(motion::planFixtureEntry(nan,head,b.runtime.feedback,c,4).status==motion::EntryStatus::INVALID_INPUT);
 // The narrow computational entry tube covers asynchronous writes too.
 auto narrow=c;narrow.tubeRad=.004;auto policy=b.runtime.feedback;policy.maxTrackingErrorRad=.0035;
 auto slow=motion::planFixtureEntry(start,head,policy,narrow,4);CHECK(slow.status==motion::EntryStatus::OK);
 CHECK(slow.durationS[1]>plan.durationS[1]);
 for(unsigned stage=0;stage<2;++stage){
  const auto& a=stage==0?slow.start:slow.middle;const auto& z=stage==0?slow.middle:slow.goal;
  for(unsigned i=0;i<12;++i){const double delta=std::abs(slot(z,i)-slot(a,i));
   const double step=1.875*delta*policy.samplePeriodS/slow.durationS[stage];
   CHECK(step/2+policy.maxTrackingErrorRad+1e-6<=narrow.tubeRad);}
 }
 // A shifted HIP/UPPER consumes more of the tube and lengthens ALIGN.
 auto shifted=start;shifted.legs[3].upper+=.0002;policy.segmentTimeoutS=30;
 auto adjusted=motion::planFixtureEntry(shifted,head,policy,narrow,4);
 CHECK(adjusted.status==motion::EntryStatus::OK&&adjusted.durationS[1]>slow.durationS[1]);
 // Unsafe duration is refused rather than relaxing a timeout or a tube.
 policy.segmentTimeoutS=4;
 CHECK(motion::planFixtureEntry(start,head,policy,narrow,4).status==motion::EntryStatus::RATE_POLICY);
}

// Full supervised STAND from the measured prone pose, all approvals explicit.
void standFromMeasuredPose(bool approve_phases,double tube=.01,double tracking=.006){
 Bench b(tracking);if(approve_phases)b.params.approval=execution::SegmentApproval::PHASE;
 auto c=b.certificate(tube); // measured torque-OFF rest, as the host certificate tool uses
 CHECK(b.configure(c)); // ADMIT configures before GRANT
 CHECK(b.arm());CHECK(b.r.executor.active());
 const auto ack=b.stand();CHECK(ack.outcome==motion::CommandOutcome::ACCEPTED);
 motion::AdaptiveObservation o;double q[12];for(unsigned i=0;i<12;++i)q[i]=b.r.feedback.actual_rad[i];
 const double armed_lf_lower=q[2];unsigned frames=0,entry_frames=0,max_steps_waiting=0;double max_step=0,max_hip_upper_stage0=0;bool lower_monotone=true;double last_lower[4]={q[2],q[5],q[8],q[11]};
 const size_t writes_at_arm=b.r.backend.writes.size();
 uint32_t& now=b.now;
 while(std::strcmp(b.coordinator.state(),"STAND")&&now<50000000){
  now+=20;observe(o,q,now);b.coordinator.observe(o);b.coordinator.update(b.r.facts,b.r.feedback,now);
  const auto& stand=b.coordinator.stand();
  if(b.coordinator.frame().reason==motion::FeedbackReason::ASSESSMENT_REQUIRED&&!b.coordinator.frame().target.valid){
   ++max_steps_waiting;
   // Nothing moves until the operator approves the current stage/segment.
   if(stand.entryPending())CHECK(b.coordinator.approveEntryStage(stand.entryStage()));
   else if(approve_phases){unsigned last=stand.proneSegment();while(last+1<motion::kPairedProneSegments&&motion::pairedPronePhase(last+1)==motion::pairedPronePhase(stand.proneSegment()))++last;CHECK(b.coordinator.approveProneThrough(last));}
   else CHECK(b.coordinator.approveProneSegment(stand.proneSegment()));
   continue;
  }
  auto frame=b.coordinator.frame();
  if(frame.target.valid&&frame.target.sequence>b.r.executor.status().completed_sequence){
   motion::SegmentAssessment a;a.sequence=frame.target.sequence;a.stampS=now/1000.;a.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
   a.geometryIdentityVerified=a.sweptCollisionClear=a.supportCompatible=a.continuousBoundsConfirmed=a.fixtureTrialEnvelopeReviewed=true;
   CHECK(b.coordinator.assess(a,now));b.coordinator.update(b.r.facts,b.r.feedback,now);
   CHECK(b.r.executor.status().state==ExecutorState::WRITING);
   for(unsigned i=0;i<12;++i)b.r.executor.update(b.r.facts,b.r.feedback,++now);
   for(unsigned i=0;i<12;++i){const double t=b.r.executor.targetRadians()[i];max_step=std::max(max_step,std::abs(t-q[i]));q[i]=t;b.r.feedback.actual_rad[i]=t;b.r.feedback.sampled_at_ms[i]=now+1;}
   ++now;b.r.executor.update(b.r.facts,b.r.feedback,now);
   CHECK(b.r.executor.status().completed_sequence==frame.target.sequence);++frames;
   if(stand.entryPending()||!std::strcmp(b.coordinator.state(),"ENTRY_ALIGN")){++entry_frames;}
   if(!std::strcmp(b.coordinator.state(),"ENTRY_DISENGAGE")){
    for(unsigned i=0;i<12;++i)if(i%3!=2)max_hip_upper_stage0=std::max(max_hip_upper_stage0,std::abs(q[i]-b.session.primedRadians()[i]));
    for(unsigned l=0;l<4;++l){if(q[l*3+2]<last_lower[l]-1e-12)lower_monotone=false;last_lower[l]=q[l*3+2];}}
  }
  if(!std::strcmp(b.coordinator.state(),"REJECTED")){std::printf("commissioning stand rejected: %s\n",motion::feedbackReasonName(b.coordinator.frame().reason));break;}
 }
 CHECK(!std::strcmp(b.coordinator.state(),"STAND"));
 CHECK(frames>200&&entry_frames>0&&max_steps_waiting>=3);
 CHECK(max_step<=b.params.max_step_rad+1e-9);CHECK(lower_monotone);CHECK(max_hip_upper_stage0<1e-12);
 CHECK(q[2]>armed_lf_lower);
 // Every write passed the unchanged Safe Actuator limits (backend saw them).
 for(size_t k=writes_at_arm;k<b.r.backend.writes.size();++k){bool inside=false;
  for(unsigned i=0;i<12;++i){const auto* l=b.r.policy.limits().find(slotIdentity(i),b.r.geometry.provenanceTag());const auto* g=b.r.geometry.findJoint(slotIdentity(i));
   if(g&&g->bus_id==b.r.backend.writes[k].bus)inside=l&&l->contains(b.r.backend.writes[k].tick);}
  CHECK(inside);}
 // STAND reached through the certified fixture route (C4 targets, not body height).
 const auto c4=motion::pairedProneSample(250,1,true).target;for(unsigned i=0;i<12;++i)CHECK(std::abs(q[i]-slot(c4,i))<1e-9);
 std::printf("COMMISSIONING_STAND approval=%s tube=%.6f tracking=%.6f frames=%u entry_frames=%u writes=%zu entry_s=%.2f+%.2f max_step=%.6f\n",approve_phases?"PHASE":"SEGMENT",tube,tracking,frames,entry_frames,b.r.backend.writes.size(),b.coordinator.entryPlan().durationS[0],b.coordinator.entryPlan().durationS[1],max_step);
}

void standRefusalsAndStop(){
 // Unknown pose: classifier rejects before any entry frame.
 {Bench b;auto c=b.certificate();CHECK(b.configure(c));CHECK(b.arm());CHECK(b.stand().outcome==motion::CommandOutcome::ACCEPTED);const size_t writes=b.r.backend.writes.size();
  motion::AdaptiveObservation o;double q[12];for(unsigned i=0;i<12;++i)q[i]=b.r.feedback.actual_rad[i];q[0]=.6; // hip way off: not prone
  for(unsigned k=0;k<4;++k){b.now+=20;observe(o,q,b.now);b.coordinator.observe(o);b.coordinator.update(b.r.facts,b.r.feedback,b.now);}
  REASON(b.coordinator.state(),"REJECTED");CHECK(b.r.backend.writes.size()==writes);CHECK(b.r.executor.status().safe_off_required);}
 // Certificate from a different start: STAND refused at the service.
 {Bench b;auto c=b.certificate();c.start.legs[1].upper+=.02;CHECK(b.configure(c));CHECK(b.arm());
  const auto ack=b.stand();CHECK(ack.outcome==motion::CommandOutcome::REJECTED);REASON(ack.reason,"START_OUTSIDE_CERTIFIED_REGION");}
 // No certificate: commissioning cannot even be configured.
 {Bench b;auto c=b.certificate();c.present=false;CHECK(!b.configure(c));}
 // STOP during the LOWER egress: SAFE_OFF, no further writes.
 {Bench b;auto c=b.certificate();CHECK(b.configure(c));CHECK(b.arm());CHECK(b.stand().outcome==motion::CommandOutcome::ACCEPTED);
  motion::AdaptiveObservation o;double q[12];for(unsigned i=0;i<12;++i)q[i]=b.r.feedback.actual_rad[i];
  for(unsigned k=0;k<3;++k){b.now+=20;observe(o,q,b.now);b.coordinator.observe(o);b.coordinator.update(b.r.facts,b.r.feedback,b.now);}
  CHECK(b.coordinator.approveEntryStage(0));
  b.now+=20;observe(o,q,b.now);b.coordinator.observe(o);b.coordinator.update(b.r.facts,b.r.feedback,b.now);
  REASON(b.coordinator.state(),"ENTRY_DISENGAGE");
  auto stop=command(motion::Intent::STOP,2,motion::ClientOwner::LOCAL);stop.issued_at_ms=b.now;b.service.command(stop,b.now);
  CHECK(b.session.safeOffRequired()&&b.r.executor.status().safe_off_required);const size_t writes=b.r.backend.writes.size();
  for(unsigned k=0;k<20;++k)b.r.executor.update(b.r.facts,b.r.feedback,++b.now);
  CHECK(b.r.backend.writes.size()==writes);
  // Approvals are refused while not current/pending.
  CHECK(!b.coordinator.approveProneSegment(0));}
 // Out-of-cap runtime parameters are refused, never clamped.
 {execution::CommissioningParameters p;const char* why=nullptr;
  CHECK(!execution::setCommissioningParameter(p,"tracking_tolerance_rad","0.05",&why));REASON(why,"TRACKING_TOLERANCE");
  CHECK(!execution::setCommissioningParameter(p,"max_velocity_rad_s","1",&why));
  CHECK(!execution::setCommissioningParameter(p,"prime_clamp_ticks","17",&why));REASON(why,"PRIME_CLAMP_TICKS");
  CHECK(execution::setCommissioningParameter(p,"prime_clamp_ticks","16",&why)&&p.prime_clamp_ticks==16);p.prime_clamp_ticks=12;
  CHECK(!execution::setCommissioningParameter(p,"inset_ticks","1",&why));
  CHECK(!execution::setCommissioningParameter(p,"approval","ANY",&why));
  CHECK(!execution::setCommissioningParameter(p,"bypass_limits","1",&why));REASON(why,"UNKNOWN_PARAMETER");
  CHECK(execution::setCommissioningParameter(p,"tracking_tolerance_rad","0.008",&why)&&p.tracking_tolerance_rad==.008);
  CHECK(execution::setCommissioningParameter(p,"approval","PHASE",&why)&&p.approval==execution::SegmentApproval::PHASE);
  execution::RuntimeTestMode m;CHECK(execution::parseRuntimeTestMode("FIXTURE_COMMISSIONING",&m)&&m==execution::RuntimeTestMode::FIXTURE_COMMISSIONING);
  CHECK(!execution::parseRuntimeTestMode("UNLOCKED",&m));
  const TrialScope scopes[]={TrialScope::FIXTURE_MICRO,TrialScope::SUPERVISED_FIXTURE_STAND,
   TrialScope::SUPERVISED_FIXTURE_WALK,TrialScope::AUTONOMOUS_STAND,TrialScope::AUTONOMOUS_WALK,TrialScope::FIXTURE_COMMISSIONING};
  for(auto scope:scopes){
   CHECK(!execution::runtimeModeAllowsScope(execution::RuntimeTestMode::DIAGNOSTIC,scope));
   CHECK(execution::runtimeModeAllowsScope(execution::RuntimeTestMode::FIXTURE_COMMISSIONING,scope)==(scope==TrialScope::FIXTURE_COMMISSIONING));
   CHECK(execution::runtimeModeAllowsScope(execution::RuntimeTestMode::GROUND_VALIDATION,scope)==(scope!=TrialScope::FIXTURE_COMMISSIONING));
   CHECK(execution::runtimeModeAllowsScope(execution::RuntimeTestMode::AUTONOMOUS,scope)==(scope==TrialScope::AUTONOMOUS_STAND||scope==TrialScope::AUTONOMOUS_WALK));
  }
  char text[900];CHECK(execution::formatCommissioningParameters(p,text,sizeof(text)));}
}
}

#ifndef MATDOG_FIXTURE_COMMISSIONING_NO_MAIN
int main(){
 classifierOnMeasuredPose();primeClampAndRefusals();entryPlanner();
 standFromMeasuredPose(false);standFromMeasuredPose(true);standRefusalsAndStop();
 standFromMeasuredPose(false,.004,.0035);standFromMeasuredPose(true,.004,.0035);
 std::printf("FIXTURE_COMMISSIONING %s checks=%d failures=%d hardware=false\n",failures?"FAIL":"PASS",checks,failures);return failures?1:0;
}
#endif
