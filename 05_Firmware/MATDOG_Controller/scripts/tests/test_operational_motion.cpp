// Operator directive 2026-10-14, end to end, offline: READY with MOTION
// authority held idle by the Controller -> authenticated Dashboard STAND ->
// session arms itself (prime at measured pose -> energize -> arm, Safe
// Actuator) -> PRONE recognized (broad band) -> entry planned FROM THE MEASURED
// pose (no certificate tube) and checked against limits + CAD keep-out ->
// continuous certified fixture route -> STAND verified -> STOP = SAFE_OFF.
// Production MotionService/Coordinator/AdaptiveStand/Session/Executor/policy;
// kinematic servo double; measured 2026-10-13 prone. NO hardware.
#define MATDOG_FIXTURE_COMMISSIONING_NO_MAIN
#include "test_fixture_commissioning.cpp"

namespace {
struct TestArmer : execution::SessionArmer {
  Bench* b=nullptr;int calls=0;const char* refuse=nullptr;
  const char* armForCommand(uint32_t now) override {
    ++calls;if(refuse)return refuse;
    // As Controller::armOperationalSession: evidence from facts, adopt the held lease.
    b->evidence.generation=100+calls;b->evidence.expires_at_ms=now+3000;b->r.facts.permit_generation=b->evidence.generation;b->r.facts.permit_expires_at_ms=now+3000;
    return b->session.request(b->evidence,b->r.facts,b->r.envelope,b->r.feedback,true,now)?nullptr:b->session.reason();
  }
};
struct Operational {
  Bench b;TestArmer armer;core::AuthorityLease held{};
  explicit Operational(double fold_offset_deg=0,double keepout_deg=-92.070442,double keepout_sign=1){
    for(unsigned i=0;i<12;++i)if(i%3==1){b.r.feedback.actual_rad[i]+=fold_offset_deg*kDeg;}
    // READY: the Controller holds MOTION authority idle (OperationalReadiness GRANT_AUTHORITY).
    CHECK(b.r.arbiter.request(ActuatorAuthority::MOTION,OperatingMode::RUN,&held)==AuthorityResult::GRANTED);
    b.r.facts.lease=held;
    b.coordinator.begin(&b.r.executor,&b.r.policy);b.coordinator.bindSession(&b.session);
    armer.b=&b;b.coordinator.bindArmer(&armer);
    motion::AcquisitionRoute route;route.pairedProne=true;route.supervisedSegments=false;route.timeScale=b.runtime.stand_time_scale;
    route.geometryIdentityVerified=true;route.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
    CHECK(b.coordinator.configure(b.runtime.feedback,b.runtime.walk,&route));
    CommissioningStandConfig cc;cc.enabled=true;cc.measuredStart=true;cc.minStageDurationS=b.params.entry_min_stage_s;
    cc.proneMaxDeviationRad=b.params.prone_max_deviation_rad;cc.proneMaxRmsRad=b.params.prone_max_rms_rad;
    cc.keepoutCount=1;auto& k=cc.keepouts[0];k.id="LF_UPPER_LOWER_CONTACT";k.a=k.b=2;k.ca=keepout_sign;k.cb=0;k.bound=keepout_sign*keepout_deg*kDeg;
    CHECK(b.coordinator.configureCommissioning(cc));   // no certificate needed
    b.service.begin(&b.r.executor,&b.coordinator);refresh();
  }
  MotionReadiness readiness(){auto r=ready();r.walk_qualified=false;r.local_operator_owns_session=false;return r;}
  void refresh(){b.service.refresh(readiness(),b.now);}
  // As every Controller tick: the measured pose is published before a command is served.
  void publish(){motion::AdaptiveObservation o;double q[12];for(unsigned i=0;i<12;++i)q[i]=b.r.feedback.actual_rad[i];observe(o,q,b.now);b.coordinator.observe(o);}
  motion::MotionAck stand(uint32_t seq=1){publish();auto c=command(motion::Intent::STAND,seq,motion::ClientOwner::DASHBOARD);c.target_height_m=.150;c.issued_at_ms=b.now;c.ttl_ms=1000;return b.service.command(c,b.now);}
  // One Controller tick: session lifecycle, coordinator, service, with a servo that follows.
  void tickArming(){
    b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);
    if(b.session.state()==MotionSessionState::ARMING){
      for(auto& on:b.r.feedback.torque_on)on=true;
      for(unsigned i=0;i<12;++i)b.r.feedback.actual_rad[i]=b.session.primedRadians()[i];
    }
    // The Controller publishes the (declared-support) observation every tick.
    motion::AdaptiveObservation o;double q[12];for(unsigned i=0;i<12;++i)q[i]=b.r.feedback.actual_rad[i];
    observe(o,q,b.now);b.coordinator.observe(o);
    b.coordinator.update(b.r.facts,b.r.feedback,b.now);refresh();
  }
};

void operationalStandEndToEnd(double fold_offset_deg){
  Operational op(fold_offset_deg);auto& b=op.b;
  CHECK(!b.r.executor.active()&&b.r.backend.writes.empty());          // READY: authority held, nothing energized
  const auto ack=op.stand();
  CHECK(ack.outcome==motion::CommandOutcome::ACCEPTED);REASON(ack.reason,"OK");
  CHECK(op.armer.calls==1&&b.coordinator.arming());
  for(unsigned k=0;k<40&&b.coordinator.arming();++k)op.tickArming();
  CHECK(!b.coordinator.arming()&&b.session.state()==MotionSessionState::ARMED&&b.r.executor.active());
  CHECK(b.session.lease().generation==op.held.generation);              // adopted, not re-minted
  CHECK(b.service.activeAck().outcome==motion::CommandOutcome::ACCEPTED); // not aborted by the arm generation change
  REASON(b.coordinator.lastEntryReason(),"");
  CHECK(b.coordinator.entryPlan().status==motion::EntryStatus::OK);
  motion::AdaptiveObservation o;double q[12];for(unsigned i=0;i<12;++i)q[i]=b.r.feedback.actual_rad[i];
  unsigned frames=0,approval_waits=0;double max_step=0;uint32_t& now=b.now;
  while(std::strcmp(b.coordinator.state(),"STAND")&&now<50000000){
    now+=20;b.r.facts.permit_expires_at_ms=b.evidence.expires_at_ms=now+3000;  // Controller renews while the operator heartbeat is fresh
    observe(o,q,now);b.coordinator.observe(o);b.coordinator.update(b.r.facts,b.r.feedback,now);op.refresh();
    if(b.coordinator.frame().reason==motion::FeedbackReason::ASSESSMENT_REQUIRED&&!b.coordinator.frame().target.valid){++approval_waits;continue;}
    auto frame=b.coordinator.frame();
    if(frame.target.valid&&frame.target.sequence>b.r.executor.status().completed_sequence){
      motion::SegmentAssessment a;a.sequence=frame.target.sequence;a.stampS=now/1000.;a.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
      a.geometryIdentityVerified=a.sweptCollisionClear=a.supportCompatible=a.continuousBoundsConfirmed=a.fixtureTrialEnvelopeReviewed=true;
      CHECK(b.coordinator.assess(a,now));b.coordinator.update(b.r.facts,b.r.feedback,now);
      for(unsigned i=0;i<12;++i)b.r.executor.update(b.r.facts,b.r.feedback,++now);
      if(b.r.executor.status().state==ExecutorState::SAFE_OFF_REQUIRED||b.r.executor.status().state==ExecutorState::FAULT){std::printf("executor: %s\n",b.r.executor.status().reason);break;}
      for(unsigned i=0;i<12;++i){const double t=b.r.executor.targetRadians()[i];max_step=std::max(max_step,std::abs(t-q[i]));q[i]=t;b.r.feedback.actual_rad[i]=t;b.r.feedback.sampled_at_ms[i]=now+1;}
      ++now;b.r.executor.update(b.r.facts,b.r.feedback,now);++frames;
    }
    if(!std::strcmp(b.coordinator.state(),"REJECTED")){std::printf("operational stand rejected: %s\n",motion::feedbackReasonName(b.coordinator.frame().reason));break;}
  }
  CHECK(!std::strcmp(b.coordinator.state(),"STAND"));
  CHECK(approval_waits==0);                                             // no per-stage/segment operator approvals
  CHECK(frames>200&&max_step<=b.params.max_step_rad+1e-9);
  const auto c4=motion::pairedProneSample(250,1,true).target;for(unsigned i=0;i<12;++i)CHECK(std::abs(q[i]-slot(c4,i))<1e-9);
  op.refresh();CHECK(b.service.activeAck().outcome==motion::CommandOutcome::COMPLETED);REASON(b.service.activeAck().reason,"STAND_VERIFIED");
  // Every write stayed inside the unchanged hard limits.
  for(const auto& w:b.r.backend.writes){bool inside=false;
    for(unsigned i=0;i<12;++i){const auto* l=b.r.policy.limits().find(slotIdentity(i),b.r.geometry.provenanceTag());const auto* g=b.r.geometry.findJoint(slotIdentity(i));
      if(g&&g->bus_id==w.bus)inside=l&&l->contains(w.tick);}
    CHECK(inside);}
  // STOP from STAND: SAFE_OFF, no further position writes.
  auto stop=command(motion::Intent::STOP,9,motion::ClientOwner::DASHBOARD);stop.issued_at_ms=now;b.service.command(stop,now);
  CHECK(b.session.safeOffRequired()&&b.r.executor.status().safe_off_required);
  const size_t writes=b.r.backend.writes.size();for(unsigned k=0;k<20;++k)b.r.executor.update(b.r.facts,b.r.feedback,++now);
  CHECK(b.r.backend.writes.size()==writes);
  std::printf("OPERATIONAL_STAND fold_offset_deg=%.2f entry=%s entry_s=%.2f+%.2f frames=%u writes=%zu approvals=0 old_tube_needed=no\n",fold_offset_deg,
    motion::entryStatusName(b.coordinator.entryPlan().status),b.coordinator.entryPlan().durationS[0],b.coordinator.entryPlan().durationS[1],frames,writes);
}

void operationalRefusals(){
  // A start inside the keep-out: refused BEFORE arming (2026-10-15 pre-arm dry run):
  // no session, no prime, no torque, not a single write.
  { Operational op(0,-80.0);auto& b=op.b;const auto ack=op.stand();
    CHECK(ack.outcome==motion::CommandOutcome::REJECTED);REASON(ack.reason,"PATH_START_IN_KEEPOUT");
    REASON(b.coordinator.lastEntryReason(),"PATH_START_IN_KEEPOUT");CHECK(b.coordinator.entryPlan().failedJoint==2);
    CHECK(op.armer.calls==0&&b.session.state()==MotionSessionState::OFF&&b.r.backend.writes.empty()&&!b.coordinator.arming()); }
  { // A contact on the UNFOLDING side (free only for LF LOWER <= -89.5 deg): the start is clear,
    // the specific path towards the -88.5 deg route head is not. Denied with joint, sample and
    // clearance, before any torque.
    Operational op(0,-89.5,-1);auto& b=op.b;const auto ack=op.stand();
    const auto& p=b.coordinator.entryPlan();
    std::printf("keepout unfolding side: %s joint=%u sample=%u clearance_mrad=%.2f required_mrad=%.2f\n",b.coordinator.lastEntryReason(),p.failedJoint,p.failedSample,p.clearanceRad*1e3,p.requiredClearanceRad*1e3);
    CHECK(ack.outcome==motion::CommandOutcome::REJECTED);REASON(ack.reason,"PATH_KEEPOUT");
    REASON(b.coordinator.lastEntryReason(),"PATH_KEEPOUT");CHECK(p.failedJoint==2&&p.failedSample>0&&p.clearanceRad<p.requiredClearanceRad);
    CHECK(op.armer.calls==0&&b.session.state()==MotionSessionState::OFF&&b.r.backend.writes.empty()); }
  // Not PRONE at command time (e.g. legs hanging after a SAFE_OFF): refused before
  // arming with the recognizer's reason; nothing is energized.
  { Operational op;auto& b=op.b;b.r.feedback.actual_rad[0]=.6;const auto ack=op.stand();
    CHECK(ack.outcome==motion::CommandOutcome::REJECTED);REASON(ack.reason,"PRE_ARM_NOT_PRONE");REASON(b.coordinator.lastEntryReason(),"HIP_BAND");
    CHECK(op.armer.calls==0&&b.session.state()==MotionSessionState::OFF&&b.r.backend.writes.empty()); }
  { Operational op;auto& b=op.b;b.r.feedback.actual_rad[4]=94.3*kDeg;const auto ack=op.stand();   // inside the family band, >10 deg off the 83.7 deg head
    CHECK(ack.outcome==motion::CommandOutcome::REJECTED);REASON(ack.reason,"PRE_ARM_NOT_PRONE");REASON(b.coordinator.lastEntryReason(),"MAX_DEVIATION");
    CHECK(op.armer.calls==0&&b.r.backend.writes.empty()); }
  // The runtime cannot leave the offline-certified entry region (2026-10-15):
  // a wider recognition band or tracking tube is refused at configuration.
  { Operational op;auto& b=op.b;CommissioningStandConfig cc=b.coordinator.commissioning();
    CHECK(cc.measuredStart&&cc.proneMaxDeviationRad<=CommissioningStandConfig::kCertifiedBandRad);
    cc.proneMaxDeviationRad=CommissioningStandConfig::kCertifiedBandRad*1.01;CHECK(!b.coordinator.configureCommissioning(cc));
    cc.proneMaxDeviationRad=CommissioningStandConfig::kCertifiedBandRad;CHECK(b.coordinator.configureCommissioning(cc)); }
  { Bench wide(.008);CommissioningStandConfig cc;cc.enabled=true;cc.measuredStart=true;cc.minStageDurationS=4;
    cc.proneMaxDeviationRad=wide.params.prone_max_deviation_rad;cc.proneMaxRmsRad=wide.params.prone_max_rms_rad;
    wide.coordinator.begin(&wide.r.executor,&wide.r.policy);motion::AcquisitionRoute route;route.pairedProne=true;route.geometryIdentityVerified=true;
    route.source=motion::EvidenceSource::OFFLINE_CERTIFIED;route.timeScale=wide.runtime.stand_time_scale;
    CHECK(wide.coordinator.configure(wide.runtime.feedback,wide.runtime.walk,&route));
    CHECK(!wide.coordinator.configureCommissioning(cc)); }                       // 8 mrad tube > certified 6 mrad
  // Feedback not published / moving: refused before arming.
  { Operational op;auto& b=op.b;auto c=command(motion::Intent::STAND,1,motion::ClientOwner::DASHBOARD);c.target_height_m=.150;c.issued_at_ms=b.now;c.ttl_ms=1000;
    const auto ack=b.service.command(c,b.now);CHECK(ack.outcome==motion::CommandOutcome::REJECTED);REASON(ack.reason,"PRE_ARM_FEEDBACK_REQUIRED");CHECK(op.armer.calls==0); }
  // Arming refused (e.g. no operator heartbeat): command rejected, nothing written.
  { Operational op;op.armer.refuse="OPERATOR_HEARTBEAT_REQUIRED";const auto ack=op.stand();
    CHECK(ack.outcome==motion::CommandOutcome::REJECTED);REASON(ack.reason,"OPERATOR_HEARTBEAT_REQUIRED");CHECK(op.b.r.backend.writes.empty()); }
  // Not a PRONE: the broad recognizer rejects before any entry frame.
  { Operational op;auto& b=op.b;CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
    for(unsigned k=0;k<40&&b.coordinator.arming();++k)op.tickArming();
    const size_t writes=b.r.backend.writes.size();motion::AdaptiveObservation o;double q[12];for(unsigned i=0;i<12;++i)q[i]=b.r.feedback.actual_rad[i];q[0]=.6;
    for(unsigned k=0;k<4;++k){b.now+=20;observe(o,q,b.now);b.coordinator.observe(o);b.coordinator.update(b.r.facts,b.r.feedback,b.now);}
    REASON(b.coordinator.state(),"REJECTED");CHECK(b.r.backend.writes.size()==writes); }
  // No command => no session, no torque, no write (no spontaneous motion).
  { Operational op;for(unsigned k=0;k<50;++k){op.b.now+=20;op.tickArming();}
    CHECK(op.b.session.state()==MotionSessionState::OFF&&op.b.r.backend.writes.empty()&&op.armer.calls==0); }
  // Service-level gates still apply: without authority the command is refused before arming.
  { Operational op;auto r=op.readiness();r.authority_live=false;op.b.service.refresh(r,op.b.now);
    auto c=command(motion::Intent::STAND,1,motion::ClientOwner::DASHBOARD);c.target_height_m=.150;c.issued_at_ms=op.b.now;c.ttl_ms=1000;
    const auto ack=op.b.service.command(c,op.b.now);CHECK(ack.outcome==motion::CommandOutcome::REJECTED);REASON(ack.reason,"MOTION_AUTHORITY_REQUIRED");CHECK(op.armer.calls==0); }
  // Losing authority mid-arming (readiness revoked it) aborts the pending STAND.
  { Operational op;auto& b=op.b;CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
    b.r.arbiter.forceClear(AuthorityClearReason::OWNER_TEARDOWN);
    for(unsigned k=0;k<40&&b.coordinator.arming();++k)op.tickArming();
    CHECK(!b.r.executor.active()); }
  // The measured-start planner by itself: re-folded start far outside the old tube is fine.
  { Bench b;auto head=motion::pairedProneSample(0,0,true).target;motion::JointTargetFrame start;start.valid=true;
    for(unsigned i=0;i<12;++i){auto& q=start.legs[i/3];(i%3==0?q.hip:i%3==1?q.upper:q.lower)=b.measured[i]+(i%3==1?3*kDeg:0);}
    motion::MeasuredEntryConfig m;const auto plan=motion::planMeasuredEntry(start,head,b.runtime.feedback,m,4);
    CHECK(plan.status==motion::EntryStatus::OK);
    auto none=b.certificate();CHECK(motion::planFixtureEntry(start,head,b.runtime.feedback,none,4).status==motion::EntryStatus::START_OUTSIDE_CERTIFIED_REGION);
    auto nan=start;nan.legs[1].hip=NAN;CHECK(motion::planMeasuredEntry(nan,head,b.runtime.feedback,m,4).status==motion::EntryStatus::INVALID_INPUT);
    auto out=start;out.legs[2].hip=1.5;CHECK(motion::planMeasuredEntry(out,head,b.runtime.feedback,m,4).status==motion::EntryStatus::PATH_START_OUTSIDE_LIMITS);
    CHECK(motion::planMeasuredEntry(out,head,b.runtime.feedback,m,4).failedJoint==6); }
}
}  // namespace

int main(){
  operationalStandEndToEnd(0);operationalStandEndToEnd(.8);operationalStandEndToEnd(-2.0);
  operationalRefusals();
  std::printf("OPERATIONAL_MOTION %s checks=%d failures=%d hardware=false\n",failures?"FAIL":"PASS",checks,failures);return failures?1:0;
}
