// Production composition, simulated servo backend; no hardware validation.
#define MATDOG_OPERATIONAL_NO_MAIN
#include "test_operational_motion.cpp"

void noProceduralPermit(){
 Operational op;auto& b=op.b;
 b.evidence.generation=0;b.evidence.expires_at_ms=0;b.evidence.local_operator_present=false;
 b.evidence.fixture_support_verified=b.evidence.physical_power_cut_verified=false;
 b.evidence.installation_verified=b.evidence.servo_output_ceiling_verified=false;
 b.r.facts.operator_permit=false;b.r.facts.permit_generation=0;b.r.facts.permit_expires_at_ms=0;
 auto r=op.readiness();r.operational_permit=false;b.service.refresh(r,b.now);
 CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
 for(unsigned k=0;k<40&&b.coordinator.arming();++k){
  b.evidence.expires_at_ms=b.r.facts.permit_expires_at_ms=0;
  b.evidence.local_operator_present=false;b.r.facts.operator_permit=false;
  op.tickArming();
 }
 CHECK(b.session.state()==MotionSessionState::ARMED);
 CHECK(b.service.activeAck().outcome==motion::CommandOutcome::ACCEPTED);
 b.r.executor.stop("STALE_FEEDBACK");b.r.executor.stop("SESSION_CHANGED");
 REASON(b.r.executor.status().reason,"STALE_FEEDBACK");
}
void retryFeedback(){
 Operational op;auto& b=op.b;execution::OperationalFeedback producer;
 execution::RawJointObservation raw;raw.read_ok=true;raw.torque=0;raw.temperature=25;raw.voltage=120;raw.status=0;
 raw.sampled_at_ms=100;raw.sampled_at_us=100000;
 for(unsigned j=0;j<12;++j){raw.position=tickOf(b.r,j,b.r.feedback.actual_rad[j]);producer.ingest(j,raw,b.r.geometry,actuator::geometry_data::kProvenance,b.r.policy.transforms());}
 const auto before=producer.snapshot(100,400);CHECK(before.read_ok[0]);
 auto bad=raw;bad.read_ok=false;bad.sampled_at_ms=105;bad.sampled_at_us=105000;
 producer.ingest(0,bad,b.r.geometry,actuator::geometry_data::kProvenance,b.r.policy.transforms());
 auto pending=producer.snapshot(105,400);CHECK(pending.pending()&&!pending.read_ok[0]);
 CHECK(pending.sampled_at_ms[0]==100&&pending.sampled_at_us[0]==100000);
 CHECK(!producer.snapshot(501,400).pending()); // loss persists beyond existing deadline
 raw.position=tickOf(b.r,0,b.r.feedback.actual_rad[0]);raw.sampled_at_ms=110;raw.sampled_at_us=110000;
 producer.ingest(0,raw,b.r.geometry,actuator::geometry_data::kProvenance,b.r.policy.transforms());
 CHECK(!producer.snapshot(110,400).pending()&&producer.snapshot(110,400).read_ok[0]);
 raw.temperature=80;producer.ingest(0,raw,b.r.geometry,actuator::geometry_data::kProvenance,b.r.policy.transforms());
 CHECK(!producer.snapshot(110,400).pending()&&!producer.snapshot(110,400).read_ok[0]);
 // A single unreadable joint holds PRIME; a fresh retry resumes; persistent
 // age and real power loss still end the session, without more writes.
 CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
 b.fresh();auto fb=b.r.feedback;fb.recovering[0]=true;fb.read_ok[0]=false;
 const auto n=b.r.backend.writes.size();b.session.update(b.evidence,b.r.facts,fb,b.now);
 CHECK(!b.session.safeOffRequired()&&b.r.backend.writes.size()==n);
 fb.recovering[0]=false;fb.read_ok[0]=true;b.session.update(b.evidence,b.r.facts,fb,b.now);
 CHECK(b.r.backend.writes.size()==n+1);
 b.r.facts.power_ok=false;b.session.update(b.evidence,b.r.facts,fb,b.now);
 CHECK(b.session.safeOffRequired()&&b.r.backend.writes.size()==n+1);
}
void directWalk(){
 Operational op(0,-92.070442,1,true);auto& b=op.b;op.walk_ok=true;op.refresh();op.publish();
 CHECK(walkCommand(op,1,motion::operational_walk::strideVelocity(1)).outcome==motion::CommandOutcome::ACCEPTED);
 CHECK(b.coordinator.arming()&&op.armer.calls==1);
 for(unsigned k=0;k<40&&b.coordinator.arming();++k){b.service.renewTeleop(b.now,3000);op.tickArming();}
 CHECK(b.session.state()==MotionSessionState::ARMED);
 Driver d(op);bool ok=true;
 for(unsigned k=0;k<100000&&!b.coordinator.walking()&&ok;++k){
  b.service.renewTeleop(b.now,3000);for(auto& t:b.r.feedback.sampled_at_us)t=(b.now+20)*1000ull;ok=d.tick(20,true);
 }
 std::printf("DIRECT_WALK ok=%d state=%s executor=%s ack=%s\n",ok,b.coordinator.state(),b.r.executor.status().reason,b.service.activeAck().reason);
 CHECK(ok&&b.coordinator.walking());
 CHECK(b.service.activeAck().outcome==motion::CommandOutcome::ACCEPTED);
 // Lost transport during continuous work invokes STOP without any readiness.
 b.service.disconnected(b.now);
 CHECK(b.session.safeOffRequired());
}
void stopDuringStartup(){
 for(auto phase:{MotionSessionState::PRIMING,MotionSessionState::ENERGIZING,MotionSessionState::ARMING}){
  Operational op;auto& b=op.b;CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
  for(unsigned k=0;k<30&&b.session.state()!=phase;++k){b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);}
  CHECK(b.session.state()==phase);const auto writes=b.r.backend.writes.size();
  auto stop=command(motion::Intent::STOP,2);stop.issued_at_ms=0;stop.ttl_ms=1;
  CHECK(b.service.command(stop,b.now).outcome==motion::CommandOutcome::ACCEPTED);
  CHECK(b.session.safeOffRequired()&&b.r.executor.status().safe_off_required);
  b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);b.r.executor.update(b.r.facts,b.r.feedback,b.now);
  CHECK(b.r.backend.writes.size()==writes);
  b.r.executor.confirmSafeOff(false);CHECK(b.r.executor.status().safe_off_required);
  b.r.executor.confirmSafeOff(true);b.session.confirmSafeOff();op.refresh();
  CHECK(b.service.activeAck().outcome==motion::CommandOutcome::COMPLETED);
  REASON(b.service.activeAck().reason,"SAFE_OFF_VERIFIED");
 }
}
void retryBeforeFrame(){
 Rig r;r.envelope.frame_writes=true;r.envelope.timed_segments=true;
 r.envelope.arrival_tolerance_rad=motion::tolerance::kArrivalRad;CHECK(r.arm());
 auto frame=r.frame(1,0);auto validation=r.validation(1);
 CHECK(r.executor.submit(frame,validation,100,1000,.020,r.facts,r.feedback,100));
 const auto n=r.backend.writes.size();r.feedback.read_ok[0]=false;r.feedback.recovering[0]=true;
 r.executor.update(r.facts,r.feedback,101);CHECK(r.backend.writes.size()==n&&!r.executor.status().safe_off_required);
 r.feedback.read_ok[0]=true;r.feedback.recovering[0]=false;r.fresh(105);
 r.executor.update(r.facts,r.feedback,105);CHECK(r.backend.writes.size()==n+12);CHECK(r.executor.status().tx_sequence==1);
 r.executor.update(r.facts,r.feedback,206);CHECK(r.executor.status().safe_off_required);
 REASON(r.executor.status().reason,"STALE_FEEDBACK");
}
// Real production executor and control scheduling predicate, with deterministic
// serial transaction durations. This reproduces starvation, not a claim that
// the hardware log identifies this as its first cause.
void controlCacheRecovery(){
 for(bool fixed:{false,true}){
  Rig r;r.envelope.frame_writes=true;r.envelope.timed_segments=true;
  r.envelope.arrival_tolerance_rad=motion::tolerance::kArrivalRad;CHECK(r.arm());
  uint32_t control[12]{};for(auto& t:control)t=100;
  // One expired control block, while the slow rotation points elsewhere.
  control[0]=0;unsigned slot=0,next=1;uint32_t last=400;
  r.fresh(400);
  for(uint32_t now=405;now<=600&&!r.executor.status().safe_off_required;now+=5){
   const bool full=fixed?controlReadDue(true,slot,next,now,last,20,control[slot],400):
     slot==next&&now-last>=20;
   if(full){control[slot]=now;if(slot==next){last=now;next=(next+1)%12;}}
   const bool valid=now-control[slot]<=400;
   r.feedback.read_ok[slot]=valid;r.feedback.recovering[slot]=!valid;
   if(valid){r.feedback.sampled_at_ms[slot]=now;r.feedback.sampled_at_us[slot]=now*1000ull;slot=(slot+1)%12;}
   r.executor.update(r.facts,r.feedback,now,now*1000ull);
  }
  if(!fixed){CHECK(r.executor.status().safe_off_required);REASON(r.executor.status().reason,"STALE_FEEDBACK");
   CHECK(r.executor.status().feedback_checked_ms==505);CHECK(r.executor.status().feedback_budget_ms==100);}
  else{CHECK(!r.executor.status().safe_off_required);
   // Silence remains a fault under exactly the same 100 ms bound.
   r.executor.update(r.facts,r.feedback,701,701000);CHECK(r.executor.status().safe_off_required);}
 }
 // Wrapping millisecond epoch and an expired cache independent of cursor.
 CHECK(controlReadDue(true,3,7,10,9,20,UINT32_MAX-400,400));
}
// Scheduling experiment: real Executor + frame backend + production batch rule.
// Transaction/loop durations are explicit synthetic inputs, NOT hardware WCET.
// Coordinator maxima are taken from the five 19-Oct traces; the remaining
// loop cost is varied. Samples are stamped only when their read completes.
void timedPolling(){
 const uint32_t work_us[]={10587,12654,10400,10395,10620};
 for(unsigned overhead_us:{3000u,5000u})for(unsigned scenario=0;scenario<4;++scenario)for(bool fixed:{false,true}){
  Rig r;r.envelope.frame_writes=r.envelope.timed_segments=true;r.envelope.feedback_max_age_ms=400;
  r.envelope.arrival_tolerance_rad=motion::tolerance::kArrivalRad;CHECK(r.arm());
  uint64_t clock=100000;unsigned slot=0,control_next=0;uint32_t control_at=100;
  uint32_t controls[12];unsigned seen[12]{};uint32_t max_gap=0,max_check_age=0;
  for(unsigned j=0;j<12;++j){r.feedback.sampled_at_ms[j]=24+6*j;r.feedback.sampled_at_us[j]=(24+6*j)*1000;controls[j]=24+6*j;}
  bool injected=false,stopped=false;uint32_t failure_sample=0;unsigned ticks=0;
  for(;ticks<300&&!r.executor.status().safe_off_required;++ticks){
   const uint64_t batch_started=clock;bool ok=true;
   for(unsigned k=0;fixed?operationalPollReadAllowed(true,k,ok):(k<3&&(k==0||clock-batch_started<2000));++k){
    if(scenario==3&&ticks==30&&k==1){r.executor.stop("OPERATOR_STOP");stopped=true;break;}
    const uint32_t before=clock/1000;
    const bool full=!r.feedback.read_ok[slot]||controlReadDue(true,slot,control_next,before,control_at,20,controls[slot],400);
    const bool fail=scenario>0&&scenario<3&&ticks>=30&&slot==7&&(!injected||scenario==2);
    clock+=fail&&scenario==2?8000:full?2000:1000;
    const uint32_t at=clock/1000;
    if(fail){if(!injected)failure_sample=r.feedback.sampled_at_ms[slot];injected=true;
     r.feedback.read_ok[slot]=false;r.feedback.recovering[slot]=true;ok=false;}
    else{
     if(full){controls[slot]=at;if(slot==control_next){control_at=at;control_next=(control_next+1)%12;}}
     max_gap=std::max(max_gap,at-r.feedback.sampled_at_ms[slot]);
     r.feedback.actual_rad[slot]=r.executor.targetRadians()[slot];
     r.feedback.sampled_at_ms[slot]=at;r.feedback.sampled_at_us[slot]=clock;
     r.feedback.read_ok[slot]=true;r.feedback.recovering[slot]=false;++seen[slot];slot=(slot+1)%12;
    }
   }
   const auto snapshot=r.feedback;const unsigned frames_before=r.backend.frames;
   clock+=work_us[ticks%5];const uint32_t now=clock/1000;
   for(unsigned j=0;j<12;++j)max_check_age=std::max(max_check_age,now-snapshot.sampled_at_ms[j]);
   r.executor.update(r.facts,snapshot,now,clock);
   if(r.executor.status().state==ExecutorState::READY&&!snapshot.pending()){
    const unsigned seq=r.executor.status().accepted_sequence+1;
    auto frame=r.frame(seq,.001*std::sin(seq*.01));auto v=r.validation(seq);v.final=false;
    CHECK(r.executor.submit(frame,v,now,1000,.020,r.facts,snapshot,now));
    r.executor.update(r.facts,snapshot,now,clock);
   }
   if(snapshot.pending()||stopped)CHECK(r.backend.frames==frames_before);
   clock+=(r.backend.frames-frames_before)*r.backend.tx_us+overhead_us;
  }
  std::printf("POLL_TIMING fixed=%u scenario=%u overhead_us=%u ticks=%u frames=%u max_gap_ms=%u max_check_age_ms=%u reason=%s\n",
   fixed,scenario,overhead_us,ticks,r.backend.frames,max_gap,max_check_age,r.executor.status().reason);
  CHECK(r.backend.attempts==0);CHECK(r.backend.writes.size()==r.backend.frames*12);
  if(!fixed){CHECK(r.executor.status().safe_off_required);REASON(r.executor.status().reason,"STALE_FEEDBACK");}
  else if(scenario<2){CHECK(!r.executor.status().safe_off_required);CHECK(r.backend.frames>50);CHECK(max_check_age<=100);
   for(unsigned n:seen){CHECK(n>30);}
   if(scenario==1)CHECK(injected);}
  else if(scenario==2){CHECK(injected&&r.executor.status().safe_off_required);REASON(r.executor.status().reason,"STALE_FEEDBACK");
   CHECK(r.executor.status().feedback_sample_ms==failure_sample);CHECK(r.executor.status().feedback_budget_ms==100);
   const unsigned n=r.backend.frames;r.executor.update(r.facts,r.feedback,clock/1000,clock);CHECK(r.backend.frames==n);}
  else{CHECK(stopped&&r.executor.status().safe_off_required);REASON(r.executor.status().reason,"OPERATOR_STOP");}
 }
}
int main(){
 {Operational op;auto stop=command(motion::Intent::STOP,1);stop.issued_at_ms=0;stop.ttl_ms=1;
  auto r=op.readiness();r.fault=true;r.operational_permit=false;r.hardware_approved=false;op.b.service.refresh(r,10000);
  CHECK(op.b.service.command(stop,10000).outcome==motion::CommandOutcome::ACCEPTED);
  CHECK(op.b.r.executor.status().safe_off_required);}
 timedPolling();controlCacheRecovery();stopDuringStartup();retryBeforeFrame();noProceduralPermit();retryFeedback();directWalk();
 stand_backend::run();postValidationAdmissionClock();operationalRefusals();
 operationalStandEndToEnd(0);phaseTimedRouteContract();
 std::printf("MOTION_REMEDIATION %s checks=%d failures=%d hardware=false\n",failures?"FAIL":"PASS",checks,failures);
 return failures?1:0;
}
