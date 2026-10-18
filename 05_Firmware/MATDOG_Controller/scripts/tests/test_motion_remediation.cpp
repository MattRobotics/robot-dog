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
 r.executor.update(r.facts,r.feedback,105);CHECK(r.backend.writes.size()==n+12);
 r.executor.update(r.facts,r.feedback,206);CHECK(r.executor.status().safe_off_required);
 REASON(r.executor.status().reason,"STALE_FEEDBACK");
}
int main(){
 {Operational op;auto stop=command(motion::Intent::STOP,1);stop.issued_at_ms=0;stop.ttl_ms=1;
  auto r=op.readiness();r.fault=true;r.operational_permit=false;r.hardware_approved=false;op.b.service.refresh(r,10000);
  CHECK(op.b.service.command(stop,10000).outcome==motion::CommandOutcome::ACCEPTED);
  CHECK(op.b.r.executor.status().safe_off_required);}
 stopDuringStartup();retryBeforeFrame();noProceduralPermit();retryFeedback();directWalk();
 stand_backend::run();postValidationAdmissionClock();operationalRefusals();
 operationalStandEndToEnd(0);phaseTimedRouteContract();
 std::printf("MOTION_REMEDIATION %s checks=%d failures=%d hardware=false\n",failures?"FAIL":"PASS",checks,failures);
 return failures?1:0;
}
