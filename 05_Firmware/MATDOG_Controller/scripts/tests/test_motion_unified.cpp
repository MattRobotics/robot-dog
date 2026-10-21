// Production-linked unified STAND/WALK, synthetic physics only. No transport.
#define MATDOG_OPERATIONAL_NO_MAIN
#include "test_operational_motion.cpp"
#include <chrono>
#include "../../src/motion/TimedStand.h"
#include "../../src/motion/StandHeightRegion.h"
namespace {
struct StreamRig {
 Rig r;double lo[12],hi[12];
 StreamRig(uint64_t seed_us=100000,bool path_box=true){
  r.envelope.frame_writes=r.envelope.timed_segments=true;
  r.envelope.max_velocity_rad_s=.19;r.envelope.max_acceleration_rad_s2=2;
  r.envelope.arrival_tolerance_rad=motion::tolerance::kArrivalRad;r.envelope.tracking_timeout_ms=4000;
  for(unsigned i=0;i<12;++i){lo[i]=-.1;hi[i]=.1;r.feedback.sampled_at_us[i]=100000;}
  CHECK(r.arm());StreamConfig c;c.tube_rad=.020;c.local_path_box=path_box;c.max_lag_s=path_box?0:.001;c.max_step_rad=.004;c.reconstruction_acceleration_rad_s2=motion::operational_walk::kStandDriveAccelerationRadS2;
  c.settle_speed_rad_s=motion::operational_walk::kStreamSettleSpeedRadS;c.feedback_max_age_us=100000;c.hold_timeout_us=4000000;c.max_frame_gap_us=100000;c.persist_samples=3;
  c.region_min_rad=lo;c.region_max_rad=hi;for(unsigned i=0;i<12;++i){r.feedback.sampled_at_us[i]=seed_us;r.feedback.sampled_at_ms[i]=seed_us/1000;}
  CHECK(r.executor.beginStream(c,r.feedback,seed_us));
 }
 void fresh(uint64_t t,double q){for(unsigned i=0;i<12;++i)sampleJoint(r.feedback,i,t,q);}
 bool send(unsigned seq,double q,uint64_t t,bool final=false){return r.executor.stream(r.frame(seq,q),r.validation(seq),t,t+20000,final,r.facts,r.feedback,t/1000,t);}
 void monitor(uint64_t t){r.executor.monitor(r.facts,r.feedback,t/1000,t);}
};
// The UART clock must include validation/commit time before UART start.
void backendClockArrival(){
 {StreamRig s;s.r.backend.absolute_frame_clock=true;s.r.backend.frame_clock_us=125000;
  CHECK(s.send(1,.003,120000,true));const auto complete=s.r.executor.status().last_tx_complete_us;
  CHECK(complete==126040);
  s.fresh(122000,.003);s.monitor(130000); // cached sample before REAL TX-complete
  s.fresh(222000,.003);s.monitor(222000);
  const auto premature=s.r.executor.status().settled_sequence;CHECK(premature==0);
  s.fresh(322000,.003);s.monitor(322000);CHECK(s.r.executor.status().settled_sequence==1);
  std::printf("BACKEND_CLOCK caller=120000 tx_start=125000 tx_end=126040 recorded=%llu premature_settled=%u final_settled=%u hardware=false\n",(unsigned long long)complete,premature,s.r.executor.status().settled_sequence);
 }
 {const uint64_t epoch=1ull<<32;StreamRig s(epoch-40000);
  s.r.backend.absolute_frame_clock=true;s.r.backend.frame_clock_us=static_cast<uint32_t>(epoch-1000);s.r.backend.tx_us=2040;
  CHECK(s.send(1,.003,epoch-20000));CHECK(s.r.executor.status().last_tx_complete_us==epoch+1040);
  std::printf("BACKEND_CLOCK_WRAP expected=%llu actual=%llu hardware=false\n",(unsigned long long)(epoch+1040),(unsigned long long)s.r.executor.status().last_tx_complete_us);
 }
 {StreamRig s;s.r.backend.absolute_frame_clock=true;s.r.backend.frame_clock_us=119999;
  CHECK(!s.send(1,.003,120000));REASON(s.r.executor.status().reason,"FRAME_TX_CLOCK_INVALID");
  const auto sent=s.r.backend.frames;CHECK(!s.send(2,.003,140000));CHECK(s.r.backend.frames==sent);}
 {Rig r;r.envelope.frame_writes=true;r.backend.absolute_frame_clock=true;r.backend.frame_clock_us=106000;
  CHECK(r.arm());CHECK(r.submit(r.frame(1,.003)));r.executor.update(r.facts,r.feedback,101,101000);
  CHECK(r.executor.status().last_tx_complete_us==107040);
  r.fresh(102,.003);r.executor.update(r.facts,r.feedback,108,108000);CHECK(r.executor.status().completed_sequence==0);
  r.fresh(109,.003);r.executor.update(r.facts,r.feedback,109,109000);CHECK(r.executor.status().completed_sequence==1);}

}
// Counterexamples use existing production executor, mapping and certified path.
// Round-robin acquisition as the Controller: one joint every 7 ms, so each joint
// is resampled every 84 ms and a snapshot spans up to 77 ms (ASYNC_84MS model).
void residualGeometry(bool fixed){
 Bench b;auto& r=b.r;motion::PhaseTimedRoute route;const auto& profile=motion::operational_walk::kStandProfile;
 CHECK(motion::buildPhaseTimedRoute(true,.999*profile.velocityRadS,.999*profile.accelerationRadS2/2,profile.samplePeriodS,&route));
 const double t0=route.durationS[4]/2;
 auto path=[&](double t){return motion::phaseTimedSample(route,4,t0+t,true).target;};
 const auto start=path(0);double lo[12],hi[12];
 CHECK(r.arbiter.request(ActuatorAuthority::MOTION,OperatingMode::RUN,&r.facts.lease)==AuthorityResult::GRANTED);
 for(unsigned i=0;i<12;++i){r.feedback.actual_rad[i]=slot(start,i);r.feedback.sampled_at_us[i]=100000;r.feedback.torque_on[i]=true;
  lo[i]=b.limits[i][0];hi[i]=b.limits[i][1];}
 r.envelope.frame_writes=r.envelope.timed_segments=true;r.envelope.max_velocity_rad_s=.19;r.envelope.max_acceleration_rad_s2=2;
 r.envelope.tracking_tolerance_rad=.020;r.envelope.arrival_tolerance_rad=motion::tolerance::kArrivalRad;CHECK(r.arm());
 StreamConfig c;c.local_path_box=true;c.max_step_rad=.004;c.tube_rad=.020;c.settle_speed_rad_s=0;c.reconstruction_acceleration_rad_s2=motion::operational_walk::kStandDriveAccelerationRadS2;
 c.feedback_max_age_us=c.max_frame_gap_us=100000;c.hold_timeout_us=4000000;c.persist_samples=3;c.region_min_rad=lo;c.region_max_rad=hi;
 CHECK(r.executor.beginStream(c,r.feedback,100000));
 std::vector<std::pair<uint64_t,motion::JointTargetFrame>> sent{{100000,start}};
 // The true simultaneous pose path(t+50 ms) is ON the certified polyline (quantized samples).
 auto acquire=[&](uint64_t now){for(unsigned i=0;i<12;++i){
  const uint64_t phase=now>=100000+i*7000?(now-100000-i*7000)/84000:0;const uint64_t at=100000+i*7000+phase*84000;
  if(at>now||at==r.feedback.sampled_at_us[i])continue;
  sampleJoint(r.feedback,i,at,qOf(r,i,tickOf(r,i,slot(path((at-100000)*1e-6+.050),i))));}};
 double tracking=0,oldest_error=0,witness_error=0;bool ok=true;unsigned frames=0;
 for(unsigned k=1;k<=12&&ok;++k){const uint64_t now=100000+k*20000;acquire(now);
  uint64_t oldest=UINT64_MAX;
  for(unsigned i=0;i<12;++i){const uint64_t at=r.feedback.sampled_at_us[i];oldest=std::min(oldest,at);
   const auto* ref=&sent.front().second;for(const auto& h:sent)if(h.first<=at)ref=&h.second;
   tracking=std::max(tracking,std::abs(r.feedback.actual_rad[i]-slot(*ref,i)));}
  const auto* old=&sent.front().second;for(const auto& h:sent)if(h.first<=oldest)old=&h.second;
  oldest_error=0;for(unsigned i=0;i<12;++i)oldest_error=std::max(oldest_error,std::abs(r.feedback.actual_rad[i]-slot(*old,i)));
  witness_error=1e9;for(const auto& h:sent){double e=0;for(unsigned i=0;i<12;++i)e=std::max(e,std::abs(r.feedback.actual_rad[i]-slot(h.second,i)));witness_error=std::min(witness_error,e);}
  auto f=path(k*.020);f.sequence=k;
  ok=r.executor.stream(f,r.validation(k),now,now+20000,false,r.facts,r.feedback,now/1000,now);
  if(ok){sent.push_back({r.executor.status().last_tx_complete_us,f});++frames;}
 }
 std::printf("RESIDUAL_GEOMETRY fixed=%d running=%d frames=%u tracking_mrad=%.3f oldest_box_mrad=%.3f best_snapshot_witness_mrad=%.3f reason=%s certified_polyline=true hardware=false\n",fixed,ok,frames,tracking*1000,oldest_error*1000,witness_error*1000,r.executor.status().reason);
 // V6: the configuration is judged at a common instant, so an asynchronous snapshot on the certified polyline runs.
 CHECK(ok&&tracking<=.020);
}
// Core Motion V6 path box (MotionExecutor::pathBox): the measured CONFIGURATION at a common instant against the
// endpoint active then, with the reconstruction uncertainty charged inside the unchanged 20 mrad tube.
void pathBoxCases(){
 // (1) RCA phase-1 counterexample: targets 3.8 mrad every 20 ms (TX 1.04 ms after release); true motion 0.19 rad/s
 // on every joint with constant -15/+15 mrad on joints 0/1; joint 0 sampled at 232 ms, the others at 302 ms (70 ms
 // skew); previous samples 84 ms earlier on the same true motion. Old box: no common endpoint within 20 mrad.
 {StreamRig s;std::vector<std::pair<uint64_t,double>> hist{{100000,0}};
  for(unsigned k=1;k<=10;++k){const uint64_t t=100000+k*20000;s.fresh(t,.0038*(k-1));CHECK(s.send(k,.0038*k,t));hist.push_back({s.r.executor.status().last_tx_complete_us,.0038*k});}
  auto truth=[](unsigned i,uint64_t t){return .19*(t-100000)*1e-6+(i==0?-.015:i==1?.015:0);};
  for(unsigned i=0;i<12;++i){const uint64_t at=i==0?232000:302000;
   s.r.feedback.previous_at_us[i]=at-84000;s.r.feedback.previous_rad[i]=truth(i,at-84000);
   s.r.feedback.sampled_at_us[i]=at;s.r.feedback.sampled_at_ms[i]=at/1000;s.r.feedback.actual_rad[i]=truth(i,at);}
  double best=1e9;for(const auto& h:hist){double e=0;for(unsigned i=0;i<12;++i)e=std::max(e,std::abs(s.r.feedback.actual_rad[i]-h.second));best=std::min(best,e);}
  s.monitor(302000);s.monitor(302001);
  std::printf("PATH_BOX_ASYNC_RCA skew_ms=70 best_snapshot_witness_mrad=%.3f reason=%s hardware=false\n",best*1000,s.r.executor.status().reason);
  CHECK(best>.020&&!s.r.executor.status().safe_off_required);}
 // (2) Truly simultaneous configuration: outside the tube refused at once, inside admitted (tube unchanged).
 for(double q:{.019,.021}){StreamRig s;s.fresh(120000,q);s.monitor(120000);
  std::printf("PATH_BOX_SYNCHRONOUS q=%.3f tube=.020 reason=%s\n",q,s.r.executor.status().reason);
  if(q>.020){REASON(s.r.executor.status().reason,"OUTSIDE_PATH_BOX");CHECK(s.r.executor.refusal().joint==0&&std::strcmp(s.r.executor.refusal().site,"PATH_SAMPLE")==0);}
  else CHECK(!s.r.executor.status().safe_off_required);}
 // (3) Common delay without skew: twelve samples 70 ms old are judged against the endpoint active THEN.
 {StreamRig s;for(unsigned k=1;k<=6;++k){const uint64_t t=100000+k*20000;s.fresh(t-70000>100000?t-70000:100000,.0038*(k>4?k-4:0));CHECK(s.send(k,.0038*k,t));}
  s.monitor(220000);CHECK(!s.r.executor.status().safe_off_required);}
 // (4) Asynchronous configuration provably outside at the common instant: joint 3 is 26 mrad off the active endpoint
 // between two of its own samples (small uncertainty) while every joint sampled at the instant is in the tube.
 {StreamRig s;CHECK(s.send(1,.003,120000));s.fresh(140000,.003);
  s.r.feedback.previous_at_us[3]=139000;s.r.feedback.previous_rad[3]=.003+.026;s.r.feedback.sampled_at_us[3]=141000;s.r.feedback.actual_rad[3]=.003+.026;
  s.monitor(141000);REASON(s.r.executor.status().reason,"OUTSIDE_PATH_BOX");
  const auto& w=s.r.executor.refusal();CHECK(w.joint==3&&w.atUs==141000&&w.uncertainty==0);
  std::printf("PATH_BOX_PROVEN_OUTSIDE joint=%d error_mrad=%.3f uncertainty_mrad=%.4f instant_us=%llu\n",w.joint,w.value*1000,w.uncertainty*1000,(unsigned long long)w.atUs);}
 // (5) Not certifiable: 19 mrad at the instant inside a 135 ms bracket (uncertainty ~6 mrad) - no immediate stop, and
 // an unchanged snapshot is no new evidence; three DISTINCT unprovable instants stop.
 {StreamRig s;CHECK(s.send(1,.003,120000));unsigned instants=0;
  for(unsigned n=0;n<4&&!s.r.executor.status().safe_off_required;++n){const uint64_t t=140000+n*10000;s.fresh(t,.003);
   s.r.feedback.previous_at_us[3]=t-90000;s.r.feedback.previous_rad[3]=.003+.019;s.r.feedback.sampled_at_us[3]=t+45000;s.r.feedback.actual_rad[3]=.003+.019;
   s.monitor(t+45000);s.monitor(t+45000);++instants;}
  REASON(s.r.executor.status().reason,"OUTSIDE_PATH_BOX");CHECK(instants==3);
  std::printf("PATH_BOX_UNCERTIFIABLE distinct_instants=%u uncertainty_mrad=%.3f reason=%s\n",instants,s.r.executor.refusal().uncertainty*1000,s.r.executor.status().reason);}
 // (6) A path-box stream without a reconstruction bound is refused at begin, never degraded silently.
 {Rig r;r.envelope.frame_writes=r.envelope.timed_segments=true;r.envelope.arrival_tolerance_rad=motion::tolerance::kArrivalRad;CHECK(r.arm());
  double lo[12],hi[12];for(unsigned i=0;i<12;++i){lo[i]=-.1;hi[i]=.1;r.feedback.sampled_at_us[i]=100000;}
  StreamConfig c;c.tube_rad=.020;c.local_path_box=true;c.max_step_rad=.004;c.feedback_max_age_us=100000;c.hold_timeout_us=4000000;c.max_frame_gap_us=100000;c.persist_samples=3;
  c.region_min_rad=lo;c.region_max_rad=hi;CHECK(!r.executor.beginStream(c,r.feedback,100000));}
 // (7) Interpolation alone hides an actual excursion: the midpoint is 3 mrad,
 // perfectly at the reference, but the later real sample is 26 mrad away.
 {StreamRig s;CHECK(s.send(1,.003,120000));s.fresh(140000,.003);
  s.r.feedback.previous_at_us[3]=139000;s.r.feedback.previous_rad[3]=.003-.026;
  s.r.feedback.sampled_at_us[3]=141000;s.r.feedback.actual_rad[3]=.003+.026;
  s.monitor(141000);REASON(s.r.executor.status().reason,"OUTSIDE_PATH_BOX");
  CHECK(std::strcmp(s.r.executor.refusal().site,"PATH_SAMPLE")==0&&s.r.executor.refusal().atUs==141000);
  std::puts("PATH_SAMPLE actual excursion cannot be hidden by nominal interpolation: PASS");}
 // (8) A malformed previous sample is unknown, never silently consistent
 // through NaN comparison semantics. Three distinct unknown instants stop.
 {StreamRig s;for(unsigned n=0;n<3;++n){const uint64_t t=110000+n*1000;s.fresh(t,0);
   s.r.feedback.previous_at_us[3]=t-1000;s.r.feedback.previous_rad[3]=NAN;
   s.r.feedback.sampled_at_us[3]=t+1000;s.monitor(t+1000);}
  REASON(s.r.executor.status().reason,"OUTSIDE_PATH_BOX");}
 std::puts("PATH_BOX PASS: asynchronous RCA vector at a common instant, synchronous outside refused, common delay, proven outside, uncertifiable persistence, bound required");
}
void residualHold(bool fixed){
 for(unsigned mode=0;mode<3;++mode){StreamRig s;CHECK(s.send(1,.003,120000,true));bool arrived=false;
  for(unsigned n=0;n<52&&!s.r.executor.status().safe_off_required;++n){const uint64_t now=140000+n*80000;
   for(unsigned i=0;i<12;++i){
    // Zero; adjacent-position-count oscillation with +/-1 speed count;
    // actual motion at ten counts/s while momentarily inside arrival band.
    sampleJoint(s.r.feedback,i,122000+n*80000+i*1000,mode==1?(1+((n+i)%2))*motion::tolerance::kTickRad:mode==2?.003+.0012*(n%4<2?n%4:4-n%4):.003);
    s.r.feedback.speed_rad_s[i]=mode==0?0:mode==1?((n+i)%2?1:-1)*motion::tolerance::kTickRad:10*motion::tolerance::kTickRad;}
   s.monitor(now);if(s.r.executor.status().settled_sequence){arrived=true;break;}}
  std::printf("RESIDUAL_HOLD mode=%u fixed=%d arrived=%d reason=%s hardware=false\n",mode,fixed,arrived,s.r.executor.status().reason);
  CHECK(arrived==(mode==0||(fixed&&mode==1)));
  if(mode==1&&!fixed)REASON(s.r.executor.status().reason,"HOLD_SETTLE_TIMEOUT");
  if(mode==2)REASON(s.r.executor.status().reason,"HOLD_SETTLE_TIMEOUT");
 }
}

void residualSpeedUnits(){
 Rig r;OperationalFeedback producer;double max_error=0;
 for(int sign:{-1,1})for(unsigned i=0;i<12;++i){
  RawJointObservation sample;sample.read_ok=true;sample.position=tickOf(r,i,.0015);sample.torque=1;sample.status=0;
  sample.temperature=30;sample.voltage=114;sample.sampled_at_ms=100;sample.sampled_at_us=100000;sample.speed_signed=sign;
  producer.ingest(i,sample,r.geometry,geometry_data::kProvenance,r.policy.transforms());
  const auto fb=producer.snapshot(100,100);CHECK(fb.read_ok[i]);
  const double expected=sign*actuator::jointDirection(r.geometry,slotIdentity(i))*motion::tolerance::kTickRad;
  max_error=std::max(max_error,std::abs(fb.speed_rad_s[i]-expected));
 }
 CHECK(max_error<1e-15);std::printf("RESIDUAL_SPEED_UNIT rad_s_per_count=%.12f rounding_error=%.12f\n",motion::tolerance::kTickRad,max_error);
}
void temporalContracts(){
 // The old PRE_TX branch rejects this future target (.022 rad away).
 // Only the held reference (0, .019 rad away) is relevant before this TX.
 {StreamRig s;s.fresh(120000,-.019);CHECK(std::abs(-.019-.003)>.020);
  CHECK(s.send(1,.003,120000));CHECK(s.r.backend.frames==1&&s.r.executor.status().completed_sequence==0);
  CHECK(s.r.executor.status().stream_peak_error_rad<=.020);}
 // A final target already matched by cached feedback is not ARRIVED.
 {StreamRig s;CHECK(s.send(1,.003,120000,true));
  s.fresh(120000,.003);s.monitor(122000);CHECK(s.r.executor.status().settled_sequence==0&&!s.r.executor.endStream());
  s.fresh(122000,.003);s.r.feedback.sampled_at_us[11]=120000;s.monitor(123000);
  CHECK(s.r.executor.status().settled_sequence==0);
  s.r.feedback.sampled_at_us[11]=122001;s.r.feedback.recovering[11]=true;s.monitor(123000);
  CHECK(s.r.executor.status().settled_sequence==0);
  s.r.feedback.recovering[11]=false;s.r.feedback.speed_rad_s[3]=.01;s.monitor(123000);CHECK(s.r.executor.status().settled_sequence==0);
  s.r.feedback.speed_rad_s[3]=0;s.monitor(123000);CHECK(s.r.executor.status().settled_sequence==0);
  s.fresh(223000,.003);s.monitor(223000);CHECK(s.r.executor.status().settled_sequence==1);
  // New feedback leaving the tighter arrival band revokes settlement.
  s.fresh(224000,-.014);s.monitor(224000);CHECK(s.r.executor.status().settled_sequence==0);
  s.fresh(225000,.003);s.monitor(225000);CHECK(!s.r.executor.endStream());
  s.fresh(325000,.003);s.monitor(325000);CHECK(s.r.executor.endStream());CHECK(s.r.executor.status().completed_sequence==1);}
 // WALK tracking persistence: three NEW divergent samples beyond the lag
 // window. Old snapshots cannot count as extra samples. STAND's immediate
 // path-sample geometry is tested separately above.
 {StreamRig s(100000,false);CHECK(s.send(1,.003,120000));CHECK(s.send(2,.006,140000));CHECK(s.send(3,.009,160000));
  for(unsigned k=0;k<3;++k){s.r.feedback.sampled_at_us[0]=163000+k*1000;s.r.feedback.actual_rad[0]=-.014;
   s.monitor(163000+k*1000);if(k<2){CHECK(!s.r.executor.status().safe_off_required);s.monitor(163000+k*1000);CHECK(!s.r.executor.status().safe_off_required);}}
  REASON(s.r.executor.status().reason,"TRACKING_LAG_PERSISTENT");}
 std::puts("TEMPORAL_CONTRACTS PASS: no future-target gate, post-TX all-12 arrival, quiet speed, new-sample persistence");
}
// Identical reference clock, differing bus completion durations. These are
// deterministic counterexamples, not a reconstruction of hardware frame 218.
void proneStatus(){
 core::MotionService service;core::MotionTelemetry t;
 const auto head=motion::pairedProneSample(0,0,true);CHECK(head.target.valid);
 t.prone_reference_valid=t.prone_stationary=true;t.feedback_max_age_ms=100;
 t.prone_max_deviation_rad=.17453292519943295;t.prone_max_rms_rad=.12217304763960307;
 for(unsigned i=0;i<12;++i){t.prone_nominal_rad[i]=t.actual_rad[i]=slot(head.target,i);t.joint_at_ms[i]=100;t.joint_mapped[i]=t.raw_ok[i]=true;}
 char json[6144];
 auto expect=[&](const char* state,uint32_t now=100){service.publishTelemetry(t);CHECK(service.statusJson(json,sizeof(json),now));CHECK(std::strstr(json,state)!=nullptr);};
 expect("\"state\":\"IN_PRONE\"");CHECK(std::strstr(json,"\"in_window\":12"));
 t.actual_rad[0]+=.18;expect("\"state\":\"OUT_OF_PRONE\"");CHECK(std::strstr(json,"\"in_window\":11"));
 t.actual_rad[0]=t.prone_nominal_rad[0];
 expect("\"state\":\"UNKNOWN\"",201);t.joint_at_ms[11]=101;expect("\"state\":\"UNKNOWN\"");
 t.joint_at_ms[11]=100;t.feedback_recovering=true;expect("\"state\":\"UNKNOWN\"");
 t.feedback_recovering=false;t.joint_mapped[11]=false;expect("\"state\":\"UNKNOWN\"");
 t.joint_mapped[11]=true;t.prone_stationary=false;expect("\"state\":\"UNKNOWN\"");
 t.prone_stationary=true;t.prone_reference_valid=false;expect("\"state\":\"UNKNOWN\"");
 std::puts("PRONE_STATUS PASS in/out/stale/future/recovering/unmapped/moving/no-reference; recognition only");
}
void recoveryAndImu(){
 entryPlanner();classifierOnMeasuredPose();
 {Bench b;b.evidence.prime_clamp_max_ticks=0;CHECK(!b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));CHECK(b.r.backend.writes.empty());}
 {Bench b;b.evidence.prime_clamp_max_ticks=2;CHECK(!b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));CHECK(b.r.backend.writes.empty());}
 {Bench b;CHECK(b.session.request(b.evidence,b.r.facts,b.r.envelope,b.r.feedback,true,b.now));
  CHECK(b.session.primeClampMask()==((1u<<2)|(1u<<5)|(1u<<8)|(1u<<11)));}
 {Operational op;op.b.r.feedback.actual_rad[0]=.6;
  const auto ack=op.stand();REASON(ack.reason,"PRE_ARM_NOT_PRONE");CHECK(op.armer.calls==0&&op.b.r.backend.writes.empty());}
 {Rig r;r.feedback.actual_rad[0]=.2;CHECK(!r.arm());REASON(r.executor.status().reason,"START_OUTSIDE_LIMITS");CHECK(r.backend.writes.empty());}
 {Operational op;CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
  for(unsigned k=0;k<40&&op.b.coordinator.arming();++k)op.tickArming();
  CHECK(op.b.session.state()==MotionSessionState::ARMED);
  op.b.evidence.imu_fresh=false;op.b.evidence.scope_loss="IMU_STALE";
  const auto n=op.b.r.backend.writes.size();op.b.session.update(op.b.evidence,op.b.r.facts,op.b.r.feedback,op.b.now);
  CHECK(op.b.session.safeOffRequired());REASON(op.b.session.reason(),"IMU_STALE");
  op.b.r.executor.update(op.b.r.facts,op.b.r.feedback,op.b.now+1);CHECK(op.b.r.backend.writes.size()==n);}
 std::puts("RECOVERY_IMU PASS measured prime/entry and out-of-domain refusal; stale evidence latches SAFE_OFF; no sensor-service claim");
}
void releaseClockRates(){
 {StreamRig s;uint64_t time=100000;double q=0;const uint32_t intervals[]={17000,23000,21000,19000};
  for(unsigned k=0;k<4;++k){time+=intervals[k];s.fresh(time,q);q+=(.04+.01*k)*intervals[k]*1e-6;s.r.backend.tx_us=1040+500*k;CHECK(s.send(k+1,q,time));}}
 for(bool excessive:{false,true}){
  StreamRig s;s.r.backend.tx_us=19000;
  CHECK(s.send(1,.0016,120000));
  s.r.backend.tx_us=1040;s.fresh(140000,.0016);CHECK(s.send(2,.0032,140000));
  s.fresh(160000,.0032);
  const double next=excessive?.0062:.0055;
  const bool sent=s.send(3,next,160000);
  const double v=(next-.0032)/.020;
  std::printf("RELEASE_CLOCK excessive=%d dt_ms=20 dt_prev_release_ms=20 dt_prev_tx_ms=2.040 v_prev=.080 v_next=%.6f a_release=%.6f a_mixed=%.6f sent=%d reason=%s hardware=false\n",
   excessive,v,(v-.08)/.020,(v-.08)/.011020,sent,s.r.executor.status().reason);
  CHECK(sent==!excessive);
  if(excessive){REASON(s.r.executor.status().reason,"ACCELERATION_LIMIT");
   const auto n=s.r.backend.frames;s.send(4,.0063,180000);CHECK(s.r.backend.frames==n);}
 }
}
void safetyFaults(){
 const char* reasons[]={"OPERATOR_STOP","OUTSIDE_PATH_BOX","STREAM_FEEDBACK_STALE","FEEDBACK_CLOCK_INVALID","TARGET_OR_MAPPING","SEGMENT_UNVALIDATED","FRAME_TX_UNCERTAIN","FRAME_REFUSED_ZERO_BYTES","POWER_OR_BMS","TORQUE_LOST","FEEDBACK_INVALID","STREAM_CLOCK_INVALID","PATH_STEP_LIMIT","HOLD_SETTLE_TIMEOUT","STREAM_FEEDBACK_STALE","STREAM_STARVED","ACTUAL_OUTSIDE_LIMITS","TARGET_OUTSIDE_CERTIFIED_REGION"};
 for(unsigned kind=0;kind<18;++kind){StreamRig s;
  if(kind==0)s.r.executor.stop("OPERATOR_STOP");
  if(kind==1){s.fresh(120000,.021);s.monitor(120000);}
  if(kind==2){s.r.feedback.sampled_at_us[2]=99999;s.monitor(200000);}
  if(kind==3){s.r.feedback.sampled_at_us[2]=120001;s.monitor(120000);}
  if(kind==4)s.send(1,.2,120000);
  if(kind==5){auto v=s.r.validation(1);v.collision_clear=false;CHECK(!s.r.executor.stream(s.r.frame(1,.003),v,120000,140000,false,s.r.facts,s.r.feedback,120,120000));}
  if(kind==6){s.r.backend.frame_mode=Backend::FrameMode::PARTIAL;s.send(1,.003,120000);}
  if(kind==7){s.r.backend.frame_mode=Backend::FrameMode::REFUSE;s.send(1,.003,120000);}
  if(kind==8){s.r.facts.power_ok=false;s.send(1,.003,120000);}
  if(kind==9){s.r.feedback.torque_on[2]=false;s.send(1,.003,120000);}
  if(kind==10){s.r.feedback.read_ok[2]=false;s.send(1,.003,120000);}
  if(kind==11)s.monitor(99999);
  if(kind==12)s.send(1,.0041,125000);
  if(kind==13){CHECK(s.send(1,.003,120000,true));s.fresh(4120001,.003);s.r.feedback.speed_rad_s[0]=.01;s.monitor(4120001);}
  if(kind==14){s.r.feedback.recovering[2]=true;s.r.feedback.sampled_at_us[2]=99999;s.monitor(200000);}
  if(kind==15)s.monitor(200001);
  if(kind==16){s.fresh(120000,.16);s.monitor(120000);}
  if(kind==17)s.send(1,.101,120000);
  CHECK(s.r.executor.status().safe_off_required);if(std::strcmp(s.r.executor.status().reason,reasons[kind]))std::printf("FAULT kind=%u actual=%s expected=%s\n",kind,s.r.executor.status().reason,reasons[kind]);REASON(s.r.executor.status().reason,reasons[kind]);
  const auto frames=s.r.backend.frames;const auto writes=s.r.backend.writes.size();
  s.send(2,.002,4130000);s.r.executor.update(s.r.facts,s.r.feedback,4130,4130000);
  CHECK(s.r.backend.frames==frames&&s.r.backend.writes.size()==writes);
  s.r.executor.confirmSafeOff(false);CHECK(s.r.executor.status().safe_off_required);
  s.r.executor.confirmSafeOff(true);CHECK(s.r.executor.status().state==ExecutorState::OFF);
 }
 // STEP and real interval rate ceilings cannot be raised by a late release.
 {StreamRig s;CHECK(s.send(1,.003,120000));s.fresh(126000,.003);CHECK(!s.send(2,.006,126000));REASON(s.r.executor.status().reason,"RATE_LIMIT");}
 {StreamRig s;CHECK(s.send(1,.001,120000));s.fresh(170000,.001);CHECK(s.send(2,.001,170000));s.fresh(190000,.001);CHECK(!s.send(3,.004,190000));REASON(s.r.executor.status().reason,"ACCELERATION_LIMIT");}
 // STOP delivered between any two reads in the existing fast batch.
 for(unsigned stop_slot=0;stop_slot<execution::kOperationalFastReads;++stop_slot){StreamRig s;unsigned reads=0;
  for(unsigned k=0;execution::operationalPollReadAllowed(true,k,true);++k){
   if(k==stop_slot){s.r.executor.stop("MID_BATCH_STOP");break;}++reads;}
  CHECK(reads==stop_slot);CHECK(!s.send(1,.003,120000)&&s.r.backend.frames==0);
 }
 std::puts("SAFETY_FAULTS PASS: STOP, divergence, lost/recovering/future feedback, raw limit, collision, uncertain/refused TX, power, torque, clock, step/rate/acceleration, HOLD timeout, SAFE_OFF latch");
}
struct RunStats {Dist gaps,regular,age,sample_period,work;unsigned frames=0,checkpoints=0;double elapsed=0,tracking=0,step=0;};
RunStats standReplay(StreamModel m,bool walk=false,bool from_ready=false,bool explore=false){
 Operational op(2.,-92.070442,1,walk);auto& b=op.b;
 if(explore){
  // OFFLINE ONLY: no production profile is changed. Keep the certified step,
  // tube, measured entry and seven HOLDs; test a faster reference clock.
  // MATDOG_EXPLORE="v,a,period_ms" overrides the default 1 rad/s, 5 rad/s^2, 4 ms (sensitivity sweeps only).
  double ev=1,ea=5,ep=4;if(const char* e=std::getenv("MATDOG_EXPLORE"))std::sscanf(e,"%lf,%lf,%lf",&ev,&ea,&ep);
  auto p=b.runtime.feedback;p.samplePeriodS=ep/1000.;
  for(unsigned i=0;i<4;++i){p.maxVelocityRadS[i]={ev,ev,ev};p.maxAccelerationRadS2[i]={ea,ea,ea};}
  motion::AcquisitionRoute route;route.pairedProne=route.phaseTimed=route.geometryIdentityVerified=true;
  route.source=motion::EvidenceSource::OFFLINE_CERTIFIED;route.phaseVelocityRadS=ev;route.phaseAccelerationRadS2=ea;
  CHECK(b.coordinator.configure(p,b.runtime.walk,&route));
  b.r.envelope.max_velocity_rad_s=ev;b.r.envelope.max_acceleration_rad_s2=ea;
 }
 const uint32_t requested_ms=b.now;
 if(from_ready){op.publish();op.walk_ok=true;op.refresh();const auto ack=walkCommand(op,1,motion::operational_walk::profileVelocity(motion::operational_walk::kStandardProfile,1));std::printf("AUTO_STAND admission=%s\n",ack.reason);CHECK(ack.outcome==motion::CommandOutcome::ACCEPTED);}
 else CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
 for(unsigned k=0;k<40&&b.coordinator.arming();++k)op.tickArming();
 CHECK(b.session.state()==MotionSessionState::ARMED&&op.armer.calls==1);
 m.speed_rad_s=motion::operational_walk::kStandProfile.drive.goalSpeed*motion::tolerance::kTickRad;
 m.acc_rad_s2=motion::operational_walk::kStandProfile.drive.acceleration*100*motion::tolerance::kTickRad;
 if(explore){double gs=0,ac=0;if(const char* e=std::getenv("MATDOG_EXPLORE_DRIVE")){std::sscanf(e,"%lf,%lf",&gs,&ac);m.speed_rad_s=gs*motion::tolerance::kTickRad;m.acc_rad_s2=ac*100*motion::tolerance::kTickRad;}}
 StreamDriver d(op,m);const uint64_t start=d.now_us;uint64_t last_tx=0;unsigned previous=0;
 uint64_t samples[12]{};double last_q[12];for(unsigned i=0;i<12;++i)last_q[i]=b.r.executor.targetRadians()[i];
 struct PhaseStats {double seconds=0,hold=0,length=0,first[12]{},last[12]{};unsigned frames=0,holds=0;bool seen=false;};
 PhaseStats phases[16]{};
 RunStats st;bool ok=true,last_final=false,no_micro_arrival=true,one_frame=true,all_post=true;
 while(ok&&std::strcmp(b.coordinator.state(),"STAND")&&!b.coordinator.walking()&&d.now_us-start<90000000){
  if(from_ready&&d.present())b.service.renewTeleop(b.now,250); // Controller renews also during automatic STAND
  const auto phase_before=b.coordinator.stand().state();const bool held_before=b.r.executor.status().state==ExecutorState::HOLD;
  const uint64_t before=d.now_us;
  const auto c0=std::chrono::steady_clock::now();ok=d.tick();const auto c1=std::chrono::steady_clock::now();
  st.work.add(std::chrono::duration<double,std::micro>(c1-c0).count());
  const unsigned ph=static_cast<unsigned>(phase_before);if(ph<16){phases[ph].seconds+=(d.now_us-before)*1e-6;if(held_before)phases[ph].hold+=(d.now_us-before)*1e-6;}
  const auto& es=b.r.executor.status();st.tracking=std::max(st.tracking,es.stream_peak_error_rad);
  for(unsigned i=0;i<12;++i){st.age.add((d.now_us-d.sample_us[i])/1000.);
   if(samples[i]!=d.sample_us[i]){if(samples[i])st.sample_period.add((d.sample_us[i]-samples[i])/1000.);samples[i]=d.sample_us[i];}}
  if(d.frames_seen!=previous){one_frame=one_frame&&d.frames_seen==previous+1;
   if(last_tx){st.gaps.add((es.last_tx_complete_us-last_tx)/1000.);if(!last_final)st.regular.add((es.last_tx_complete_us-last_tx)/1000.);}
   last_tx=es.last_tx_complete_us;previous=d.frames_seen;last_final=b.coordinator.stand().pendingFinal();
   const unsigned ph=static_cast<unsigned>(b.coordinator.stand().state());if(ph<16){auto& p=phases[ph];double length=0;
    for(unsigned i=0;i<12;++i){if(!p.seen)p.first[i]=last_q[i];p.last[i]=b.r.executor.targetRadians()[i];length=std::max(length,std::abs(p.last[i]-last_q[i]));}
    p.seen=true;++p.frames;p.length+=length;if(last_final)++p.holds;}
   if(last_final)++st.checkpoints;
   else no_micro_arrival=no_micro_arrival&&es.completed_sequence<es.tx_sequence;
   for(unsigned i=0;i<12;++i){st.step=std::max(st.step,std::abs(b.r.executor.targetRadians()[i]-last_q[i]));last_q[i]=b.r.executor.targetRadians()[i];}}
  if(es.settled_sequence)for(unsigned i=0;i<12;++i)all_post=all_post&&d.sample_us[i]>es.last_tx_complete_us;
 }
 st.frames=d.frames_seen;st.elapsed=(d.now_us-start)/1e6;
 std::printf("STAND_REPLAY model=%s ok=%d elapsed_s=%.6f frames=%u checkpoints=%u gap_ms_p50=%.3f p99=%.3f max=%.3f regular_ms_p50=%.3f p99=%.3f max=%.3f feedback_age_ms_max=%.3f sample_period_ms_p50=%.3f max=%.3f tracking_peak_mrad=%.3f step_mrad=%.3f host_tick_work_us_p50=%.1f p99=%.1f max=%.1f reason=%s frame=%s hardware=false\n",
  m.name,ok,st.elapsed,st.frames,st.checkpoints,st.gaps.pct(.5),st.gaps.pct(.99),st.gaps.max(),st.regular.pct(.5),st.regular.pct(.99),st.regular.max(),st.age.max(),st.sample_period.pct(.5),st.sample_period.max(),st.tracking*1000,st.step*1000,st.work.pct(.5),st.work.pct(.99),st.work.max(),b.r.executor.status().reason,motion::feedbackReasonName(b.coordinator.frame().reason));
 for(unsigned ph=0;ph<16;++ph)if(phases[ph].seconds||phases[ph].seen){const auto& p=phases[ph];double net=0;
  for(unsigned i=0;i<12;++i)net=std::max(net,std::abs(p.last[i]-p.first[i]));
  std::printf("PHASE model=%s phase=%s seconds=%.6f frames=%u net_inf_rad=%.9f length_inf_rad=%.9f holds=%u hold_s=%.6f\n",m.name,motion::adaptiveStandStateName(static_cast<motion::AdaptiveStandState>(ph)),p.seconds,p.frames,net,p.length,p.holds,p.hold);}
 std::printf("END_TO_END model=%s prime_arm_s=%.6f total_s=%.6f five_second_result=%s exploratory=%d hardware=false\n",m.name,(start-requested_ms*1000ull)*1e-6,(d.now_us-requested_ms*1000ull)*1e-6,ok&&d.now_us-requested_ms*1000ull<=5000000&&!std::strcmp(b.coordinator.state(),"STAND")?"PASS":"FAIL",explore);
 if(!explore)CHECK(ok&&(!std::strcmp(b.coordinator.state(),"STAND")||b.coordinator.walking()));
 if(ok)CHECK(st.checkpoints==7&&no_micro_arrival&&one_frame&&all_post&&st.step<=.004+1e-12);
 CHECK(b.r.backend.frames==st.frames&&writesInsideLimits(b));
 if(!ok||(!b.coordinator.walking()&&std::strcmp(b.coordinator.state(),"STAND")))return st;
 if(walk){
  for(unsigned k=0;k<120&&ok;++k)ok=d.tick();
  op.walk_ok=true;op.refresh();
  if(!from_ready){
   d.physics(d.now_us+1000);b.now=d.now_us/1000;d.clock.extend(static_cast<uint32_t>(d.now_us));
   motion::AdaptiveObservation o;double q[12];
   for(unsigned i=0;i<12;++i){q[i]=b.r.feedback.actual_rad[i];b.r.feedback.sampled_at_us[i]=d.now_us-i*20;b.r.feedback.sampled_at_ms[i]=b.r.feedback.sampled_at_us[i]/1000;
    b.r.feedback.speed_rad_s[i]=(i%2?1:-1)*motion::tolerance::kTickRad;}
   observe(o,q,b.now);o.stationary=false;b.coordinator.observe(o);b.coordinator.update(b.r.facts,b.r.feedback,b.now,d.now_us);
   CHECK(!std::strcmp(b.coordinator.state(),"STAND")&&!b.coordinator.walking());
   std::puts("RESIDUAL_STAND_NOISE waiting_for_WALK=true no_extra_operator_gate=true");
  }
  if(!from_ready)CHECK(walkCommand(op,30,motion::operational_walk::profileVelocity(motion::operational_walk::kStandardProfile,1)).outcome==motion::CommandOutcome::ACCEPTED);
  d.m.speed_rad_s=motion::operational_walk::kStandardDrive.goalSpeed*motion::tolerance::kTickRad;
  d.m.acc_rad_s2=motion::operational_walk::kStandardDrive.acceleration*100*motion::tolerance::kTickRad;
  const auto w0=d.now_us;while(ok&&b.coordinator.walkCycles()<1.1&&d.now_us-w0<16000000)ok=d.tick();
  CHECK(ok&&b.coordinator.walkCycles()>=1.1);
  CHECK(walkCommand(op,31,0).outcome==motion::CommandOutcome::ACCEPTED);
  const auto z0=d.now_us;while(ok&&b.coordinator.walking()&&d.now_us-z0<30000000)ok=d.tick();
  CHECK(ok&&!b.coordinator.walking()&&b.r.executor.status().state==ExecutorState::READY);
  if(!from_ready){
   CHECK(walkCommand(op,32,motion::operational_walk::profileVelocity(motion::operational_walk::kStandardProfile,1)).outcome==motion::CommandOutcome::ACCEPTED);
   for(unsigned k=0;k<50&&ok;++k)ok=d.tick();
   d.heartbeat=false;const auto deadman_start=d.now_us;
   while(ok&&d.now_us-deadman_start<4000000)ok=d.tick();
   CHECK(!ok&&b.r.executor.status().safe_off_required);
   const auto stopped=b.r.backend.frames;for(unsigned k=0;k<5;++k)d.tick();CHECK(b.r.backend.frames==stopped);
   std::printf("WALK_DEADMAN reason=%s zero_post_stop_frames=true hardware=false\n",b.r.executor.status().reason);
  }
  std::printf("WALK_STANDARD auto_stand=%d coordinated_stop_checked=true state=%s hardware=false\n",from_ready,b.coordinator.state());
 }
 auto stop=command(motion::Intent::STOP,90,motion::ClientOwner::DASHBOARD);stop.issued_at_ms=b.now;b.service.command(stop,b.now);
 const auto n=b.r.backend.frames;for(unsigned k=0;k<5;++k)d.tick();CHECK(b.r.executor.status().safe_off_required&&b.session.safeOffRequired()&&b.r.backend.frames==n);
 return st;
}
// Offline candidate uses the SAME fixed-contact quintic and protected height box.
// Dense numerical survey is not a continuous proof or hardware qualification.
void heightTimingSurvey(){
 Bench b;double peak_v=0,peak_a=0,peak_step=0;unsigned paths=0,samples=0;
 std::vector<std::pair<double,double>> cases;
 for(unsigned from=60;from<=150;from+=5)for(unsigned to=60;to<=150;to+=5)if(from!=to)cases.push_back({double(from),double(to)});
 for(double from:{60.,100.,149.,150.})for(double span:{.00001,.001,1.})
  cases.push_back({from,from==150?from-span:from+span});
 for(const auto& heights:cases){
  const double from=heights.first,to=heights.second;
  auto d=motion::canonicalStandDefinition();d.lowBodyHeightM=std::min(from,to)/1000.;d.standBodyHeightM=std::max(from,to)/1000.;
  // Production law (AdaptiveStand::requestHeightAdjust): envelope .19 rad/s / 2 rad/s^2, 12 ms lateness tolerance.
  motion::StandHeightLaw law;const auto seed=motion::generateStandTarget(d);motion::LegJointAngles seeds[4];for(unsigned i=0;i<4;++i)seeds[i]=seed.target.legs[i];
  const double vref=motion::velocityForLateness(.19,2.,.020,.004,motion::kStreamLatenessToleranceS);
  CHECK(vref>0&&motion::planStandHeightLaw(d,seeds,vref,1.,motion::kHeightAccelRampS,.040,&law));
  const double duration=law.durationS;
  const unsigned n=std::max(2u,static_cast<unsigned>(std::ceil(duration/.020)));   // real 20 ms frame steps
  auto previous=motion::evaluateStandWithLaw(d,law.at(0),d.standSeed);CHECK(previous.status==motion::TimedStatus::OK);
  for(unsigned k=0;k<=n;++k){const auto f=motion::evaluateStandWithLaw(d,law.at(duration*k/n),previous.geometry.target.legs);
   CHECK(f.status==motion::TimedStatus::OK&&motion::heightTargetInRegion(f.geometry.target));
   for(unsigned i=0;i<12;++i){
    const auto& v=f.velocityRadS[i/3];const auto& a=f.accelerationRadS2[i/3];
    peak_v=std::max(peak_v,std::abs(i%3==0?v.hip:i%3==1?v.upper:v.lower));
    peak_a=std::max(peak_a,std::abs(i%3==0?a.hip:i%3==1?a.upper:a.lower));
    if(k)peak_step=std::max(peak_step,std::abs(slot(f.geometry.target,i)-slot(previous.geometry.target,i)));
    CHECK(slot(f.geometry.target,i)>=b.runtime.feedback.jointLimits[i/3][i%3].lower&&slot(f.geometry.target,i)<=b.runtime.feedback.jointLimits[i/3][i%3].upper);
   }
   previous=f;++samples;
  }++paths;
 }
 CHECK(peak_v<=.19&&peak_a<=2&&peak_step<=.004);
 std::printf("HEIGHT_TIMING_SURVEY paths=%u samples=%u v_max=%.9f a_max=%.9f step_max=%.9f current_limits=true hardware=false\n",paths,samples,peak_v,peak_a,peak_step);
}
void heightReplay(bool jitter=false){
 Operational op(2.);auto& b=op.b;
 CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
 for(unsigned k=0;k<40&&b.coordinator.arming();++k)op.tickArming();
 StreamModel m;m.loop_us=4000;m.reads_per_tick=3;m.jitter_us=jitter?1200:0;StreamDriver d(op,m);
 bool ok=true;
 while(ok&&std::strcmp(b.coordinator.state(),"STAND")&&d.now_us<60000000)ok=d.tick();
 CHECK(ok&&!std::strcmp(b.coordinator.state(),"STAND"));if(!ok)return;
 unsigned seq=10;
 for(double h:{.149,.140,.150,.100,.060,.150}){
  const auto start=d.now_us;const auto before=b.r.backend.frames;
  const auto ack=op.standAt(h,seq++);CHECK(ack.outcome==motion::CommandOutcome::ACCEPTED);
  while(ok&&std::strcmp(b.coordinator.state(),"STAND")&&d.now_us-start<60000000)ok=d.tick();
  std::printf("HEIGHT_CHAIN jitter=%d target_mm=%.0f elapsed_s=%.6f frames=%u state=%s reason=%s hardware=false\n",jitter,h*1000,(d.now_us-start)/1e6,b.r.backend.frames-before,b.coordinator.state(),b.r.executor.status().reason);
  CHECK(ok&&!std::strcmp(b.coordinator.state(),"STAND")&&std::abs(b.coordinator.stand().standHeightM()-h)<1e-9);
  if(!ok)return;
 }
 for(double h:{0.,.020,.040}){const auto before=b.r.backend.frames;const auto ack=op.standAt(h,seq++);
  CHECK(ack.outcome==motion::CommandOutcome::REJECTED&&b.r.backend.frames==before);}
}
}
// ===================== Core Motion V6 regressions (RCA phase 1 -> phase 2) =====================
// C1 numeric contract: ONE RAW -> radians conversion for samples and limit ends, exact round trip, inclusive edges.
void numericContract(){
 Bench b;auto& r=b.r;unsigned ticks=0,edges=0;bool exact=true,roundtrip=true,agree=true;
 for(unsigned i=0;i<12;++i){const auto id=slotIdentity(i);const auto* t=r.policy.transforms().find(id,r.geometry.provenanceTag());const auto* l=r.policy.limits().find(id,r.geometry.provenanceTag());
  CHECK(t&&l);if(!t||!l)continue;
  for(int tick=static_cast<int>(l->min_tick)-3;tick<=static_cast<int>(l->max_tick)+3;++tick){double q=0;
   if(tick<0||tick>4095||resolveRawToRadians(r.geometry,geometry_data::kProvenance,*t,static_cast<uint16_t>(tick),&q)!=TargetResolveStatus::OK)continue;
   uint16_t back=0;roundtrip=roundtrip&&resolveRadiansToRaw(r.geometry,geometry_data::kProvenance,*t,q,&back)==TargetResolveStatus::OK&&back==tick;
   // The producer gives the SAME radians as the conversion of the limit end on the same tick.
   execution::OperationalFeedback prod;execution::RawJointObservation raw;raw.read_ok=true;raw.position=static_cast<uint16_t>(tick);raw.torque=0;raw.status=0;raw.voltage=120;raw.temperature=25;raw.sampled_at_ms=100;raw.sampled_at_us=100000;
   prod.ingest(i,raw,r.geometry,geometry_data::kProvenance,r.policy.transforms());const double fq=prod.snapshot(100,400).actual_rad[i];exact=exact&&fq==q;
   const bool in_raw=l->contains(static_cast<uint16_t>(tick)),in_rad=fq>=b.limits[i][0]&&fq<=b.limits[i][1];agree=agree&&in_raw==in_rad;
   if(tick==l->min_tick||tick==l->max_tick){++edges;CHECK(in_rad&&in_raw);}
   ++ticks;}}
 CHECK(exact&&roundtrip&&agree&&edges==24);
 // The phase-1 counterexample integer (LF LOWER limit end): the two old formulas round apart; the canonical one is used everywhere.
 const MicroRad u=-1598408;
 std::printf("NUMERIC_CONTRACT ticks=%u limit_edges_inclusive=%u producer_equals_limit_conversion=%d raw_roundtrip=%d rad_raw_membership_agree=%d old_div=%.17g old_mul=%.17g canonical=%.17g hardware=false\n",
  ticks,edges,exact,roundtrip,agree,u/1e6,u*1e-6,microRadToRadians(u));
 CHECK(u/1e6!=u*1e-6&&microRadToRadians(u)==u/1e6);
 // Slot identity: the one mapping. Every slot names its canonical servo (leg prefix, joint suffix, bus leg*10+{3,2,1})
 // and the four LOWER slots keep their qualified encoder directions.
 const char* legs[4]={"LF_","RF_","RH_","LH_"};const char* kinds[3]={"HIP","UPPER","LOWER"};const int8_t dir[4]={-1,1,1,-1};
 for(unsigned i=0;i<12;++i){JointIdentity id;const auto* sv=slotServo(i);CHECK(sv&&slotIdentity(i,&id)&&static_cast<unsigned>(id.leg)==i/3&&static_cast<unsigned>(id.joint)==i%3);
  CHECK(sv&&std::strncmp(sv->joint,legs[i/3],3)==0&&std::strcmp(sv->joint+3,kinds[i%3])==0&&sv->bus_id==(i/3+1)*10+3-i%3&&std::strcmp(id.physical_unit,sv->physical_unit)==0);
  if(i%3==2)CHECK(jointDirection(r.geometry,id)==dir[i/3]);}
}
// C1 end to end: a LOWER whose RAW sits EXACTLY on its inclusive folded limit (both RAW ends, both directions) arms,
// is observed and streams - the phase-1 run stopped JOINT_LIMIT with zero frames on the same bench.
void rawEdgeEndToEnd(){
 for(unsigned joint:{2u,5u,8u,11u}){
  Operational op;auto& b=op.b;auto& r=b.r;
  for(unsigned i=2;i<12;i+=3)r.feedback.actual_rad[i]=b.limits[i][0]+.01;
  const auto* lim=r.policy.limits().find(slotIdentity(joint),r.geometry.provenanceTag());
  const uint16_t edge=qOf(r,joint,lim->min_tick)<qOf(r,joint,lim->max_tick)?lim->min_tick:lim->max_tick;
  r.feedback.actual_rad[joint]=qOf(r,joint,edge);CHECK(r.feedback.actual_rad[joint]==b.limits[joint][0]);
  const auto ack=op.stand();for(unsigned k=0;k<40&&b.coordinator.arming();++k)op.tickArming();
  StreamModel m;m.ideal=true;m.loop_us=4000;m.reads_per_tick=3;StreamDriver d(op,m);bool ok=true;
  for(unsigned k=0;k<200&&ok&&r.executor.status().tx_sequence<3;++k)ok=d.tick();
  std::printf("RAW_EDGE joint=%u raw=%u side=%s q=%.17g limit=%.17g command=%s session=%s tx=%u reason=%s hardware=false\n",joint,edge,edge==lim->min_tick?"MIN":"MAX",
   r.feedback.actual_rad[joint],b.limits[joint][0],ack.reason,b.session.stateName(),r.executor.status().tx_sequence,r.executor.status().reason);
  CHECK(ack.outcome==motion::CommandOutcome::ACCEPTED&&ok&&r.executor.status().tx_sequence>=3&&writesInsideLimits(b));}
 // Outside the clamp evidence (and every HIP/UPPER outside its limit) stays refused before any torque.
 {Operational op;auto& b=op.b;b.r.feedback.actual_rad[2]=b.limits[2][0]-20*motion::tolerance::kTickRad;
  const auto ack=op.stand();REASON(ack.reason,"PRIME_REQUIRES_OFF_AND_LIMITS");CHECK(op.armer.calls==0&&b.r.backend.writes.empty());}
 {Operational op;auto& b=op.b;b.r.feedback.actual_rad[1]=b.limits[1][0]-motion::tolerance::kTickRad;
  const auto ack=op.stand();CHECK(ack.outcome!=motion::CommandOutcome::ACCEPTED&&b.r.backend.writes.empty());}
}
// C1 contact exit: the measured-prone bench rests every LOWER 1..3 ticks past its folded limit (on the stop). PRIME
// holds the inside tick; a LOWER that stops short (still on the contact side) is admitted at ARM, the ENTRY moves it
// away; one that goes DEEPER than its measured rest is refused. No target ever leaves the RAW limits.
void contactExit(){
 for(unsigned variant=0;variant<2;++variant){
  Operational op;auto& b=op.b;auto& r=b.r;
  CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
  for(unsigned k=0;k<40&&b.coordinator.arming();++k){
   b.fresh();b.session.update(b.evidence,r.facts,r.feedback,b.now);
   if(b.session.state()==MotionSessionState::ARMING){for(auto& on:r.feedback.torque_on)on=true;
    // The servos reach their PRIME goal, except the clamped LOWERs which stop at their rest (on the contact side).
    for(unsigned i=0;i<12;++i)r.feedback.actual_rad[i]=b.session.primeClampMask()&(1u<<i)?b.measured[i]:b.session.primedRadians()[i];}
   motion::AdaptiveObservation o;double q[12];for(unsigned i=0;i<12;++i)q[i]=r.feedback.actual_rad[i];observe(o,q,b.now);b.coordinator.observe(o);
   b.coordinator.update(r.facts,r.feedback,b.now);op.refresh();}
  CHECK(b.session.state()==MotionSessionState::ARMED&&r.executor.contactMask()==b.session.primeClampMask());
  // The ENTRY seed IS the PRIME hold the executor holds: one description, no reconstruction.
  bool seed=true;for(unsigned i=0;i<12;++i)seed=seed&&motion::jointSlot(b.coordinator.entryPlan().start,i)==r.executor.targetRadians()[i]&&r.executor.targetRadians()[i]==b.session.primedRadians()[i];
  CHECK(seed);
  StreamModel m;m.loop_us=4000;m.reads_per_tick=3;StreamDriver d(op,m);bool ok=true;
  if(variant==1){d.stuck[5]=true;for(unsigned i=0;i<12;++i)if(b.session.primeClampMask()&(1u<<i)){d.q[i]=b.measured[i];}
   // RF LOWER pushed one tick DEEPER into its contact than its measured rest.
   const auto* lim=r.policy.limits().find(slotIdentity(5),r.geometry.provenanceTag());const uint16_t rest=tickOf(r,5,b.measured[5]);
   d.q[5]=qOf(r,5,static_cast<uint16_t>(rest<lim->min_tick?rest-1:rest+1));}
  while(ok&&r.executor.status().completed_sequence==0&&d.now_us<20000000)ok=d.tick();
  std::printf("CONTACT_EXIT variant=%s clamp=0x%03x ok=%d completed=%u contact_band_after=0x%03x reason=%s writes_inside=%d hardware=false\n",variant?"DEEPER":"STOPPED_SHORT",
   b.session.primeClampMask(),ok,r.executor.status().completed_sequence,r.executor.contactMask(),r.executor.status().reason,writesInsideLimits(b));
  CHECK(writesInsideLimits(b));
  if(variant==0)CHECK(ok&&r.executor.status().completed_sequence>0&&r.executor.contactMask()==0);
  else{CHECK(!ok);REASON(r.executor.status().reason,"ACTUAL_OUTSIDE_LIMITS");CHECK(r.executor.refusal().joint==5);}
 }
}
// C2 retiming: a straight reference at cruise, the executor's committed state, late releases and the absolute-deadline
// return. MotionLaw::retimeRelease against the V5 clamp min(elapsed, step/vmax).
namespace {struct Line{double v;};bool linePath(const void* c,double tau,double q[12]){const double v=static_cast<const Line*>(c)->v;for(unsigned i=0;i<12;++i)q[i]=v*tau;return true;}}
void releaseRetiming(){
 const motion::ReleaseEnvelope env{.19,2.,.004};const double vlaw=.999*.19;Line line{vlaw};
 for(double late_ms:{21.855,22.471,27.322,28.,30.453}){
  // Committed: cruise, last frame at tau 0.2 s released at 200 ms after a 20 ms interval.
  double q[12],v[12];for(unsigned i=0;i<12;++i){q[i]=vlaw*.2;v[i]=vlaw;}
  motion::ReleaseState st{q,v,200000,20000};const uint64_t late=200000+static_cast<uint64_t>(late_ms*1000+.5);
  const auto r1=motion::retimeRelease(st,late,env,.2,.2+late_ms/1000,linePath,&line);
  // V5 clamp: advance min(elapsed, step/vmax), then the return to the law at the next absolute deadline (240 ms).
  const double adv=std::min(late_ms/1000,.004/.19),v5late=vlaw*adv/(late_ms/1000),a5late=(v5late-vlaw)/(.5*(late_ms/1000+.02));
  const double back_ms=40-late_ms,a5back=(vlaw-v5late)/(.5*(late_ms+back_ms)/1000);
  double a6back=0;bool back_ok=false;
  if(r1.found){double q2[12],v2[12];for(unsigned i=0;i<12;++i){q2[i]=vlaw*r1.tau;v2[i]=(q2[i]-q[i])/((late-200000)*1e-6);}
   motion::ReleaseState st2{q2,v2,late,late-200000};
   const auto r2=motion::retimeRelease(st2,240000,env,r1.tau,r1.tau+back_ms/1000,linePath,&line);back_ok=r2.found;
   if(r2.found){const double vb=vlaw*(r2.tau-r1.tau)/(back_ms/1000);a6back=(vb-v2[0])/(.5*(late_ms+back_ms)/1000);}}
  const double a6late=r1.found?(vlaw*(r1.tau-.2)/(late_ms/1000)-vlaw)/(.5*(late_ms/1000+.02)):r1.check.value;
  std::printf("RETIME late_ms=%.3f v5_late_a=%.6f v5_return_a=%.6f v5_ok=%d | v6_found=%d v6_late_a=%.6f v6_return_ok=%d v6_return_a=%.6f evaluations=%u verdict=%s hardware=false\n",
   late_ms,a5late,a5back,std::fabs(a5late)<=2+1e-9&&std::fabs(a5back)<=2+1e-9,r1.found,a6late,back_ok,a6back,r1.evaluations,motion::releaseVerdictName(r1.check.verdict));
  if(late_ms<=28.){CHECK(r1.found&&back_ok&&std::fabs(a6late)<=2+1e-9&&std::fabs(a6back)<=2+1e-9);}
  else {CHECK(!r1.found&&r1.check.verdict==motion::ReleaseVerdict::ACCELERATION&&r1.check.tooSlow&&std::fabs(r1.check.value)>2);}
 }
 // The common stream lateness contract (12 ms) sets the cruise the laws are planned with: the 30.453 ms release of
 // hardware stop 7 and its return to the absolute-deadline interval are then feasible at full cruise.
 {const double vc=motion::velocityForLateness(.19,2,.02,.004,motion::kStreamLatenessToleranceS);Line cruise{vc};double q[12],v[12];
  for(unsigned i=0;i<12;++i){q[i]=vc*.2;v[i]=vc;}motion::ReleaseState st{q,v,200000,20000};
  const auto r1=motion::retimeRelease(st,230453,env,.2,.230453,linePath,&cruise);bool back=false;double aback=0;
  if(r1.found){double q2[12],v2[12];for(unsigned i=0;i<12;++i){q2[i]=vc*r1.tau;v2[i]=(q2[i]-q[i])/.030453;}motion::ReleaseState st2{q2,v2,230453,30453};
   const auto r2=motion::retimeRelease(st2,240000,env,r1.tau,r1.tau+.009547,linePath,&cruise);back=r2.found;
   if(back)aback=(vc*(r2.tau-r1.tau)/.009547-v2[0])/(.5*(.030453+.009547));}
  std::printf("RETIME_CONTRACT cruise=%.6f late_ms=30.453 found=%d return_ok=%d return_a=%.6f hardware=false\n",vc,r1.found,back,aback);
  CHECK(r1.found&&back&&std::fabs(aback)<=2+1e-9);}
 // The same 30.453 ms release during the ramp (0.15 rad/s): feasible, no discontinuity.
 {Line ramp{.15};double q[12],v[12];for(unsigned i=0;i<12;++i){q[i]=.15*.2;v[i]=.15;}motion::ReleaseState st{q,v,200000,20000};
  const auto r=motion::retimeRelease(st,230453,env,.2,.230453,linePath,&ramp);CHECK(r.found);
  std::printf("RETIME_RAMP v=0.15 late_ms=30.453 found=%d tau_ms=%.3f hardware=false\n",r.found,r.tau*1000);}
 // A real but very narrow feasible interval: at 32 ms, .177 rad/s can
 // decelerate to .125 exactly at the 4 mrad step cap. Fourteen bisections
 // miss it; this must not become ACCELERATION_LIMIT because of resolution.
 {Line cruise{.177};double q[12]{},v[12];for(double& x:v)x=cruise.v;
  motion::ReleaseState st{q,v,200000,20000};
  const auto r=motion::retimeRelease(st,232000,env,0,.032,linePath,&cruise);
  CHECK(r.found);double target[12];linePath(&cruise,r.tau,target);
  CHECK(motion::checkRelease(st,target,232000,env).verdict==motion::ReleaseVerdict::OK);
  const auto impossible=motion::retimeRelease(st,233000,env,0,.033,linePath,&cruise);
  CHECK(!impossible.found&&impossible.check.verdict==motion::ReleaseVerdict::ACCELERATION);
  std::printf("RETIME_BOUNDARY dt32_found=%d evaluations=%u dt33_found=%d a_min_dt33=%.9f limit=2\n",
   r.found,r.evaluations,impossible.found,(.177-.004/.033)/(.5*(.033+.020)));}
 // On time: one evaluation, the law itself.
 {double q[12],v[12];for(unsigned i=0;i<12;++i){q[i]=vlaw*.2;v[i]=vlaw;}motion::ReleaseState st{q,v,200000,20000};
  const auto r=motion::retimeRelease(st,220000,env,.2,.22,linePath,&line);CHECK(r.found&&r.evaluations==1&&r.tau==.22);}
 // A true acceleration violation of the law itself is refused, never smoothed away.
 {double q[12],v[12];for(unsigned i=0;i<12;++i){q[i]=0;v[i]=0;}motion::ReleaseState st{q,v,200000,20000};
  const auto c=motion::checkRelease(st,[]{static double t[12];for(double& x:t)x=.0038;return t;}(),220000,env);CHECK(c.verdict==motion::ReleaseVerdict::ACCELERATION);}
}
// Release window (MotionClock): a tick that ends just before a deadline serves it then instead of one whole tick later.
// Serving early moves the next deadline too: steady 19 ms ticks must never
// accumulate phase error and manufacture a 38 ms gap.
void releaseWindow(){
 auto run=[](uint32_t early,uint32_t tick_lo,uint32_t tick_span,unsigned* skips){core::PeriodicScheduler s;CHECK(s.start(20000,20000,early));
  uint64_t last=0;double worst=0;uint32_t lcg=7;*skips=0;
  for(uint64_t now=0;now<20000000;){lcg=lcg*1103515245u+12345u;now+=tick_lo+(tick_span?(lcg>>8)%tick_span:0);
   if(s.poll(now).release){if(last){worst=std::max(worst,(now-last)/1000.);if(now-last>30000)++*skips;}last=now;}}
  return worst;};
 for(uint32_t tick:{19000u,17000u,12000u,4000u}){unsigned a=0,b=0;const double w0=run(0,tick,0,&a),w1=run(20000/core::kStreamReleaseWindowDivisor,tick,0,&b);
  std::printf("RELEASE_WINDOW tick_ms=%.0f max_ms without=%.1f with=%.1f long_intervals without=%u with=%u hardware=false\n",tick/1000.,w0,w1,a,b);CHECK(w1<=w0);if(tick>=15000)CHECK(w1==tick/1000.&&b==0);}
 {unsigned a=0,b=0;const double w0=run(0,19000,12000,&a),w1=run(20000/core::kStreamReleaseWindowDivisor,19000,12000,&b);
  std::printf("RELEASE_WINDOW tick_ms=19..31 max_ms without=%.1f with=%.1f intervals_over_30ms without=%u with=%u hardware=false\n",w0,w1,a,b);CHECK(w1<w0&&b<a);}
 {core::PeriodicScheduler s;CHECK(!s.start(0,20000,20000));}   // a window must stay inside the period
}
// C2 end to end: the Controller's cost between the first and the second coordinator call (frame materialized ->
// assessed frame transmitted). Phase 1: 9 ms there stopped the STAND with ACCELERATION_LIMIT (zero analytic a).
void generationReleaseDelay(){
 for(unsigned mode=0;mode<6;++mode){
  Operational op(2.);auto& b=op.b;CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
  for(unsigned k=0;k<40&&b.coordinator.arming();++k)op.tickArming();
  StreamModel m;m.loop_us=4000;m.reads_per_tick=3;m.ideal=mode==0;
  if(mode==0)m.assess_delay_us=9000;else if(mode==1)m.assess_jitter_us=9000;else if(mode==2){m.assess_delay_us=2000;m.assess_jitter_us=12000;m.jitter_us=4000;}
  // Hardware-like (stand150 log 21/10: poll ~3 ms, planner 9-11 ms, ticks 21-30 ms): 19-31 ms ticks, 9 ms between calls.
  else{m.loop_us=10000;m.jitter_us=12000;m.assess_delay_us=9000;}
  // Charge the additional three reads on TOP of the old measured cadence;
  // they are not free time hidden inside the old loop. The cost is a model.
  if(mode>=4){if(mode==5)m.read_us=1500;m.reads_per_tick=execution::kOperationalFastReads;m.loop_us+=(m.reads_per_tick-3)*m.read_us;}
  StreamDriver d(op,m);bool ok=true;unsigned searched=0;const uint64_t t0=d.now_us;Dist interval;uint64_t last=0,max_age=0;
  while(ok&&std::strcmp(b.coordinator.state(),"STAND")&&d.now_us-t0<90000000){ok=d.tick();if(b.coordinator.stand().lastRetimeEvaluations()>1)++searched;
   for(unsigned i=0;i<12;++i)max_age=std::max(max_age,d.now_us-d.sample_us[i]);
   const auto at=b.r.executor.releaseState().atUs;if(at!=last){if(last&&at-last<60000)interval.add((at-last)/1000.);last=at;}}
  std::printf("GEN_RELEASE_DELAY mode=%s ok=%d state=%s frames=%u elapsed_s=%.3f release_interval_ms_p50=%.3f p99=%.3f max=%.3f retime_searches=%u reason=%s refusal_site=%s hardware=false\n",
   mode==0?"CONST_9MS":mode==1?"JITTER_0_9MS":mode==2?"LOOP4+JITTER_2_14MS":mode==3?"OLD_POLL_19_31MS":mode==4?"HALF_SWEEP_19_31MS":"HALF_SWEEP_READ_1P5MS",ok,b.coordinator.state(),d.frames_seen,(d.now_us-t0)/1e6,
   interval.pct(.5),interval.pct(.99),interval.max(),searched,b.r.executor.status().reason,b.r.executor.refusal().site);
  std::printf("FEEDBACK_ACQUISITION reads=%u read_us=%u max_age_ms=%.3f limit_ms=100\n",m.reads_per_tick,m.read_us,max_age/1000.);
  if(mode!=3)CHECK(ok&&!std::strcmp(b.coordinator.state(),"STAND")&&writesInsideLimits(b));
  else{CHECK(writesInsideLimits(b));
   // Negative control: the old acquisition quota still reaches the unchanged
   // freshness limit. Startup/HOLD-to-stream intervals are not cruise gaps;
   // assert the actual cause and operands, not an incidental cadence maximum.
   CHECK(!ok&&max_age>100000);REASON(b.r.executor.status().reason,"STREAM_FEEDBACK_STALE");
   const auto& w=b.r.executor.refusal();
   if(!ok)std::printf("GEN_RELEASE_DELAY_REFUSAL site=%s reason=%s joint=%d q=%.9f q_last=%.9f v=%.6f v_prev=%.6f dt_ms=%.3f dt_prev_ms=%.3f value=%.6f limit=%.6f hardware=false\n",
    w.site,w.reason,w.joint,w.q,w.qLast,w.v,w.vPrev,w.dtS*1000,w.dtPrevS*1000,w.value,w.limit);
   if(!ok)CHECK(w.valid&&*w.site&&w.joint>=0);}
 }
}
// Production quota and genuine loss, isolated from trajectory dynamics. A
// successful read gets a new timestamp; an absent reply keeps the old one.
void feedbackCadence(){
 for(bool lost:{false,true}){StreamRig s;CHECK(s.send(1,0,120000,true));
  uint64_t now=120000,max_age=0;unsigned rr=0,reads=0;uint32_t random=7;
  for(unsigned turn=0;turn<100&&!s.r.executor.status().safe_off_required;++turn){
   random=random*1103515245u+12345u;now+=19000+(random>>8)%12000;
   bool ok=true;for(unsigned k=0;execution::operationalPollReadAllowed(true,k,ok);++k){
    now+=1500;ok=!(lost&&rr==5);if(!ok)break;
    sampleJoint(s.r.feedback,rr,now,0);rr=(rr+1)%12;++reads;
   }
   now+=9000; // planning/other work after the samples; no free or restamped time
   for(unsigned i=0;i<12;++i)max_age=std::max(max_age,now-s.r.feedback.sampled_at_us[i]);
   s.monitor(now);
  }
  if(lost){REASON(s.r.executor.status().reason,"STREAM_FEEDBACK_STALE");CHECK(s.r.executor.refusal().sampleUs==100000);}
  else CHECK(!s.r.executor.status().safe_off_required&&max_age<=100000);
  std::printf("FEEDBACK_CADENCE lost=%d reads=%u max_age_ms=%.3f reason=%s hardware=false\n",lost,reads,max_age/1000.,s.r.executor.status().reason);
 }
 CHECK(!execution::operationalPollReadAllowed(true,1,false));
 CHECK(execution::operationalPollReadAllowed(false,0,true)&&!execution::operationalPollReadAllowed(false,1,true));
}
// No state advances on a failed TX: the executor's committed release and the stand's committed reference stay put.
void txFailureNoAdvance(){
 {StreamRig s;CHECK(s.send(1,.0038,120000));const auto before=s.r.executor.releaseState();double q=before.q[0],v=before.v[0];uint64_t at=before.atUs,in=before.intervalUs;
  s.r.backend.frame_mode=Backend::FrameMode::REFUSE;s.fresh(140000,.0038);CHECK(!s.send(2,.0076,140000));
  const auto after=s.r.executor.releaseState();CHECK(after.q[0]==q&&after.v[0]==v&&after.atUs==at&&after.intervalUs==in&&s.r.executor.status().tx_sequence==1);}
 {Operational op;auto& b=op.b;CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);for(unsigned k=0;k<40&&b.coordinator.arming();++k)op.tickArming();
  StreamModel m;m.ideal=true;m.loop_us=4000;m.reads_per_tick=3;StreamDriver d(op,m);bool ok=true;
  while(ok&&b.r.executor.status().tx_sequence<20)ok=d.tick();
  double sent[12];for(unsigned i=0;i<12;++i)sent[i]=b.r.executor.targetRadians()[i];const auto tx=b.r.executor.status().tx_sequence;
  b.r.backend.frame_mode=Backend::FrameMode::PARTIAL;for(unsigned k=0;k<20&&ok;++k)ok=d.tick();
  // The stand commits a frame only once it was transmitted (tx_sequence): after the failed TX its reference is the
  // last TRANSMITTED target, which the executor still holds; the failed candidate committed nowhere.
  bool same=true,held=true,candidate_differs=false;
  for(unsigned i=0;i<12;++i){same=same&&motion::jointSlot(b.coordinator.stand().referenceTarget(),i)==b.r.executor.targetRadians()[i];
   held=held&&b.r.executor.targetRadians()[i]==sent[i];candidate_differs=candidate_differs||motion::jointSlot(b.coordinator.frame().target,i)!=sent[i];}
  std::printf("TX_FAILURE_NO_ADVANCE reason=%s tx_before=%u tx_after=%u executor_held=%d stand_reference_is_last_tx=%d failed_candidate_differs=%d hardware=false\n",
   b.r.executor.status().reason,tx,b.r.executor.status().tx_sequence,held,same,candidate_differs);
  CHECK(!ok&&b.r.executor.status().tx_sequence==tx&&held&&same&&candidate_differs);REASON(b.r.executor.status().reason,"FRAME_TX_UNCERTAIN");}
}
// Coherence of every operational profile (reference, period, step cap, drive pair) with the lateness each can absorb.
void dynamicProfiles(){
 const char* why="";CHECK(motion::operational_walk::dynamicProfilesCoherent(&why));
 using namespace motion::operational_walk;using motion::DynamicProfile;using motion::analyzeDynamicProfile;
 DynamicProfile list[]={dynamicProfile(kStandProfile),dynamicProfile(kStandardProfile),dynamicProfile(kFastProfile),
   // OFFLINE candidate, NOT enabled (needs kMinSamplePeriodS 0.02->0.01): faster reference, finer period, the already reviewed WALK drive pair.
   DynamicProfile{"STAND_CANDIDATE_OFFLINE",.22,2.,0,.010,.004,1000,50}};
 for(const auto& d:list){const auto a=analyzeDynamicProfile(d);CHECK(a.valid);
  std::printf("DYNAMIC_PROFILE %-24s v=%.3f a=%.2f T_ms=%.0f step_mrad=%.2f step_headroom_ms=%.2f accel_safe_lateness_ms=%.2f drive=%u/%u ratio_v=%.2f ratio_a=%.2f valid=%d\n",
   d.name,d.velocityRadS,d.accelerationRadS2,d.periodS*1000,a.stepAtNominalRad*1000,a.stepHeadroomMs,a.accelSafeLatenessMs,d.goalSpeed,d.acc,a.driveSpeedRatio,a.driveAccelRatio,a.valid);}
 // fail-closed examples
 CHECK(!analyzeDynamicProfile(DynamicProfile{"BAD_STEP",.3,2,0,.020,.004,300,20}).valid);
 CHECK(!analyzeDynamicProfile(DynamicProfile{"BAD_DRIVE",.19,2,0,.020,.004,100,20}).valid);
}
int main(){
 std::setvbuf(stdout,nullptr,_IONBF,0);
 if(std::getenv("MATDOG_EXPLORE_ONLY")){StreamModel m;m.name="EXPLORE";m.loop_us=std::getenv("MATDOG_EXPLORE_LOOP_US")?std::atoi(std::getenv("MATDOG_EXPLORE_LOOP_US")):2000;m.reads_per_tick=3;m.ideal=std::getenv("MATDOG_EXPLORE_REAL")==nullptr;m.stop_short_ticks=5;if(const char* e=std::getenv("MATDOG_EXPLORE_STALL")){unsigned every=0,us=0;std::sscanf(e,"%u,%u",&every,&us);m.stall_every_ms=every;m.stall_us=us;}standReplay(m,false,false,true);return failures?1:0;}
 if(std::getenv("MATDOG_TX_CLOCK_ONLY")){backendClockArrival();return failures?1:0;}
 if(std::getenv("MATDOG_V6_ONLY")){numericContract();rawEdgeEndToEnd();contactExit();releaseRetiming();releaseWindow();generationReleaseDelay();feedbackCadence();txFailureNoAdvance();pathBoxCases();return failures?1:0;}
 if(std::getenv("MATDOG_HEIGHT_SURVEY_ONLY")){heightTimingSurvey();return failures?1:0;}
 if(std::getenv("MATDOG_HEIGHT_CHAIN_ONLY")){heightReplay();heightReplay(true);return failures?1:0;}
 backendClockArrival();dynamicProfiles();
 const bool fixed=std::getenv("MATDOG_RESIDUAL_BASELINE")==nullptr;
 residualGeometry(fixed);pathBoxCases();residualHold(fixed);if(fixed)residualSpeedUnits();
 if(std::getenv("MATDOG_RESIDUAL_ONLY"))return failures?1:0;
 temporalContracts();proneStatus();recoveryAndImu();releaseClockRates();safetyFaults();
 numericContract();rawEdgeEndToEnd();contactExit();releaseRetiming();releaseWindow();generationReleaseDelay();feedbackCadence();txFailureNoAdvance();
 heightTimingSurvey();heightReplay();heightReplay(true);
 StreamModel m;m.name="NOMINAL_4MS";m.loop_us=4000;m.reads_per_tick=3;m.stop_short_ticks=5;standReplay(m,true);
 m.name="JITTER_0_1P2MS";m.jitter_us=1200;standReplay(m);
 m.name="ASYNC_84MS";m.jitter_us=0;m.loop_us=21000;m.read_us=1000;standReplay(m,true,true);
 m=StreamModel{};m.name="OFFLINE_1RAD_5RAD_4MS";m.loop_us=4000;m.reads_per_tick=3;m.ideal=true;standReplay(m,false,false,true);
 std::printf("UNIFIED_MOTION failures=%d hardware=false\n",failures);return failures?1:0;
}
