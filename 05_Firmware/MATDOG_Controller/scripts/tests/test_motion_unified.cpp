// Production-linked unified STAND/WALK, synthetic physics only. No transport.
#define MATDOG_OPERATIONAL_NO_MAIN
#include "test_operational_motion.cpp"
#include <chrono>
namespace {
struct StreamRig {
 Rig r;double lo[12],hi[12];
 StreamRig(){
  r.envelope.frame_writes=r.envelope.timed_segments=true;
  r.envelope.max_velocity_rad_s=.19;r.envelope.max_acceleration_rad_s2=2;
  r.envelope.arrival_tolerance_rad=motion::tolerance::kArrivalRad;r.envelope.tracking_timeout_ms=4000;
  for(unsigned i=0;i<12;++i){lo[i]=-.1;hi[i]=.1;r.feedback.sampled_at_us[i]=100000;}
  CHECK(r.arm());StreamConfig c;c.tube_rad=.020;c.local_path_box=true;c.max_step_rad=.004;
  c.settle_speed_rad_s=motion::operational_walk::kStreamSettleSpeedRadS;c.feedback_max_age_us=100000;c.hold_timeout_us=4000000;c.max_frame_gap_us=100000;c.persist_samples=3;
  c.region_min_rad=lo;c.region_max_rad=hi;CHECK(r.executor.beginStream(c,r.feedback,100000));
 }
 void fresh(uint64_t t,double q){for(unsigned i=0;i<12;++i){r.feedback.sampled_at_ms[i]=t/1000;r.feedback.sampled_at_us[i]=t;r.feedback.actual_rad[i]=q;}}
 bool send(unsigned seq,double q,uint64_t t,bool final=false){return r.executor.stream(r.frame(seq,q),r.validation(seq),t,t+20000,final,r.facts,r.feedback,t/1000,t);}
 void monitor(uint64_t t){r.executor.monitor(r.facts,r.feedback,t/1000,t);}
};
// Counterexamples use existing production executor, mapping and certified path.
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
 StreamConfig c;c.local_path_box=true;c.max_step_rad=.004;c.tube_rad=.020;c.settle_speed_rad_s=0;
 c.feedback_max_age_us=c.max_frame_gap_us=100000;c.hold_timeout_us=4000000;c.persist_samples=3;c.region_min_rad=lo;c.region_max_rad=hi;
 CHECK(r.executor.beginStream(c,r.feedback,100000));
 std::vector<std::pair<uint64_t,motion::JointTargetFrame>> sent{{100000,start}};
 double tracking=0,oldest_error=0,witness_error=0;bool ok=true,whole_box=false;unsigned frames=0;
 for(unsigned k=1;k<=8&&ok;++k){const uint64_t now=100000+k*20000;
  // The true simultaneous pose path(t+50 ms) is ON the certified polyline.
  // Samples are quantized, with 7 ms between slots (77 ms snapshot skew).
  uint64_t oldest=UINT64_MAX,newest=0;
  for(unsigned i=0;i<12;++i){const uint64_t at=std::max<uint64_t>(100000,now-(11-i)*7000);
   const auto actual=path((at-100000)*1e-6+.050);
   r.feedback.actual_rad[i]=qOf(r,i,tickOf(r,i,slot(actual,i)));
   r.feedback.sampled_at_us[i]=at;r.feedback.sampled_at_ms[i]=at/1000;
   oldest=std::min(oldest,at);newest=std::max(newest,at);
   const auto* ref=&sent.front().second;for(const auto& h:sent)if(h.first<=at)ref=&h.second;
   tracking=std::max(tracking,std::abs(r.feedback.actual_rad[i]-slot(*ref,i)));}
  const auto* old=&sent.front().second;for(const auto& h:sent)if(h.first<=oldest)old=&h.second;
  oldest_error=0;for(unsigned i=0;i<12;++i)oldest_error=std::max(oldest_error,std::abs(r.feedback.actual_rad[i]-slot(*old,i)));
  whole_box=false;witness_error=1e9;
  for(const auto& h:sent){if(h.first>newest)continue;double e=0;
   for(unsigned i=0;i<12;++i)e=std::max(e,std::abs(r.feedback.actual_rad[i]-slot(h.second,i)));
   witness_error=std::min(witness_error,e);whole_box=whole_box||e<=.020;}
  CHECK(tracking<=.020&&whole_box);auto f=path(k*.020);f.sequence=k;
  ok=r.executor.stream(f,r.validation(k),now,now+20000,false,r.facts,r.feedback,now/1000,now);
  if(ok){sent.push_back({r.executor.status().last_tx_complete_us,f});++frames;}
 }
 std::printf("RESIDUAL_GEOMETRY fixed=%d running=%d frames=%u tracking_mrad=%.3f oldest_box_mrad=%.3f common_witness_mrad=%.3f reason=%s certified_polyline=true hardware=false\n",fixed,ok,frames,tracking*1000,oldest_error*1000,witness_error*1000,r.executor.status().reason);
 if(fixed){
  CHECK(ok);const uint64_t now=280000;const auto frames_before=r.backend.frames;
  for(unsigned i=0;i<12;++i){const uint64_t at=now-(11-i)*7000;const auto actual=path((at-100000)*1e-6+.050);
   r.feedback.actual_rad[i]=qOf(r,i,tickOf(r,i,slot(actual,i)));r.feedback.sampled_at_us[i]=at;r.feedback.sampled_at_ms[i]=at/1000;}
  r.feedback.sampled_at_us[1]=203000;r.feedback.sampled_at_us[4]=now;
  for(unsigned i:{1u,4u}){const auto* ref=&sent.front().second;for(const auto& h:sent)if(h.first<=r.feedback.sampled_at_us[i])ref=&h.second;
   r.feedback.actual_rad[i]=slot(*ref,i)+(i==1?.019:-.019);}
  bool common=false;double per_joint=0;
  for(unsigned i=0;i<12;++i){const auto* ref=&sent.front().second;for(const auto& h:sent)if(h.first<=r.feedback.sampled_at_us[i])ref=&h.second;
   per_joint=std::max(per_joint,std::abs(r.feedback.actual_rad[i]-slot(*ref,i)));}
  for(const auto& h:sent){bool inside=true;for(unsigned i=0;i<12;++i)inside=inside&&std::abs(r.feedback.actual_rad[i]-slot(h.second,i))<=.020;common=common||inside;}
  CHECK(per_joint<=.020&&!common);r.executor.monitor(r.facts,r.feedback,now/1000,now);
  REASON(r.executor.status().reason,"OUTSIDE_PATH_BOX");CHECK(r.backend.frames==frames_before);
  std::printf("RESIDUAL_GEOMETRY_UNSAFE per_joint_mrad=%.3f common_box=%d reason=%s\n",per_joint*1000,common,r.executor.status().reason);
 }else{CHECK(!ok&&oldest_error>.020);REASON(r.executor.status().reason,"OUTSIDE_PATH_BOX");}
}
void residualHold(bool fixed){
 for(unsigned mode=0;mode<3;++mode){StreamRig s;CHECK(s.send(1,.003,120000,true));bool arrived=false;
  for(unsigned n=0;n<52&&!s.r.executor.status().safe_off_required;++n){const uint64_t now=140000+n*80000;
   for(unsigned i=0;i<12;++i){
    s.r.feedback.sampled_at_us[i]=122000+n*80000+i*1000;s.r.feedback.sampled_at_ms[i]=s.r.feedback.sampled_at_us[i]/1000;
    // Zero; adjacent-position-count oscillation with +/-1 speed count;
    // actual motion at ten counts/s while momentarily inside arrival band.
    s.r.feedback.actual_rad[i]=mode==1?(1+((n+i)%2))*motion::tolerance::kTickRad:mode==2?.003+.0012*(n%4<2?n%4:4-n%4):.003;
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
 // Three NEW divergent samples stop tracking even inside the independent
 // historical geometric box. Old snapshots cannot count as extra samples.
 {StreamRig s;CHECK(s.send(1,.003,120000));CHECK(s.send(2,.006,140000));CHECK(s.send(3,.009,160000));
  for(unsigned k=0;k<3;++k){s.r.feedback.sampled_at_us[0]=162000+k*1000;s.r.feedback.actual_rad[0]=-.014;
   s.monitor(162000+k*1000);if(k<2){CHECK(!s.r.executor.status().safe_off_required);s.monitor(162000+k*1000);CHECK(!s.r.executor.status().safe_off_required);}}
  REASON(s.r.executor.status().reason,"TRACKING_LAG_PERSISTENT");}
 std::puts("TEMPORAL_CONTRACTS PASS: no future-target gate, post-TX all-12 arrival, quiet speed, new-sample persistence");
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
 for(unsigned stop_slot=0;stop_slot<3;++stop_slot){StreamRig s;unsigned reads=0;
  for(unsigned k=0;execution::operationalPollReadAllowed(true,k,true);++k){
   if(k==stop_slot){s.r.executor.stop("MID_BATCH_STOP");break;}++reads;}
  CHECK(reads==stop_slot);CHECK(!s.send(1,.003,120000)&&s.r.backend.frames==0);
 }
 std::puts("SAFETY_FAULTS PASS: STOP, divergence, lost/recovering/future feedback, raw limit, collision, uncertain/refused TX, power, torque, clock, step/rate/acceleration, HOLD timeout, SAFE_OFF latch");
}
struct RunStats {Dist gaps,regular,age,sample_period,work;unsigned frames=0,checkpoints=0;double elapsed=0,tracking=0,step=0;};
RunStats standReplay(StreamModel m,bool walk=false,bool from_ready=false){
 Operational op(2.,-92.070442,1,walk);auto& b=op.b;
 if(from_ready){op.publish();op.walk_ok=true;op.refresh();const auto ack=walkCommand(op,1,motion::operational_walk::profileVelocity(motion::operational_walk::kStandardProfile,1));std::printf("AUTO_STAND admission=%s\n",ack.reason);CHECK(ack.outcome==motion::CommandOutcome::ACCEPTED);}
 else CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
 for(unsigned k=0;k<40&&b.coordinator.arming();++k)op.tickArming();
 CHECK(b.session.state()==MotionSessionState::ARMED&&op.armer.calls==1);
 m.speed_rad_s=motion::operational_walk::kStandProfile.drive.goalSpeed*motion::tolerance::kTickRad;
 m.acc_rad_s2=motion::operational_walk::kStandProfile.drive.acceleration*100*motion::tolerance::kTickRad;
 StreamDriver d(op,m);const uint64_t start=d.now_us;uint64_t last_tx=0;unsigned previous=0;
 uint64_t samples[12]{};double last_q[12];for(unsigned i=0;i<12;++i)last_q[i]=b.r.executor.targetRadians()[i];
 RunStats st;bool ok=true,last_final=false,no_micro_arrival=true,one_frame=true,all_post=true;
 while(ok&&std::strcmp(b.coordinator.state(),"STAND")&&!b.coordinator.walking()&&d.now_us-start<90000000){
  if(from_ready&&d.present())b.service.renewTeleop(b.now,250); // Controller renews also during automatic STAND
  const auto c0=std::chrono::steady_clock::now();ok=d.tick();const auto c1=std::chrono::steady_clock::now();
  st.work.add(std::chrono::duration<double,std::micro>(c1-c0).count());
  const auto& es=b.r.executor.status();st.tracking=std::max(st.tracking,es.stream_peak_error_rad);
  for(unsigned i=0;i<12;++i){st.age.add((d.now_us-d.sample_us[i])/1000.);
   if(samples[i]!=d.sample_us[i]){if(samples[i])st.sample_period.add((d.sample_us[i]-samples[i])/1000.);samples[i]=d.sample_us[i];}}
  if(d.frames_seen!=previous){one_frame=one_frame&&d.frames_seen==previous+1;
   if(last_tx){st.gaps.add((es.last_tx_complete_us-last_tx)/1000.);if(!last_final)st.regular.add((es.last_tx_complete_us-last_tx)/1000.);}
   last_tx=es.last_tx_complete_us;previous=d.frames_seen;last_final=b.coordinator.stand().pendingFinal();
   if(last_final)++st.checkpoints;
   else no_micro_arrival=no_micro_arrival&&es.completed_sequence<es.tx_sequence;
   for(unsigned i=0;i<12;++i){st.step=std::max(st.step,std::abs(b.r.executor.targetRadians()[i]-last_q[i]));last_q[i]=b.r.executor.targetRadians()[i];}}
  if(es.settled_sequence)for(unsigned i=0;i<12;++i)all_post=all_post&&d.sample_us[i]>es.last_tx_complete_us;
 }
 st.frames=d.frames_seen;st.elapsed=(d.now_us-start)/1e6;
 std::printf("STAND_REPLAY model=%s ok=%d elapsed_s=%.6f frames=%u checkpoints=%u gap_ms_p50=%.3f p99=%.3f max=%.3f regular_ms_p50=%.3f p99=%.3f max=%.3f feedback_age_ms_max=%.3f sample_period_ms_p50=%.3f max=%.3f tracking_peak_mrad=%.3f step_mrad=%.3f host_tick_work_us_p50=%.1f p99=%.1f max=%.1f reason=%s frame=%s hardware=false\n",
  m.name,ok,st.elapsed,st.frames,st.checkpoints,st.gaps.pct(.5),st.gaps.pct(.99),st.gaps.max(),st.regular.pct(.5),st.regular.pct(.99),st.regular.max(),st.age.max(),st.sample_period.pct(.5),st.sample_period.max(),st.tracking*1000,st.step*1000,st.work.pct(.5),st.work.pct(.99),st.work.max(),b.r.executor.status().reason,motion::feedbackReasonName(b.coordinator.frame().reason));
 CHECK(ok&&(!std::strcmp(b.coordinator.state(),"STAND")||b.coordinator.walking()));
 CHECK(st.checkpoints==7&&no_micro_arrival&&one_frame&&all_post&&st.step<=.004+1e-12);
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
}
int main(){
 std::setvbuf(stdout,nullptr,_IONBF,0);
 const bool fixed=std::getenv("MATDOG_RESIDUAL_BASELINE")==nullptr;
 residualGeometry(fixed);residualHold(fixed);if(fixed)residualSpeedUnits();
 if(std::getenv("MATDOG_RESIDUAL_ONLY"))return failures?1:0;
 temporalContracts();safetyFaults();
 StreamModel m;m.name="NOMINAL_4MS";m.loop_us=4000;m.reads_per_tick=3;m.stop_short_ticks=5;standReplay(m,true);
 m.name="JITTER_0_1P2MS";m.jitter_us=1200;standReplay(m);
 m.name="ASYNC_84MS";m.jitter_us=0;m.loop_us=21000;m.read_us=1000;standReplay(m,true,true);
 std::printf("UNIFIED_MOTION failures=%d hardware=false\n",failures);return failures?1:0;
}
