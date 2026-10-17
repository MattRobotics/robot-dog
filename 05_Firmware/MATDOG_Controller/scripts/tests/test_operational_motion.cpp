// Operator directive 2026-10-14, end to end, offline: READY with MOTION
// authority held idle by the Controller -> authenticated Dashboard STAND ->
// session arms itself (prime at measured pose -> energize -> arm, Safe
// Actuator) -> PRONE recognized (broad band) -> entry planned FROM THE MEASURED
// pose (no certificate tube) and checked against limits + CAD keep-out ->
// continuous certified fixture route -> STAND verified -> STOP = SAFE_OFF.
// Production MotionService/Coordinator/AdaptiveStand/Session/Executor/policy;
// kinematic servo double; measured 2026-10-13 prone. NO hardware.
#define MATDOG_FIXTURE_COMMISSIONING_NO_MAIN
#include "../../src/motion/FirstStrideReference.h"
#include "../../src/motion/OperationalWalkProfile.h"
#include "../../src/execution/OperationalFeedback.h"
#include <algorithm>
#include <array>
#include "../../src/core/MotionClock.h"
#include <cstdlib>
#include <deque>
#include <vector>
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
  bool walk_ok=false;   // what the Controller computes: verified STAND + bound stride + operator present
  bool legacy_stand=false;
  explicit Operational(double fold_offset_deg=0,double keepout_deg=-92.070442,double keepout_sign=1,bool walk=false,bool legacy=false):legacy_stand(legacy){
    // As Controller::admitRuntimePolicy(operational): the Issue #52 STAND profile
    // (rates, 20 ms frames, derived entry), junction-limited route time law and
    // coordinated frame writes. legacy=true keeps the baseline for A/B.
    if(!legacy){const char* why="";CHECK(execution::applyOperationalStandProfile(b.params,&why));
      CHECK(execution::buildCommissioningRuntime(b.params,b.limits,1,b.r.policy.epoch(),&b.runtime));
      b.runtime.execution.frame_writes=true;b.r.envelope=b.runtime.execution;}
    if(walk){  // operational admission binds the continuous fixture WALK profile
      b.runtime.walk=motion::operational_walk::specification();
      CHECK(motion::operational_walk::fitsLimits(b.runtime.feedback,b.runtime.execution.tracking_tolerance_rad));
      // As Commissioning.cpp: the experimental fast profile raises the cap only when its box fits.
      if(motion::operational_walk::fitsLimits(motion::operational_walk::kFastProfile,b.runtime.feedback,b.runtime.execution.tracking_tolerance_rad))
        b.runtime.walk.maxVxMps=std::max(b.runtime.walk.maxVxMps,motion::operational_walk::profileVelocity(motion::operational_walk::kFastProfile,0));
    }
    for(unsigned i=0;i<12;++i)if(i%3==1){b.r.feedback.actual_rad[i]+=fold_offset_deg*kDeg;}
    // READY: the Controller holds MOTION authority idle (OperationalReadiness GRANT_AUTHORITY).
    CHECK(b.r.arbiter.request(ActuatorAuthority::MOTION,OperatingMode::RUN,&held)==AuthorityResult::GRANTED);
    b.r.facts.lease=held;
    b.coordinator.begin(&b.r.executor,&b.r.policy);b.coordinator.bindSession(&b.session);
    armer.b=&b;b.coordinator.bindArmer(&armer);
    motion::AcquisitionRoute route;route.pairedProne=true;route.supervisedSegments=false;route.timeScale=b.runtime.stand_time_scale;
    route.geometryIdentityVerified=true;route.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
    if(!legacy){route.phaseTimed=true;route.phaseVelocityRadS=b.params.max_velocity_rad_s;route.phaseAccelerationRadS2=b.params.max_acceleration_rad_s2;}
    CHECK(b.coordinator.configure(b.runtime.feedback,b.runtime.walk,&route));
    CommissioningStandConfig cc;cc.enabled=true;cc.measuredStart=true;cc.minStageDurationS=b.params.entry_min_stage_s;
    cc.proneMaxDeviationRad=b.params.prone_max_deviation_rad;cc.proneMaxRmsRad=b.params.prone_max_rms_rad;
    cc.keepoutCount=1;auto& k=cc.keepouts[0];k.id="LF_UPPER_LOWER_CONTACT";k.a=k.b=2;k.ca=keepout_sign;k.cb=0;k.bound=keepout_sign*keepout_deg*kDeg;
    CHECK(b.coordinator.configureCommissioning(cc));   // no certificate needed
    b.service.begin(&b.r.executor,&b.coordinator);refresh();
  }
  MotionReadiness readiness(){auto r=ready();r.walk_qualified=walk_ok;r.first_stride_only=b.runtime.walk.firstStrideOnly;
    r.max_vx=b.runtime.walk.maxVxMps;r.max_vy=0;r.max_yaw_rate=0;r.local_operator_owns_session=false;return r;}
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
  std::printf("OPERATIONAL_STAND peak_joint_step_rad=%.6f per_%.3fs -> peak_joint_velocity_rad_s=%.4f\n",max_step,b.params.sample_period_s,max_step/b.params.sample_period_s);
  std::printf("OPERATIONAL_STAND fold_offset_deg=%.2f entry=%s entry_s=%.2f+%.2f frames=%u writes=%zu approvals=0 old_tube_needed=no\n",fold_offset_deg,
    motion::entryStatusName(b.coordinator.entryPlan().status),b.coordinator.entryPlan().durationS[0],b.coordinator.entryPlan().durationS[1],frames,writes);
}

// PRONE -> STAND -> first stride -> STAND on the fixture (directive 2026-10-15).
// One explicit Dashboard command per stride; the internal 250 ms stride TTL is
// renewed only while the operator heartbeat is current (Controller behaviour).
// ---- Continuous fixture WALK (operator directive 2026-10-15) ----
// Production MotionService/Coordinator/FeedbackWalk/Session/Executor/Safe
// Actuator; servo backend simulated. Two servo models: IDEAL (target reached at
// once) and REALISTIC (200 Hz Controller loop, one joint write per tick,
// GOAL_SPEED rate ceiling, first-order lag, delayed feedback).
struct ServoModel { bool ideal=true; double speed_rad_s=1000*2*3.14159265358979/4096; double tau_s=.03; uint32_t latency_ms=20; uint32_t loop_ms=5; };
struct WalkStats {
  double vmax=0,amax=0,stepmax=0,trackmax=0,margin=1e9;unsigned worst=12;
  double foot_x[4][2],foot_z[4][2];std::vector<double> cycle_t;unsigned frames=0;size_t writes=0;
  WalkStats(){for(unsigned l=0;l<4;++l){foot_x[l][0]=foot_z[l][0]=1e9;foot_x[l][1]=foot_z[l][1]=-1e9;}}
};
struct Driver {
  Operational& op;ServoModel servo;double q[12]{},written[12]{};bool stuck[12]{};size_t seen_writes=0;
  std::deque<std::pair<uint32_t,std::array<double,12>>> history;
  WalkStats st;uint32_t last_seq=0;double prev_q[12]{};bool have_prev=false;int last_cycle=-1;
  std::FILE* trace=nullptr;bool feedback_ok=true;unsigned frames=0;double max_step=0;
  explicit Driver(Operational& o):op(o){for(unsigned i=0;i<12;++i)q[i]=written[i]=op.b.r.feedback.actual_rad[i];}
  void record(){
    auto& b=op.b;const auto& f=b.coordinator.frame();
    if(!f.target.valid||f.target.sequence==last_seq||!b.coordinator.walking())return;
    last_seq=f.target.sequence;++st.frames;double t[12];
    for(unsigned i=0;i<12;++i){const auto& l=f.target.legs[i/3];t[i]=i%3==0?l.hip:i%3==1?l.upper:l.lower;
      const auto& v=f.velocityRadS[i/3];const auto& a=f.accelerationRadS2[i/3];
      st.vmax=std::max(st.vmax,std::abs(i%3==0?v.hip:i%3==1?v.upper:v.lower));st.amax=std::max(st.amax,std::abs(i%3==0?a.hip:i%3==1?a.upper:a.lower));
      if(have_prev){st.stepmax=std::max(st.stepmax,std::abs(t[i]-prev_q[i]));}
      prev_q[i]=t[i];
      const double m=std::min(t[i]-b.limits[i][0],b.limits[i][1]-t[i]);if(m<st.margin){st.margin=m;st.worst=i;}}
    have_prev=true;
    if(!std::strcmp(b.coordinator.state(),"WALK"))for(unsigned l=0;l<4;++l){motion::Vector3 r;motion::pointToBase(f.body,f.feetWorldM[l],r);
      st.foot_x[l][0]=std::min(st.foot_x[l][0],r.x);st.foot_x[l][1]=std::max(st.foot_x[l][1],r.x);
      st.foot_z[l][0]=std::min(st.foot_z[l][0],r.z);st.foot_z[l][1]=std::max(st.foot_z[l][1],r.z);}
    if(trace){for(unsigned i=0;i<12;++i)std::fprintf(trace,"%s%.9f",i?",":"",t[i]);std::fprintf(trace,"\n");}
    const int c=static_cast<int>(std::floor(b.coordinator.walkCycles()+1e-9));
    if(c!=last_cycle){if(last_cycle>=0)st.cycle_t.push_back(b.now/1000.0);last_cycle=c;}
  }
  void renew(bool heartbeat){auto& b=op.b;
    if(heartbeat){b.r.facts.permit_expires_at_ms=b.evidence.expires_at_ms=b.now+3000;
      if(b.coordinator.walking()){b.coordinator.renewWalk(b.now);b.service.renewTeleop(b.now,static_cast<uint32_t>(motion::operational_walk::kCommandTtlS*1000));}}}
  void assessPending(){auto& b=op.b;const auto& frame=b.coordinator.frame();
    if(frame.target.valid&&!frame.executable){motion::SegmentAssessment a;a.sequence=frame.target.sequence;a.stampS=b.now/1000.;a.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
      a.geometryIdentityVerified=a.sweptCollisionClear=a.supportCompatible=a.continuousBoundsConfirmed=a.fixtureTrialEnvelopeReviewed=true;
      if(b.coordinator.assess(a,b.now))b.coordinator.update(b.r.facts,b.r.feedback,b.now);}}
  bool failed(){auto& b=op.b;return b.r.executor.status().state==ExecutorState::SAFE_OFF_REQUIRED||b.r.executor.status().state==ExecutorState::FAULT||
    !std::strcmp(b.coordinator.state(),"REJECTED")||b.session.safeOffRequired();}
  // IDEAL servo: one Controller tick, frames executed synchronously.
  bool tick(uint32_t dt,bool heartbeat){
    auto& b=op.b;uint32_t& now=b.now;now+=dt;
    for(unsigned i=0;i<12;++i)b.r.feedback.sampled_at_ms[i]=now;
    renew(heartbeat);
    motion::AdaptiveObservation o;observe(o,q,now);b.coordinator.observe(o);b.coordinator.update(b.r.facts,b.r.feedback,now);op.refresh();
    auto frame=b.coordinator.frame();
    if(frame.target.valid&&frame.target.sequence>b.r.executor.status().completed_sequence){
      assessPending();record();
      for(unsigned i=0;i<12;++i)b.r.executor.update(b.r.facts,b.r.feedback,++now);
      if(failed())return false;
      for(unsigned i=0;i<12;++i){const double t=b.r.executor.targetRadians()[i];max_step=std::max(max_step,std::abs(t-q[i]));q[i]=t;b.r.feedback.actual_rad[i]=t;b.r.feedback.sampled_at_ms[i]=now+1;}
      ++now;b.r.executor.update(b.r.facts,b.r.feedback,now);++frames;
    }
    b.r.executor.update(b.r.facts,b.r.feedback,now);
    return !failed();
  }
  // REALISTIC servo: 5 ms Controller loop.
  bool rtick(bool heartbeat){
    auto& b=op.b;b.now+=servo.loop_ms;const double dt=servo.loop_ms/1000.0;
    for(;seen_writes<b.r.backend.writes.size();++seen_writes){const auto& w=b.r.backend.writes[seen_writes];
      for(unsigned i=0;i<12;++i){const auto* g=b.r.geometry.findJoint(slotIdentity(i));if(g&&g->bus_id==w.bus)written[i]=qOf(b.r,i,w.tick);}}
    for(unsigned i=0;i<12;++i){if(stuck[i])continue;double d=(written[i]-q[i])*(1-std::exp(-dt/servo.tau_s));const double cap=servo.speed_rad_s*dt;
      d=std::max(-cap,std::min(cap,d));q[i]+=d;}
    std::array<double,12> now_q;for(unsigned i=0;i<12;++i)now_q[i]=q[i];history.push_back({b.now,now_q});
    while(history.size()>1&&history[1].first+servo.latency_ms<=b.now)history.pop_front();
    const auto& seen=history.front();
    for(unsigned i=0;i<12;++i){b.r.feedback.actual_rad[i]=seen.second[i];b.r.feedback.sampled_at_ms[i]=seen.first;b.r.feedback.read_ok[i]=feedback_ok;b.r.feedback.torque_on[i]=true;}
    renew(heartbeat);
    motion::AdaptiveObservation o;double fq[12];for(unsigned i=0;i<12;++i)fq[i]=seen.second[i];observe(o,fq,b.now);b.coordinator.observe(o);
    b.coordinator.update(b.r.facts,b.r.feedback,b.now);op.refresh();assessPending();record();
    b.r.executor.update(b.r.facts,b.r.feedback,b.now);
    for(unsigned i=0;i<12;++i)st.trackmax=b.coordinator.walking()?std::max(st.trackmax,std::abs(q[i]-b.r.executor.targetRadians()[i])):st.trackmax;
    return !failed();
  }
};
bool reachStand(Operational& op,Driver& d){
  auto& b=op.b;if(op.stand().outcome!=motion::CommandOutcome::ACCEPTED)return false;
  for(unsigned k=0;k<40&&b.coordinator.arming();++k)op.tickArming();
  for(unsigned i=0;i<12;++i)d.q[i]=d.written[i]=b.r.feedback.actual_rad[i];
  while(std::strcmp(b.coordinator.state(),"STAND")&&b.now<50000000)if(!d.tick(20,true))return false;
  op.refresh();for(unsigned i=0;i<12;++i)d.q[i]=d.written[i]=b.r.feedback.actual_rad[i];d.seen_writes=b.r.backend.writes.size();
  return b.service.activeAck().outcome==motion::CommandOutcome::COMPLETED;
}
// ==== Issue #52 (2026-10-15): time-driven WALK, coordinated frames, asynchronous feedback ====
// Production MotionService/Coordinator/FeedbackWalk(streamUpdate)/Executor(STREAMING/HOLD)/
// SafeActuator frame transaction/ActuatorRuntime::executeFrame. Simulated (SIMULATO):
// UART delivery delay, per-servo first-order lag with GoalSpeed and ACC saturation,
// encoder and speed quantization, ROUND-ROBIN per-joint reads with their own
// completion timestamps (never all 12 at once), Controller loop jitter and stalls,
// 1 Hz operator heartbeat (presence only). Targets are NEVER used as measurements.
constexpr double kPi=3.14159265358979323846;
struct StreamModel {
  const char* name="B";bool ideal=false;
  uint32_t loop_us=2000,read_us=420,tx_us=1100,jitter_us=0;unsigned reads_per_tick=2;
  double tau_s=.03,speed_rad_s=1000*2*kPi/4096,acc_rad_s2=50*100*2*kPi/4096;
  uint32_t hb_period_ms=1000,stall_every_ms=0,stall_us=0;
};
struct Dist {
  std::vector<double> v;void add(double x){v.push_back(x);}
  double pct(double p){if(v.empty())return 0;std::vector<double> s=v;std::sort(s.begin(),s.end());return s[std::min(s.size()-1,static_cast<size_t>(p*(s.size()-1)+.5))];}
  double rms() const {double a=0;for(double x:v)a+=x*x;return v.empty()?0:std::sqrt(a/v.size());}
  double max() const {double m=0;for(double x:v)m=std::max(m,x);return m;}
};
struct StreamDriver {
  Operational& op;StreamModel m;core::MonotonicClock64 clock;uint64_t now_us=0,last_hb_us=0;uint32_t lcg=12345;
  double q[12]{},v[12]{},goal[12]{};std::deque<std::pair<uint64_t,std::array<double,12>>> inflight;size_t seen_writes=0;
  unsigned rr=0;uint64_t sample_us[12]{};double sample_q[12]{},sample_v[12]{};bool heartbeat=true,feedback_ok=true,stuck[12]{};
  Dist monitor[12],lead[12],tick_us;std::vector<double> cycle_t;int last_cycle=-1;uint64_t next_stall_us=0;
  unsigned frames_seen=0;uint32_t last_tx_seq=0;bool trace_refs=false;double ref_lo[12],ref_hi[12];
  StreamDriver(Operational& o,StreamModel mm):op(o),m(mm){
    auto& b=op.b;now_us=static_cast<uint64_t>(b.now)*1000;clock.extend(static_cast<uint32_t>(now_us));b.coordinator.bindClock(&clock);last_hb_us=now_us;
    for(unsigned i=0;i<12;++i){q[i]=goal[i]=sample_q[i]=b.r.feedback.actual_rad[i];sample_us[i]=now_us;ref_lo[i]=1e9;ref_hi[i]=-1e9;}
    seen_writes=b.r.backend.writes.size();next_stall_us=now_us+m.stall_every_ms*1000ull;
  }
  unsigned slotOfBus(uint8_t bus){for(unsigned i=0;i<12;++i){const auto* g=op.b.r.geometry.findJoint(slotIdentity(i));if(g&&g->bus_id==bus)return i;}return 12;}
  void physics(uint64_t to){
    while(now_us<to){const uint64_t step=std::min<uint64_t>(100,to-now_us);const double dt=step*1e-6;now_us+=step;
      while(!inflight.empty()&&inflight.front().first<=now_us){for(unsigned i=0;i<12;++i)goal[i]=inflight.front().second[i];inflight.pop_front();}
      for(unsigned i=0;i<12;++i){if(stuck[i]){v[i]=0;continue;}
        if(m.ideal){q[i]=goal[i];v[i]=0;continue;}
        double vd=(goal[i]-q[i])/m.tau_s;vd=std::max(-m.speed_rad_s,std::min(m.speed_rad_s,vd));
        const double dv=std::max(-m.acc_rad_s2*dt,std::min(m.acc_rad_s2*dt,vd-v[i]));v[i]+=dv;q[i]+=v[i]*dt;}}
  }
  void read(unsigned j){auto& r=op.b.r;sample_q[j]=qOf(r,j,tickOf(r,j,q[j]));sample_us[j]=now_us;
    const double steps=std::round(v[j]*4096/(2*kPi));sample_v[j]=steps*2*kPi/4096;}
  bool failed(){auto& b=op.b;return b.r.executor.status().state==ExecutorState::SAFE_OFF_REQUIRED||b.r.executor.status().state==ExecutorState::FAULT||
    !std::strcmp(b.coordinator.state(),"REJECTED")||b.session.safeOffRequired();}
  bool present() const {return now_us-last_hb_us<=3000000ull;}
  // One Controller loop iteration, in the Controller's order.
  bool tick(){
    auto& b=op.b;lcg=lcg*1103515245u+12345u;
    uint64_t dt=m.loop_us+(m.jitter_us?(lcg>>8)%m.jitter_us:0);
    if(m.stall_every_ms&&now_us>=next_stall_us){dt+=m.stall_us;next_stall_us+=m.stall_every_ms*1000ull;}
    tick_us.add(static_cast<double>(dt));
    const uint64_t start=now_us,end=start+dt;
    // Round-robin fast reads at the end of the tick, each with its own completion time.
    const uint64_t reads_from=end>m.reads_per_tick*m.read_us?end-m.reads_per_tick*m.read_us:start;
    physics(std::max(start,reads_from));
    for(unsigned k=0;k<m.reads_per_tick;++k){physics(std::max(now_us,reads_from+(k+1)*m.read_us));if(feedback_ok)read(rr);rr=(rr+1)%12;}
    physics(end);b.now=static_cast<uint32_t>(now_us/1000);clock.extend(static_cast<uint32_t>(now_us));
    for(unsigned i=0;i<12;++i){b.r.feedback.actual_rad[i]=sample_q[i];b.r.feedback.sampled_at_us[i]=sample_us[i];b.r.feedback.sampled_at_ms[i]=static_cast<uint32_t>(sample_us[i]/1000);
      b.r.feedback.speed_rad_s[i]=sample_v[i];b.r.feedback.read_ok[i]=true;b.r.feedback.torque_on[i]=true;}
    // 1 Hz authenticated heartbeat = presence (3 s window), as Controller::operatorPresent.
    if(heartbeat&&now_us-last_hb_us>=m.hb_period_ms*1000ull)last_hb_us=now_us;
    if(present()){b.r.facts.permit_expires_at_ms=b.evidence.expires_at_ms=static_cast<uint32_t>(last_hb_us/1000)+3000;
      if(b.coordinator.walking()){b.coordinator.renewWalk(b.now);b.service.renewTeleop(b.now,static_cast<uint32_t>(motion::operational_walk::kCommandTtlS*1000));}}
    else if(b.session.state()!=MotionSessionState::OFF&&!b.session.safeOffRequired())b.session.revoke("LOCAL_PRESENCE_DEADMAN_EXPIRED");
    // Observation: encoder vector stamped by its OLDEST joint sample; support = declaration while present.
    motion::AdaptiveObservation o;observe(o,sample_q,b.now);uint64_t oldest=UINT64_MAX;bool still=true;
    for(unsigned i=0;i<12;++i){oldest=std::min(oldest,sample_us[i]);still=still&&std::abs(sample_v[i])<1e-12;}
    o.stampS=oldest/1e6;o.stationary=still;o.externalSupportConfirmed=present();
    b.coordinator.observe(o);b.coordinator.update(b.r.facts,b.r.feedback,b.now,now_us);
    // As the Controller: reviewed offline-certified assessment of a planned segment frame.
    {const auto& frame=b.coordinator.frame();
     if(frame.target.valid&&!frame.executable&&!b.coordinator.walking()){motion::SegmentAssessment a;a.sequence=frame.target.sequence;a.stampS=now_us/1e6;a.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
      a.geometryIdentityVerified=a.sweptCollisionClear=a.supportCompatible=a.continuousBoundsConfirmed=a.fixtureTrialEnvelopeReviewed=true;
      if(b.coordinator.assess(a,b.now))b.coordinator.update(b.r.facts,b.r.feedback,b.now,now_us);}}
    op.refresh();b.r.executor.update(b.r.facts,b.r.feedback,b.now);
    // Writes transmitted this tick (a 12-joint frame or single legacy writes)
    // reach their servos after the UART delay.
    if(seen_writes<b.r.backend.writes.size()){std::array<double,12> g;
      for(unsigned i=0;i<12;++i)g[i]=inflight.empty()?goal[i]:inflight.back().second[i];
      const size_t n=b.r.backend.writes.size()-seen_writes;
      for(;seen_writes<b.r.backend.writes.size();++seen_writes){const auto& w=b.r.backend.writes[seen_writes];const unsigned s=slotOfBus(w.bus);if(s<12)g[s]=qOf(b.r,s,w.tick);}
      frames_seen+=static_cast<unsigned>(n/12);inflight.push_back({now_us+m.tx_us,g});}
    const auto& st=b.r.executor.status();
    if(b.coordinator.walking())for(unsigned i=0;i<12;++i){monitor[i].add(st.stream_error_rad[i]);lead[i].add(std::abs(q[i]-b.r.executor.targetRadians()[i]));
      if(trace_refs){ref_lo[i]=std::min(ref_lo[i],b.r.executor.targetRadians()[i]);ref_hi[i]=std::max(ref_hi[i],b.r.executor.targetRadians()[i]);}}
    const int c=static_cast<int>(std::floor(b.coordinator.walkCycles()+1e-9));
    if(b.coordinator.walking()&&c!=last_cycle){if(last_cycle>=0)cycle_t.push_back(now_us/1e6);last_cycle=c;}
    return !failed();
  }
};
motion::MotionAck walkCommand(Operational& op,uint32_t seq,double vx){
  auto c=command(motion::Intent::BASE_VELOCITY,seq,motion::ClientOwner::DASHBOARD);c.vx=vx;c.issued_at_ms=op.b.now;c.ttl_ms=350;
  return op.b.service.command(c,op.b.now);
}
bool writesInsideLimits(Bench& b){
  for(const auto& w:b.r.backend.writes){bool inside=false;
    for(unsigned i=0;i<12;++i){const auto* l=b.r.policy.limits().find(slotIdentity(i),b.r.geometry.provenanceTag());const auto* g=b.r.geometry.findJoint(slotIdentity(i));
      if(g&&g->bus_id==w.bus)inside=l&&l->contains(w.tick);}
    if(!inside)return false;}
  return true;
}
struct StreamResult { bool ok=false;double walk_s=0,stop_s=0,period_s=0;unsigned frames=0;std::vector<double> cycles;double cycles_done=0;const char* reason="";};
// PRONE -> STAND -> WALK START -> N cycles (or stop_at) -> WALK STOP -> measured STAND.
StreamResult streamWalk(unsigned preset,StreamModel model,double cycles,bool print=true,double* lo=nullptr,double* hi=nullptr,
                        const motion::operational_walk::WalkProfile& prof=motion::operational_walk::kStandardProfile,double stop_at_cycles=-1){
  Operational op(0,-92.070442,1,true);auto& b=op.b;Driver d0(op);StreamResult r;
  if(!reachStand(op,d0)){CHECK(false);return r;}
  StreamDriver d(op,model);d.trace_refs=lo!=nullptr;
  for(unsigned k=0;k<200;++k)if(!d.tick()){CHECK(false);return r;}
  const size_t stand_writes=b.r.backend.writes.size();CHECK(!b.coordinator.walking());
  op.walk_ok=true;op.refresh();
  const auto ack=walkCommand(op,10+preset,motion::operational_walk::profileVelocity(prof,preset));CHECK(ack.outcome==motion::CommandOutcome::ACCEPTED);REASON(ack.reason,"OK");
  const uint64_t t0=d.now_us;bool ok=true;if(stop_at_cycles>=0)cycles=stop_at_cycles;
  while(ok&&b.coordinator.walkCycles()<cycles&&d.now_us-t0<4000000000ull)ok=d.tick();
  r.walk_s=(d.now_us-t0)/1e6;r.cycles_done=b.coordinator.walkCycles();
  if(!ok){r.reason=b.r.executor.status().reason;std::printf("STREAM_WALK FAILED model=%s preset=%u at cycles=%.3f executor=%s frame=%s\n",model.name,preset,r.cycles_done,b.r.executor.status().reason,motion::feedbackReasonName(b.coordinator.frame().reason));return r;}
  const auto stop=walkCommand(op,40+preset,0);CHECK(stop.outcome==motion::CommandOutcome::ACCEPTED);
  const uint64_t s0=d.now_us;
  while(ok&&b.coordinator.walking()&&d.now_us-s0<120000000ull)ok=d.tick();
  for(unsigned k=0;k<200&&ok;++k)ok=d.tick();
  r.stop_s=(d.now_us-s0)/1e6;
  if(!ok||b.coordinator.walking()){r.reason=b.r.executor.status().reason;std::printf("STREAM_WALK STOP FAILED model=%s ok=%d state=%s frame=%s executor=%s cycles=%.6f\n",model.name,ok,b.coordinator.state(),motion::feedbackReasonName(b.coordinator.frame().reason),b.r.executor.status().reason,b.coordinator.walkCycles());return r;}
  CHECK(!std::strcmp(b.coordinator.state(),"STAND")&&b.r.executor.status().state==ExecutorState::READY&&!b.session.safeOffRequired());
  op.refresh();CHECK(b.service.activeAck().outcome==motion::CommandOutcome::COMPLETED);REASON(b.service.activeAck().reason,"ZERO_VELOCITY_VERIFIED");
  CHECK(writesInsideLimits(b));
  r.ok=true;r.frames=d.frames_seen;r.cycles=d.cycle_t;
  // Steady period: boundaries of full cycles after the ramp, before the stop.
  {std::vector<double> ct;for(double t:d.cycle_t)if(t<=t0/1e6+r.walk_s+1e-9)ct.push_back(t);
   if(ct.size()>=3)r.period_s=(ct.back()-ct[1])/(ct.size()-2);}
  if(lo)for(unsigned i=0;i<12;++i){lo[i]=std::min(lo[i],d.ref_lo[i]);hi[i]=std::max(hi[i],d.ref_hi[i]);}
  if(print){
    const auto& sc=b.coordinator.schedulerCounters();
    double mon_max=0;for(auto& x:d.monitor)mon_max=std::max(mon_max,x.max());
    std::printf("STREAM_WALK [SIMULATO] model=%s preset=%.0fmm cycles=%.2f walk_s=%.2f period_s=%.3f equiv_mm_s=%.2f stop_s=%.2f frames=%u frame_hz=%.1f writes=%zu scheduler releases=%u overruns=%u skipped=%u max_late_ms=%.2f loop_us p50=%.0f p99=%.0f max=%.0f monitor_peak_mrad=%.3f\n",
      model.name,prof.strides[preset]*1e3,r.cycles_done,r.walk_s,r.period_s,r.period_s>0?prof.strides[preset]*1e3/r.period_s:0,r.stop_s,r.frames,
      r.frames/(r.walk_s+r.stop_s),b.r.backend.writes.size()-stand_writes,sc.releases,sc.overruns,sc.skipped,sc.max_lateness_us/1e3,d.tick_us.pct(.5),d.tick_us.pct(.99),d.tick_us.max(),mon_max*1e3);
    for(unsigned i=0;i<12;++i)std::printf("STREAM_TRACK [SIMULATO] model=%s J%02u monitor_mrad rms=%.3f p95=%.3f p99=%.3f max=%.3f | lead_to_latest_target_mrad rms=%.2f p95=%.2f p99=%.2f max=%.2f\n",
      model.name,i,d.monitor[i].rms()*1e3,d.monitor[i].pct(.95)*1e3,d.monitor[i].pct(.99)*1e3,d.monitor[i].max()*1e3,d.lead[i].rms()*1e3,d.lead[i].pct(.95)*1e3,d.lead[i].pct(.99)*1e3,d.lead[i].max()*1e3);
  }
  return r;
}
// Union of every target the time-driven reference can emit, sampled at 1 kHz
// (any release instant), presets x {stop in the ramp, stop after 3 cycles}.
// The firmware pins this union (+1 urad outward); between 1 ms samples the
// excursion is < 0.5*a*(0.5 ms)^2 = 0.6 urad at the 4.5 rad/s^2 envelope.
void denseWalkRanges(){
  // Each profile is surveyed separately: its own presets, ramp-in, a stop in
  // the ramp and a stop after 3 cycles, every 1 ms (survey build) or every
  // 20 ms release (normal build, checked against the pinned box).
  struct Case{const motion::operational_walk::WalkProfile* p;const char* name;};
  for(const Case c:{Case{&motion::operational_walk::kStandardProfile,"STANDARD"},Case{&motion::operational_walk::kFastProfile,"FAST"}}){
    double lo[12],hi[12];for(unsigned i=0;i<12;++i){lo[i]=1e9;hi[i]=-1e9;}bool all=true;
    for(unsigned k=0;k<c.p->strideCount;++k)for(double stop_at:{0.3,3.0}){
      Operational op(0,-92.070442,1,true);auto& b=op.b;
      Driver d0(op);CHECK(reachStand(op,d0));StreamModel m;m.ideal=true;m.loop_us=1000;m.reads_per_tick=12;m.read_us=50;StreamDriver d(op,m);d.trace_refs=true;
      op.walk_ok=true;op.refresh();const auto ack=walkCommand(op,10,motion::operational_walk::profileVelocity(*c.p,k));
      if(ack.outcome!=motion::CommandOutcome::ACCEPTED){std::printf("DENSE %s refused: %s\n",c.name,ack.reason);all=false;continue;}
      bool ok=true;while(ok&&b.coordinator.walkCycles()<stop_at)ok=d.tick();
      CHECK(walkCommand(op,30,0).outcome==motion::CommandOutcome::ACCEPTED);
      while(ok&&b.coordinator.walking())ok=d.tick();
      if(!(ok&&!std::strcmp(b.coordinator.state(),"STAND"))){std::printf("DENSE %s stride %u stop@%.1f failed: %s / %s\n",c.name,k,stop_at,b.r.executor.status().reason,motion::feedbackReasonName(b.coordinator.frame().reason));all=false;CHECK(false);}
      for(unsigned i=0;i<12;++i){lo[i]=std::min(lo[i],d.ref_lo[i]);hi[i]=std::max(hi[i],d.ref_hi[i]);}
    }
    std::printf("OPERATIONAL_WALK_DENSE_RANGES [MISURATO HOST] profile=%s complete=%d\n  min={",c.name,all);for(unsigned i=0;i<12;++i)std::printf("%s%.6f",i?",":"",std::floor(lo[i]*1e6-1)/1e6);
    std::printf("}\n  max={");for(unsigned i=0;i<12;++i)std::printf("%s%.6f",i?",":"",std::ceil(hi[i]*1e6+1)/1e6);std::printf("}\n");
#ifndef MATDOG_HOST_REGION_SURVEY
    for(unsigned i=0;i<12;++i)CHECK(lo[i]>=c.p->regionMinRad[i]&&hi[i]<=c.p->regionMaxRad[i]);
#endif
  }
}
// PRONE -> STAND timing, legacy (rest-to-rest per segment, single writes,
// commissioning drive pair 20/5 = 0.031 rad/s servo ceiling) vs Issue #52
// (junction-limited law, 20 ms frames, frame writes, STAND pair 300/20).
// Same measured-start entry, same per-frame tracking, realistic servo.
double standTiming(bool legacy,StreamModel m,double fold,bool print=true){
  Operational op(fold,-92.070442,1,false,legacy);auto& b=op.b;
  if(op.stand().outcome!=motion::CommandOutcome::ACCEPTED){CHECK(false);return -1;}
  for(unsigned k=0;k<40&&b.coordinator.arming();++k)op.tickArming();
  if(!(b.session.state()==MotionSessionState::ARMED)){CHECK(false);return -1;}
  // Servo GoalSpeed/ACC ceilings of the pair that drives the STAND frames.
  const double step=2*kPi/4096;
  if(!m.ideal){if(legacy){m.speed_rad_s=b.params.drive_speed*step;m.acc_rad_s2=b.params.drive_acceleration*100*step;}
    else{m.speed_rad_s=motion::operational_walk::kStandProfile.drive.goalSpeed*step;m.acc_rad_s2=motion::operational_walk::kStandProfile.drive.acceleration*100*step;}}
  StreamDriver d(op,m);const uint64_t t0=d.now_us;bool ok=true;double peak=0;
  while(ok&&std::strcmp(b.coordinator.state(),"STAND")&&d.now_us-t0<900000000ull){ok=d.tick();
    for(unsigned i=0;i<12;++i)peak=std::max(peak,std::abs(d.q[i]-b.r.executor.targetRadians()[i]));}
  const double s=(d.now_us-t0)/1e6;
  const bool stood=ok&&!std::strcmp(b.coordinator.state(),"STAND");CHECK(stood);
  if(stood){op.refresh();CHECK(b.service.activeAck().outcome==motion::CommandOutcome::COMPLETED);REASON(b.service.activeAck().reason,"STAND_VERIFIED");}
  const auto c4=motion::pairedProneSample(250,1,true).target;double final_err=0;for(unsigned i=0;i<12;++i)final_err=std::max(final_err,std::abs(d.q[i]-slot(c4,i)));
  CHECK(writesInsideLimits(b));
  if(print)std::printf("STAND_TIMING [SIMULATO] %s model=%s fold_deg=%.1f ok=%d prone_to_stand_s=%.2f entry_s=%.2f+%.2f frames=%u writes=%zu peak_lag_mrad=%.2f final_error_mrad=%.2f (%s)\n",
    legacy?"LEGACY":"ISSUE52",m.name,fold,stood,s,b.coordinator.entryPlan().durationS[0],b.coordinator.entryPlan().durationS[1],d.frames_seen,b.r.backend.writes.size(),peak*1e3,final_err*1e3,
    stood?"STAND_VERIFIED":b.r.executor.status().reason);
  return stood?s:-1;
}
// The phase-timed STAND law changes TIME only: every frame target is a point of
// the certified paired polyline (same knots, fixture variant), rates inside the
// envelope, rest at the 5 phase transitions, ends exactly at the route end.
void phaseTimedRouteContract(){
  const auto& S=motion::operational_walk::kStandProfile;motion::PhaseTimedRoute r;
  CHECK(motion::buildPhaseTimedRoute(true,.999*S.velocityRadS,.999*S.accelerationRadS2/2,S.samplePeriodS,&r));
  double vmax=0,amax=0,fda=0,offpath=0,prev[12]{},pv[12]{};bool have=false;unsigned frames=0;
  for(unsigned ph=0;ph<5;++ph){const unsigned n=static_cast<unsigned>(std::ceil(r.durationS[ph]/S.samplePeriodS-1e-9));
    for(unsigned k=1;k<=n;++k){unsigned seg=0;const double t=r.durationS[ph]*k/n;auto f=motion::phaseTimedSample(r,ph,t,true,&seg);CHECK(f.target.valid);++frames;
      const auto a=motion::pairedProneSampleAt(seg,0,0,0,true).target,b=motion::pairedProneSampleAt(seg,1,0,0,true).target;
      // distance of the target from the straight segment a->b (per joint, common fraction)
      double frac=-1;for(unsigned i=0;i<12;++i){const double d=slot(b,i)-slot(a,i);if(std::abs(d)>1e-9){frac=(slot(f.target,i)-slot(a,i))/d;break;}}
      for(unsigned i=0;i<12;++i){const double q=slot(f.target,i),on=frac<0?slot(a,i):slot(a,i)+frac*(slot(b,i)-slot(a,i));offpath=std::max(offpath,std::abs(q-on));
        const auto& vl=f.velocityRadS[i/3];const auto& al=f.accelerationRadS2[i/3];vmax=std::max(vmax,std::abs(i%3==0?vl.hip:i%3==1?vl.upper:vl.lower));amax=std::max(amax,std::abs(i%3==0?al.hip:i%3==1?al.upper:al.lower));
        const double h=r.durationS[ph]/n;if(have){const double v=(q-prev[i])/h;fda=std::max(fda,std::abs(v-pv[i])/h);pv[i]=v;}prev[i]=q;}
      have=true;}}
  const auto end=motion::pairedProneSample(250,1,true).target;double enderr=0;for(unsigned i=0;i<12;++i)enderr=std::max(enderr,std::abs(prev[i]-slot(end,i)));
  CHECK(offpath<1e-9&&vmax<=S.velocityRadS+1e-9&&amax<=S.accelerationRadS2/2+1e-9&&enderr<1e-12);
  CHECK(fda<=S.accelerationRadS2+1e-6);   // finite-difference incl. corner jumps, inside the executor envelope
  double legacy=0;for(unsigned i=0;i<motion::kPairedProneSegments;++i)legacy+=motion::pairedProneDuration(i);
  std::printf("STAND_TIME_LAW [MISURATO HOST] same polyline (max off-path %.1e rad), route %.2f s (phases %.2f/%.2f/%.2f/%.2f/%.2f) vs legacy %.2f s x route_time_scale, frames=%u, |qd|max=%.3f rad/s, |qdd|max=%.3f rad/s2, finite-diff max=%.3f rad/s2\n",
    offpath,r.totalS,r.durationS[0],r.durationS[1],r.durationS[2],r.durationS[3],r.durationS[4],legacy,frames,vmax,amax,fda);
}
// Bug A reproduction: the legacy feedback-paced walk with the heartbeat-stamped
// observation (baseline Controller.cpp:1251) advances one frame per heartbeat.
void heartbeatCoupling(){
  Operational op(0,-92.070442,1,true);auto& b=op.b;Driver d0(op);CHECK(reachStand(op,d0));
  StreamModel m;m.hb_period_ms=1000;StreamDriver d(op,m);op.walk_ok=true;op.refresh();
  CHECK(walkCommand(op,20,motion::operational_walk::strideVelocity(1)).outcome==motion::CommandOutcome::ACCEPTED);
  const unsigned f0=d.frames_seen;const uint64_t t0=d.now_us;bool ok=true;
  while(ok&&d.now_us-t0<10000000ull)ok=d.tick();
  const double rate=(d.frames_seen-f0)/10.0;
  CHECK(ok&&rate>45);
  std::printf("HEARTBEAT_1HZ [SIMULATO] time-driven frames/s=%.1f cycles_in_10s=%.3f expected=%.3f (heartbeat 1 Hz = presence only)\n",rate,b.coordinator.walkCycles(),(10-.5*motion::operational_walk::kRampS)/motion::operational_walk::kPeriodS);
}
// The legacy (baseline) checkTracking contract, isolated: observation stamp =
// heartbeat (1 Hz) -> one 50 ms frame unblocked per heartbeat.
void legacyHeartbeatContract(){
  motion::FeedbackPolicy p;p.samplePeriodS=.05;p.maxTrackingErrorRad=.006;p.mode=motion::FeedbackMode::FIXTURE_REVIEW;
  motion::FeedbackFrame f;f.target.valid=true;f.target.sequence=7;f.timeS=10.000;
  motion::AdaptiveObservation o;o.acknowledged=true;o.acknowledgedSequence=7;o.actual=f.target;o.externalSupportConfirmed=true;
  unsigned unblocked=0;for(int k=0;k<1000;++k){const double now=10.000+k*.001;o.stampS=std::floor(now);   // heartbeat-stamped (baseline)
    if(motion::checkTracking(o,f,p)==motion::FeedbackReason::OK){++unblocked;break;}}
  std::printf("LEGACY_HEARTBEAT_STAMP [MISURATO HOST] baseline contract: 50 ms frame unblocked within 1 s: %s (stamp only advances at the next heartbeat)\n",unblocked?"yes":"NO");
  CHECK(unblocked==0);   // documents the baseline coupling the new path removes
}
void streamFaults(){
  const unsigned preset=1;StreamModel m;m.name="B";
  auto start=[&](Operational& op,StreamDriver*& d,Driver& d0)->bool{const bool stood=reachStand(op,d0);d=new StreamDriver(op,m);if(!stood)return false;op.walk_ok=true;op.refresh();
    return walkCommand(op,50,motion::operational_walk::strideVelocity(preset)).outcome==motion::CommandOutcome::ACCEPTED;};
  auto runUntil=[&](StreamDriver& d,double c){bool ok=true;while(ok&&d.op.b.coordinator.walkCycles()<c)ok=d.tick();return ok;};
  auto noWritesAfter=[&](StreamDriver& d){const size_t w=d.op.b.r.backend.writes.size();for(unsigned k=0;k<500;++k)d.tick();return d.op.b.r.backend.writes.size()==w;};
  // Emergency STOP in LF swing / stance: SAFE_OFF, zero frames after the latch.
  for(double at:{1.05,1.40,1.85}){Operational op(0,-92.070442,1,true);Driver d0(op);StreamDriver* d=nullptr;CHECK(start(op,d,d0));CHECK(runUntil(*d,at));
    const bool swing=!op.b.coordinator.frame().phases[0].stance;const uint32_t tx=op.b.r.executor.status().tx_sequence;
    auto stop=command(motion::Intent::STOP,60,motion::ClientOwner::DASHBOARD);stop.issued_at_ms=op.b.now;op.b.service.command(stop,op.b.now);
    CHECK(op.b.session.safeOffRequired()&&op.b.r.executor.status().safe_off_required&&op.b.r.executor.status().tx_sequence==tx&&noWritesAfter(*d));
    std::printf("STREAM_FAULT emergency_stop LF_%s cycles=%.2f -> SAFE_OFF, frames after latch=0\n",swing?"SWING":"STANCE",at);delete d;}
  // Heartbeat lost (presence 3 s): deadman -> SAFE_OFF, no frame after it.
  {Operational op(0,-92.070442,1,true);Driver d0(op);StreamDriver* d=nullptr;CHECK(start(op,d,d0));CHECK(runUntil(*d,1.3));d->heartbeat=false;const uint64_t t=d->now_us;bool ok=true;
   while(ok&&d->now_us-t<6000000ull)ok=d->tick();
   CHECK(!ok&&d->now_us-t<=3100000ull&&noWritesAfter(*d));std::printf("STREAM_FAULT heartbeat_lost -> SAFE_OFF after %.0f ms (presence window 3000 ms, last heartbeat up to 1000 ms earlier)\n",(d->now_us-t)/1e3);delete d;}
  // Feedback stops (read failures keep old stamps): stale -> SAFE_OFF within the age bound.
  {Operational op(0,-92.070442,1,true);Driver d0(op);StreamDriver* d=nullptr;CHECK(start(op,d,d0));CHECK(runUntil(*d,1.2));d->feedback_ok=false;const uint64_t t=d->now_us;bool ok=true;
   while(ok&&d->now_us-t<3000000ull)ok=d->tick();
   CHECK(!ok&&d->now_us-t<=150000ull);std::printf("STREAM_FAULT feedback_lost -> SAFE_OFF after %.0f ms (%s)\n",(d->now_us-t)/1e3,op.b.r.executor.status().reason);delete d;}
  // Stuck joint: persistent delay-aligned tracking error -> SAFE_OFF (never a skipped target).
  {Operational op(0,-92.070442,1,true);Driver d0(op);StreamDriver* d=nullptr;CHECK(start(op,d,d0));CHECK(runUntil(*d,1.1));d->stuck[2]=true;const uint64_t t=d->now_us;bool ok=true;
   while(ok&&d->now_us-t<30000000ull)ok=d->tick();
   CHECK(!ok);std::printf("STREAM_FAULT stuck_LF_LOWER -> SAFE_OFF after %.0f ms (%s)\n",(d->now_us-t)/1e3,op.b.r.executor.status().reason);delete d;}
  // Slow drive (commissioning 20 steps/s while the reference runs): physical failure to follow -> stop, never time dilation.
  {Operational op(0,-92.070442,1,true);Driver d0(op);StreamDriver* d=nullptr;CHECK(start(op,d,d0));d->m.speed_rad_s=20*2*kPi/4096;const uint64_t t=d->now_us;bool ok=true;
   while(ok&&d->now_us-t<30000000ull)ok=d->tick();
   CHECK(!ok);std::printf("STREAM_FAULT slow_drive_20steps_s -> SAFE_OFF after %.0f ms (%s): the reference is never stretched\n",(d->now_us-t)/1e3,op.b.r.executor.status().reason);delete d;}
  // Controller stall longer than the gap bound: the walk stops instead of planning through it.
  {Operational op(0,-92.070442,1,true);Driver d0(op);StreamDriver* d=nullptr;CHECK(start(op,d,d0));CHECK(runUntil(*d,1.2));d->m.stall_every_ms=1;d->m.stall_us=150000;bool ok=d->tick();
   CHECK(!ok);std::printf("STREAM_FAULT controller_stall_150ms -> %s (%s), frames after=0:%d\n",ok?"RUNNING":"SAFE_OFF",op.b.r.executor.status().reason,noWritesAfter(*d)?1:0);delete d;}
  // Backend refuses a frame (zero bytes) / TX not completed: both end the stream.
  for(int mode=0;mode<2;++mode){Operational op(0,-92.070442,1,true);Driver d0(op);StreamDriver* d=nullptr;CHECK(start(op,d,d0));CHECK(runUntil(*d,1.1));
   op.b.r.backend.frame_mode=mode?Backend::FrameMode::PARTIAL:Backend::FrameMode::REFUSE;bool ok=true;for(unsigned k=0;k<50&&ok;++k)ok=d->tick();
   CHECK(!ok);REASON(op.b.r.executor.status().reason,mode?"FRAME_TX_UNCERTAIN":"FRAME_REFUSED_ZERO_BYTES");delete d;}
}
// Executor frame contract with the bare rig: zero bytes for any refused frame.
void frameTransactionContract(){
  auto rigArm=[](Rig& r){for(unsigned i=0;i<12;++i){r.feedback.sampled_at_us[i]=100000;}return r.arm();};
  StreamConfig c;double lo[12],hi[12];for(unsigned i=0;i<12;++i){lo[i]=-.5;hi[i]=.5;}
  c.tube_rad=.006;c.max_lag_s=.15;c.settle_speed_rad_s=.002;c.feedback_max_age_us=100000;c.hold_timeout_us=3000000;c.max_frame_gap_us=60000;c.persist_samples=3;c.region_min_rad=lo;c.region_max_rad=hi;
  auto frame=[](uint32_t seq,double q){motion::JointTargetFrame f;f.valid=true;f.sequence=seq;for(auto& l:f.legs)l.hip=l.upper=l.lower=q;return f;};
  // 12th joint unresolvable -> zero bytes.
  {Rig r;r.envelope.tracking_tolerance_rad=.006;CHECK(rigArm(r));CHECK(r.executor.beginStream(c,r.feedback,100000));auto f=frame(1,.001);f.legs[3].lower=5;
   CHECK(!r.executor.stream(f,r.validation(1),120000,140000,false,r.facts,r.feedback,120,120000));CHECK(r.backend.writes.empty()&&r.backend.frames==0);}
  // Lease revoked before commit -> zero bytes.
  {Rig r;r.envelope.tracking_tolerance_rad=.006;CHECK(rigArm(r));CHECK(r.executor.beginStream(c,r.feedback,100000));r.arbiter.forceClear(AuthorityClearReason::OWNER_TEARDOWN);
   CHECK(!r.executor.stream(frame(1,.001),r.validation(1),120000,140000,false,r.facts,r.feedback,120,120000));CHECK(r.backend.writes.empty());}
  // Policy level: replay, copy, wrong shape (duplicate joint), epoch reset.
  {Rig r;actuator::ActuatorFrameCommand fc;
   for(unsigned i=0;i<12;++i){auto& j=fc.joints[i];j.operation=actuator::ActuatorOperation::POSITION_COMMAND;j.joint=slotIdentity(i);
     const auto* t=r.policy.transforms().find(j.joint,r.geometry.provenanceTag());j.target_tick=static_cast<uint16_t>(t->q0_tick);j.target_urad=0;}
   actuator::ActuatorFrameTransaction tx;CHECK(r.policy.planFrame(fc,r.facts.lease,r.facts.mode,&tx)==actuator::WriteDecision::ACCEPT);
   auto copy=tx;actuator::ActuatorTransaction single;
   CHECK(r.policy.plan(fc.joints[0],r.facts.lease,r.facts.mode,&single)==actuator::WriteDecision::REJECT_TRANSACTION_STATE);   // one outstanding
   uint8_t ids[12];for(unsigned k=0;k<12;++k)ids[k]=servo::legServoAt((k/3)*3+(2-k%3))->bus_id;
   CHECK(r.runtime.executeFrame(&tx,ids,nullptr,nullptr)==actuator::FrameExecuteResult::TX_COMPLETE_UNCONFIRMED&&r.backend.frames==1);
   CHECK(r.runtime.executeFrame(&tx,ids,nullptr,nullptr)==actuator::FrameExecuteResult::NOT_EXECUTED);    // replay
   CHECK(r.runtime.executeFrame(&copy,ids,nullptr,nullptr)==actuator::FrameExecuteResult::NOT_EXECUTED);  // copy
   auto dup=fc;dup.joints[11].joint=dup.joints[10].joint;CHECK(r.policy.planFrame(dup,r.facts.lease,r.facts.mode,&tx)==actuator::WriteDecision::REJECT_FRAME_SHAPE&&tx.failed_joint==11);
   auto torque=fc;torque.joints[4].operation=actuator::ActuatorOperation::TORQUE_ENABLE;CHECK(r.policy.planFrame(torque,r.facts.lease,r.facts.mode,&tx)==actuator::WriteDecision::REJECT_FRAME_SHAPE);
   auto oob=fc;oob.joints[7].target_tick=static_cast<uint16_t>(oob.joints[7].target_tick+500);CHECK(r.policy.planFrame(oob,r.facts.lease,r.facts.mode,&tx)!=actuator::WriteDecision::ACCEPT&&tx.failed_joint==7);
   CHECK(r.policy.planFrame(fc,r.facts.lease,r.facts.mode,&tx)==actuator::WriteDecision::ACCEPT);r.policy.reset();
   CHECK(r.runtime.executeFrame(&tx,ids,nullptr,nullptr)==actuator::FrameExecuteResult::NOT_EXECUTED);       // stale epoch
   const unsigned frames=r.backend.frames;
   // Auxiliary / duplicate bus id -> refused before commit, zero bytes.
   Rig r2;CHECK(r2.policy.planFrame(fc,r2.facts.lease,r2.facts.mode,&tx)==actuator::WriteDecision::ACCEPT);auto bad=ids;uint8_t ids2[12];std::memcpy(ids2,bad,12);ids2[5]=51;
   CHECK(r2.runtime.executeFrame(&tx,ids2,nullptr,nullptr)==actuator::FrameExecuteResult::INVALID_BUS_IDS&&r2.backend.frames==0&&!r2.policy.hasOutstandingTransaction());
   CHECK(r.backend.frames==frames);}
  std::printf("FRAME_TRANSACTION [MISURATO HOST] invalid 12th joint / revoked lease / replay / copy / duplicate / torque-in-frame / out-of-limits / stale epoch / aux id: zero bytes\n");
}
// Monotonic clock across the 32-bit micros() wrap and the absolute-deadline scheduler.
void clockAndScheduler(){
  core::MonotonicClock64 c;uint32_t t=0xFFFFFF00u;uint64_t prev=c.extend(t);bool mono=true;
  for(int k=0;k<1000;++k){t+=997;const uint64_t n=c.extend(t);mono=mono&&n==prev+997;prev=n;}
  CHECK(mono&&prev>0xFFFFFFFFull);
  core::PeriodicScheduler s;CHECK(s.start(1000,20000));
  auto r=s.poll(1000);CHECK(r.release&&r.sequence==1&&r.deadline_us==1000);
  CHECK(!s.poll(20999).release);r=s.poll(21000);CHECK(r.release&&r.deadline_us==21000&&!r.skipped);
  r=s.poll(41000+65000);CHECK(r.release&&r.skipped==3&&s.nextReleaseUs()==41000+4*20000ull);   // late: one release, 3 dropped, no catch-up
  CHECK(!s.poll(41000+65001).release&&s.counters().overruns==1&&s.counters().skipped==3);
  // 64-bit deadlines stay exact far beyond 2^32 us.
  core::PeriodicScheduler w;CHECK(w.start(0xFFFFFFF0ull,20000));unsigned n=0;for(uint64_t now=0xFFFFFFF0ull;now<0xFFFFFFF0ull+10000000ull;now+=1000)n+=w.poll(now).release;
  CHECK(n==500&&w.counters().overruns==0);
  std::printf("CLOCK_SCHEDULER [MISURATO HOST] 32-bit wrap monotonic, next_release+=period, late poll -> 1 release + %u skipped, 500 releases/10 s across 2^32 us\n",3u);
}
void walkGates(){
  const double vx=motion::operational_walk::strideVelocity(1);
  { Operational op(0,-92.070442,1,true);CHECK(walkCommand(op,1,vx).outcome==motion::CommandOutcome::REJECTED);       // walk_qualified false
    op.walk_ok=true;op.refresh();CHECK(walkCommand(op,2,vx).outcome==motion::CommandOutcome::REJECTED);REASON(op.b.service.lastAck().reason,"EXECUTOR_NOT_ARMED");
    CHECK(op.b.r.backend.writes.empty()&&!op.b.coordinator.walking()); }
  Operational op(0,-92.070442,1,true);auto& b=op.b;Driver d(op);CHECK(reachStand(op,d));op.walk_ok=true;op.refresh();
  CHECK(walkCommand(op,10,vx*.9).outcome==motion::CommandOutcome::REJECTED);REASON(b.service.lastAck().reason,"WALK_SPEED_NOT_CERTIFIED");
  { auto c=command(motion::Intent::BASE_VELOCITY,11,motion::ClientOwner::DASHBOARD);c.vx=vx;c.vy=.001;c.issued_at_ms=b.now;c.ttl_ms=350;CHECK(b.service.command(c,b.now).outcome==motion::CommandOutcome::REJECTED); }
  CHECK(walkCommand(op,12,vx).outcome==motion::CommandOutcome::ACCEPTED);
  CHECK(walkCommand(op,13,motion::operational_walk::strideVelocity(2)).outcome==motion::CommandOutcome::REJECTED);REASON(b.service.lastAck().reason,"WALK_IN_PROGRESS");
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

// Reproduce loop-entry vs read-completion ordering at every RR phase.
// Future samples stay invalid; changing the evaluation clock fixes admission.
void postValidationAdmissionClock(){
 for(unsigned newest=0;newest<12;++newest){
  Operational op;auto& b=op.b;b.now=10000;
  for(unsigned j=0;j<12;++j)b.r.feedback.sampled_at_ms[j]=9940+((j+12-newest)%12)*5;
  b.r.feedback.sampled_at_ms[newest]=10005;
  execution::OperationalFeedback producer;
  for(unsigned j=0;j<12;++j){execution::RawJointObservation raw;raw.read_ok=true;raw.position=tickOf(b.r,j,b.r.feedback.actual_rad[j]);
    raw.torque=0;raw.temperature=30;raw.voltage=114;raw.status=0;raw.speed=0;
    raw.sampled_at_ms=b.r.feedback.sampled_at_ms[j];raw.sampled_at_us=raw.sampled_at_ms*1000ull;
    producer.ingest(j,raw,b.r.geometry,geometry_data::kProvenance,b.r.policy.transforms());}
  CHECK(!producer.snapshot(b.now,b.r.envelope.feedback_max_age_ms).read_ok[newest]);
  CHECK(producer.snapshot(10006,b.r.envelope.feedback_max_age_ms).read_ok[newest]);
  CHECK(!producer.rawFresh(10407,b.r.envelope.feedback_max_age_ms));
  auto rejected=op.stand();CHECK(rejected.outcome==motion::CommandOutcome::REJECTED);REASON(rejected.reason,"STALE_FEEDBACK");
  CHECK(b.r.backend.writes.empty()&&b.session.state()==MotionSessionState::OFF);
  b.now=10006;auto accepted=op.stand(2);CHECK(accepted.outcome==motion::CommandOutcome::ACCEPTED);
  CHECK(b.session.state()==MotionSessionState::PRIMING&&b.r.backend.writes.empty());
 }
 for(unsigned stale=0;stale<12;++stale){
  Operational op;auto& b=op.b;b.now=10000;for(auto& t:b.r.feedback.sampled_at_ms)t=b.now;
  b.r.feedback.sampled_at_ms[stale]=b.now-b.r.envelope.feedback_max_age_ms-1;
  auto rejected=op.stand();CHECK(rejected.outcome==motion::CommandOutcome::REJECTED);REASON(rejected.reason,"STALE_FEEDBACK");CHECK(b.r.backend.writes.empty());
 }
}


// Regression 2026-10-17 (hardware: STAND ACCEPTED, no motion, planner
// PRIME_BACKEND_OR_POLICY, clamp mask 260, executor INPUT_LOST_OR_EXPIRED).
// An admitted STAND must reach the backend: 12 PRIME goal writes inside the
// limits (LF/LH LOWER clamped inward = 0x104), 12 verified torque enables, ARM,
// coordinated targets, executor progress. A lost reply is retried; a refused
// write aborts the command with ITS cause, never relabelled as input loss.
namespace stand_backend {
void foldOnlyLfLh(Operational& op){
  auto& b=op.b;
  // RF and RH LOWER rest inside their limits on this pose: only 2 and 8 clamp.
  for(unsigned i:{5u,11u}){const auto* l=b.r.policy.limits().find(slotIdentity(i),b.r.geometry.provenanceTag());
    const uint16_t rest=tickOf(b.r,i,b.r.feedback.actual_rad[i]);const uint16_t inside=rest<l->min_tick?l->min_tick+3:rest>l->max_tick?l->max_tick-3:rest;
    b.r.feedback.actual_rad[i]=qOf(b.r,i,inside);}
}
MotionReadiness gated(Operational& op){auto r=op.readiness();
  // As Controller: SAFE_OFF_REQUIRED drops the operational permit.
  if(op.b.session.state()==MotionSessionState::SAFE_OFF_REQUIRED){r.operational_permit=false;}
  return r;}
void tick(Operational& op){auto& b=op.b;b.fresh();b.session.update(b.evidence,b.r.facts,b.r.feedback,b.now);
  if(b.session.state()==MotionSessionState::ARMING){for(auto& on:b.r.feedback.torque_on)on=true;
    for(unsigned i=0;i<12;++i)b.r.feedback.actual_rad[i]=b.session.primedRadians()[i];}
  motion::AdaptiveObservation o;double q[12];for(unsigned i=0;i<12;++i)q[i]=b.r.feedback.actual_rad[i];
  observe(o,q,b.now);b.coordinator.observe(o);b.coordinator.update(b.r.facts,b.r.feedback,b.now);b.service.refresh(gated(op),b.now);}
void reachesBackend(int lost_replies){
  Operational op;auto& b=op.b;foldOnlyLfLh(op);
  b.r.backend.fail_next=lost_replies;b.r.backend.fail_outcome=BackendWriteOutcome::UNCERTAIN;
  const auto ack=op.stand();CHECK(ack.outcome==motion::CommandOutcome::ACCEPTED);REASON(ack.reason,"OK");
  CHECK(b.session.primeClampMask()==260);                                   // 0x104, as on hardware
  for(unsigned k=0;k<60&&b.coordinator.arming();++k)tick(op);
  // The backend saw the whole torque sequence, not just an HTTP ACCEPTED.
  CHECK(b.r.backend.attempts==24+lost_replies);
  CHECK(b.r.backend.torque_calls==12);
  CHECK(b.session.state()==MotionSessionState::ARMED&&b.r.executor.active());
  REASON(b.coordinator.lastEntryReason(),"");CHECK(b.coordinator.armingFailure()==nullptr);
  size_t prime=0;for(unsigned i=0;i<12;++i){const auto& w=b.r.backend.writes[prime++];
    const auto* l=b.r.policy.limits().find(slotIdentity(i),b.r.geometry.provenanceTag());CHECK(l&&l->contains(w.tick));
    CHECK(w.tick==tickOf(b.r,i,b.session.primedRadians()[i]));}
  // Planner targets reach the executor and it advances.
  motion::AdaptiveObservation o;double q[12];for(unsigned i=0;i<12;++i)q[i]=b.r.feedback.actual_rad[i];
  unsigned frames=0;const unsigned frames_before=b.r.backend.frames;
  for(unsigned k=0;k<4000&&frames<25;++k){
    b.now+=20;b.r.facts.permit_expires_at_ms=b.evidence.expires_at_ms=b.now+3000;
    observe(o,q,b.now);b.coordinator.observe(o);b.coordinator.update(b.r.facts,b.r.feedback,b.now);b.service.refresh(gated(op),b.now);
    auto frame=b.coordinator.frame();
    if(frame.target.valid&&frame.target.sequence>b.r.executor.status().completed_sequence){
      motion::SegmentAssessment a;a.sequence=frame.target.sequence;a.stampS=b.now/1000.;a.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
      a.geometryIdentityVerified=a.sweptCollisionClear=a.supportCompatible=a.continuousBoundsConfirmed=a.fixtureTrialEnvelopeReviewed=true;
      b.coordinator.assess(a,b.now);b.coordinator.update(b.r.facts,b.r.feedback,b.now);
      for(unsigned i=0;i<12;++i)b.r.executor.update(b.r.facts,b.r.feedback,++b.now);
      for(unsigned i=0;i<12;++i){q[i]=b.r.executor.targetRadians()[i];b.r.feedback.actual_rad[i]=q[i];b.r.feedback.sampled_at_ms[i]=b.now+1;}
      ++b.now;b.r.executor.update(b.r.facts,b.r.feedback,b.now);++frames;}
    if(b.r.executor.status().safe_off_required)break;
  }
  CHECK(frames>=25&&b.r.executor.status().completed_sequence>=25&&b.r.backend.frames>frames_before);
  CHECK(!b.r.executor.status().safe_off_required);
  CHECK(b.service.activeAck().outcome==motion::CommandOutcome::ACCEPTED);   // still rising, not aborted
  std::printf("STAND_BACKEND lost_replies=%d prime+energize_attempts=%d torque=%d frames=%u completed=%lu\n",lost_replies,b.r.backend.attempts,
              b.r.backend.torque_calls,frames,(unsigned long)b.r.executor.status().completed_sequence);
}
void refusedWriteKeepsItsCause(BackendWriteOutcome outcome,const char* expected,int fail_after){
  Operational op;auto& b=op.b;foldOnlyLfLh(op);
  CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
  b.r.backend.fail_outcome=outcome;
  for(unsigned k=0;k<60&&b.coordinator.arming();++k){if(b.r.backend.attempts==fail_after)b.r.backend.fail_next=MotionSession::kWriteAttempts;tick(op);}
  CHECK(!b.coordinator.arming()&&b.session.state()==MotionSessionState::SAFE_OFF_REQUIRED);
  REASON(b.session.reason(),expected);CHECK(b.session.failedJoint()==(fail_after%12));
  REASON(b.coordinator.armingFailure(),expected);REASON(b.coordinator.lastEntryReason(),expected);
  tick(op);
  CHECK(b.service.activeAck().outcome==motion::CommandOutcome::ABORTED);REASON(b.service.activeAck().reason,expected);
  CHECK(std::strcmp(b.r.executor.status().reason,"INPUT_LOST_OR_EXPIRED")!=0);REASON(b.r.executor.status().reason,expected);
  CHECK(b.r.backend.attempts==fail_after+(int)MotionSession::kWriteAttempts);  // bounded retries, then nothing
}
void run(){
  reachesBackend(0);reachesBackend(1);reachesBackend(2);
  refusedWriteKeepsItsCause(BackendWriteOutcome::VERIFIED_NOT_APPLIED,"PRIME_BACKEND_NOT_APPLIED",0);
  refusedWriteKeepsItsCause(BackendWriteOutcome::UNCERTAIN,"PRIME_BACKEND_NO_RESPONSE",2);
  refusedWriteKeepsItsCause(BackendWriteOutcome::VERIFIED_NOT_APPLIED,"ENERGIZE_BACKEND_NOT_APPLIED",12+8);
}
}
int main(){
  stand_backend::run();
  if(std::getenv("MATDOG_STAND_BACKEND_ONLY")){std::printf("OPERATIONAL_MOTION %s checks=%d failures=%d hardware=false (STAND backend)\n",failures?"FAIL":"PASS",checks,failures);return failures?1:0;}
  postValidationAdmissionClock();
  clockAndScheduler();frameTransactionContract();legacyHeartbeatContract();
  const bool stand_only=std::getenv("MATDOG_STAND_ONLY")!=nullptr;
  if(!stand_only)denseWalkRanges();
  if(std::getenv("MATDOG_RANGES_ONLY")){std::printf("OPERATIONAL_MOTION %s checks=%d failures=%d hardware=false\n",failures?"FAIL":"PASS",checks,failures);return failures?1:0;}
  phaseTimedRouteContract();
  operationalStandEndToEnd(0);operationalStandEndToEnd(.8);operationalStandEndToEnd(-2.0);
  // STAND A/B (SIMULATO): legacy baseline vs Issue #52 law, several PRONE poses, servo models.
  { StreamModel a;a.name="A_IDEAL";a.ideal=true;StreamModel b;b.name="B_TAU30_LOOP2_RR2";StreamModel c;c.name="C_TAU60_LOOP5_RR1_JIT2";c.tau_s=.06;c.loop_us=5000;c.reads_per_tick=1;c.jitter_us=2000;
    const double legacy=standTiming(true,b,0);
    for(double fold:{0.,.8,-2.0}){standTiming(false,a,fold);const double t=standTiming(false,b,fold);if(fold==0&&legacy>0&&t>0)std::printf("STAND_AB [SIMULATO] model=B legacy=%.1f s issue52=%.1f s speedup=%.1fx\n",legacy,t,legacy/t);}
    standTiming(false,c,0); }
  if(stand_only){operationalRefusals();heartbeatCoupling();std::printf("OPERATIONAL_MOTION %s checks=%d failures=%d hardware=false (STAND only)\n",failures?"FAIL":"PASS",checks,failures);return failures?1:0;}
  operationalRefusals();
  walkGates();heartbeatCoupling();streamFaults();
  const double step=2*kPi/4096;
  { StreamModel ideal;ideal.name="A_IDEAL";ideal.ideal=true;
    for(unsigned p=0;p<3;++p){auto r=streamWalk(p,ideal,10);CHECK(r.ok&&r.cycles_done>=10);} }
  { StreamModel b;b.name="B_TAU30_LOOP2_RR2";auto r=streamWalk(1,b,10);CHECK(r.ok);
    StreamModel c;c.name="C_TAU60_LOOP5_RR1_JIT2";c.tau_s=.06;c.loop_us=5000;c.reads_per_tick=1;c.jitter_us=2000;auto rc=streamWalk(1,c,10);CHECK(rc.ok);
    StreamModel s;s.name="D_TAU30_STALL40_EVERY1S";s.stall_every_ms=1000;s.stall_us=40000;auto rs=streamWalk(1,s,10);CHECK(rs.ok); }
  // WALK FAST (EXPERIMENTAL profile, drive 1200/100): A/B servo models, 10 cycles.
  { const auto& F=motion::operational_walk::kFastProfile;
    StreamModel a;a.name="FAST_A_IDEAL";a.ideal=true;auto ra=streamWalk(0,a,10,true,nullptr,nullptr,F);CHECK(ra.ok&&ra.cycles_done>=10);
    StreamModel b;b.name="FAST_B_TAU30_LOOP2_RR2";b.speed_rad_s=F.drive.goalSpeed*step;b.acc_rad_s2=F.drive.acceleration*100*step;auto rb=streamWalk(0,b,10,true,nullptr,nullptr,F);
    StreamModel c=b;c.name="FAST_C_TAU60_LOOP5_RR1_JIT2";c.tau_s=.06;c.loop_us=5000;c.reads_per_tick=1;c.jitter_us=2000;auto rc=streamWalk(0,c,10,true,nullptr,nullptr,F);
    StreamModel o=b;o.name="FAST_ON_STANDARD_DRIVE_1000_50";o.speed_rad_s=1000*step;o.acc_rad_s2=50*100*step;auto ro=streamWalk(0,o,10,true,nullptr,nullptr,F);
    std::printf("FAST_RESULT [SIMULATO] ideal=%d tau30=%d tau60=%d on_1000_50_drive=%d (a FAIL here is a physical following limit, not hidden)\n",ra.ok,rb.ok,rc.ok,ro.ok);
    CHECK(rb.ok);
    // Coordinated stop at swing / stance / ramp (acceleration) / decel boundaries.
    for(double at:{0.5,1.0,1.37,2.0,2.62,3.0}){auto r=streamWalk(0,b,10,false,nullptr,nullptr,F,at);CHECK(r.ok);
      std::printf("FAST_STOP_AT cycles=%.2f -> %s stop_s=%.2f\n",at,r.ok?"STAND":"FAIL",r.stop_s);} }
  std::printf("OPERATIONAL_MOTION %s checks=%d failures=%d hardware=false\n",failures?"FAIL":"PASS",checks,failures);return failures?1:0;
}
