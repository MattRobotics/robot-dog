// Reuse the operational mock port/rig, not a second servo or controller model.
#define main operationalRegressionMain
#include "test_simple_motion.cpp"
#undef main

static motion::MotionCommand observeCommand(Rig& r,const char* leg,unsigned seq=1) {
  motion::MotionCommand c;c.intent=motion::Intent::GROUND_OBSERVE;c.owner=motion::ClientOwner::DASHBOARD;
  std::snprintf(c.id,sizeof c.id,"ground-%u",seq);std::strcpy(c.leg,leg);
  c.sequence=seq;c.issued_at_ms=r.p.now/1000;c.ttl_ms=5000;return c;
}
static void hold(Rig& r) {
  assert(!r.m.stand(.12,r.p.now));assert(r.reachStand());for(unsigned n=0;n<30;++n)r.tick();
}
int main() {
  // Geometry sanity independent of the projection helper: parallel tread,
  // tilted finite width, and the solver's Jacobian vs finite differences.
  for(auto leg:{motion::LegId::LF,motion::LegId::RF,motion::LegId::LH,motion::LegId::RH}){
    motion::LegJointAngles q{.12,.7,-.2};motion::Vector3 up{0,0,1};
    const auto c=motion::contactForwardKinematics(leg,q,up);double h,J[3][3];
    assert(motion::lowestTreadHeightM(c,up,h));
    const auto& model=motion::footContactModel();
    const double expected=c.referenceM.z-model.supportWidthM*.5*std::sin(c.axisTiltRad)-model.filletRadiusM*(1-std::cos(c.axisTiltRad));
    assert(std::abs(h-expected)<1e-12);
    assert(motion::contactReferenceJacobian(leg,q,up,c,J));
    for(unsigned j=0;j<3;++j){auto a=q,b=q;double* x[]={&a.hip,&a.upper,&a.lower};double* y[]={&b.hip,&b.upper,&b.lower};
      *x[j]+=1e-6;*y[j]-=1e-6;
      const auto pa=motion::contactForwardKinematics(leg,a,up),pb=motion::contactForwardKinematics(leg,b,up);
      assert(std::abs((pa.referenceM.z-pb.referenceM.z)/2e-6-J[2][j])<1e-8);
    }
  }
  assert(simple::signMagnitude(1024+40,10)==-40&&simple::signMagnitude(32768+7,15)==-7);
  assert(simple::groundLeg("LH")==3&&simple::groundLeg("RH")==2&&simple::groundLeg("XX")==-1);
  for(const char* leg:{"LF","RF","LH","RH"}){
    Rig r,control;hold(r);hold(control);core::MotionService service;service.begin(&r.m,nullptr,nullptr);service.setReady(true,"READY");
    const auto c=observeCommand(r,leg);const auto a=service.command(c,r.p.now/1000);assert(a.outcome==motion::CommandOutcome::ACCEPTED);
    assert(service.command(c,r.p.now/1000).outcome==a.outcome); // replay never restarts
    core::MotionStatusSnapshot status;service.captureStatus(status,r.p.now/1000);assert(status.ground.samples==0);
    for(unsigned n=0;n<100&&status.ground.running;++n){
      r.tick();control.tick();test_clock_us=r.p.now;service.update();service.captureStatus(status,r.p.now/1000);
      const auto count=status.ground.samples;service.update();service.captureStatus(status,r.p.now/1000);
      assert(status.ground.samples==count); // repeated cache is not a new sample
    }
    assert(!status.ground.running&&status.ground.samples==simple::kGroundObservationSamples);
    assert(!std::strcmp(status.activeOutcome,"COMPLETED")&&!std::strcmp(status.ground.reason,"OBSERVATION_ONLY_NO_CONTACT"));
    assert(!std::strcmp(simple::groundLegName(status.ground.leg),leg));
    assert(status.ground.maxSpanUs>0); // sequential read population, NOT synchronous
    for(unsigned j=0;j<3;++j)assert(status.ground.stampUs[j]>status.ground.startedUs&&status.ground.tracking[j]==0);
    // Diagnostic command never adds servo reads/writes or changes the law.
    assert(r.p.reads==control.p.reads&&r.p.frames==control.p.frames&&r.p.enables==control.p.enables&&r.p.offCalls==control.p.offCalls);
    assert(r.p.sent==control.p.sent&&r.m.state()==simple::State::STANDING);
    char json[core::kMotionStatusJsonBytes];assert(service.statusJson(json,sizeof json-640,r.p.now/1000));
    assert(std::strstr(json,"\"ground_valid\":false")&&std::strstr(json,"\"release_mm\":null")&&std::strstr(json,"\"touchdown_mm\":null"));
    if(!std::strcmp(leg,"LF"))std::printf("GROUND_JSON=%s\n",json);
    assert(status.ground.referenceSentUs<=status.ground.startedUs);
    const auto saved=status.ground;
    assert(service.command(observeCommand(r,leg,2),r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);
    motion::MotionCommand stop=observeCommand(r,leg,3);stop.intent=motion::Intent::STOP;
    assert(service.command(stop,r.p.now/1000).outcome==motion::CommandOutcome::ACCEPTED);
    for(unsigned n=0;n<100;++n){r.tick();service.update();}
    service.captureStatus(status,r.p.now/1000);assert(!status.ground.running&&status.ground.samples==0&&!std::strcmp(status.ground.reason,"GROUND_STOPPED"));
    assert(service.command(observeCommand(r,leg,4),r.p.now/1000).outcome==motion::CommandOutcome::REJECTED);
    // Worst-case retained record after a fault/stale status still fits the
    // original transport's 640-byte reserve and emits valid finite numbers.
    status.ground=saved;status.referenceFaultSlot=2;status.referenceFaultRaw=4095;
    status.referenceFaultFeedback=status.feedback[2];status.referenceFaultFeedback.controlStampUs=UINT64_MAX;
    status.capturedUs=test_clock_us=UINT64_C(5000000000000);status.capturedMs=UINT32_MAX;
    assert(core::MotionService::formatStatus(status,json,sizeof json-640));
    assert(!std::strstr(json,"nan")&&!std::strstr(json,"inf"));
    std::printf("Ground status retained worst-case %zu / %zu bytes\n",std::strlen(json),sizeof json-640);
  }
  // Faults/refusals affect only the diagnostic, never turn a failed
  // observation into a locomotion gate or change torque.
  for(unsigned scenario=0;scenario<5;++scenario){
    Rig r;hold(r);simple::GroundContactDiagnostic g;g.start(0,r.p.now);
    for(unsigned n=0;n<8;++n)r.tick();
    const auto frames=r.p.frames,off=r.p.offCalls,reads=r.p.reads;
    const char* expected=nullptr;
    if(scenario==0){auto& f=const_cast<simple::Feedback&>(r.m.feedback()[0]);f.stampUs=r.p.now-300000;expected="GROUND_FEEDBACK_STALE";}
    if(scenario==1){auto& f=const_cast<simple::Feedback&>(r.m.feedback()[1]);f.stampUs=r.p.now+1000;expected="GROUND_FUTURE_FEEDBACK";}
    if(scenario==2){auto& f=const_cast<simple::Feedback&>(r.m.feedback()[2]);f.speed=1;expected="GROUND_SERVO_MOVING";}
    if(scenario==3){r.in.imuValid=false;r.m.inputs(r.in);expected="GROUND_IMU_UNAVAILABLE";}
    if(scenario==4){g.start(0,r.p.now-simple::kGroundObservationTimeoutUs);expected="GROUND_OBSERVATION_TIMEOUT";}
    g.update(r.m,r.p.now);assert(!g.running()&&!std::strcmp(g.snapshot().reason,expected));
    assert(r.p.frames==frames&&r.p.offCalls==off&&r.p.reads==reads&&r.m.state()==simple::State::STANDING);
  }
  {Rig r;hold(r);simple::GroundContactDiagnostic g;g.start(0,r.p.now);
   r.in.quaternion[0]=std::cos(.002);r.in.quaternion[2]=std::sin(.002);
   for(unsigned n=0;n<30&&g.running();++n){r.tick();g.update(r.m,r.p.now);}
   assert(!g.running()&&!std::strcmp(g.snapshot().reason,"GROUND_REFERENCE_CHANGED"));
   g.start(0,r.p.now);assert(!r.m.stand(.13,r.p.now));r.tick();g.update(r.m,r.p.now);
   assert(!g.running()&&!std::strcmp(g.snapshot().reason,"GROUND_REQUIRES_STAND_HOLD"));}
  const char* body="{\"version\":1,\"id\":\"g-1\",\"sequence\":1,\"issued_at_ms\":1000,\"ttl_ms\":5000,\"owner\":\"dashboard\",\"type\":\"GROUND_CONTACT.OBSERVE\",\"params\":{\"leg\":\"LH\"}}";
  motion::MotionCommand c;const char* why=nullptr;assert(core::parseMotionJson(body,std::strlen(body),&c,&why));
  assert(c.intent==motion::Intent::GROUND_OBSERVE&&!std::strcmp(c.leg,"LH")&&motion::validCommand(c,1000,&why));
  std::strcpy(c.leg,"XX");assert(!motion::validCommand(c,1000,&why));
  std::string unexpected=body;unexpected.replace(unexpected.find("\"leg\":\"LH\""),10,"\"gait\":\"LH\"");
  assert(!core::parseMotionJson(unexpected.c_str(),unexpected.size(),&c,&why));
  puts("PASS ground observation: geometry, asynchronous samples, no extra servo IO, refusal, STOP, API; NO HARDWARE QUALIFICATION");
}
