// LOADDIAG offline checks: the first-exit record is frozen before SAFE_OFF,
// survives the later polling and a STOP, is re-armed by the next accepted
// STAND, and RAM 48 from control reads is carried across fast reads.
// Reuses the unchanged scripted harness of test_simple_motion.cpp.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main simple_motion_main
#include "test_simple_motion.cpp"
#undef main
#pragma GCC diagnostic pop
// Fast reads (FastFeedback) carry no RAM 48; control reads carry 1000.
struct LimitPort: ScriptedObservationPort {
  bool read(unsigned i,bool control,simple::Feedback& out) override {
    if(!ScriptedObservationPort::read(i,control,out))return false;
    out.torqueLimit=control?1000:-1;return true;
  }
};
struct DiagRig {
  simple::SimpleMotion m;LimitPort p;simple::Inputs in;
  DiagRig(){m.begin(&p);p.motion=&m;in.ready=in.powerOk=in.authorityOk=in.imuValid=true;in.refusal="READY";
    for(unsigned i=0;i<12;++i){in.minRad[i]=-3;in.maxRad[i]=3;}
    for(unsigned i=0;i<15;++i)tick(false);assert(m.offVerified());}
  void tick(bool publish=true){p.now+=simple::kPeriodUs;in.imuStampUs=p.now;m.inputs(in);if(publish)p.publishCommandedPose();test_clock_us=p.now;m.update(p.now);m.updateTelemetry(p.now);}
};
int main(){
  {DiagRig r;assert(!r.m.exitSnapshot().valid);
    assert(!r.m.stand(.05,r.p.now));
    // Run into the energized route, then raise a native alarm bit on slot 5.
    unsigned n=0;while(!(r.m.energized()&&r.m.phase()!=std::string("ENABLE"))&&n++<2000)r.tick();
    assert(r.m.energized()&&r.m.state()==simple::State::MOVING);
    for(unsigned i=0;i<40;++i)r.tick();
    const std::string phase=r.m.phase();
    uint16_t cmd[12];for(unsigned i=0;i<12;++i)cmd[i]=r.m.commandedRaw()[i];
    r.p.observed[5].status=0x20;r.p.observed[5].current=420;
    n=0;while(r.m.state()!=simple::State::FAULT&&n++<20)r.tick();
    const auto& e=r.m.exitSnapshot();
    assert(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT"));
    assert(e.valid&&e.fault&&e.energized&&e.state==simple::State::MOVING&&phase==e.phase);
    assert(!strcmp(e.reason,"SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT")&&e.lastPolled==5);
    assert(e.fb[5].status==0x20&&e.fb[5].current==420&&e.atUs>=e.fb[5].stampUs);
    for(unsigned i=0;i<12;++i)assert(e.commandedRaw[i]==r.m.commandedRaw()[i]);
    (void)cmd;
    // The alarm clears; SAFE_OFF completes and idle polling overwrites fb_.
    r.p.observed[5].status=0;r.p.observed[5].current=0;
    for(unsigned i=0;i<40;++i)r.tick(false);
    assert(r.m.offVerified()&&r.m.feedback()[5].status==0);
    assert(e.valid&&e.fb[5].status==0x20&&!strcmp(e.reason,"SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT"));
    // An operator STOP after the fault does not replace the first record.
    r.m.stop();for(unsigned i=0;i<20;++i)r.tick(false);
    assert(e.valid&&e.fault&&!strcmp(e.reason,"SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT"));
    std::puts("PASS first fault frozen before SAFE_OFF, kept after polling and STOP");
  }
  {DiagRig r;
    assert(!r.m.stand(.05,r.p.now));unsigned n=0;
    while(r.m.state()!=simple::State::STANDING&&n++<3000)r.tick();
    assert(r.m.state()==simple::State::STANDING&&!r.m.exitSnapshot().valid);
    for(unsigned i=0;i<200;++i)r.tick();
    for(unsigned i=0;i<12;++i)assert(r.m.feedback()[i].torqueLimit==1000);
    std::puts("PASS RAM 48 from control reads carried across fast reads");
    r.m.stop();for(unsigned i=0;i<20;++i)r.tick(false);
    const auto& e=r.m.exitSnapshot();
    assert(e.valid&&!e.fault&&!strcmp(e.reason,"STOP_REQUESTED")&&e.state==simple::State::STANDING);
    for(unsigned i=0;i<12;++i)assert(e.fb[i].torqueLimit==1000);
    // The next accepted STAND re-arms the record.
    n=0;while(!r.m.offVerified()&&n++<100)r.tick(false);
    for(unsigned i=0;i<20;++i)r.tick(false);
    assert(!r.m.stand(.05,r.p.now)&&!r.m.exitSnapshot().valid);
    std::puts("PASS STOP recorded; next accepted STAND re-arms");
  }
  return 0;
}
