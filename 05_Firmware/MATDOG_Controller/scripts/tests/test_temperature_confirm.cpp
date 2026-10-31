// Temperature confirmation (rc.11). Scripted observations, NO HARDWARE.
// Real cases from the 29/10 exit snapshots: temperatures of all 12 servos as
// recorded, one hot sample on the faulting servo, the next sample back to
// normal (as the STATUS after each fault showed). Positions/loads come from
// the scripted loaded trajectory. Build with -DEXPECT_RC10 against rc.10
// sources: the two real cases must then FAULT (reproduction).
#define STAND_LOAD_NO_MAIN
#include "test_stand_load_completion.cpp"
static void temps(LoadRig& r,const int t[12]){for(unsigned i=0;i<12;++i)r.p.observed[i].temp=t[i];}
static bool until(LoadRig& r,const char* phase,unsigned n=8000){while(n--&&r.m.state()!=simple::State::FAULT&&std::string(r.m.phase())!=phase)r.tick();return std::string(r.m.phase())==phase;}
// One hot sample on slot s, held until the temperature-gated FAST read has
// seen it (a control read also refreshes the feedback but is not gated, in
// rc.10 as in rc.11). Returns that read's time.
static uint64_t spike(LoadRig& r,unsigned s,int hot,int after){
  r.p.observed[s].temp=hot;unsigned n=0;
#ifdef EXPECT_RC10
  while(r.m.state()!=simple::State::FAULT&&n++<20)r.tick();
#else
  while(!(r.m.hotPendingMask()>>s&1)&&r.m.state()!=simple::State::FAULT&&n++<20)r.tick();
#endif
  const uint64_t at=r.m.feedback()[s].stampUs;r.p.observed[s].temp=after;return at;}
// Exactly one fast read of slot s at `hot`: fast reads of a slot are 6
// releases apart, so 6 releases at `hot` contain exactly one of them (a
// window may already be open, so the window mask cannot tell when it is read).
[[maybe_unused]] static void sampleOnce(LoadRig& r,unsigned s,int hot,int after){
  r.p.observed[s].temp=hot;for(unsigned i=0;i<6&&r.m.state()!=simple::State::FAULT;++i)r.tick();r.p.observed[s].temp=after;}
[[maybe_unused]] static uint64_t faultAt(LoadRig& r,unsigned n=200){while(n--&&r.m.state()!=simple::State::FAULT)r.tick();return r.p.now;}
int main(){
  // Snapshot temperatures (slot order LF hip/up/low, RF, RH, LH).
  const int snapA[12]={26,25,27,25,26,27,25,25,27,27,25,27};  // STAND150 DEPLOY, ID 21 (slot 5) 27->81->27
  const int snapB[12]={27,25,28,26,26,28,25,25,28,27,25,27};  // STAND100 HOLD, ID 31 (slot 8) 28->83->28
  {LoadRig r;temps(r,snapA);lowers(r,13,9,23,15);
    CHECK(!r.m.stand(.15,r.p.now),"STAND150 accepted");CHECK(until(r,"STAND_DEPLOY"),"DEPLOY reached");
    for(unsigned i=0;i<60;++i)r.tick();
    spike(r,5,81,27);for(unsigned i=0;i<100;++i)r.tick();
#ifdef EXPECT_RC10
    CHECK(r.m.state()==simple::State::FAULT&&!strcmp(r.m.reason(),"SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT")&&r.m.exitSnapshot().fb[5].temp==81,"rc.10 faults on the ID 21 81 C sample");
    std::printf("REPRODUCED rc.10: ID 21 81 C single sample in STAND_DEPLOY -> %s\n",r.m.reason());
#else
    CHECK(r.m.state()!=simple::State::FAULT&&r.m.tempSpikes()==1&&!r.m.hotPendingMask(),"ID 21 81 C single sample rejected");
    r.shaping=false;CHECK(r.run(8000,simple::State::STANDING),"STAND150 completes after the rejected sample");
    std::printf("PASS real case ID 21: 27->81->27 C in STAND_DEPLOY rejected, STAND150 reached (spikes=%u)\n",unsigned(r.m.tempSpikes()));
#endif
  }
  {LoadRig r;temps(r,snapB);lowers(r,24,3,21,19);
    CHECK(!r.m.stand(.10,r.p.now),"STAND100 accepted");r.shaping=true;
    CHECK(r.run(8000,simple::State::STANDING),"STAND100 HOLD reached");
    for(unsigned i=0;i<50;++i)r.tick();
    spike(r,8,83,28);for(unsigned i=0;i<150;++i)r.tick();
#ifdef EXPECT_RC10
    CHECK(r.m.state()==simple::State::FAULT&&r.m.exitSnapshot().fb[8].temp==83,"rc.10 faults on the ID 31 83 C sample");
    std::printf("REPRODUCED rc.10: ID 31 83 C single sample in HOLD -> %s\n",r.m.reason());
    std::printf("TEMPERATURE_CONFIRM rc.10 reproduction checks=%d\n",checks);return 0;
#else
    CHECK(r.m.state()==simple::State::STANDING&&r.m.tempSpikes()==1,"ID 31 83 C single sample in HOLD rejected, HOLD kept");
    std::printf("PASS real case ID 31: 28->83->28 C in HOLD rejected, HOLD kept 3 s\n");
#endif
  }
#ifndef EXPECT_RC10
  [[maybe_unused]] auto hold=[](LoadRig& r){lowers(r,24,3,21,19);CHECK(!r.m.stand(.10,r.p.now),"STAND accepted");CHECK(r.run(8000,simple::State::STANDING),"HOLD");for(unsigned i=0;i<30;++i)r.tick();};
  // Persistent heating: fault and SAFE_OFF between 0.5 s and 0.5 s + one poll.
  {LoadRig r;hold(r);r.p.observed[8].temp=75;const uint64_t t0=spike(r,8,75,75);const uint64_t tf=faultAt(r);
    CHECK(!strcmp(r.m.reason(),"SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT")&&r.m.exitSnapshot().fb[8].temp==75,"persistent 75 C faults");
    CHECK(tf-t0>=500000&&tf-t0<=500000+130000,"confirmed after 0.5 s, within one poll");
    for(unsigned i=0;i<30;++i)r.tick(false);CHECK(r.m.offVerified(),"SAFE_OFF 13/13");
    std::printf("PASS persistent 75 C -> fault after %.2f s, SAFE_OFF verified\n",(tf-t0)/1e6);}
  // Hovering at the threshold (69/71 alternating) is not reset by 69 C.
  {LoadRig r;hold(r);const uint64_t t0=spike(r,8,71,69);uint64_t tf=0;
    for(unsigned k=0;k<60&&r.m.state()!=simple::State::FAULT;++k){r.p.observed[8].temp=(k/6)%2?69:71;r.tick();}tf=r.p.now;
    CHECK(r.m.state()==simple::State::FAULT&&tf-t0<=1200000,"69/71 hovering confirms (65 C reset hysteresis)");
    std::printf("PASS hovering 69/71 C -> fault after %.2f s\n",(tf-t0)/1e6);}
  // Hot, then 66 C (above the 65 C reset), then 72 C: still one window.
  {LoadRig r;hold(r);const uint64_t t0=spike(r,8,80,66);for(unsigned i=0;i<15;++i)r.tick();r.p.observed[8].temp=72;const uint64_t tf=faultAt(r);
    CHECK(r.m.state()==simple::State::FAULT&&tf-t0>=500000&&tf-t0<=500000+130000,"66 C does not close the window");
    std::printf("PASS 80 -> 66 -> 72 C -> fault after %.2f s\n",(tf-t0)/1e6);}
  // Two isolated spikes, each followed by a normal sample: no fault.
  {LoadRig r;hold(r);spike(r,8,83,28);for(unsigned i=0;i<15;++i)r.tick();spike(r,2,81,27);for(unsigned i=0;i<100;++i)r.tick();
    CHECK(r.m.state()==simple::State::STANDING&&r.m.tempSpikes()==2,"two isolated spikes rejected");
    std::printf("PASS two isolated spikes rejected\n");}
  // Native status alarm and voltage stay immediate (same read).
  {LoadRig r;hold(r);r.p.observed[8].status=0x20;const uint64_t t0=r.p.now;const uint64_t tf=faultAt(r);
    CHECK(!strcmp(r.m.reason(),"SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT")&&tf-t0<=130000&&!r.m.hotPendingMask(),"status alarm immediate");
    std::printf("PASS native status alarm -> fault after %.2f s (immediate)\n",(tf-t0)/1e6);}
  {LoadRig r;hold(r);r.p.observed[3].voltage=35;const uint64_t t0=r.p.now;const uint64_t tf=faultAt(r);
    CHECK(!strcmp(r.m.reason(),"SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT")&&tf-t0<=130000,"voltage immediate");
    std::printf("PASS undervoltage -> fault after %.2f s (immediate)\n",(tf-t0)/1e6);}
  // Communication interrupted after a hot sample: never "normal", Motion faults.
  {LoadRig r;hold(r);spike(r,8,83,83);r.p.missing=true;bool kept=true;
    for(unsigned i=0;i<60&&r.m.state()!=simple::State::FAULT;++i){r.tick();kept=kept&&(r.m.hotPendingMask()&0x100);}
    CHECK(kept&&r.m.state()==simple::State::FAULT&&(!strcmp(r.m.reason(),"SERVO_COMMUNICATION_LOST")||!strcmp(r.m.reason(),"SERVO_FEEDBACK_STALE")),"missing reads keep the window and fault");
    std::printf("PASS hot then no feedback -> window kept, %s\n",r.m.reason());}
  // Short read loss inside the window does not reset it: 83 C persists.
  {LoadRig r;hold(r);const uint64_t t0=spike(r,8,83,83);r.p.missing=true;r.tick();r.tick();r.p.missing=false;const uint64_t tf=faultAt(r);
    CHECK(!strcmp(r.m.reason(),"SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT")&&tf-t0>=500000&&tf-t0<=500000+130000,"window survives a short read loss");
    std::printf("PASS 83 C with a short read loss -> fault after %.2f s\n",(tf-t0)/1e6);}
  // STOP inside the window: SAFE_OFF, no fault.
  {LoadRig r;hold(r);spike(r,8,83,83);r.m.stop();for(unsigned i=0;i<30;++i)r.tick(false);
    CHECK(r.m.state()==simple::State::STOPPED&&r.m.offVerified(),"STOP wins inside the window");
    std::printf("PASS STOP inside the window -> STOPPED, SAFE_OFF 13/13\n");}
  // Torque OFF admission is unchanged: 75 C at ACQUIRE refuses at once.
  {LoadRig r;r.p.observed[8].temp=75;CHECK(!r.m.stand(.05,r.p.now),"STAND accepted");r.run(200,simple::State::FAULT);
    CHECK(!strcmp(r.m.reason(),"INITIAL_SERVO_NOT_READY"),"ACQUIRE immediate");
    std::printf("PASS 75 C before enable -> INITIAL_SERVO_NOT_READY (unchanged)\n");}
#endif
#if !defined(EXPECT_RC10)
  // ---- Window lifecycle across STOP / SAFE_OFF / fault / new STAND ----
  auto toStandAgain=[](LoadRig& r){for(unsigned i=0;i<30;++i)r.tick(false);CHECK(r.m.offVerified(),"SAFE_OFF 13/13");};
  // L1 residual window at STOP, new STAND at once: idle samples close it; a
  //    new isolated spike in the next motion does not fault.
  {LoadRig r;hold(r);spike(r,8,83,28);r.m.stop();toStandAgain(r);
    CHECK(!r.m.stand(.10,r.p.now),"new STAND accepted");unsigned n=0;
    while(!r.m.energized()&&r.m.state()!=simple::State::FAULT&&n++<500)r.tick();
    CHECK(!(r.m.hotPendingMask()&0x100),"residual window closed before torque");
    for(unsigned i=0;i<10;++i)r.tick();sampleOnce(r,8,81,28);for(unsigned i=0;i<20;++i)r.tick();
    CHECK(r.m.state()!=simple::State::FAULT&&!(r.m.hotPendingMask()&0x100),"new spike rejected");
    CHECK(r.run(8000,simple::State::STANDING),"new STAND reaches HOLD despite the residual window and a new spike");
    std::printf("PASS L1 residual window at STOP + immediate new STAND + new spike -> HOLD (spikes=%u)\n",unsigned(r.m.tempSpikes()));}
  // L2 gap: window open at STOP, then 1 s not READY (no samples) with the servo
  //    warm (67 C), new STAND, one 72 C sample: continuity is lost, no fault.
  {LoadRig r;hold(r);spike(r,8,83,67);r.m.stop();toStandAgain(r);
    r.in.ready=false;for(unsigned i=0;i<50;++i)r.tick();r.in.ready=true;for(unsigned i=0;i<20;++i)r.tick();
    CHECK(!r.m.stand(.10,r.p.now),"STAND accepted after the gap");unsigned n=0;
    while(!r.m.energized()&&r.m.state()!=simple::State::FAULT&&n++<500)r.tick();
    for(unsigned i=0;i<10;++i)r.tick();sampleOnce(r,8,72,67);for(unsigned i=0;i<10;++i)r.tick();
#ifdef EXPECT_RC11R1
    CHECK(r.m.state()==simple::State::FAULT,"rc.11 r1: the old window confirms a single new sample");
    std::printf("REPRODUCED rc.11 r1: residual window across a 1 s gap + one 72 C sample -> %s\n",r.m.reason());
#else
    CHECK(r.m.state()!=simple::State::FAULT,"gap restarts the window: one 72 C sample does not fault");
    std::printf("PASS L2 window across a 1 s gap (not READY) + one 72 C sample -> no fault\n");
#endif
  }
#ifndef EXPECT_RC11R1
  // L3 continuous evidence survives STOP/SAFE_OFF: 75 C before confirmation,
  //    STOP, still 75 C -> the next STAND is refused before torque.
  {LoadRig r;hold(r);spike(r,8,75,75);for(unsigned i=0;i<10;++i)r.tick();r.m.stop();toStandAgain(r);
    CHECK(r.m.hotPendingMask()&0x100,"continuous window kept through STOP/SAFE_OFF");
    CHECK(!r.m.stand(.10,r.p.now),"STAND accepted");r.run(300,simple::State::FAULT);
    CHECK(!strcmp(r.m.reason(),"INITIAL_SERVO_NOT_READY"),"refused before torque");
    std::printf("PASS L3 75 C continuous across STOP -> new STAND refused (INITIAL_SERVO_NOT_READY)\n");}
  //    Same evidence at 68 C (accepted before torque), then 72 C persistent once
  //    energized: the SAFE_OFF/new-STAND gap (>250 ms without samples) drops the
  //    time credit, so persistent heat is re-confirmed in 0.5 s, not ignored.
  {LoadRig r;hold(r);spike(r,8,75,68);for(unsigned i=0;i<10;++i)r.tick();r.m.stop();toStandAgain(r);
    CHECK(!r.m.stand(.10,r.p.now),"STAND accepted at 68 C");unsigned n=0;
    while(!r.m.energized()&&r.m.state()!=simple::State::FAULT&&n++<500)r.tick();
    for(unsigned i=0;i<20;++i)r.tick();r.p.observed[8].temp=72;const uint64_t t0=r.p.now;const uint64_t tf=faultAt(r);
    CHECK(!strcmp(r.m.reason(),"SERVO_NATIVE_ALARM_OR_ELECTRICAL_LIMIT")&&tf-t0>=500000&&tf-t0<=500000+250000,"persistent heat after the gap re-confirmed in 0.5 s");
    std::printf("PASS L3b 75 C in HOLD, 68 C through STOP, 72 C persistent after new enable -> fault after %.2f s\n",(tf-t0)/1e6);}
  // L4 after a confirmed thermal fault, cooling, STOP and a new STAND: no fault.
  {LoadRig r;hold(r);r.p.observed[8].temp=75;faultAt(r,200);CHECK(r.m.state()==simple::State::FAULT,"thermal fault");
    r.p.observed[8].temp=30;for(unsigned i=0;i<30;++i)r.tick(false);r.m.stop();toStandAgain(r);
    CHECK(!r.m.hotPendingMask()&&!r.m.stand(.10,r.p.now),"window closed, new STAND accepted");
    CHECK(r.run(8000,simple::State::STANDING),"new STAND reaches HOLD");
    std::printf("PASS L4 thermal fault -> cooled -> STOP -> new STAND reaches HOLD\n");}
  std::printf("TEMPERATURE_CONFIRM PASS checks=%d (scripted observations; NO HARDWARE)\n",checks);
#else
  std::printf("TEMPERATURE_CONFIRM rc.11 r1 reproduction checks=%d\n",checks);
#endif
#endif
  return 0;
}
