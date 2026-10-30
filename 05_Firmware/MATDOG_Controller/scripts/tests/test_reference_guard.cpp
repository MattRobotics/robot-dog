// Production SimpleMotion + ServoBus + SDK + gen1 transforms + MotionService.
// UART registers and arrival times are modeled; no hardware is accessed.
#define main existing_preenable_suite
#include "test_preenable_bus.cpp"
#undef main
#include "../../src/core/MotionService.h"
#include <fstream>
#include <string>
static unsigned checks=0,failures=0;
#define DIAG_CHECK(x) do{++checks;if(!(x)){++failures;std::printf("DIAG_FAIL line=%u %s\n",__LINE__,#x);}}while(0)
static std::ofstream wire;
static bool published(Rig& r){
 core::MotionService s;s.begin(&r.m,nullptr,nullptr);s.setReady(true,"READY");
 char text[6144]{};assert(s.statusJson(text,sizeof(text),host_us/1000));
 return std::strstr(text,"\"reference_guard\":{")!=nullptr;
}
static void trace(const char* name,const Rig& r){
 wire<<name<<' '<<r.p.enables<<' '<<r.p.frames.size()<<' '<<simple::name(r.m.state())<<' '<<r.m.reason()<<'\n';
 const char digits[]="0123456789abcdef";
 for(const auto& packet:r.p.uart->requests){for(auto c:packet)wire<<digits[c>>4]<<digits[c&15];wire<<'\n';}
}
static void off(Rig& r){r.m.stop();for(unsigned i=0;i<30;++i)r.tick();assert(r.m.offVerified()&&r.m.state()==simple::State::STOPPED);}
static void driftFault(Rig& r,unsigned slot){
 assert(!r.m.stand(.05,host_us));r.toEnable(slot);
 auto& q=r.p.uart->regs[Port::id(slot)];const unsigned moved=rawPose[slot]+13;q[56]=moved&255;q[57]=moved>>8;r.tick();
 assert(r.m.state()==simple::State::FAULT&&r.p.enables==slot&&!strcmp(r.m.reason(),"REFERENCE_CHANGED_BEFORE_ENABLE"));
 DIAG_CHECK(published(r)&&r.m.referenceFaultSlot()==slot&&r.m.referenceFaultFeedback().raw==moved&&r.m.referenceFaultRaw()==rawPose[slot]);
 DIAG_CHECK(r.m.referenceFaultFeedback().torque==0&&r.m.referenceFaultFeedback().goal==rawPose[slot]&&r.m.referenceFaultFeedback().controlStampUs>0);
}
int main(int argc,char** argv){
 assert(argc==2);wire.open(argv[1]);assert(wire);
 for(unsigned slot:{0u,5u,11u}){
  {Rig r;assert(!r.m.stand(.05,host_us));r.toEnable(slot);r.p.uart->regs[Port::id(slot)][40]=1;
   const auto n=r.p.frames.size();r.tick();assert(r.p.enables==slot&&r.p.frames.size()==n&&r.p.uart->regs[Port::id(slot)][40]==0);
   DIAG_CHECK(!published(r)&&r.m.referenceFaultSlot()==12); // retry is not an incident
   r.tick();assert(r.p.enables==slot+1&&r.p.checked[slot].torque==0);assert(r.prepare());
   DIAG_CHECK(!published(r)&&r.m.referenceFaultSlot()==12);
   r.p.follow=true;for(unsigned n=0;n<1800&&r.m.state()!=simple::State::STANDING;++n)r.tick();assert(r.m.state()==simple::State::STANDING);
   DIAG_CHECK(!published(r));
   assert(!r.m.stand(.15,host_us));assert(!strcmp(r.m.phase(),"HEIGHT"));DIAG_CHECK(!published(r)&&r.m.referenceFaultSlot()==12);
   trace("recovered_and_height",r);}
  {Rig r;driftFault(r,slot);const auto incident=r.m.referenceFaultFeedback();
   DIAG_CHECK(r.m.stand(.05,host_us)!=nullptr&&published(r)); // FAULT refusal retains it
   DIAG_CHECK(r.m.walk(3,host_us)!=nullptr&&published(r)); // invalid WALK refusal retains it
   r.m.stop();DIAG_CHECK(published(r));for(unsigned n=0;n<30;++n)r.tick();assert(r.m.offVerified());
   DIAG_CHECK(published(r)&&r.m.referenceFaultFeedback().raw==incident.raw&&r.m.referenceFaultFeedback().controlStampUs==incident.controlStampUs);
   DIAG_CHECK(r.m.stand(.01,host_us)!=nullptr&&published(r)); // invalid height does not acquire a command
   auto saved=r.in;r.in.powerOk=false;r.m.inputs(r.in);DIAG_CHECK(r.m.stand(.05,host_us)!=nullptr&&published(r));r.in=saved;r.m.inputs(r.in);
   assert(!r.m.stand(.05,host_us));DIAG_CHECK(r.m.referenceFaultSlot()==12&&!published(r));assert(!strcmp(r.m.phase(),"ACQUIRE"));
   trace("fault_stop_new_stand",r);}
  {Rig r;driftFault(r,slot);off(r);DIAG_CHECK(published(r));
   assert(!r.m.walk(1,host_us));DIAG_CHECK(r.m.referenceFaultSlot()==12&&!published(r));assert(!strcmp(r.m.phase(),"ACQUIRE"));
   trace("fault_stop_new_walk",r);}
 }
 wire.close();std::printf("REFERENCE_GUARD %s checks=%u failures=%u; actual MotionService JSON; hardware=false\n",failures?"FAIL":"PASS",checks,failures);
 return failures?1:0;
}
