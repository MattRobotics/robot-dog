#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main original_main
#include "test_simple_motion.cpp"
#undef main
#pragma GCC diagnostic pop
int main(){
 for(double h:{.05,.10,.15}){Rig r;assert(!r.m.stand(h,r.p.now));assert(r.reachStand());
  core::MotionService svc;svc.begin(&r.m,nullptr,nullptr);svc.setReady(true,"READY");
  motion::MotionCommand c;c.intent=motion::Intent::LIE_DOWN;c.owner=motion::ClientOwner::DASHBOARD;c.sequence=1;c.issued_at_ms=r.p.now/1000;c.ttl_ms=1000;strcpy(c.id,"prone-1");
  unsigned frames=r.p.frames,off=r.p.offCalls,reads=r.p.reads;
  auto a=svc.command(c,r.p.now/1000);assert(a.outcome==motion::CommandOutcome::ACCEPTED);
  assert(r.m.state()==simple::State::MOVING&&r.m.energized());
  assert(frames==r.p.frames&&off==r.p.offCalls&&reads==r.p.reads);
  r.m.stop();a=svc.command(c,r.p.now/1000);r.tick();assert(r.m.state()==simple::State::STOPPED);
  for(int i=0;i<20;++i)r.tick();assert(r.m.offVerified());
 }
 puts("PRONE admission: accepted intent preserves torque, zero servo I/O, STOP priority at 50/100/150; terminal support NOT QUALIFIED");
}
