// Core Motion V5: Motion cadence with slow/blocking services. Production
// Coordinator/Executor/AdaptiveStand through the existing StreamDriver; the
// Controller loop period is a MODEL (service costs below are assumptions
// derived from the vendored IMU library source, NOT hardware measurements).
#define MATDOG_OPERATIONAL_NO_MAIN
#include "test_operational_motion.cpp"
#include "../../src/imu/ImuFeed.h"
#include "imu_hub_model.h"
#include <functional>
namespace {
struct Result{bool stood=false;double elapsed_s=0,entry_s=-1,gap_p50=0,gap_p99=0,gap_max=0,age_max=0,loop_p50=0,loop_p99=0,loop_max=0;unsigned frames=0;std::string reason;
  uint64_t imu_stale_at_us=0;};
// dt_fn(now_us) = duration of the NEXT Controller loop iteration; imu_hook(now_us) advances the IMU task.
Result runStand(const char* name,std::function<uint32_t(uint64_t)> dt_fn,unsigned reads,uint32_t stall_every_ms=0,uint32_t stall_us=0,
                std::function<void(uint64_t)> imu_hook=nullptr){
 Operational op(2.,-92.070442,1,false);auto& b=op.b;CHECK(op.stand().outcome==motion::CommandOutcome::ACCEPTED);
 for(unsigned k=0;k<40&&b.coordinator.arming();++k)op.tickArming();
 StreamModel m;m.name=name;m.loop_us=4000;m.reads_per_tick=reads;m.stop_short_ticks=5;m.stall_every_ms=stall_every_ms;m.stall_us=stall_us;
 m.speed_rad_s=300*motion::tolerance::kTickRad;m.acc_rad_s2=20*100*motion::tolerance::kTickRad;
 StreamDriver d(op,m);Result r;const uint64_t start=d.now_us;uint64_t entry_end=0,last_tx=0;Dist intervals,age,loop;bool last_final=false,ok=true;unsigned previous=0;
 while(ok&&std::strcmp(b.coordinator.state(),"STAND")&&d.now_us-start<120000000ull){
  d.m.loop_us=dt_fn(d.now_us);loop.add(d.m.loop_us/1000.+0);
  ok=d.tick();if(imu_hook)imu_hook(d.now_us);
  if(!entry_end&&!b.coordinator.stand().entryPending()&&d.frames_seen)entry_end=d.now_us;
  if(d.frames_seen!=previous){if(last_tx&&!last_final)intervals.add((d.now_us-last_tx)/1000.);last_final=b.coordinator.stand().pendingFinal();last_tx=d.now_us;previous=d.frames_seen;}
  for(unsigned i=0;i<12;++i)age.add((d.now_us-d.sample_us[i])/1000.);
 }
 r.stood=ok&&!std::strcmp(b.coordinator.state(),"STAND");r.elapsed_s=(d.now_us-start)/1e6;r.entry_s=entry_end?(entry_end-start)/1e6:-1;r.frames=d.frames_seen;
 r.gap_p50=intervals.pct(.5);r.gap_p99=intervals.pct(.99);r.gap_max=intervals.max();r.age_max=age.max();r.loop_p50=loop.pct(.5);r.loop_p99=loop.pct(.99);r.loop_max=loop.max();
 r.reason=b.r.executor.status().reason;
 std::printf("ISOLATION %-34s stood=%d elapsed_s=%7.3f frames=%4u loop_ms p50=%6.2f p99=%7.2f max=%7.2f | frame_gap_ms p50=%6.2f p99=%6.2f max=%7.2f | joint_age_ms max=%7.2f reason=%s\n",
  name,r.stood,r.elapsed_s,r.frames,r.loop_p50,r.loop_p99,r.loop_max,r.gap_p50,r.gap_p99,r.gap_max,r.age_max,r.reason.c_str());
 return r;
}
}
int main(){
 using namespace imu_model;
 const uint32_t kWorkUs=1500;   // model: loop work excluding any IMU wait
 // ---- AS-IS: IMU acquisition inside the Motion loop (one getSensorEvent per iteration, blocks for INT) ----
 auto asisDt=[](Hub& hub,uint64_t* resets){return [&hub,resets](uint64_t t)->uint32_t{
   hub.advance(t);uint64_t wait=0;
   if(!hub.intLow()){const uint64_t nt=hub.nextEventUs();wait=nt>t?nt-t:0;
     if(wait>500000){wait=500000;++*resets;hub.mute_until_us=t+wait;}     // spihal_wait_for_int: 500 x delay(1), then hardware reset
     hub.advance(t+wait);}
   hub.cargo();return static_cast<uint32_t>(kWorkUs+wait);};};
 uint64_t resets=0;
 Hub hub1(1,1);auto asis=runStand("AS_IS imu paced loop (170 ev/s)",asisDt(hub1,&resets),3);
 Hub hub2(1,2);hub2.mute_from_us=800000;hub2.mute_until_us=1500000;resets=0;
 auto asis_silent=runStand("AS_IS + sensor silent 700 ms @0.8 s",asisDt(hub2,&resets),3);
 std::printf("ISOLATION_NOTE as_is_500ms_blocking_waits_with_sensor_reset=%llu (spihal_wait_for_int)\n",(unsigned long long)resets);
 CHECK(resets>=1);
 // ---- TO-BE: IMU in its own task; Motion loop period set by the poll gate only ----
 auto tobeDt=[](uint64_t)->uint32_t{return 4000;};
 auto tobe=runStand("TO_BE motion loop, imu in task",tobeDt,3);
 // IMU task advanced alongside (1 ms passes) over the same time axis, with the same 700 ms silence
 {imu_model::Hub hub(1,3);hub.mute_from_us=5000000;hub.mute_until_us=5700000;matdog::imu::ImuFeed feed;uint64_t task_t=0,last_rv=0,stale_at=0,max_age=0;
  auto hook=[&](uint64_t now){for(;task_t<=now;task_t+=1000){hub.advance(task_t);const float q[4]={1,0,0,0};
     matdog::imu::servicePass([&]{return hub.intLow();},[&]{for(const auto& r:hub.cargo())if(r.kind==imu_model::RV){feed.onRotationVector(q,.01f,3,0,static_cast<uint32_t>(task_t),static_cast<uint32_t>(task_t/1000));last_rv=task_t;}},
       [&]{return static_cast<uint32_t>(task_t);},8,3000);
     const uint64_t age=last_rv?task_t-last_rv:0;max_age=std::max(max_age,age);if(age>600000&&!stale_at)stale_at=task_t;}};
  Hub dummy(1,0);(void)dummy;
  auto tobe_silent=runStand("TO_BE + sensor silent 700 ms @5 s",tobeDt,3,0,0,hook);
  std::printf("ISOLATION_IMU to_be_max_rv_age_ms=%.1f imu_stale_declared_at_ms=%.1f (real silence still reaches the 600 ms IMU_STALE gate)\n",max_age/1000.,stale_at/1000.);
  CHECK(stale_at>5000000&&stale_at<5700000+1000);CHECK(max_age>=600000);
  CHECK(tobe_silent.stood);   // Motion cadence is not touched by the sensor outage (the Controller still reaches SAFE_OFF through the IMU_STALE gate)
 }
 CHECK(tobe.stood);CHECK(tobe.gap_max<=24.5);
 CHECK(!asis_silent.stood);CHECK(asis_silent.loop_max>=500);   // the library's own 500 ms wait lands inside the Motion loop
 // V5: the IMU-paced loop's jitter alone stopped the qualified STAND (ACCELERATION_LIMIT, as on the 19/10 hardware run).
 // Core Motion V6: the same jitter is retimed coherently (MotionLaw retimeRelease) and the STAND completes.
 CHECK(asis.stood);
 // ---- release jitter that REMAINS in the Motion thread. The qualified STAND profile moves at v=.19 rad/s with a 4 mrad
 // step cap per 20 ms frame. V5 (clamp + re-stamped release) stopped at a 6 ms stall. V6 retimes every release against the
 // committed frame: a late frame decelerates within the step and the return is ramped. A release later than the profile
 // can decelerate within its step is a true infeasibility, refused before TX (fail-safe, with operands). ----
 for(uint32_t stall_ms:{1u,2u,3u,4u,5u,6u,7u,8u,9u,10u,12u,20u,57u,120u}){char n[64];std::snprintf(n,sizeof(n),"TO_BE qualified profile stall %u ms/500 ms",stall_ms);
  const auto r=runStand(n,tobeDt,3,500,stall_ms*1000);
  if(stall_ms<=12)CHECK(r.stood);
  std::printf("STALL_SWEEP stall_ms=%u stood=%d reason=%s\n",stall_ms,r.stood,r.reason.c_str());}
 std::printf("MOTION_ISOLATION %s checks=%d failures=%d hardware=false\n",failures?"FAIL":"PASS",checks,failures);
 return failures?1:0;
}
