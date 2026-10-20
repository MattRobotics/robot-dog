// Core Motion V5: IMU acquisition core (src/imu/ImuFeed.h). Host, virtual time.
// The SH-2 hub is a MODEL (report cadence from Bno085Imu.cpp: RV/game/mag 50 Hz,
// acc/gyro 10 Hz); the library defects it exposes are VERIFIED in the vendored
// source (see ImuFeed.h header). Figures are simulated, not hardware measurements.
#include <algorithm>
#include <cstdio>
#include <deque>
#include <thread>
#include <vector>
#include "../../src/imu/ImuFeed.h"
#include "imu_hub_model.h"
using namespace matdog::imu;
using namespace imu_model;
static int checks=0,failures=0;
#define CHECK(c) do{++checks;if(!(c)){++failures;std::printf("FAIL %d: %s\n",__LINE__,#c);}}while(0)
namespace {
// Library behaviour BEFORE V5: one getSensorEvent() per loop; every report of a
// cargo overwrites the same value, so only the last one is visible.
struct Legacy {
  uint64_t seen_rv=0,max_gap_us=0,last_rv_us=0,max_age_us=0,windows_over_600ms=0;
  void loopOnce(Hub& hub,uint64_t now_us){
    if(!hub.intLow())return;
    const auto c=hub.cargo();if(c.empty())return;
    const Report last=c.back();                      // sensorHandler: same sh2_SensorValue_t each report
    if(last.kind==RV){++seen_rv;if(last_rv_us&&now_us-last_rv_us>max_gap_us){max_gap_us=now_us-last_rv_us;}
      if(last_rv_us&&now_us-last_rv_us>600000){++windows_over_600ms;}
      last_rv_us=now_us;max_age_us=std::max(max_age_us,now_us-last.gen_us);}
  }
};
struct Modern {
  ImuFeed feed;uint64_t max_gap_us=0,last_rv_us=0,max_age_us=0,windows_over_600ms=0,transfers_while_idle=0;
  void service(Hub& hub,uint64_t now_us){
    const float q[4]={1,0,0,0};
    servicePass([&]{return hub.intLow();},
      [&]{if(!hub.intLow())++transfers_while_idle;
          for(const auto& r:hub.cargo()){
            if(r.kind==RV){feed.onRotationVector(q,.01f,3,static_cast<int32_t>(now_us-r.gen_us),static_cast<uint32_t>(now_us),static_cast<uint32_t>(now_us/1000));
              if(last_rv_us&&now_us-last_rv_us>max_gap_us){max_gap_us=now_us-last_rv_us;}
              if(last_rv_us&&now_us-last_rv_us>600000){++windows_over_600ms;}
              last_rv_us=now_us;max_age_us=std::max(max_age_us,now_us-r.gen_us);}
            else feed.noteOther(r.kind);}},
      [&]{return static_cast<uint32_t>(now_us);},8,3000);
  }
};
}
int main(){
  // ===== A. mechanism: one event per loop with last-report-wins vs per-report delivery =====
  std::printf("IMU_MODEL batching service_period_ms legacy_rv_seen/generated legacy_max_rv_gap_ms | v5_rv_seen/generated v5_max_rv_gap_ms v5_max_data_age_ms (10 s, simulated)\n");
  for(size_t batch:{size_t(1),size_t(1000)})for(uint32_t period_ms:{4u,10u,20u,60u}){
    Hub hl(batch,period_ms),hm(batch,period_ms);Legacy legacy;Modern modern;const uint64_t end=10000000;
    uint64_t next_legacy=0;
    for(uint64_t t=0;t<=end;t+=1000){                  // 1 ms ticks: the IMU task period
      hl.advance(t);hm.advance(t);
      if(t>=next_legacy){legacy.loopOnce(hl,t);next_legacy=t+period_ms*1000;}
      modern.service(hm,t);
    }
    std::printf("IMU_MODEL batch=%zu loop=%2u legacy %4llu/%llu gap=%7.1f | v5 %4u/%llu gap=%6.1f age=%6.2f drops=%llu\n",batch,period_ms,
      (unsigned long long)legacy.seen_rv,(unsigned long long)hl.generated_rv,legacy.max_gap_us/1000.0,
      modern.feed.rvCount(),(unsigned long long)hm.generated_rv,modern.max_gap_us/1000.0,modern.max_age_us/1000.0,(unsigned long long)hm.dropped);
    CHECK(modern.feed.rvCount()==hm.generated_rv-0||modern.feed.rvCount()+2>=hm.generated_rv);   // every generated RV delivered (last one may be in flight)
    CHECK(modern.transfers_while_idle==0);CHECK(modern.windows_over_600ms==0);CHECK(modern.max_age_us<=2000);
    if(batch==1000&&period_ms>=20)CHECK(legacy.seen_rv<hl.generated_rv);                         // overwrite loses Rotation Vector events
    if(batch==1&&period_ms>=20)CHECK(hl.dropped>0||legacy.max_age_us>modern.max_age_us*10);      // backlog: stale-in-time or dropped
  }
  // ===== B. a pass never waits and is bounded =====
  {
    Hub idle(1,0);Modern m;uint32_t calls=0;
    const auto pass=servicePass([&]{return idle.intLow();},[&]{++calls;},[]{return 0u;},8,3000);
    CHECK(pass.transfers==0&&calls==0&&!pass.budget_stop&&!pass.backlog_stop);          // INT high: not a single SH-2 call
    Hub busy(1,0);busy.advance(2000000);const size_t pending=busy.fifo.size();CHECK(pending>8);
    uint32_t tick=0;const auto p2=servicePass([&]{return busy.intLow();},[&]{busy.cargo();},[&]{return tick+=1;},8,3000);
    CHECK(p2.transfers==8&&p2.backlog_stop&&busy.fifo.size()==pending-8);
    uint32_t t3=0;Hub slow(1,0);slow.advance(2000000);
    const auto p3=servicePass([&]{return slow.intLow();},[&]{slow.cargo();t3+=1000;},[&]{return t3;},8,3000);
    CHECK(p3.transfers==3&&p3.budget_stop&&!p3.backlog_stop);                           // 1 ms per transfer, 3 ms budget
    std::printf("IMU_PASS idle_sh2_calls=0 cap_transfers=%u budget_transfers=%u (bounded, never waits)\n",p2.transfers,p3.transfers);
  }
  // ===== C. snapshot contents, receive time, wrap =====
  {
    ImuFeed f;RotationSample s;CHECK(!f.snapshot(&s));
    const float q[4]={.5f,.5f,.5f,.5f};
    f.onRotationVector(q,.02f,3,1500,0xFFFFF000u,123456);
    f.onRotationVector(q,.02f,3,1500,0xFFFFF000u+20000u,123476);          // wraps the 32-bit clock
    CHECK(f.snapshot(&s)&&s.valid&&s.seq==2&&s.rx_ms==123476&&s.sensor_age_us==1500&&s.wxyz[0]==.5f);
    CHECK(f.stats().rv_gap_last_us==20000&&f.stats().rv_gap_min_us==20000&&f.stats().rv_gap_max_us==20000);
    std::printf("IMU_SNAPSHOT rx_ms=%u sensor_age_us=%d rv_gap_us=%u wrap_safe=1\n",s.rx_ms,s.sensor_age_us,f.stats().rv_gap_last_us);
  }
  // ===== D. IMU_STALE stays real: silence is visible, a late-but-real sample is not hidden =====
  {
    auto fresh=[](uint32_t now,uint32_t at,uint32_t max){return static_cast<int32_t>(now-at)<=static_cast<int32_t>(max);};  // Controller.cpp sampleFresh
    ImuFeed f;const float q[4]={1,0,0,0};f.onRotationVector(q,.01f,3,0,1000000,1000);RotationSample s;CHECK(f.snapshot(&s));
    CHECK(fresh(1500,s.rx_ms,600));CHECK(fresh(1600,s.rx_ms,600));CHECK(!fresh(1601,s.rx_ms,600));CHECK(!fresh(3000,s.rx_ms,600));   // real silence => stale => SAFE_OFF path
    CHECK(fresh(990,s.rx_ms,600));   // a sample received later in the same Controller loop is fresh, never 'stale by underflow'
  }
#ifdef IMU_FEED_TSAN
  // ===== E. single-writer snapshot under real concurrency (TSan) =====
  {
    ImuFeed f;std::atomic<bool> stop{false};std::atomic<uint64_t> torn{0},reads{0};
    std::thread writer([&]{for(uint32_t i=1;i<=400000;++i){const float q[4]={float(i),float(i),float(i),float(i)};f.onRotationVector(q,float(i),3,int32_t(i),i,i);}stop=true;});
    std::thread reader([&]{RotationSample s;uint32_t last=0;while(!stop){if(f.snapshot(&s)){++reads;
      if(s.wxyz[0]!=s.wxyz[3]||s.wxyz[1]!=s.wxyz[2]||s.rx_ms!=s.seq||s.accuracy_rad!=float(s.seq)||s.seq<last){++torn;}
      last=s.seq;}}});
    writer.join();reader.join();CHECK(torn==0);
    std::printf("IMU_SEQLOCK writer=400000 reads=%llu torn=%llu\n",(unsigned long long)reads.load(),(unsigned long long)torn.load());
  }
#endif
  std::printf("IMU_FEED %s checks=%d failures=%d hardware=false\n",failures?"FAIL":"PASS",checks,failures);
  return failures?1:0;
}
