#pragma once
// SH-2 sensor hub MODEL used by the IMU tests (report cadence from Bno085Imu.cpp:
// RV/game/mag 50 Hz, acc/gyro 10 Hz). Not a hardware measurement; the finite hub
// buffer depth and the cargo batching are HYPOTHESES, so tests sweep both.
#include <cstdint>
#include <deque>
#include <vector>
namespace imu_model {
enum Kind{GAME=0,MAG=1,ACC=2,GYR=3,RV=4};
struct Report{Kind kind;uint64_t gen_us;};
struct Hub {                         // SH-2 sensor hub model: reports queue until the host reads
  std::deque<Report> fifo;uint64_t next[5];uint32_t period[5]={20000,20000,100000,100000,20000};
  uint64_t mute_from_us=0,mute_until_us=0;   // sensor silent: reports generated in [from,until) never exist
  size_t batch_max;uint64_t generated_rv=0,dropped=0;size_t fifo_cap=64;   // finite hub buffer (HYPOTHESIS: real depth unknown)
  Hub(size_t batch,uint32_t seed):batch_max(batch){const uint32_t ph[5]={3000+seed%2000,1000+seed%500,7000,9000,6000+seed%3000};
    for(unsigned i=0;i<5;++i)next[i]=ph[i];}
  void advance(uint64_t to_us){
    for(;;){unsigned k=0;for(unsigned i=1;i<5;++i)if(next[i]<next[k])k=i;if(next[k]>to_us)break;
      const bool muted=next[k]>=mute_from_us&&next[k]<mute_until_us;
      if(!muted){if(fifo.size()>=fifo_cap){fifo.pop_front();++dropped;}
        fifo.push_back({static_cast<Kind>(k),next[k]});if(k==RV)++generated_rv;}
      next[k]+=period[k];}
  }
  bool intLow()const{return !fifo.empty();}
  // First instant a report will actually be available (muted generations do not exist).
  uint64_t nextEventUs()const{uint64_t m=next[0];for(unsigned i=1;i<5;++i)m=m<next[i]?m:next[i];return(m>=mute_from_us&&m<mute_until_us)?mute_until_us:m;}
  std::vector<Report> cargo(){std::vector<Report> c;while(!fifo.empty()&&c.size()<batch_max){c.push_back(fifo.front());fifo.pop_front();}return c;}
};
}
