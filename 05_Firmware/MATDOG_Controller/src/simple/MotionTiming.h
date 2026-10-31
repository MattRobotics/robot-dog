#pragma once
#include <algorithm>
#include <cstdint>
#include <esp_timer.h>

namespace matdog { namespace simple { namespace timing {
// Controller-owned observations only. No task, UART, allocation, synchronization
// or control decisions. All intervals are WALL time (preemption is included).
enum Part : unsigned { READ, NOMINAL, COMPENSATE, CONVERT, FRAME, REST, PARTS };
enum External : unsigned { USB_CAPTURE, USB_FORMAT, USB_SERIAL, ROUTER, HTTP,
                          IMU_BMS, CONTROLLER_SERVICES, TELEMETRY, EXTERNALS };
// Deterministic numeric work of one release: compensate solve() calls, tilted
// leg IK calls, Newton steps and contact FK evaluations inside that IK.
enum Work : unsigned { SOLVES, IK, NEWTON, FK, WORKS };
struct Record {
  bool valid=false;
  const char* phase="IDLE";
  const char* previousPhase="IDLE";
  uint64_t stampUs=0,latenessUs=0,visitGapUs=0,outsideVisitUs=0;
  uint32_t release=0,missed=0,skipped=0,workUs=0;
  uint32_t us[PARTS]{},previousUs[PARTS]{},externalUs[EXTERNALS]{};
  uint32_t fastReads=0,controlReads=0,frames=0;
  uint32_t work[WORKS]{},previousWork[WORKS]{};
};
struct State {
  Record firstMiss{},maximum{},working{};
  bool armed=false,active=false;
  unsigned part=REST;
  uint64_t partStart=0,start=0,lastVisit=0,lastExit=0,visitGap=0,outside=0;
  uint64_t maxVisitGap=0,maxOutside=0;
  const char* previousPhase="IDLE";
  uint32_t sectionMax[PARTS]{};
  uint32_t previous[PARTS]{},external[EXTERNALS]{},externalMax[EXTERNALS]{};
  uint32_t previousWork[WORKS]{};
  uint32_t missed=0;  // release periods lost since the command armed this record
};
inline State state{};
inline void arm() { state=State{};state.armed=true; }
inline void count(Work w,uint32_t n=1) { if(state.active)state.working.work[w]+=n; }
struct Visit {
  bool armed;
  Visit():armed(state.armed) {
    if(!armed)return;
    const uint64_t now=esp_timer_get_time();
    state.visitGap=state.lastVisit?now-state.lastVisit:0;
    state.outside=state.lastExit?now-state.lastExit:0;
    state.maxVisitGap=std::max(state.maxVisitGap,state.visitGap);
    state.maxOutside=std::max(state.maxOutside,state.outside);state.lastVisit=now;
  }
  ~Visit(){if(armed)state.lastExit=esp_timer_get_time();}
};
inline void switchPart(unsigned part) {
  if(!state.active)return;
  const uint64_t now=esp_timer_get_time();
  state.working.us[state.part]+=static_cast<uint32_t>(now-state.partStart);
  state.partStart=now;state.part=part;
}
struct Section {
  unsigned previous;
  explicit Section(Part p):previous(state.part){switchPart(p);}
  ~Section(){switchPart(previous);}
};
struct Outside {
  External section;uint64_t start;bool armed;
  explicit Outside(External p):section(p),start(esp_timer_get_time()),armed(state.armed){}
  ~Outside(){if(!armed)return;const auto us=static_cast<uint32_t>(esp_timer_get_time()-start);
    state.external[section]+=us;state.externalMax[section]=std::max(state.externalMax[section],us);}
};
struct Release {
  uint64_t start;uint32_t& maximum;
  Release(uint32_t& m,uint64_t stamp,const char* phase,uint32_t number,
          uint64_t lateness,uint32_t missed,uint32_t skipped):start(esp_timer_get_time()),maximum(m) {
    if(!state.armed)return;
    state.missed+=skipped;
    state.working=Record{};auto& r=state.working;r.valid=true;r.phase=phase;
    r.stampUs=stamp;r.release=number;r.latenessUs=lateness;r.missed=missed;r.skipped=skipped;
    r.visitGapUs=state.visitGap;r.outsideVisitUs=state.outside;r.previousPhase=state.previousPhase;
    for(unsigned i=0;i<PARTS;++i)r.previousUs[i]=state.previous[i];
    for(unsigned i=0;i<EXTERNALS;++i){r.externalUs[i]=state.external[i];state.external[i]=0;}
    for(unsigned i=0;i<WORKS;++i)r.previousWork[i]=state.previousWork[i];
    state.start=start;state.partStart=start;state.part=REST;state.active=true;
  }
  ~Release(){
    if(state.active){switchPart(REST);state.active=false;auto& r=state.working;
      // The same endpoint closes the partition: sum(us)==workUs exactly.
      r.workUs=static_cast<uint32_t>(state.partStart-start);
      for(unsigned i=0;i<PARTS;++i){state.previous[i]=r.us[i];state.sectionMax[i]=std::max(state.sectionMax[i],r.us[i]);}
      for(unsigned i=0;i<WORKS;++i)state.previousWork[i]=r.work[i];
      state.previousPhase=r.phase;
      if(r.skipped&&!state.firstMiss.valid)state.firstMiss=r;
      if(!state.maximum.valid||r.workUs>state.maximum.workUs)state.maximum=r;
      maximum=std::max(maximum,r.workUs);
    }else maximum=std::max(maximum,static_cast<uint32_t>(esp_timer_get_time()-start));
  }
};
} } }
