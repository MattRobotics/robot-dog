#pragma once
#include "../simple/SimpleMotion.h"
namespace matdog { namespace core {
// Transport adapter only. SimpleMotion is the one operational state owner.
class MotionService {
 public:
  using Prepare=const char*(*)(void*);
  void begin(simple::SimpleMotion* m,Prepare p,void* c){motion_=m;prepare_=p;context_=c;}
  void setReady(bool r,const char* reason){ready_=r;reason_=reason;}
  void update();
  motion::MotionAck command(const motion::MotionCommand&,uint32_t now_ms);
  bool statusJson(char*,size_t,uint32_t) const;
  bool capabilitiesJson(char*,size_t) const;
  bool ackJson(const motion::MotionAck&,char*,size_t) const;
  bool busy() const { return motion_&&motion_->engaged(); }
  bool engaged() const {return busy();}
  bool slack(uint64_t now,uint32_t reserve) const {
    return !busy()||motion_->nextReleaseUs()>now+reserve;
  }
 private:
  simple::SimpleMotion* motion_=nullptr;
  Prepare prepare_=nullptr;void* context_=nullptr;
  bool ready_=false,active_=false;
  const char* reason_="BOOTING";
  motion::MotionAck last_{},activeAck_{};
  struct Replay {uint32_t sequence=0;char id[49]{};motion::MotionAck ack{};} replay_[3]{};
};
} }
