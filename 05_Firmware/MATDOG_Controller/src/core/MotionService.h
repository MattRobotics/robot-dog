#pragma once
#include "../simple/SimpleMotion.h"
namespace matdog { namespace core {
constexpr size_t kMotionStatusJsonBytes=5120; // fixed HTTPD buffer, including transport diagnostics
// Immutable per-request observation, transferred by the existing HTTP mailbox.
// Contains no owner, port, command or writable Motion state. String pointers
// refer only to firmware-lifetime reason/phase literals.
struct MotionStatusSnapshot {
  uint64_t capturedUs=0,imuStampUs=0,maxGapUs=0,offVerifiedUs=0;
  uint32_t capturedMs=0,releases=0,missed=0,bmsAgeMs=0,maxReleaseWorkUs=0;
  uint32_t commandMissed=0,firstMissLatenessUs=0,commandMaxWorkUs=0,commandMaxCompensateUs=0,commandMaxSolves=0;
  const char* firstMissPhase="";const char* commandMaxWorkPhase="";
  simple::State state=simple::State::STOPPED;
  const char *phase="IDLE",*poseRegion="UNKNOWN",*reason="BOOTING",*readinessReason="BOOTING",*powerReason="POWER_NOT_READY";
  char activeId[49]{};
  uint32_t activeSequence=0,acceptedMs=0,finishedMs=0;
  uint16_t commandedRaw[12]{};
  const char *activeReason="",*activeOutcome="";
  bool ready=false,fresh=false,offVerified=false,energized=false,heightValid=false,stabilizationLimited=false,imuValid=false;
  uint16_t offMask=0,referenceFaultRaw=0;
  double requestedHeightM=0,heightM=0,walkCycles=0,quaternion[4]{};
  unsigned referenceFaultSlot=12;
  simple::Feedback feedback[12]{},referenceFaultFeedback{};
};
static_assert(sizeof(MotionStatusSnapshot)<=1536,"bounded per-request Motion observation");
// Transport adapter only. SimpleMotion is the one operational state owner.
class MotionService {
 public:
  using Prepare=const char*(*)(void*);
  void begin(simple::SimpleMotion* m,Prepare p,void* c){motion_=m;prepare_=p;context_=c;}
  void setReady(bool r,const char* reason){ready_=r;reason_=reason;}
  void update();
  motion::MotionAck command(const motion::MotionCommand&,uint32_t now_ms);
  bool statusJson(char*,size_t,uint32_t) const;
  bool captureStatus(MotionStatusSnapshot&,uint32_t) const;
  static bool formatStatus(const MotionStatusSnapshot&,char*,size_t);
  bool capabilitiesJson(char*,size_t) const;
  bool ackJson(const motion::MotionAck&,char*,size_t) const;
  bool busy() const { return motion_&&motion_->engaged(); }
  const simple::SimpleMotion* motion() const { return motion_; } // LOADDIAG read-only
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
  uint32_t acceptedMs_=0,finishedMs_=0;
  struct Replay {uint32_t sequence=0;char id[49]{};motion::MotionAck ack{};} replay_[3]{};
};
} }
