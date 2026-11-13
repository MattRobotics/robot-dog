#pragma once
#include "SimpleMotion.h"
#include "../motion/TiltedContactIk.h"
#include "../motion/ImuAttitude.h"
#include <algorithm>
#include <cmath>
#include <cstring>

namespace matdog { namespace simple {
// Phase 1 fallback: bounded, passive observation in STAND. No ServoPort,
// actuator writes, inferred contact, force calibration, or persistent offsets.
// 12 fresh sweeps / 5 seconds bound storage and work; NOT hardware thresholds.
constexpr unsigned kGroundObservationSamples=12;
constexpr uint64_t kGroundObservationTimeoutUs=5000000;
inline int groundLeg(const char* name) {
  if(!name)return -1;
  const char* names[]={"LF","RF","RH","LH"}; // canonical firmware order
  for(unsigned i=0;i<4;++i)if(!std::strcmp(name,names[i]))return int(i);
  return -1;
}
inline const char* groundLegName(unsigned leg) {
  const char* names[]={"LF","RF","RH","LH"};return leg<4?names[leg]:"NONE";
}
inline int signMagnitude(int raw,unsigned bit) {
  const int magnitude=raw&((1<<bit)-1);return raw&(1<<bit)?-magnitude:magnitude;
}
struct GroundContactObservation {
  uint64_t startedUs=0,finishedUs=0,stampUs[3]{},imuStampUs=0,referenceSentUs=0;
  const char* reason="NOT_STARTED";
  float lowestMm=0,targetLowestMm=0,minMm=0,maxMm=0;
  float relativeMm=0,relativeMinMm=0,relativeMaxMm=0,bodyDeltaMm=0,supportSpreadMm=0;
  float referenceJacobianMm[3]{}; // d(reference height)/dq; NOT force gains
  uint32_t spanUs=0,maxSpanUs=0;
  uint16_t raw[3]{},commandedRaw[3]{};
  int16_t load[3]{},current[3]{},speed[3]{},tracking[3]{};
  int16_t loadDelta[3]{},loadMin[3]{},loadMax[3]{};
  uint8_t leg=4,samples=0;
  bool running=false;
};
// Fits inside the existing status mailbox with an explicit bounded addition.
static_assert(sizeof(GroundContactObservation)<=240,"bounded ground observation");

class GroundContactDiagnostic {
 public:
  const GroundContactObservation& snapshot() const { return out_; }
  bool running() const { return out_.running; }
  static const char* admission(const SimpleMotion& m,uint64_t now) {
    if(m.stopping())return "GROUND_STOP_ACTIVE";
    if(m.state()!=State::STANDING||std::strcmp(m.phase(),"HOLD")||!m.energized())return "GROUND_REQUIRES_STAND_HOLD";
    if(!m.fresh(now))return "GROUND_FEEDBACK_STALE";
    if(!m.imuValid()||now<m.imuStampUs()||now-m.imuStampUs()>200000)return "GROUND_IMU_UNAVAILABLE";
    return nullptr;
  }
  void start(unsigned leg,uint64_t now) {
    out_={};out_.leg=static_cast<uint8_t>(leg);out_.startedUs=now;
    out_.running=true;out_.reason="ACQUIRING_OBSERVATION";
    for(auto& stamp:lastStamp_)stamp=now; // never reuse pre-request feedback
  }
  void abort(const char* why,uint64_t now) {
    if(!running())return;
    out_.running=false;out_.reason=why;out_.finishedUs=now;
  }
  // Controller thread, cooperative slack only; uses cached feedback, never
  // polls the bus. Every joint must have advanced since the preceding sweep.
  void update(const SimpleMotion& m,uint64_t now) {
    if(!running())return;
    if(const char* why=admission(m,now)){abort(why,now);return;}
    if(now<out_.startedUs||now-out_.startedUs>=kGroundObservationTimeoutUs){abort("GROUND_OBSERVATION_TIMEOUT",now);return;}
    // goalSentUs dates every successful changed RAW/drive frame. A whole
    // observation window after it permits tracking against commandedRaw,
    // without pretending the 1.44 s control-register read is contemporary.
    if(m.goalSentUs()>out_.startedUs){abort("GROUND_REFERENCE_CHANGED",now);return;}
    const auto* fb=m.feedback();uint64_t oldest=now,newest=0;
    for(unsigned i=0;i<12;++i){
      if(!fb[i].valid||fb[i].stampUs<=lastStamp_[i])return;
      if(fb[i].stampUs>now){abort("GROUND_FUTURE_FEEDBACK",now);return;}
      oldest=std::min(oldest,fb[i].stampUs);newest=std::max(newest,fb[i].stampUs);
      if(fb[i].status||fb[i].torque!=1){abort("GROUND_SERVO_STATUS",now);return;}
      // Zero is a conservative observation filter, not proof of immobility.
      if(signMagnitude(fb[i].speed,15)!=0){abort("GROUND_SERVO_MOVING",now);return;}
    }
    oldest=std::min(oldest,m.imuStampUs());newest=std::max(newest,m.imuStampUs());
    motion::Attitude attitude;const auto* q=m.quaternion();
    if(!motion::tiltFromQuaternion(q[0],q[1],q[2],q[3],attitude)){abort("GROUND_IMU_UNAVAILABLE",now);return;}
    const motion::Vector3 up{attitude.upBase[0],attitude.upBase[1],attitude.upBase[2]};
    double heights[4];motion::ContactResult selected;motion::LegJointAngles selectedQ;
    for(unsigned leg=0;leg<4;++leg){
      const motion::LegJointAngles actual{fb[leg*3].q,fb[leg*3+1].q,fb[leg*3+2].q};
      const auto contact=motion::contactForwardKinematics(static_cast<motion::LegId>(leg),actual,up);
      if(!motion::lowestTreadHeightM(contact,up,heights[leg])){abort("GROUND_FK_UNAVAILABLE",now);return;}
      if(leg==out_.leg){selected=contact;selectedQ=actual;}
    }
    double predicted=0,J[3][3];const auto leg=static_cast<motion::LegId>(out_.leg);
    if(!m.target().valid||!motion::lowestTreadHeightM(motion::contactForwardKinematics(leg,m.target().legs[out_.leg],up),up,predicted)||
       !motion::contactReferenceJacobian(leg,selectedQ,up,selected,J)){
      abort("GROUND_FK_UNAVAILABLE",now);return;
    }
    const bool first=!out_.samples;
    if(first)std::copy(heights,heights+4,anchors_);
    // Hypothesis only: the OTHER three tread points remain fixed vertically.
    // Median translation and disagreement are exposed, never called support
    // proof. Common slip/compression remains unobservable.
    double shift[3];unsigned k=0;
    for(unsigned l=0;l<4;++l)if(l!=out_.leg)shift[k++]=anchors_[l]-heights[l];
    std::sort(shift,shift+3);
    out_.bodyDeltaMm=shift[1]*1000;out_.supportSpreadMm=(shift[2]-shift[0])*1000;
    out_.lowestMm=heights[out_.leg]*1000;out_.targetLowestMm=predicted*1000;
    out_.relativeMm=(heights[out_.leg]+shift[1]-anchors_[out_.leg])*1000;
    out_.minMm=first?out_.lowestMm:std::min(out_.minMm,out_.lowestMm);
    out_.maxMm=first?out_.lowestMm:std::max(out_.maxMm,out_.lowestMm);
    out_.relativeMinMm=first?out_.relativeMm:std::min(out_.relativeMinMm,out_.relativeMm);
    out_.relativeMaxMm=first?out_.relativeMm:std::max(out_.relativeMaxMm,out_.relativeMm);
    for(unsigned j=0;j<3;++j){const unsigned i=out_.leg*3+j;const auto& f=fb[i];
      out_.stampUs[j]=f.stampUs;out_.raw[j]=f.raw;out_.commandedRaw[j]=m.commandedRaw()[i];
      out_.load[j]=signMagnitude(f.load,10);out_.current[j]=signMagnitude(f.current,15);out_.speed[j]=signMagnitude(f.speed,15);
      out_.tracking[j]=int(f.raw)-int(m.commandedRaw()[i]);
      if(first)baselineLoad_[j]=out_.load[j];
      out_.loadDelta[j]=out_.load[j]-baselineLoad_[j];
      out_.loadMin[j]=first?out_.load[j]:std::min(out_.loadMin[j],out_.load[j]);
      out_.loadMax[j]=first?out_.load[j]:std::max(out_.loadMax[j],out_.load[j]);
      out_.referenceJacobianMm[j]=(up.x*J[0][j]+up.y*J[1][j]+up.z*J[2][j])*1000;
    }
    for(unsigned i=0;i<12;++i)lastStamp_[i]=fb[i].stampUs;
    out_.referenceSentUs=m.goalSentUs();out_.imuStampUs=m.imuStampUs();out_.spanUs=static_cast<uint32_t>(newest-oldest);
    out_.maxSpanUs=std::max(out_.maxSpanUs,out_.spanUs);
    if(++out_.samples==kGroundObservationSamples)abort("OBSERVATION_ONLY_NO_CONTACT",now);
  }
 private:
  GroundContactObservation out_{};
  uint64_t lastStamp_[12]{};
  double anchors_[4]{};
  // HOLD evidence (REPORT_C_IDENT_HOLD, 2026-11-13): load, current and
  // tracking are correlated regulator outputs, NOT independent contact cues.
  int16_t baselineLoad_[3]{}; // first STAND observation; NOT a free-leg baseline
};
} }
