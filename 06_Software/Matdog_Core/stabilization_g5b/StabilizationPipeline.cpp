#include "StabilizationPipeline.h"
#include <cmath>
#include <limits>

namespace matdog { namespace stabilization {
namespace {
bool finite(Vector3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
double dot(Vector3 a, Vector3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vector3 sub(Vector3 a, Vector3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
double distance(Vector3 a, Vector3 b) { const auto d = sub(a,b); return std::sqrt(dot(d,d)); }
double joint(const motion::LegJointAngles& q, unsigned j) { return j == 0 ? q.hip : j == 1 ? q.upper : q.lower; }
bool same(Vector3 a, Vector3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
bool evidence(Evidence e) { return e == Evidence::OFFLINE_FIXTURE || e == Evidence::EXTERNAL_MEASUREMENT; }
bool validConfig(const PipelineConfig& c) {
  if (!motion::validAttitudePolicy(c.attitude) || !motion::validStabilizerConfig(c.stabilizer) ||
      !evidence(c.referenceEvidence) || c.referenceRevision == 0 || !evidence(c.bounds.evidence) || c.bounds.revision == 0) return false;
  const double positive[] = {c.maxSampleGapS, c.maxMeasurementToCommandS, c.maxSupportAgeS,
    c.minContactConfidence, c.planeToleranceM, c.maxPlaneTiltRad, c.maxContactSpeedMS,
    c.maxNormalRateRadS, c.maxJointStepRad, c.maxJointRateRadS, c.maxIkCondition, c.ikToleranceM};
  for (double v : positive) if (!std::isfinite(v) || v <= 0) return false;
  if (!std::isfinite(c.maxCommandLeadS) || c.maxCommandLeadS < 0 ||
      !std::isfinite(c.minJointMarginRad) || c.minJointMarginRad < 0 ||
      c.minContactConfidence > 1 || c.maxPlaneTiltRad >= 1.0 ||
      c.maxIkIterations == 0 || c.maxIkIterations > 40 || c.ikToleranceM > 1e-5 ||
      c.maxSampleGapS > 4*c.stabilizer.sampleDtS || c.maxSampleGapS < c.stabilizer.sampleDtS ||
      c.stabilizer.integralGainPerS >= motion::maxStableIntegralGain(c.maxSampleGapS, c.stabilizer.loopDelaySteps)) return false;
  if (!std::isfinite(c.dampingTimeS) || c.dampingTimeS < 0 || !std::isfinite(c.maxTiltRateRadS) || c.maxTiltRateRadS < 0 ||
      (c.dampingTimeS > 0 && (!c.allowDampingExperiment || c.maxTiltRateRadS <= 0))) return false;
  for (unsigned i=0; i<4; ++i) for (unsigned j=0; j<3; ++j) {
    const auto& b = c.bounds.joints[i][j]; const auto& u = motion::legModel(static_cast<motion::LegId>(i))->limits[j];
    if (!std::isfinite(b.lower) || !std::isfinite(b.upper) || b.lower < u.lower || b.upper > u.upper ||
        b.upper - b.lower <= 2*c.minJointMarginRad) return false;
  }
  return true;
}
bool validAnchor(const NominalPose& n, const PipelineConfig& c) {
  if (!motion::validBodyPose(n.pose) || !n.anchor.valid || n.revision == 0 || std::abs(n.pose.rotation[2][0]) >= .99) return false;
  for (unsigned i=0; i<4; ++i) for (unsigned j=0; j<3; ++j) {
    const double q = joint(n.anchor.legs[i],j); const auto& b = c.bounds.joints[i][j];
    if (!std::isfinite(q) || q < b.lower+c.minJointMarginRad || q > b.upper-c.minJointMarginRad) return false;
  }
  return true;
}
bool sameSample(const Bno085Sample& a, const Bno085Sample& b) {
  const auto& x=a.orientation; const auto& y=b.orientation;
  return a.connected == b.connected && a.acquisitionEpoch == b.acquisitionEpoch &&
    x.w==y.w && x.x==y.x && x.y==y.y && x.z==y.z && x.stampS==y.stampS && x.sequence==y.sequence &&
    x.accuracyRad==y.accuracyRad && x.accuracyStatus==y.accuracyStatus;
}
bool sameSupport(const SupportSnapshot& a, const SupportSnapshot& b) {
  if (!same(a.normalWorld,b.normalWorld) || !same(a.originWorldM,b.originWorldM) || a.revision!=b.revision ||
      a.evidence!=b.evidence || a.mode!=b.mode) return false;
  for (unsigned i=0; i<4; ++i) if (!same(a.contacts[i].worldM,b.contacts[i].worldM) ||
    a.contacts[i].state!=b.contacts[i].state || a.contacts[i].confidence!=b.contacts[i].confidence) return false;
  return true;
}
BodyPose correctedPose(const BodyPose& nominal, double rollCorrection, double pitchCorrection) {
  // Explicit ZYX correction; preserve nominal yaw and translation. IMU heading is never read here.
  const double roll = std::atan2(nominal.rotation[2][1], nominal.rotation[2][2]);
  const double pitch = std::asin(-nominal.rotation[2][0]);
  const double yaw = std::atan2(nominal.rotation[1][0], nominal.rotation[0][0]);
  BodyPose p = motion::bodyPoseFromRollPitch(nominal.translationM,roll+rollCorrection,pitch+pitchCorrection);
  const double cy=std::cos(yaw), sy=std::sin(yaw);
  for (unsigned j=0; j<3; ++j) {
    const double x=p.rotation[0][j], y=p.rotation[1][j];
    p.rotation[0][j]=cy*x-sy*y; p.rotation[1][j]=sy*x+cy*y;
  }
  return p;
}
Reason checkSupport(const SupportSnapshot& s, const PipelineConfig& c, double commandS,
                    bool havePrevious, const SupportSnapshot& previous) {
  if (!evidence(s.evidence) || s.revision==0 || !finite(s.normalWorld) || !finite(s.originWorldM) ||
      !std::isfinite(s.stampS) || s.stampS < 0 || s.stampS>commandS || std::abs(dot(s.normalWorld,s.normalWorld)-1)>1e-6 ||
      s.normalWorld.z <= 0 || std::acos(std::fmin(1.0,s.normalWorld.z))>c.maxPlaneTiltRad) return Reason::INVALID_SUPPORT;
  if (commandS-s.stampS>c.maxSupportAgeS) return Reason::STALE_SUPPORT;
  if (s.mode != SurfaceMode::WORLD_LOCKED_HORIZONTAL && s.mode != SurfaceMode::MOVING_PLANE_EXPERIMENTAL) return Reason::INVALID_SUPPORT;
  if (s.mode == SurfaceMode::MOVING_PLANE_EXPERIMENTAL && !c.allowMovingPlaneExperiment) return Reason::PLANE_GATE;
  if (s.mode == SurfaceMode::WORLD_LOCKED_HORIZONTAL && !same(s.normalWorld,{0,0,1})) return Reason::PLANE_GATE;
  for (const auto& foot : s.contacts) {
    if (foot.state != ContactState::CONFIRMED_SUPPORT || !std::isfinite(foot.confidence) ||
        foot.confidence<c.minContactConfidence || foot.confidence>1) return Reason::UNCONFIRMED_CONTACT;
    if (!finite(foot.worldM) || std::abs(dot(s.normalWorld,sub(foot.worldM,s.originWorldM)))>c.planeToleranceM) return Reason::INVALID_SUPPORT;
  }
  // Reject degenerate support geometry. This is a plane-model check, NOT a CoM/load test.
  const auto a=sub(s.contacts[1].worldM,s.contacts[0].worldM), b=sub(s.contacts[3].worldM,s.contacts[0].worldM);
  const Vector3 cross{a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
  if (dot(cross,cross)<=1e-16) return Reason::INVALID_SUPPORT;
  if (!havePrevious) return Reason::OK;
  if (s.revision!=previous.revision || s.evidence!=previous.evidence || s.mode!=previous.mode ||
      s.sequence<previous.sequence || s.stampS<previous.stampS) return Reason::SUPPORT_ORDER;
  if (s.sequence==previous.sequence) return s.stampS==previous.stampS && sameSupport(s,previous) ? Reason::OK : Reason::SUPPORT_ORDER;
  const double dt=s.stampS-previous.stampS;
  if (!(dt>0)) return Reason::SUPPORT_ORDER;
  if (s.mode==SurfaceMode::WORLD_LOCKED_HORIZONTAL) {
    if (!same(s.originWorldM,previous.originWorldM)) return Reason::PLANE_MOTION;
    for (unsigned i=0;i<4;++i) if (!same(s.contacts[i].worldM,previous.contacts[i].worldM)) return Reason::PLANE_MOTION;
  } else {
    if (std::acos(std::fmax(-1.0,std::fmin(1.0,dot(s.normalWorld,previous.normalWorld)))) > c.maxNormalRateRadS*dt+1e-10) return Reason::PLANE_MOTION;
    for (unsigned i=0;i<4;++i) if (distance(s.contacts[i].worldM,previous.contacts[i].worldM)>c.maxContactSpeedMS*dt) return Reason::PLANE_MOTION;
  }
  return Reason::OK;
}
}
const char* reasonName(Reason r) {
  switch (r) {
#define G5B_REASON(x) case Reason::x: return #x
    G5B_REASON(OK); G5B_REASON(NOT_CONFIGURED); G5B_REASON(DISABLED); G5B_REASON(FAULT_LATCHED);
    G5B_REASON(INVALID_TIME); G5B_REASON(DISCONNECTED); G5B_REASON(IMU_REJECTED); G5B_REASON(DUPLICATE_SAMPLE);
    G5B_REASON(INCOHERENT_SAMPLE); G5B_REASON(SAMPLE_GAP); G5B_REASON(EPOCH_CHANGED); G5B_REASON(COMMAND_TOO_LATE);
    G5B_REASON(INVALID_NOMINAL); G5B_REASON(INVALID_SUPPORT); G5B_REASON(STALE_SUPPORT); G5B_REASON(UNCONFIRMED_CONTACT);
    G5B_REASON(SUPPORT_ORDER); G5B_REASON(PLANE_GATE); G5B_REASON(PLANE_MOTION); G5B_REASON(INVALID_RATE);
    G5B_REASON(IK_FAILURE); G5B_REASON(CONDITION_LIMIT); G5B_REASON(OPERATIONAL_LIMIT); G5B_REASON(TARGET_DISCONTINUITY);
    G5B_REASON(OUTPUT_SEQUENCE_EXHAUSTED);
#undef G5B_REASON
  }
  return "UNKNOWN_REASON";
}
bool StabilizationPipeline::configure(const PipelineConfig& c) {
  if (enabled_ || fault_ || diagnostic_.rollCorrectionRad!=0 || diagnostic_.pitchCorrectionRad!=0) return false;
  if (!validConfig(c)) { configured_=false; return false; }
  *this = StabilizationPipeline(); cfg_=c;
  configured_ = monitor_.configure(c.attitude) && stabilizer_.configure(c.stabilizer);
  return configured_;
}
bool StabilizationPipeline::enable(const NominalPose& n, double nowS) {
  if (!configured_ || enabled_ || fault_ || !std::isfinite(nowS) || nowS<0 || !validAnchor(n,cfg_) || !stabilizer_.enable(nowS)) return false;
  enabled_=true; previous_=n.anchor; previousCommandS_=lastNewNowS_=lastCallS_=nowS;
  haveSample_=haveSupport_=false; monitor_.reset();
  return true;
}
void StabilizationPipeline::disable() { enabled_=false; stabilizer_.disable(); }
bool StabilizationPipeline::resetFault() {
  if (enabled_ || !fault_ || diagnostic_.rollCorrectionRad!=0 || diagnostic_.pitchCorrectionRad!=0 ||
      stabilizer_.state()!=motion::StabilizerState::DISABLED) return false;
  fault_=false; monitor_.reset(); haveSample_=haveSupport_=false;
  return true;
}
PipelineOutput StabilizationPipeline::reject(PipelineOutput out, Reason r, double nowS) {
  fault_=true;
  stabilizer_.disable();
  diagnostic_=stabilizer_.step(motion::AttitudeStatus::STALE,{},false,nowS);
  // Never publish a ramp-down frame when measurements or geometry are invalid.
  out.reason=r; out.correction=diagnostic_; out.faultLatched=true;
  out.target={}; for (auto& contact : out.achievedContactsWorldM) contact={};
  return out;
}
PipelineOutput StabilizationPipeline::update(const Bno085Sample& s, const NominalPose& n,
 const SupportSnapshot& support, double nowS, double commandS, const TiltRateSample& rate) {
  PipelineOutput out; out.inputSequence=s.orientation.sequence; out.acquisitionEpoch=s.acquisitionEpoch;
  out.supportSequence=support.sequence; out.nominalRevision=n.revision;
  out.referenceRevision=cfg_.referenceRevision; out.boundsRevision=cfg_.bounds.revision;
  out.generatedAtS=nowS; out.intendedCommandAtS=commandS;
  out.correction=diagnostic_; out.faultLatched=fault_;
  if (!configured_) return out;
  if (!std::isfinite(nowS) || !std::isfinite(commandS) || nowS<0 || nowS<lastCallS_ || commandS<nowS)
    return reject(out,Reason::INVALID_TIME,nowS);
  lastCallS_=nowS;
  if (fault_ || !enabled_) {
    diagnostic_=stabilizer_.step(motion::AttitudeStatus::STALE,{},false,nowS);
    out.correction=diagnostic_; out.reason=fault_ ? Reason::FAULT_LATCHED : Reason::DISABLED; return out;
  }
  if (!s.connected) return reject(out,Reason::DISCONNECTED,nowS);
  if (haveSample_ && s.acquisitionEpoch!=lastSample_.acquisitionEpoch) return reject(out,Reason::EPOCH_CHANGED,nowS);
  // Monitor is authoritative for quaternion, quality, freshness and caller-time validation.
  motion::Attitude attitude; bool isNew=false;
  out.attitudeStatus=monitor_.evaluate(s.orientation,nowS,attitude,&isNew);
  if (out.attitudeStatus!=motion::AttitudeStatus::OK) return reject(out,Reason::IMU_REJECTED,nowS);
  if (s.orientation.stampS<0 || s.orientation.accuracyRad<0 || s.orientation.accuracyStatus>3 ||
      (haveSample_ && s.orientation.sequence!=lastSample_.orientation.sequence &&
       (s.orientation.sequence<lastSample_.orientation.sequence || s.orientation.stampS<=lastSample_.orientation.stampS)))
    return reject(out,Reason::INCOHERENT_SAMPLE,nowS);
  if (!isNew) {
    if (!sameSample(s,lastSample_)) return reject(out,Reason::INCOHERENT_SAMPLE,nowS);
  }
  out.measurementAgeS=nowS-s.orientation.stampS;
  out.measurementToCommandS=commandS-s.orientation.stampS;
  out.sampleIntervalS=haveSample_ ? s.orientation.stampS-lastSample_.orientation.stampS : 0;
  if (nowS-lastNewNowS_>cfg_.maxSampleGapS+1e-12 || out.sampleIntervalS>cfg_.maxSampleGapS+1e-12)
    return reject(out,Reason::SAMPLE_GAP,nowS);
  if (commandS-nowS>cfg_.maxCommandLeadS+1e-12 || out.measurementToCommandS>cfg_.maxMeasurementToCommandS+1e-12 ||
      out.measurementToCommandS>cfg_.attitude.maxAgeS+1e-12) return reject(out,Reason::COMMAND_TOO_LATE,nowS);
  if (!motion::validBodyPose(n.pose) || n.revision==0 || std::abs(n.pose.rotation[2][0])>=.99)
    return reject(out,Reason::INVALID_NOMINAL,nowS);
  if (support.stampS>nowS) return reject(out,Reason::INVALID_SUPPORT,nowS);
  const Reason supportStatus=checkSupport(support,cfg_,commandS,haveSupport_,lastSupport_);
  if (supportStatus!=Reason::OK) return reject(out,supportStatus,nowS);
  if (!isNew) { out.reason=Reason::DUPLICATE_SAMPLE; return out; } // do NOT advance regulator time on polling
  out.experimental=support.mode==SurfaceMode::MOVING_PLANE_EXPERIMENTAL || cfg_.dampingTimeS>0;
  if (cfg_.dampingTimeS>0) {
    if (!rate.valid || !std::isfinite(rate.rollRadS) || !std::isfinite(rate.pitchRadS) ||
        rate.stampS!=s.orientation.stampS || rate.sequence!=s.orientation.sequence ||
        std::abs(rate.rollRadS)>cfg_.maxTiltRateRadS || std::abs(rate.pitchRadS)>cfg_.maxTiltRateRadS)
      return reject(out,Reason::INVALID_RATE,nowS);
    attitude.rollRad+=cfg_.dampingTimeS*rate.rollRadS; attitude.pitchRad+=cfg_.dampingTimeS*rate.pitchRadS;
  }
  // Transactional control: rejected IK/limits cannot accumulate an unpublished integral correction.
  auto candidate=stabilizer_;
  const auto correction=candidate.step(motion::AttitudeStatus::OK,attitude,true,nowS);
  out.correctedPose=correctedPose(n.pose,correction.rollCorrectionRad,correction.pitchCorrectionRad);
  if (!motion::validBodyPose(out.correctedPose)) return reject(out,Reason::INVALID_NOMINAL,nowS);
  const auto& r=out.correctedPose.rotation;
  const Vector3 normalBase{r[0][0]*support.normalWorld.x+r[1][0]*support.normalWorld.y+r[2][0]*support.normalWorld.z,
    r[0][1]*support.normalWorld.x+r[1][1]*support.normalWorld.y+r[2][1]*support.normalWorld.z,
    r[0][2]*support.normalWorld.x+r[1][2]*support.normalWorld.y+r[2][2]*support.normalWorld.z};
  JointTargetFrame target; out.minJointMarginRad=std::numeric_limits<double>::infinity();
  for (unsigned i=0; i<4; ++i) {
    Vector3 base;
    if (!motion::pointToBase(out.correctedPose,support.contacts[i].worldM,base)) return reject(out,Reason::INVALID_NOMINAL,nowS);
    motion::TiltedIkOptions options; options.seed=previous_.legs[i]; options.toleranceM=cfg_.ikToleranceM;
    options.maxIterations=cfg_.maxIkIterations; options.requireNominalStrip=true;
    const auto ik=motion::contactInverseKinematicsGroundNormal(static_cast<motion::LegId>(i),base,normalBase,options);
    if (ik.status!=motion::TiltedIkResult::Status::OK) {
      out.failedLeg=static_cast<uint8_t>(i); out.ikStatus=ik.status; return reject(out,Reason::IK_FAILURE,nowS);
    }
    if (!std::isfinite(ik.condition) || ik.condition>cfg_.maxIkCondition) return reject(out,Reason::CONDITION_LIMIT,nowS);
    target.legs[i]=ik.joints;
    if (!motion::pointToWorld(out.correctedPose,ik.contact.referenceM,out.achievedContactsWorldM[i])) return reject(out,Reason::IK_FAILURE,nowS);
    out.maxContactResidualM=std::fmax(out.maxContactResidualM,distance(out.achievedContactsWorldM[i],support.contacts[i].worldM));
    out.maxCondition=std::fmax(out.maxCondition,ik.condition);
    out.maxIterations=ik.iterations>out.maxIterations ? ik.iterations : out.maxIterations;
    for (unsigned j=0;j<3;++j) {
      const double q=joint(ik.joints,j); const auto& b=cfg_.bounds.joints[i][j];
      const double margin=std::fmin(q-b.lower,b.upper-q);
      out.minJointMarginRad=std::fmin(out.minJointMarginRad,margin);
      if (!std::isfinite(q) || margin<cfg_.minJointMarginRad) return reject(out,Reason::OPERATIONAL_LIMIT,nowS);
      const double delta=std::abs(q-joint(previous_.legs[i],j)); out.maxJointDeltaRad=std::fmax(out.maxJointDeltaRad,delta);
      if (commandS<previousCommandS_ || delta>cfg_.maxJointStepRad+1e-10 ||
          delta>cfg_.maxJointRateRadS*(commandS-previousCommandS_)+1e-10) return reject(out,Reason::TARGET_DISCONTINUITY,nowS);
    }
  }
  if (outputSequence_==std::numeric_limits<uint32_t>::max()) return reject(out,Reason::OUTPUT_SEQUENCE_EXHAUSTED,nowS);
  stabilizer_=candidate; diagnostic_=correction; out.correction=correction;
  target.valid=true; target.sequence=++outputSequence_; out.target=target; out.reason=Reason::OK;
  out.validUntilS=std::fmin(s.orientation.stampS+std::fmin(cfg_.attitude.maxAgeS,cfg_.maxMeasurementToCommandS),support.stampS+cfg_.maxSupportAgeS);
  if (haveSample_) out.skippedSamples=s.orientation.sequence-lastSample_.orientation.sequence-1;
  previous_=target; previousCommandS_=commandS; lastNewNowS_=nowS;
  lastSample_=s; haveSample_=true; lastSupport_=support; haveSupport_=true;
  return out;
}
} }
