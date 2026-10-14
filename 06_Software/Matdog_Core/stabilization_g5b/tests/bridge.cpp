#include "Fixtures.h"
#include <chrono>
using namespace fixture;
namespace { StabilizationPipeline pipeline; NominalPose nominalPose; bool movingPlane=false; }
extern "C" {
int g5b_init(double gain, double damping, unsigned delay, int moving) {
  pipeline=StabilizationPipeline(); nominalPose=nominal(); movingPlane=moving!=0;
  auto c=config(); c.stabilizer.integralGainPerS=gain; c.stabilizer.loopDelaySteps=delay;
  c.allowMovingPlaneExperiment=movingPlane; c.allowDampingExperiment=damping>0; c.dampingTimeS=damping; c.maxTiltRateRadS=4;
  return pipeline.configure(c) && pipeline.enable(nominalPose,0);
}
// Host test ABI only. input[20]: tilt r,p, acquisition stamp, seq, now, command,
// plane r,p, connected, epoch, tilt rates r,p, rateValid, qScale, confidence,
// unknownFoot(-1=none), supportStamp, nominalDz, farContact, reserved.
int g5b_step(const double* in, double* out) {
  auto s=sample(in[0],in[1],in[2],static_cast<uint32_t>(in[3]));
  s.connected=in[8]!=0; s.acquisitionEpoch=static_cast<uint32_t>(in[9]);
  s.orientation.w*=in[13]; s.orientation.x*=in[13]; s.orientation.y*=in[13]; s.orientation.z*=in[13];
  auto supportFrame=movingPlane ? moving(in[16],s.orientation.sequence,in[6],in[7]) : support(in[16],s.orientation.sequence);
  for (unsigned i=0;i<4;++i) supportFrame.contacts[i].confidence=in[14];
  if (in[15]>=0 && in[15]<4) supportFrame.contacts[static_cast<unsigned>(in[15])].state=ContactState::UNKNOWN;
  if (in[18]!=0) supportFrame.contacts[3].worldM.x+=1;
  auto n=nominalPose; n.pose.translationM.z+=in[17];
  const TiltRateSample rate{in[10],in[11],in[2],s.orientation.sequence,in[12]!=0};
  const auto o=pipeline.update(s,n,supportFrame,in[4],in[5],rate);
  for (unsigned i=0;i<64;++i) out[i]=0;
  out[0]=o.correction.rollCorrectionRad; out[1]=o.correction.pitchCorrectionRad; out[2]=o.target.valid;
  out[3]=o.correction.saturated; out[4]=o.faultLatched; out[5]=o.maxContactResidualM;
  out[6]=o.minJointMarginRad; out[7]=o.maxJointDeltaRad; out[8]=o.maxCondition; out[9]=o.maxIterations;
  out[10]=o.measurementAgeS; out[11]=o.measurementToCommandS; out[12]=o.sampleIntervalS;
  out[13]=o.skippedSamples; out[14]=o.target.sequence; out[15]=o.validUntilS;
  for (unsigned i=0;i<4;++i) {
    out[16+3*i]=o.target.legs[i].hip; out[17+3*i]=o.target.legs[i].upper; out[18+3*i]=o.target.legs[i].lower;
    out[40+3*i]=o.achievedContactsWorldM[i].x; out[41+3*i]=o.achievedContactsWorldM[i].y; out[42+3*i]=o.achievedContactsWorldM[i].z;
  }
  for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j) out[28+3*i+j]=o.correctedPose.rotation[i][j];
  out[37]=o.correctedPose.translationM.x; out[38]=o.correctedPose.translationM.y; out[39]=o.correctedPose.translationM.z;
  out[52]=static_cast<unsigned>(o.attitudeStatus); out[53]=static_cast<unsigned>(o.ikStatus); out[54]=o.failedLeg;
  out[55]=o.experimental;
  return static_cast<int>(o.reason);
}
const char* g5b_reason(int r) { return reasonName(static_cast<Reason>(r)); }
void g5b_disable() { pipeline.disable(); }
int g5b_reset() { return pipeline.resetFault(); }
int g5b_enable(double now) { return pipeline.enable(nominalPose,now); }
unsigned g5b_size() { return sizeof(StabilizationPipeline); }
unsigned g5b_output_size() { return sizeof(PipelineOutput); }
double g5b_benchmark(unsigned steps, double* maximumUs, unsigned* accepted) {
  auto p=StabilizationPipeline(); const auto n=nominal(); p.configure(config()); p.enable(n,0);
  using Clock=std::chrono::steady_clock; double total=0; *maximumUs=0; *accepted=0;
  for (unsigned i=1;i<=steps;++i) {
    const double t=.02*i; const auto s=sample(.002*std::sin(t),.003*std::cos(t),t,i); const auto f=support(t,i);
    const auto start=Clock::now(); const auto o=p.update(s,n,f,t,t); const auto end=Clock::now();
    const double us=std::chrono::duration<double,std::micro>(end-start).count();
    total+=us; *maximumUs=std::fmax(*maximumUs,us); *accepted+=o.target.valid ? 1u : 0u;
  }
  return steps ? total/steps : 0;
}
}
