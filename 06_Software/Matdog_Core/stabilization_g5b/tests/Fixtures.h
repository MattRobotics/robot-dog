#ifndef MATDOG_G5B_TEST_FIXTURES_H
#define MATDOG_G5B_TEST_FIXTURES_H
#include "StabilizationPipeline.h"
#include <cmath>
namespace fixture {
using namespace matdog::stabilization;
using namespace matdog::motion;
// Synthetic policies for repeatable software tests only; never a board configuration.
inline PipelineConfig config() {
  PipelineConfig c;
  c.attitude={.08,.1,2,1e-3,.5};
  c.stabilizer={1,.03,.04,0,.02,2,.1,0,0};
  c.referenceEvidence=Evidence::OFFLINE_FIXTURE; c.referenceRevision=1;
  c.bounds.evidence=Evidence::OFFLINE_FIXTURE; c.bounds.revision=1;
  for (unsigned i=0;i<4;++i) for (unsigned j=0;j<3;++j) c.bounds.joints[i][j]=legModel(static_cast<LegId>(i))->limits[j];
  c.maxSampleGapS=.06; c.maxCommandLeadS=.02; c.maxMeasurementToCommandS=.06;
  c.maxSupportAgeS=.1; c.minContactConfidence=.8; c.planeToleranceM=1e-7;
  c.maxPlaneTiltRad=.12; c.maxContactSpeedMS=.05; c.maxNormalRateRadS=.2;
  c.minJointMarginRad=.001; c.maxJointStepRad=.03; c.maxJointRateRadS=2;
  c.maxIkCondition=1000; c.ikToleranceM=1e-9; c.maxIkIterations=40;
  return c;
}
inline NominalPose nominal() {
  NominalPose n; n.revision=1; n.pose.translationM={0,0,canonicalStandDefinition().standBodyHeightM};
  n.anchor=generateStandTarget(canonicalStandDefinition()).target; return n;
}
inline Bno085Sample sample(double roll, double pitch, double stamp, uint32_t seq) {
  Bno085Sample s; s.connected=true; s.acquisitionEpoch=1;
  const double cr=std::cos(roll/2),sr=std::sin(roll/2),cp=std::cos(pitch/2),sp=std::sin(pitch/2);
  s.orientation={cr*cp,sr*cp,cr*sp,-sr*sp,stamp,seq,.01,3}; return s;
}
inline SupportSnapshot support(double stamp, uint32_t seq=1) {
  SupportSnapshot s; s.stampS=stamp; s.sequence=seq; s.revision=1; s.evidence=Evidence::OFFLINE_FIXTURE;
  for (unsigned i=0;i<4;++i) s.contacts[i]={canonicalStandDefinition().contactsWorldM[i],ContactState::CONFIRMED_SUPPORT,1};
  return s;
}
inline SupportSnapshot moving(double stamp, uint32_t seq, double roll, double pitch) {
  auto s=support(stamp,seq); s.mode=SurfaceMode::MOVING_PLANE_EXPERIMENTAL;
  const auto p=bodyPoseFromRollPitch({0,0,0},roll,pitch);
  s.normalWorld={p.rotation[0][2],p.rotation[1][2],p.rotation[2][2]};
  for (auto& foot : s.contacts) { Vector3 world{}; pointToWorld(p,foot.worldM,world); foot.worldM=world; }
  return s;
}
}
#endif
