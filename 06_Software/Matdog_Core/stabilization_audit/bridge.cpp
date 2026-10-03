// Host-only numeric bridge to the production G5-A units. No framework or device dependency.
#include "TiltCompensation.h"
#include "BodyStabilizer.h"
#include "ActuatorEnvelope.h"
#include "ImuAttitude.h"
using namespace matdog::motion;
namespace {
LegJointAngles q3(const double* p) { return {p[0], p[1], p[2]}; }
}
extern "C" {
int g5a_tilt(double w, double x, double y, double z, double* out) {
  Attitude a; if (!tiltFromQuaternion(w, x, y, z, a)) return 0;
  out[0] = a.rollRad; out[1] = a.pitchRad; out[2] = a.upBase[0]; out[3] = a.upBase[1]; out[4] = a.upBase[2]; return 1;
}
void g5a_stand(double* contacts12, double* seeds12, double* misc) {
  const auto& d = canonicalStandDefinition();
  for (unsigned i = 0; i < 4; ++i) { contacts12[3*i] = d.contactsWorldM[i].x; contacts12[3*i+1] = d.contactsWorldM[i].y; contacts12[3*i+2] = d.contactsWorldM[i].z;
    seeds12[3*i] = d.standSeed[i].hip; seeds12[3*i+1] = d.standSeed[i].upper; seeds12[3*i+2] = d.standSeed[i].lower; }
  misc[0] = d.standBodyHeightM; misc[1] = d.lowBodyHeightM;
}
// out: joints[12], contacts[12], metrics[4] (drift, margin, condition, iterations), info[2] (failed leg, leg status)
int g5a_compensate(double height, double roll, double pitch, const double* seeds12, int requireStrip, double* joints12, double* contacts12, double* metrics4, int* info2) {
  LegJointAngles seeds[4]; for (unsigned i = 0; i < 4; ++i) seeds[i] = q3(seeds12 + 3*i);
  const auto c = compensateStand(canonicalStandDefinition(), height, roll, pitch, seeds, requireStrip != 0);
  info2[0] = c.failedLeg; info2[1] = static_cast<int>(c.legStatus);
  if (c.status != TiltCompensation::Status::OK) return static_cast<int>(c.status);
  for (unsigned i = 0; i < 4; ++i) { joints12[3*i] = c.target.legs[i].hip; joints12[3*i+1] = c.target.legs[i].upper; joints12[3*i+2] = c.target.legs[i].lower;
    contacts12[3*i] = c.contactsWorldM[i].x; contacts12[3*i+1] = c.contactsWorldM[i].y; contacts12[3*i+2] = c.contactsWorldM[i].z; }
  metrics4[0] = c.maxContactDriftM; metrics4[1] = c.minJointMarginRad; metrics4[2] = c.maxCondition; metrics4[3] = c.maxIterations;
  return 0;
}
// base-frame tilted IK for one leg. out: joints[3], residual, condition, iterations, axisTilt
int g5a_ik(int leg, const double* target3, const double* normal3, const double* seed3, int requireStrip, double* joints3, double* misc4) {
  TiltedIkOptions o; o.seed = q3(seed3); o.requireNominalStrip = requireStrip != 0;
  const auto r = contactInverseKinematicsGroundNormal(static_cast<LegId>(leg), {target3[0], target3[1], target3[2]}, {normal3[0], normal3[1], normal3[2]}, o);
  joints3[0] = r.joints.hip; joints3[1] = r.joints.upper; joints3[2] = r.joints.lower; misc4[0] = r.residualM; misc4[1] = r.condition; misc4[2] = r.iterations; misc4[3] = r.contact.axisTiltRad;
  return static_cast<int>(r.status);
}
// canonical G3 low->stand contact-locked path: joints at progress in [0,1] (seeds = previous valid joints)
int g5a_stand_path(double progress, const double* seeds12, double* joints12, double* height) {
  LegJointAngles seeds[4]; for (unsigned i = 0; i < 4; ++i) seeds[i] = q3(seeds12 + 3*i);
  const auto s = sampleStandPath(canonicalStandDefinition(), progress, seeds);
  if (s.status != StandStatus::OK) return static_cast<int>(s.status);
  for (unsigned i = 0; i < 4; ++i) { joints12[3*i] = s.target.legs[i].hip; joints12[3*i+1] = s.target.legs[i].upper; joints12[3*i+2] = s.target.legs[i].lower; }
  *height = s.bodyHeightM; return 0;
}
struct Loop { AttitudeMonitor monitor; BodyStabilizer stabilizer; };
void* g5a_loop_new(const double* policy5, const double* cfg9, unsigned delaySteps, int* ok) {
  Loop* l = new Loop();
  AttitudePolicy p; p.maxAgeS = policy5[0]; p.maxAccuracyRad = policy5[1]; p.minAccuracyStatus = static_cast<uint8_t>(policy5[2]); p.normTolerance = policy5[3]; p.maxPlausibleTiltRad = policy5[4];
  StabilizerConfig c; c.integralGainPerS = cfg9[0]; c.maxCorrectionRad = cfg9[1]; c.maxRateRadS = cfg9[2]; c.deadbandRad = cfg9[3]; c.sampleDtS = cfg9[4]; c.loopDelaySteps = delaySteps;
  c.holdBeforeFaultS = cfg9[5]; c.rollReferenceRad = cfg9[6]; c.pitchReferenceRad = cfg9[7];
  *ok = l->monitor.configure(p) && l->stabilizer.configure(c);
  return l;
}
void g5a_loop_free(void* h) { delete static_cast<Loop*>(h); }
int g5a_loop_enable(void* h, double now) { return static_cast<Loop*>(h)->stabilizer.enable(now); }
void g5a_loop_disable(void* h) { static_cast<Loop*>(h)->stabilizer.disable(); }
void g5a_loop_reset(void* h) { static_cast<Loop*>(h)->stabilizer.resetFault(); }
// snapshot[8] = w x y z stamp sequence accuracyRad accuracyStatus; out[4] = roll_cmd pitch_cmd state saturated; returns attitude status
int g5a_loop_step(void* h, const double* s8, double now, double* out5) {
  Loop* l = static_cast<Loop*>(h); ImuSnapshot s; s.w = s8[0]; s.x = s8[1]; s.y = s8[2]; s.z = s8[3]; s.stampS = s8[4]; s.sequence = static_cast<uint32_t>(s8[5]); s.accuracyRad = s8[6]; s.accuracyStatus = static_cast<uint8_t>(s8[7]);
  Attitude a; bool isNew = false; const auto st = l->monitor.evaluate(s, now, a, &isNew);
  const auto o = l->stabilizer.step(st, a, isNew, now);
  out5[0] = o.rollCorrectionRad; out5[1] = o.pitchCorrectionRad; out5[2] = static_cast<double>(o.state); out5[3] = o.saturated; out5[4] = a.rollRad;
  return static_cast<int>(st);
}
double g5a_max_gain(double dt, unsigned n) { return maxStableIntegralGain(dt, n); }
int g5a_classify(double required, double limit, int provenance, int set, double margin) {
  ActuatorLimit l; l.set = set != 0; l.value = limit; l.provenance = static_cast<LimitProvenance>(provenance);
  return static_cast<int>(classifyRequirement(required, l, margin));
}
}
