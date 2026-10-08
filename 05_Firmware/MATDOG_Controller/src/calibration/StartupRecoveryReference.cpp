#include "StartupRecoveryReference.h"
#include "FullLegCalibrationPlan.h"
#include "../actuator/CalibrationGeometryProfileData.h"
#include "../actuator/CalibrationSequencePlanData.h"
#include <string.h>

namespace matdog { namespace calibration {
const StartupReferenceRow* startupReference(uint8_t bus) {
  for (const auto& r : kStartupReference) if (r.bus == bus) return &r;
  return nullptr;
}
bool startupReferenceMatches(const actuator::CalibrationGeometryProfile& g) {
  if (!g.bound() || !g.provenanceMatches(actuator::geometry_data::kProvenance) ||
      g.provenanceTag() != kStartupReferenceGeometry) return false;
  for (const auto& r : kStartupReference) {
    const Leg leg = static_cast<Leg>(r.bus/10-1);
    const JointKind kind = r.bus%10==1 ? JointKind::LOWER : r.bus%10==2 ? JointKind::UPPER : JointKind::HIP;
    JointIdentity identity{};identity.leg=leg;identity.joint=kind;setPhysicalUnit(&identity,r.unit);
    const auto* record=g.findJoint(identity);
    if (record==nullptr || record->bus_id!=r.bus || record->encoder_direction!=r.direction) return false;
  }
  return true;
}
bool makeStartupRecoveryRequest(const actuator::CalibrationGeometryProfile& g, FullLegCalibrationRequest* out) {
  if (out==nullptr || !startupReferenceMatches(g)) return false;
  FullLegCalibrationRequest r{};
  r.leg=Leg::RF;r.recovery_only=true;r.startup_recovery=true;
  r.torque_limit=kFullLegCalibrationTorqueLimit;r.repeatability_tolerance_ticks=kFullLegRepeatabilityToleranceTicks;
  for (const auto& data : kStartupReference) {
    const Leg leg=static_cast<Leg>(data.bus/10-1);
    const JointKind kind=data.bus%10==1 ? JointKind::LOWER : data.bus%10==2 ? JointKind::UPPER : JointKind::HIP;
    const uint8_t index=static_cast<uint8_t>(leg)*3+static_cast<uint8_t>(kind);
    FullLegJoint joint{};joint.identity.leg=leg;joint.identity.joint=kind;
    setPhysicalUnit(&joint.identity,data.unit);joint.bus_id=data.bus;joint.q0_tick=data.q0;
    r.population[index]=joint;
    if (leg==Leg::RF) {
      const uint8_t k=static_cast<uint8_t>(kind);
      r.joint[k]=joint;r.direction[k]=data.direction;
      const auto* record=g.findJoint(joint.identity);
      r.urdf_lower[k]=record->urdf_lower;r.urdf_upper[k]=record->urdf_upper;
    }
    if (data.bus==32) r.park=joint;
  }
  r.population_count=12;
  const auto* plan=actuator::findSequenceLeg(actuator::sequence_plan_data::kPlan,Leg::RF);
  if (plan==nullptr || !plan->geometry_validated || !plan->has_rear_park ||
      plan->park_leg!=Leg::RH || plan->park_joint!=JointKind::UPPER ||
      plan->upper_for_lower!=1570796 || plan->park_target!=610865) return false;
  r.has_rear_park=true;r.upper_for_lower_urad=plan->upper_for_lower;
  r.upper_for_lower_tick=2106-1024;r.park_target_urad=plan->park_target;r.park_target_tick=2058-398;
  // No contact corridors: this program can only prime at present and return
  // the three fixed joints to reference zero. No search/parking move exists.
  *out=r;return true;
}
bool startupPositionInBand(uint8_t bus, int32_t position, CalibrationPhase phase) {
  const auto* ref=startupReference(bus);
  if (!ref || position<0 || position>=4096) return false;
  const int32_t q=(position-ref->q0)*ref->direction;
  int32_t lo=-10,hi=10;
  const bool lower=phase==CalibrationPhase::PREFLIGHT || phase==CalibrationPhase::PARKING ||
                   phase==CalibrationPhase::RETURN_LOWER_HELD;
  const bool upper=phase==CalibrationPhase::RETURN_UPPER;
  const bool rear=phase==CalibrationPhase::RESTORE_PARKING;
  if (lower && bus==21) hi=367;
  if (lower && bus==22) { lo=1014;hi=1034; }
  if ((lower || upper) && bus==32) { lo=388;hi=408; }
  if (upper && bus==22) hi=1034;
  if (rear && bus==32) hi=408;
  return q>=lo && q<=hi;
}
} }
