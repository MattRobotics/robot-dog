#include "StandTrajectory.h"
#include "StandReferenceData.h"
#include <cmath>
#include <limits>

namespace matdog { namespace motion {
namespace {
bool validDefinition(const StandDefinition& d) {
  if (!std::isfinite(d.lowBodyHeightM) || !std::isfinite(d.standBodyHeightM) ||
      d.lowBodyHeightM <= 0 || d.lowBodyHeightM >= d.standBodyHeightM || d.samples < 2) return false;
  for (unsigned i=0; i<4; ++i) {
    const auto p = d.contactsWorldM[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.z != 0.0 ||
        forwardKinematics(static_cast<LegId>(i), d.standSeed[i]).status != KinematicsStatus::OK) return false;
  }
  return true;
}
double distance(Vector3 a, Vector3 b) {
  return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z);
}
StandSample solve(const StandDefinition& d, double height, const LegJointAngles seeds[4]) {
  StandSample result;
  result.bodyHeightM = height;
  for (unsigned i=0; i<4; ++i) {
    ContactIkOptions options;
    options.kinematics.seed = seeds[i];
    const Vector3 target = {d.contactsWorldM[i].x, d.contactsWorldM[i].y, -height};
    const auto ik = contactInverseKinematics(static_cast<LegId>(i), target, options);
    if (ik.status != ContactStatus::OK) {
      // Never leak a partially solved 12-joint target frame on failure.
      StandSample failed;
      failed.status = StandStatus::CONTACT_FAILURE;
      failed.contactStatus = ik.status;
      failed.failedLeg = static_cast<uint8_t>(i);
      return failed;
    }
    result.target.legs[i] = ik.joints;
    result.branches[i] = ik.branch;
    result.contactsWorldM[i] = {ik.contact.referenceM.x, ik.contact.referenceM.y,
                                ik.contact.referenceM.z + height};
    result.maxContactResidualM = std::fmax(result.maxContactResidualM,
                                           distance(result.contactsWorldM[i], d.contactsWorldM[i]));
    const auto* model = legModel(static_cast<LegId>(i));
    const double q[3] = {ik.joints.hip,ik.joints.upper,ik.joints.lower};
    double margins[3];
    for (unsigned j=0; j<3; ++j)
      margins[j] = std::fmin(q[j]-model->limits[j].lower,model->limits[j].upper-q[j]);
    result.jointLimitMarginRad[i] = {margins[0],margins[1],margins[2]};
  }
  result.status = StandStatus::OK;
  result.contactStatus = ContactStatus::OK;
  result.target.valid = true;
  return result;
}
}
const StandDefinition& canonicalStandDefinition() { return generated::kStand; }
StandSample generateStandTarget(const StandDefinition& d) {
  if (!validDefinition(d)) {
    StandSample result; result.status = StandStatus::INVALID_DEFINITION; return result;
  }
  return solve(d,d.standBodyHeightM,d.standSeed);
}
StandStatus ContactLockedStand::initialize(const StandDefinition& d) {
  ready_ = false;
  previous_ = {};
  metrics_ = {};
  const auto stand = generateStandTarget(d);
  if (stand.status != StandStatus::OK) return stand.status;
  const auto low = solve(d,d.lowBodyHeightM,stand.target.legs);
  if (low.status != StandStatus::OK) return low.status;
  definition_ = d;
  previous_ = low;
  metrics_.minJointLimitMarginRad = std::numeric_limits<double>::infinity();
  ready_ = true;
  return StandStatus::OK;
}
StandSample ContactLockedStand::next() {
  StandSample result;
  if (!ready_) return result;
  if (metrics_.emittedSamples == definition_.samples) {
    result.status = StandStatus::COMPLETE;
    return result;
  }
  const unsigned index = metrics_.emittedSamples;
  const double fraction = static_cast<double>(index)/(definition_.samples-1);
  const double height = definition_.lowBodyHeightM + fraction *
                        (definition_.standBodyHeightM-definition_.lowBodyHeightM);
  result = solve(definition_,height,previous_.target.legs);
  if (result.status != StandStatus::OK) { ready_ = false; return result; }
  result.target.sequence = index;
  for (unsigned i=0; i<4; ++i) {
    const auto q = result.target.legs[i];
    const auto prev = previous_.target.legs[i];
    const LegJointAngles delta = index == 0 ? LegJointAngles{} :
                               LegJointAngles{q.hip-prev.hip,q.upper-prev.upper,q.lower-prev.lower};
    result.jointDeltaRad[i] = delta;
    auto& maximum = metrics_.maxAbsJointDeltaRad[i];
    maximum.hip = std::fmax(maximum.hip,std::abs(delta.hip));
    maximum.upper = std::fmax(maximum.upper,std::abs(delta.upper));
    maximum.lower = std::fmax(maximum.lower,std::abs(delta.lower));
    metrics_.maxJointDeltaRad = std::fmax(metrics_.maxJointDeltaRad,
                               std::fmax(maximum.hip,std::fmax(maximum.upper,maximum.lower)));
    if (index == 0) firstContacts_[i] = result.contactsWorldM[i];
    result.maxContactDriftM = std::fmax(result.maxContactDriftM,
                                       distance(result.contactsWorldM[i], firstContacts_[i]));
    if (index != 0 && (result.branches[i].hip != previous_.branches[i].hip ||
                      result.branches[i].elbow != previous_.branches[i].elbow)) ++metrics_.branchChanges[i];
    const auto margins = result.jointLimitMarginRad[i];
    metrics_.minJointLimitMarginRad = std::fmin(metrics_.minJointLimitMarginRad,
                                     std::fmin(margins.hip,std::fmin(margins.upper,margins.lower)));
  }
  metrics_.maxContactResidualM = std::fmax(metrics_.maxContactResidualM,result.maxContactResidualM);
  metrics_.maxContactDriftM = std::fmax(metrics_.maxContactDriftM,result.maxContactDriftM);
  ++metrics_.emittedSamples;
  previous_ = result;
  return result;
}
} }
