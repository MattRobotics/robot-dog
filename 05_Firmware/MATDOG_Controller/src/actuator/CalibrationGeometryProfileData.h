// GENERATED FILE - DO NOT EDIT BY HAND.
//
// Produced by 06_Software/Matdog_Core/calibration/matdog_calibration_geometry_export.py
// from the canonical Geometry Compiler V5 bundle
//   2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4
//
// Every value below is COPIED from that bundle and converted to integer
// micro-radians. No geometry is computed here and none is computed on the
// device: the Controller verifies provenance and executes prevalidated
// plan primitives only. Regenerate with the exporter; never patch a value.
//
// max rounding error introduced by the micro-radian conversion: 4.695e-07 rad
//   (the compiler's own bisection resolution is 1.0e-4 rad)

#ifndef MATDOG_ACTUATOR_CALIBRATION_GEOMETRY_PROFILE_DATA_H
#define MATDOG_ACTUATOR_CALIBRATION_GEOMETRY_PROFILE_DATA_H

#include "CalibrationGeometryProfile.h"

namespace matdog {
namespace actuator {
namespace geometry_data {

constexpr char kSchemaVersion[] = "matdog.calibration_geometry_profile.bootstrap.v1";
constexpr char kBundleId[] = "2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4";

// The immutable inputs the canonical V5 run was gated on. The Controller
// refuses to act on a profile whose provenance it cannot match.
constexpr GeometryProvenance kProvenance = {
    "3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59",
    "60fff604eae0857c7c61f115bbe3922949ababdd827fab61c74e1673336c39e1",
    "de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e",
    "67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139",
    "e5cb2a4c33082c59c6f5d381f90fcc19680cc899e326d13c9c9129932bde5d08",
    "574dfd6e4cf88034655eea6b5f3505da06a4b2fc9cb198ac34cdd0398315b234",
};

constexpr uint8_t kJointCount = 12;
constexpr GeometryJointRecord kJoints[kJointCount] = {
    // lf_hip_joint  unit M22  bus 13
    {{calibration::Leg::LF, calibration::JointKind::HIP, "M22"}, 13, 1, -785398, 785398, 789216, 2048, 0},
    // lf_lower_leg_joint  unit M33  bus 11
    {{calibration::Leg::LF, calibration::JointKind::LOWER, "M33"}, 11, 1, -1605703, 654498, 666293, 2049, 1},
    // lf_upper_leg_joint  unit ELR01  bus 12
    {{calibration::Leg::LF, calibration::JointKind::UPPER, "ELR01"}, 12, 1, -916298, 2138028, 909821, 2048, 0},
    // lh_hip_joint  unit M43  bus 43
    {{calibration::Leg::LH, calibration::JointKind::HIP, "M43"}, 43, -1, -785398, 785398, 788057, 2049, 1},
    // lh_lower_leg_joint  unit M41  bus 41
    {{calibration::Leg::LH, calibration::JointKind::LOWER, "M41"}, 41, 1, -1605703, 654498, 666293, 2049, 1},
    // lh_upper_leg_joint  unit M42  bus 42
    {{calibration::Leg::LH, calibration::JointKind::UPPER, "M42"}, 42, 1, -916298, 2138028, 909821, 2048, 0},
    // rf_hip_joint  unit NEW01  bus 23
    {{calibration::Leg::RF, calibration::JointKind::HIP, "NEW01"}, 23, 1, -785398, 785398, 789216, 2047, -1},
    // rf_lower_leg_joint  unit NEW03  bus 21
    {{calibration::Leg::RF, calibration::JointKind::LOWER, "NEW03"}, 21, -1, -1605703, 654498, 666293, 2048, 0},
    // rf_upper_leg_joint  unit ELR03  bus 22
    {{calibration::Leg::RF, calibration::JointKind::UPPER, "ELR03"}, 22, -1, -916298, 2138028, 909821, 2049, 1},
    // rh_hip_joint  unit NEW06  bus 33
    {{calibration::Leg::RH, calibration::JointKind::HIP, "NEW06"}, 33, -1, -785398, 785398, 788057, 2047, -1},
    // rh_lower_leg_joint  unit NEW05  bus 31
    {{calibration::Leg::RH, calibration::JointKind::LOWER, "NEW05"}, 31, -1, -1605703, 654498, 666293, 2049, 1},
    // rh_upper_leg_joint  unit ELR02  bus 32
    {{calibration::Leg::RH, calibration::JointKind::UPPER, "ELR02"}, 32, -1, -916298, 2138028, 909821, 2049, 1},
};

constexpr uint8_t kEndpointCount = 24;
constexpr GeometryEndpointRecord kEndpoints[kEndpointCount] = {
    // lf_hip_joint:max  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::LF, calibration::JointKind::HIP, calibration::ContactSide::MAX_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, 789284, 789216, 785398, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // lf_hip_joint:min  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  NOT_NEEDED  UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::LF, calibration::JointKind::HIP, calibration::ContactSide::MIN_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD, -803056, -802988, -785398, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // lf_lower_leg_joint:max  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::LF, calibration::JointKind::LOWER, calibration::ContactSide::MAX_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, 666361, 666293, 654498, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // lf_lower_leg_joint:min  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  FEASIBLE_1DOF_PLAN_FOUND  UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD  [PATH_OBSTRUCTION_BRACKET]
    {calibration::Leg::LF, calibration::JointKind::LOWER, calibration::ContactSide::MIN_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND, ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD, -1606998, -1470787, -1605703, true, calibration::Leg::LF, calibration::JointKind::UPPER, 1119920},
    // lf_upper_leg_joint:max  EXECUTABLE_URDF_DOMAIN  FEASIBLE_1DOF_PLAN_FOUND  PASS  [PATH_OBSTRUCTION_BRACKET]
    {calibration::Leg::LF, calibration::JointKind::UPPER, calibration::ContactSide::MAX_SIDE, TargetDomain::EXECUTABLE_URDF_DOMAIN, ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND, ClearancePolicyResult::PASS, 2127120, 1278983, 2138028, true, calibration::Leg::LH, calibration::JointKind::UPPER, 610865},
    // lf_upper_leg_joint:min  EXECUTABLE_URDF_DOMAIN  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::LF, calibration::JointKind::UPPER, calibration::ContactSide::MIN_SIDE, TargetDomain::EXECUTABLE_URDF_DOMAIN, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, -909889, -909821, -916298, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // lh_hip_joint:max  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::LH, calibration::JointKind::HIP, calibration::ContactSide::MAX_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, 788125, 788057, 785398, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // lh_hip_joint:min  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  NOT_NEEDED  UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::LH, calibration::JointKind::HIP, calibration::ContactSide::MIN_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD, -803056, -802988, -785398, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // lh_lower_leg_joint:max  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::LH, calibration::JointKind::LOWER, calibration::ContactSide::MAX_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, 666361, 666293, 654498, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // lh_lower_leg_joint:min  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  FEASIBLE_1DOF_PLAN_FOUND  UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD  [PATH_OBSTRUCTION_BRACKET]
    {calibration::Leg::LH, calibration::JointKind::LOWER, calibration::ContactSide::MIN_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND, ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD, -1606998, -1470787, -1605703, true, calibration::Leg::LH, calibration::JointKind::UPPER, 1628974},
    // lh_upper_leg_joint:max  EXECUTABLE_URDF_DOMAIN  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::LH, calibration::JointKind::UPPER, calibration::ContactSide::MAX_SIDE, TargetDomain::EXECUTABLE_URDF_DOMAIN, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, 2127120, 2127052, 2138028, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // lh_upper_leg_joint:min  EXECUTABLE_URDF_DOMAIN  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::LH, calibration::JointKind::UPPER, calibration::ContactSide::MIN_SIDE, TargetDomain::EXECUTABLE_URDF_DOMAIN, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, -909889, -909821, -916298, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // rf_hip_joint:max  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  NOT_NEEDED  UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::RF, calibration::JointKind::HIP, calibration::ContactSide::MAX_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD, 803056, 802988, 785398, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // rf_hip_joint:min  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::RF, calibration::JointKind::HIP, calibration::ContactSide::MIN_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, -789284, -789216, -785398, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // rf_lower_leg_joint:max  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::RF, calibration::JointKind::LOWER, calibration::ContactSide::MAX_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, 666361, 666293, 654498, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // rf_lower_leg_joint:min  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  FEASIBLE_1DOF_PLAN_FOUND  UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD  [PATH_OBSTRUCTION_BRACKET]
    {calibration::Leg::RF, calibration::JointKind::LOWER, calibration::ContactSide::MIN_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND, ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD, -1606998, -1470787, -1605703, true, calibration::Leg::RF, calibration::JointKind::UPPER, 1119920},
    // rf_upper_leg_joint:max  EXECUTABLE_URDF_DOMAIN  FEASIBLE_1DOF_PLAN_FOUND  PASS  [PATH_OBSTRUCTION_BRACKET]
    {calibration::Leg::RF, calibration::JointKind::UPPER, calibration::ContactSide::MAX_SIDE, TargetDomain::EXECUTABLE_URDF_DOMAIN, ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND, ClearancePolicyResult::PASS, 2127120, 1278983, 2138028, true, calibration::Leg::RH, calibration::JointKind::UPPER, 610865},
    // rf_upper_leg_joint:min  EXECUTABLE_URDF_DOMAIN  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::RF, calibration::JointKind::UPPER, calibration::ContactSide::MIN_SIDE, TargetDomain::EXECUTABLE_URDF_DOMAIN, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, -909889, -909821, -916298, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // rh_hip_joint:max  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  NOT_NEEDED  UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::RH, calibration::JointKind::HIP, calibration::ContactSide::MAX_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD, 803056, 802988, 785398, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // rh_hip_joint:min  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::RH, calibration::JointKind::HIP, calibration::ContactSide::MIN_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, -788125, -788057, -785398, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // rh_lower_leg_joint:max  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::RH, calibration::JointKind::LOWER, calibration::ContactSide::MAX_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, 666361, 666293, 654498, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // rh_lower_leg_joint:min  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS  FEASIBLE_1DOF_PLAN_FOUND  UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD  [PATH_OBSTRUCTION_BRACKET]
    {calibration::Leg::RH, calibration::JointKind::LOWER, calibration::ContactSide::MIN_SIDE, TargetDomain::DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS, ParkingOutcome::FEASIBLE_1DOF_PLAN_FOUND, ClearancePolicyResult::UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD, -1606998, -1470787, -1605703, true, calibration::Leg::RH, calibration::JointKind::UPPER, 1628974},
    // rh_upper_leg_joint:max  EXECUTABLE_URDF_DOMAIN  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::RH, calibration::JointKind::UPPER, calibration::ContactSide::MAX_SIDE, TargetDomain::EXECUTABLE_URDF_DOMAIN, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, 2127120, 2127052, 2138028, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
    // rh_upper_leg_joint:min  EXECUTABLE_URDF_DOMAIN  NOT_NEEDED  PASS  [GEOMETRIC_CONTACT_BRACKET]
    {calibration::Leg::RH, calibration::JointKind::UPPER, calibration::ContactSide::MIN_SIDE, TargetDomain::EXECUTABLE_URDF_DOMAIN, ParkingOutcome::NOT_NEEDED, ClearancePolicyResult::PASS, -909889, -909821, -916298, false, calibration::Leg::LF, calibration::JointKind::HIP, 0},
};

}  // namespace geometry_data
}  // namespace actuator
}  // namespace matdog

#endif  // MATDOG_ACTUATOR_CALIBRATION_GEOMETRY_PROFILE_DATA_H
