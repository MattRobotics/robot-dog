// CR2-C CURRENT-INSTALLATION Q0 EVIDENCE SNAPSHOT - DO NOT EDIT VALUES.
//
// Source package:
//   09_Logs/Validation_Reports/Calibration_Q0_CR2C_2026-09-27/
// Source q0_status.txt SHA256:
//   37290f77a5a44db5cacbe145dc1014e83d7d79296bc83e5cd181c74e18548b7f
//
// This file intentionally freezes the Geometry V5 provenance that the
// measurement was taken under. Do NOT replace kSourceGeometry with an alias to
// geometry_data::kProvenance: doing so would silently rebind old measurements
// to a future URDF/mesh/profile revision.
//
// These are CANDIDATE evidence records, not operational transforms. Loading
// this file does not accept, promote or admit anything.

#ifndef MATDOG_ACTUATOR_CALIBRATION_Q0_EVIDENCE_DATA_H
#define MATDOG_ACTUATOR_CALIBRATION_Q0_EVIDENCE_DATA_H

#include "CalibrationQ0Promotion.h"

namespace matdog {
namespace actuator {
namespace q0_evidence_data {

constexpr char kPackageId[] = "Calibration_Q0_CR2C_2026-09-27";
constexpr char kQ0StatusSha256[] =
    "37290f77a5a44db5cacbe145dc1014e83d7d79296bc83e5cd181c74e18548b7f";
constexpr uint32_t kCaptureSessionId = 1;
constexpr uint8_t kSampleCount = 9;
constexpr uint16_t kStabilitySpreadTicks = 0;

constexpr GeometryProvenance kSourceGeometry = {
    "3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59",
    "60fff604eae0857c7c61f115bbe3922949ababdd827fab61c74e1673336c39e1",
    "de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e",
    "67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139",
    "e5cb2a4c33082c59c6f5d381f90fcc19680cc899e326d13c9c9129932bde5d08",
    "574dfd6e4cf88034655eea6b5f3505da06a4b2fc9cb198ac34cdd0398315b234",
};

struct Q0CandidateRecord {
  calibration::Leg leg;
  calibration::JointKind joint;
  const char* physical_unit;
  uint8_t bus_id;
  uint16_t q0_tick;
};

constexpr uint8_t kRecordCount = calibration::kLegServoSlotCount;
constexpr Q0CandidateRecord kRecords[kRecordCount] = {
    {calibration::Leg::LF, calibration::JointKind::LOWER, "M33",   11, 2087},
    {calibration::Leg::LF, calibration::JointKind::UPPER, "ELR01", 12, 2100},
    {calibration::Leg::LF, calibration::JointKind::HIP,   "M22",   13, 1996},

    {calibration::Leg::RF, calibration::JointKind::LOWER, "NEW03", 21, 1985},
    {calibration::Leg::RF, calibration::JointKind::UPPER, "ELR03", 22, 2092},
    {calibration::Leg::RF, calibration::JointKind::HIP,   "NEW01", 23, 2030},

    {calibration::Leg::RH, calibration::JointKind::LOWER, "NEW05", 31, 2034},
    {calibration::Leg::RH, calibration::JointKind::UPPER, "ELR02", 32, 2042},
    {calibration::Leg::RH, calibration::JointKind::HIP,   "NEW06", 33, 2081},

    {calibration::Leg::LH, calibration::JointKind::LOWER, "M41",   41, 2073},
    {calibration::Leg::LH, calibration::JointKind::UPPER, "M42",   42, 2089},
    {calibration::Leg::LH, calibration::JointKind::HIP,   "M43",   43, 2035},
};

}  // namespace q0_evidence_data
}  // namespace actuator
}  // namespace matdog

#endif
