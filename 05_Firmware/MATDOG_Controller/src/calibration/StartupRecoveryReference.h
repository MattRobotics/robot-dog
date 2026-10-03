#ifndef MATDOG_CALIBRATION_STARTUP_RECOVERY_REFERENCE_H
#define MATDOG_CALIBRATION_STARTUP_RECOVERY_REFERENCE_H

#include "FullLegCalibrationExecutor.h"

namespace matdog { namespace calibration {

// Immutable DATA, never admitted to JointTransformTable or restored as motion
// authority. Verified capture 1, LF session 1 / RF session 2, same POWERON.
// q0_promoted.json SHA256 cf14a2b6df6511db63c719e2325f8ae53021f416ff0b113a323acab30a787881
// original log SHA256 a9129abc1200a6bdd915334d84618c98e588bb76eada36fab983a9801085ea88
constexpr actuator::GeometryProvenanceTag kStartupReferenceGeometry = 0x3713f4ddc43b204eULL;
struct StartupReferenceRow { uint8_t bus; uint16_t q0; int8_t direction; const char* unit; };
constexpr StartupReferenceRow kStartupReference[12] = {
  {11,2102,-1,"M33"},{12,2107,1,"ELR01"},{13,1975,-1,"M22"},
  {21,1997,1,"NEW03"},{22,2106,-1,"ELR03"},{23,2024,-1,"NEW01"},
  {31,2036,1,"NEW05"},{32,2058,-1,"ELR02"},{33,2086,1,"NEW06"},
  {41,2079,-1,"M41"},{42,2076,1,"M42"},{43,2026,1,"M43"}};

const StartupReferenceRow* startupReference(uint8_t bus);
bool startupReferenceMatches(const actuator::CalibrationGeometryProfile& geometry);
bool makeStartupRecoveryRequest(const actuator::CalibrationGeometryProfile& geometry,
                                FullLegCalibrationRequest* request);
// Absolute URDF tick boxes covered by the pinned startup CAD certificate.
bool startupPositionInBand(uint8_t bus, int32_t position, CalibrationPhase phase);

} }
#endif
