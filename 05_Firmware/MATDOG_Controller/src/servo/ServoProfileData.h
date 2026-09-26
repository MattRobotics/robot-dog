// GENERATED FILE - DO NOT EDIT BY HAND.
//
// Produced by 06_Software/Matdog_Core/config/matdog_servo_profile_export.py
// from 06_Software/Matdog_Core/config/MATDOG_ST3215_C018_V1.yaml
//   sha256 70aee51244d35e1b08afc086f8b4a1479af1ba7daba2a254f2cc9740a630342a
//
// The 20 persistent-profile register values are NOT retyped here; they are
// copied from the reviewed YAML. Regenerate with the exporter; a hand-patched
// value is caught by the static audit.

#ifndef MATDOG_SERVO_SERVO_PROFILE_DATA_H
#define MATDOG_SERVO_SERVO_PROFILE_DATA_H

#include "ServoProfile.h"

namespace matdog {
namespace servo {
namespace profile_data {

constexpr char kProfileId[] = "MATDOG_C018_V1";
constexpr char kFrozenAt[] = "2026-08-27";
constexpr char kSourceSha256[] = "70aee51244d35e1b08afc086f8b4a1479af1ba7daba2a254f2cc9740a630342a";

// The persistent EEPROM contract: a unit either matches all twenty or it is
// not provisioned. Sorted by address so the verifier reads the bus in order.
constexpr uint8_t kPersistentRegisterCount = 20;
constexpr ProfileRegister kPersistentRegisters[kPersistentRegisterCount] = {
    {0x09, 2, 0, "MinAngle"},
    {0x0B, 2, 4095, "MaxAngle"},
    {0x0D, 1, 70, "MaxTemperature"},
    {0x0E, 1, 140, "MaxVoltage"},
    {0x0F, 1, 40, "MinVoltage"},
    {0x10, 2, 1000, "MaxTorque"},
    {0x15, 1, 32, "P"},
    {0x16, 1, 32, "D"},
    {0x17, 1, 0, "I"},
    {0x18, 2, 16, "MinStartupForce"},
    {0x1A, 1, 1, "CWDead"},
    {0x1B, 1, 1, "CCWDead"},
    {0x1C, 2, 310, "ProtectionCurrent"},
    {0x21, 1, 0, "Mode"},
    {0x22, 1, 20, "ProtectionTorque"},
    {0x23, 1, 200, "ProtectionTime"},
    {0x24, 1, 80, "OverloadTorque"},
    {0x25, 1, 10, "SpeedClosedLoopP"},
    {0x26, 1, 200, "OverCurrentProtectionTime"},
    {0x27, 1, 200, "VelocityClosedLoopI"},
};

// Servo invariants. Checked, but NOT part of the twenty-register delta set -
// they are properties of the unit rather than of the provisioning contract.
constexpr ServoInvariants kInvariants = {
    0x03, 777,   // model word
    0x1F, 0,     // PositionOffset, int16 LE two's complement
    0x06, 0,      // BaudRate - verification only, never written
    2048, 1,  // physical raw centre and its acceptance band
};

}  // namespace profile_data
}  // namespace servo
}  // namespace matdog

#endif  // MATDOG_SERVO_SERVO_PROFILE_DATA_H
