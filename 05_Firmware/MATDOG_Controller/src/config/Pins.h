#ifndef MATDOG_CONFIG_PINS_H
#define MATDOG_CONFIG_PINS_H

// Central GPIO ownership map for MATDOG Controller V0.1.
//
// Source: MATDOG_CONTROLLER_V01_INTEGRATION_HANDOFF_REV3_USB_ONLY_2026-09-15.md
// section 8/18. Do not remap without new hardware evidence — every pin below
// is either hardware-validated (BNO085, ST3215 bus, USB) or physically wired
// per the handoff (DALY, LED ring).
//
// Every constant must be used by exactly one module. scripts/static_audit.py
// greps this file and flags duplicate GPIO numbers as a build-breaking error.

namespace matdog {
namespace pins {

// --- BNO085 IMU (SPI) ---------------------------------------------------
constexpr int kBnoSck  = 1;   // SCL/SCK
constexpr int kBnoMiso = 2;   // SDA/MISO
constexpr int kBnoInt  = 42;  // INT
constexpr int kBnoCs   = 41;  // CS
constexpr int kBnoMosi = 40;  // DI/MOSI
constexpr int kBnoRst  = 39;  // RST
constexpr int kBnoPs0  = 38;  // P0/PS0/WAKE

// --- DALY BMS via XY-017 RS485 (UART) -----------------------------------
constexpr int kDalyTx = 15;
constexpr int kDalyRx = 16;

// --- ST3215 / Seeed servo bus (UART) ------------------------------------
constexpr int kServoTx = 17;
constexpr int kServoRx = 18;

// --- Native USB CDC (owned by the toolchain, listed for completeness) ---
constexpr int kUsbDMinus = 19;
constexpr int kUsbDPlus  = 20;

// --- WS2812B LED ring -----------------------------------------------------
constexpr int kLedRingDin = 47;

}  // namespace pins
}  // namespace matdog

#endif  // MATDOG_CONFIG_PINS_H
