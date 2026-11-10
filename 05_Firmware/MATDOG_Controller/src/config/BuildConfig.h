#ifndef MATDOG_CONFIG_BUILD_CONFIG_H
#define MATDOG_CONFIG_BUILD_CONFIG_H

#include <stdint.h>

#include "HardwareProfile.h"

// Injected by scripts/build.sh via --build-property compiler.cpp.extra_flags
// as the short git SHA (+ "-dirty" if the tree had uncommitted changes at
// build time). Falls back to "unknown" for an IDE build that skips the script.
#ifndef MATDOG_BUILD_ID
#define MATDOG_BUILD_ID "unknown"
#endif

// Full development provenance is separate from the short OTA protocol ID.
// An IDE build without the build script cannot claim clean Git provenance.
#ifndef MATDOG_GIT_SHA
#define MATDOG_GIT_SHA "unknown"
#endif
#ifndef MATDOG_GIT_DIRTY
#define MATDOG_GIT_DIRTY 1
#endif
#ifndef MATDOG_BUILD_UTC
#define MATDOG_BUILD_UTC "unknown"
#endif

// ---------------------------------------------------------------------------
// ACTIVE HARDWARE PROFILE — the single G3 authorization gate.
// ---------------------------------------------------------------------------
// The source default is USB_ONLY and must stay USB_ONLY until the powered
// no-motion hardware validation gate (G3) is explicitly authorized.
// scripts/static_audit.py FAILS the build if this default is anything else,
// so a ROBOT_POWERED image can never be produced by an unreviewed edit or
// by accident.
//
// Switching profiles is deliberately a ONE-symbol change — either this
// #define, or an explicit -DMATDOG_ACTIVE_HARDWARE_PROFILE=... build
// override for a G3 session. Nothing else in the firmware needs editing:
// every rail expectation below, and every module ExpectedState derived from
// them, follows automatically (config/HardwareProfile.h).
#ifndef MATDOG_ACTIVE_HARDWARE_PROFILE
#define MATDOG_ACTIVE_HARDWARE_PROFILE ::matdog::config::HardwareProfile::USB_ONLY
#endif

// Capability only: no authority at boot or from this macro.
#ifndef MATDOG_SUPERVISED_VALIDATION
#define MATDOG_SUPERVISED_VALIDATION 0
#endif
namespace matdog {
namespace build {
// No compile-time feature fence (ADR-006, 2026-10-17): STAND, WALK STANDARD
// and WALK FAST are exposed whenever the Controller's runtime checks admit them
// (live limits, certified boxes, feedback, power, presence). Hardware
// qualification is tracked outside the firmware and reported as such.

constexpr const char* kFirmwareName    = "MATDOG Controller";
constexpr const char* kFirmwareDescription = "Embedded Motion & Robot Control Firmware";

// Release candidate: 1.0.0 is assigned only after hardware qualification.
// SIMPLE 1.0.0-rc.3 (f838f677) remains the historical predecessor.
// rc.6 (OP1, 367e2b32, not installed): rc.5 + IK cost reduction + prompt 503.
// rc.7 (5d208c36, not installed): rc.6 + compensate IK tolerance 1e-6 m + WALK STOP without slack wait.
// rc.8: rc.7 + bounded HTTPD wait while Motion is engaged + dashboard 503 = paused telemetry.
// dev.5: dev.4 + held PRONE terminal state (STAND without STOP) + dashboard source freshness.
// dev.14: dev.13 + reference secant on the sampling clock (STOP and its WALK/HOLD pre-check).
// dev.15: dev.14 + JOYSTICK BASE_VELOCITY (vx, vy, yaw_rate) on the SE(2) WALK law, tread
//   fillet contact and certified kSteer, STAND after STOP in PRONE/route segments,
//   lighter IK, IMU slew in physical time, Reset vista +X, timing/IMU diagnostics.
// dev.16: dev.15 + PRONE HEIGHT->LIFT handover at 52 mm and contextual STOP instead of
//   SAFE_OFF when a PRONE segment cannot complete with support available; timing
//   diagnostics (overruns, lateness, outside work; max_work/first_miss keys fixed);
//   IMU correction reused on alternate settled WALK/HOLD releases (T1).
// dev.17: dev.16 + status pages without Motion slack, read lanes past TCP
//   recovery, fixed dashboard slots and joystick anchor, Twin after a reload
//   with torque ON, WALK STANDARD directly after a verified STOP, Motion
//   Cockpit and Twin view cube; pre-flash: dashboard joystick commands
//   admitted in page-session order only. Released unchanged as Stable v1.0.1.
// 1.0.2-dev.1: v1.0.1 + Motion release budget (A1 position-only gait
//   sampling, A2 exact analytic IK savings, T2 look-ahead nominal gait on
//   alternate WALK releases, T1b retarget-tolerant IMU correction reuse);
//   gait law, geometry, boxes, calibration, STOP and drive profile unchanged.
//   Identity proposed by the motion-perf-next branch, to be agreed before any
//   flash; the installed firmware stays 1.0.1-dev.17 (APP 14e055a0...).
// 1.0.2-dev.2: 1.0.2-dev.1 + WALK STANDARD v2 (gait redesign): COM-centred
//   crawl, duty .80, trapezoidal swing velocity, 50/75/100 mm per 5 s cycle
//   (10/15/20 mm/s) with the qualified drive {1000,50} and the kHeight box;
//   eased walk start clock; IMU tilt envelopes recomputed; JOYSTICK law,
//   geometry, boxes, calibration, STOP unchanged. Offline candidate.
// 1.0.2-dev.3: 1.0.2-dev.2 + pre-flash fixes: late T2 resync anchored to its
//   grid point, StopPlan search start (same plans), dynamic joystick (a mainly
//   lateral/yaw demand never reverses; the page holds the stick through late
//   polls and through a reversal). Gait law, envelopes, geometry, boxes,
//   calibration, STOP and drive unchanged. Offline candidate.
// 1.0.2-dev.4: 1.0.2-dev.3 + H1-RCA fix: the HEIGHT/PRONE_HEIGHT unwind of a
//   level correction accepted by a steered WALK_SETTLE in the fillet edge
//   model is solved in that model (dev.3 hardware: NATIVE_TARGET_NOT_REACHED
//   on the joystick restart after a reversal). Everything else unchanged.
// 1.0.2-dev.5: 1.0.2-dev.4 (installed, VH on fixture and floor 08/11) + a
//   refused entry pose recoverable without power cycle (READY again on the
//   ACQUIRE evidence, no movement); fore/aft joystick on the WALK STANDARD v2
//   law (up to 20 mm/s forward, 10 mm/s backward, touch jitter never steers);
//   in-place HTTP response reset (loop stack worst path -7 KB); refusal and
//   entry diagnostics. Steered law, boxes, envelopes, geometry, calibration,
//   STOP and drive {1000,50} unchanged. Offline candidate.
// 1.0.2-dev.6 (consolidated 2026-11-09, supersedes the 2c53ecf dev.6 build):
//   one continuous joystick law (fore/aft and steered regimes hand over
//   without WALK STOP; only a fore/aft reversal stops); WALK100 STOP proof
//   with the T2 midpoints (kWalkTilt100 re-derived, held correction on a
//   refused WALK step); first-exit cause in @MOTION DIAG. Geometry, URDF,
//   calibration, limits, STOP/SAFE_OFF and drive {1000,50} unchanged.
//   Walk-modes addendum (same version string, new build): WALK modes SLOW
//   (the law above, default at boot), STANDARD (75 mm, 3.0 s, {1500,140}) and
//   FAST (75 mm, 2.5 s, 110 mm, {1900,190}), selected by GAIT.SELECT for the
//   next walk; StopPlan braking capped at the measured 8 400 step/s^2 (SLOW
//   and STAND unchanged); joystick SLOW. STANDARD/FAST not hardware qualified.
//   Final dev.6 (same version string, new build): per-mode presets, STANDARD
//   60/75/90 mm at 3.0 s (30 mm/s, 110 mm, {1800,180}), FAST 60/70/80 mm at
//   2.5 s (32 mm/s, 105 mm, lift 6 mm, sway 1 mm, {1950,220}); IMU envelopes
//   kept by the servo response model. Technical closure (same version
//   string, new build): a walk starts only inside its own IMU envelope
//   (STANDARD recovery fault RCA); dashboard: one speed selector SLOW 20 /
//   STANDARD 25 / FAST 32 mm/s ordered after GAIT.SELECT. Installed 09/11
//   (70abcff); trace 44: its first STANDARD frame faulted SYNC_WRITE_FAILED,
//   the ServoBus drive gate refused {1800,180}. Trace-44 fix (same version
//   string, new build): the gate admits the STANDARD/FAST pairs {1800,180} and
//   {1950,220} too; WALK speed selector readable in the narrow cockpit.
//   Installed 09/11 (dd76fad); trace 45: ACC 180 written, 50 stored in
//   register 41. ACC diagnostic (same version string, new build): the
//   MAINTENANCE/idle-gated @SERVO READ also prints registers 40, 41, 85 and
//   86, read-only, as separate 1-byte reads. Installed 09/11 (975fe51): 85 =
//   50, 86 = 1 on 12/12. ACC causal test (same version string, new build):
//   @SERVO ACC BACKUP / SRAM 41 / register 85 on one servo behind a token,
//   MAINTENANCE, idle, torque OFF. Installed 10/11 (1fbee7b): register 85 is
//   the ACC ceiling, VOLATILE. Operational ACC 220 (same version string, new
//   build): register 85 set to 220 before every activation, Motion bounded by
//   the lowest value read back; WALK mode change during a walk; the joystick
//   on the selected mode, a selection with the stick held never dropped
//   (final audit). Offline candidate.
constexpr const char* kFirmwareVersion = "1.0.2-dev.6"; // Offline candidate; hardware qualification pending
constexpr const char* kBoardName       = "YD-ESP32-S3 N16R8";
constexpr const char* kBuildId         = MATDOG_BUILD_ID;
constexpr const char* kGitSha          = MATDOG_GIT_SHA;
constexpr bool kGitDirty              = MATDOG_GIT_DIRTY != 0;
constexpr const char* kBuildUtc        = MATDOG_BUILD_UTC;
constexpr const char* kMotionStack     = "MATDOG_SIMPLE_V1";
// A new source review is required for any operationally authorized image.
// There is deliberately no build override or HTTP authority flag.
constexpr bool kOperationalMotionAuthorized = false;
constexpr bool kSupervisedValidationCapable = MATDOG_SUPERVISED_VALIDATION == 1;

// The active profile and everything derived from it. kProfileExpectations
// is the value every module consults (via core::expectedStateFor*()) to
// build its ExpectedState — see core/Availability.h.
constexpr config::HardwareProfile kHardwareProfile = MATDOG_ACTIVE_HARDWARE_PROFILE;
constexpr config::ProfileExpectations kProfileExpectations =
    config::expectationsFor(kHardwareProfile);

// Profile name printed by the boot banner / @STATUS. Derived, so it can
// never contradict the rail facts below.
constexpr const char* kTestProfile = config::toString(kHardwareProfile);

// Per-rail availability under the active profile. These names are retained
// from V0.1 so no module changed shape, but they are now DERIVED values,
// not independently editable literals — flipping one in isolation (and
// thereby contradicting the other two and the profile name) is no longer
// expressible.
//
// kLedRailPowered additionally gates whether LedRing is allowed to drive
// GPIO47 at all (Session 2 hardening: anti-back-power). false means the
// NeoPixel transport is never initialized and no WS2812 frame is ever
// transmitted — see src/status/LedRing.cpp.
constexpr bool kServoPowerAvailable = kProfileExpectations.servo_power_available;
constexpr bool kBatteryAvailable    = kProfileExpectations.battery_available;
constexpr bool kLedRailPowered      = kProfileExpectations.led_rail_powered;

constexpr uint32_t kUsbSerialBaud = 115200;   // USB CDC ignores baud; kept for host-side tooling parity.
constexpr uint32_t kServoBusBaud  = 1000000;  // ST3215 / Seeed driver, hardware-proven.
constexpr uint32_t kDalyBusBaud   = 9600;     // DALY XY-017 RS485, hardware-proven.

// G3.1 USB CDC transmit policy, applied in Controller::begin(). Timeout 0
// makes HWCDC drop, instead of wait for, output no host is draining. The
// ring holds the largest single Controller::update() burst (worst census
// reply + DALY KEY write ACK and COMPLETE lines + IMU block + BMS block =
// 2674 B; 2395 B before the KEY probe), so replies are expected complete
// while a host drains the port - best effort, see VALIDATION.md § G3.1 and
// § DALY KEY.
// The core default is 256 B.
constexpr uint32_t kUsbTxTimeoutMs = 0;
// Complete bounded @TEL record (<6144 B) plus ordinary diagnostics (<2674 B).
// Timeout remains zero; telemetry is skipped if the whole record cannot fit.
constexpr uint32_t kUsbTxRingBytes = 12288;

}  // namespace build
}  // namespace matdog

#endif  // MATDOG_CONFIG_BUILD_CONFIG_H
