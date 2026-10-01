// Host-test model of ST3215 joints under position control, for the staged
// calibration endpoint search (ContactProbeEngine / FullLegCalibrationExecutor
// suites). NO HARDWARE: every figure here is a synthetic model chosen to
// reproduce the behaviours observed on LF_UPPER on 2026-09-29, never a
// measurement:
//   - speed ~1.2x the commanded profile (40 -> ~49 ticks/s was observed);
//   - a position-controlled joint settles `deadband_ticks` (4) short of its
//     goal - the "stall 4-5 ticks short of any goal" that fooled the old probe;
//   - hard stops (optionally beyond the modelled contact), friction plateaus
//     that hold until the target error exceeds a breakaway, or for a time;
//   - current: idle / moving / pressing (grows with target error while blocked,
//     up to a torque-limit saturation).
#ifndef MATDOG_TESTS_KINEMATIC_SERVO_SIM_H
#define MATDOG_TESTS_KINEMATIC_SERVO_SIM_H

#include <cmath>
#include <cstdint>
#include <vector>

#include "../../src/actuator/ActuatorRuntime.h"
#include "../../src/actuator/MotionDeadman.h"

namespace simk {

struct SimJoint {
  double pos = 2048;
  int goal = 2048;
  bool torque = false;
  matdog::actuator::MotionProfile profile = matdog::actuator::MotionProfile::BOUNDED_DEFAULT;

  // --- model ---
  double search_speed_tps = 190;   // CALIBRATION_SEARCH (160 commanded)
  double bounded_speed_tps = 49;   // BOUNDED_DEFAULT (40 commanded)
  int deadband_ticks = 4;
  bool has_stop_low = false, has_stop_high = false;
  double stop_low = 0, stop_high = 4095;
  // Friction plateau at plateau_tick: while the target error is <= breakaway
  // the joint cannot cross it; optionally it only holds for hold_ms.
  bool plateau = false;
  double plateau_tick = 0;
  int plateau_breakaway = 0;
  uint32_t plateau_hold_ms = 0;  // 0 = holds until breakaway
  uint32_t plateau_stuck_since = 0;
  bool plateau_released = false;
  int current_idle = 4, current_moving = 30, press_gain = 6;
  // Pressing current saturates where the RAM TorqueLimit caps the drive
  // (synthetic figure). A coarse contact scout leaves its target up to one
  // 64-tick step past a stop; the LF V25 hardware scouts at TorqueLimit 500
  // never reached the 200-raw hard-current abort, so the model's cap stays
  // below it. -1 = no cap (a jam that keeps drawing more current).
  int press_current_max = 150;
  int current_override = -1;  // >= 0 forces the reported current
  int speed_override = -1;    // >= 0 forces the reported speed register
  int temperature = 35;
  // RAM registers the LF V25 calibrator read back on every observation.
  // torque_limit defaults to the calibration value 500 so the single-joint
  // probe rigs (which never write it) keep their meaning; the 24-contact
  // sequence rigs start it at the EEPROM default 1000 and let the executor's
  // own TorqueLimit write lower it.
  int torque_limit = 500;
  int status = 0;             // servo error flags (register 65)
  bool read_fails = false;    // the servo stops answering (telemetry reads fail)
  // Speed-register noise on a TIME basis (keyed to 20 ms slots, so it cannot
  // alias with the engine's 20 ms consumption cadence): every
  // `speed_noise_every`-th slot reads `speed_noise_raw` above the true speed.
  int speed_noise_every = 0;
  int speed_noise_raw = 0;
  // A slow zone [slow_lo, slow_hi] (raw) crossed at `slow_speed_tps` - a
  // temporary slowdown, not a stop.
  bool slow_zone = false;
  double slow_lo = 0, slow_hi = 0, slow_speed_tps = 0;
  // Encoder dither while pressing a stop (synthetic): on every other 20 ms
  // slot the reported position reads this many ticks back off the stop, the
  // speed register still 0. It defeats the contact detector's per-sample
  // progress rule while the joint stays kinematically still - the case the
  // LF V25 kinematic-plateau path (confirm_kinematic_plateau) exists for.
  int stop_dither_ticks = 0;

  // --- outputs ---
  double velocity = 0;
  bool pressing = false;
  double lo_seen = 4096, hi_seen = -1;

  void advance(uint32_t now_ms, uint32_t dt_ms) {
    pressing = false;
    if (!torque) {
      velocity = 0;
      return;
    }
    const double err = goal - pos;
    const double mag = std::fabs(err);
    if (mag <= deadband_ticks) {
      velocity = 0;
      return;
    }
    const double speed = profile == matdog::actuator::MotionProfile::CALIBRATION_SEARCH
                             ? search_speed_tps
                             : bounded_speed_tps;
    const double eff_speed = (slow_zone && pos >= slow_lo && pos <= slow_hi) ? slow_speed_tps : speed;
    double move = eff_speed * dt_ms / 1000.0;
    if (move > mag - deadband_ticks) move = mag - deadband_ticks;
    const double dir = err > 0 ? 1.0 : -1.0;
    double next = pos + dir * move;
    if (has_stop_low && next < stop_low) {
      next = stop_low;
      if (dir < 0) pressing = true;
    }
    if (has_stop_high && next > stop_high) {
      next = stop_high;
      if (dir > 0) pressing = true;
    }
    if (plateau && !plateau_released) {
      const bool crossing = (dir > 0 && pos <= plateau_tick && next > plateau_tick) ||
                            (dir < 0 && pos >= plateau_tick && next < plateau_tick) ||
                            std::fabs(pos - plateau_tick) < 1e-9;
      if (crossing && mag <= plateau_breakaway) {
        if (plateau_stuck_since == 0) plateau_stuck_since = now_ms == 0 ? 1 : now_ms;
        if (plateau_hold_ms == 0 || now_ms - plateau_stuck_since < plateau_hold_ms) {
          next = plateau_tick;
          pressing = true;
        } else {
          plateau_released = true;
        }
      }
    }
    velocity = std::fabs(next - pos) * 1000.0 / dt_ms;
    pos = next;
    if (pos < lo_seen) lo_seen = pos;
    if (pos > hi_seen) hi_seen = pos;
  }

  int position() const { return static_cast<int>(std::lround(pos)); }
  int current() const {
    if (current_override >= 0) return current_override;
    if (!torque) return 0;
    if (pressing) {
      const int drawn = current_idle + press_gain * std::abs(goal - position());
      return (press_current_max >= 0 && drawn > press_current_max) ? press_current_max : drawn;
    }
    return velocity > 0.5 ? current_moving : current_idle;
  }

  matdog::actuator::TelemetrySample sample(uint32_t now_ms) const {
    matdog::actuator::TelemetrySample s{};
    if (read_fails) {
      s.read_ok = false;
      s.sampled_at_ms = now_ms;
      return s;
    }
    s.read_ok = true;
    s.sampled_at_ms = now_ms;
    s.present_position = position();
    if (stop_dither_ticks > 0 && pressing && (now_ms / 20) % 2 == 1) {
      const bool on_low = has_stop_low && pos <= stop_low + 0.5;
      s.present_position += on_low ? stop_dither_ticks : -stop_dither_ticks;
    }
    s.torque_enable = torque ? 1 : 0;
    const bool noisy = speed_noise_every > 0 && (now_ms / 20) % speed_noise_every == 0;
    s.present_speed = speed_override >= 0
                          ? speed_override
                          : static_cast<int32_t>(std::lround(velocity)) + (noisy ? speed_noise_raw : 0);
    s.present_current = current();
    s.present_temperature = temperature;
    s.goal_position = goal;
    s.torque_limit = torque_limit;
    s.servo_status = status;
    return s;
  }
};

struct GoalWrite {
  uint8_t bus;
  uint16_t tick;
  matdog::actuator::MotionProfile profile;
};

// Real ActuatorBackend: every accepted write drives the modelled joint.
class SimBackend : public matdog::actuator::ActuatorBackend {
 public:
  matdog::actuator::BackendWriteOutcome enableTorque(uint8_t bus_id) override {
    ++torque_writes[bus_id];
    SimJoint& j = joint[bus_id];
    j.torque = true;
    // A real ST3215 drives to whatever its GoalPosition register already
    // holds when torque comes on (the hazard V25's prepare_motor() prime
    // exists for). The single-joint probe rigs keep the older convenience of
    // holding the present pose; the sequence rigs model the real servo.
    if (torque_enable_holds_pose) j.goal = j.position();
    return matdog::actuator::BackendWriteOutcome::VERIFIED_APPLIED;
  }
  matdog::actuator::BackendWriteOutcome writeCalibrationTorqueLimit(uint8_t bus_id) override {
    ++torque_limit_writes[bus_id];
    joint[bus_id].torque_limit = 500;  // ServoBus::kReviewedRamTorqueLimit
    return matdog::actuator::BackendWriteOutcome::VERIFIED_APPLIED;
  }
  matdog::actuator::BackendWriteOutcome writeGoalPosition(
      uint8_t bus_id, uint16_t target_tick, matdog::actuator::MotionProfile profile) override {
    writes.push_back(GoalWrite{bus_id, target_tick, profile});
    SimJoint& j = joint[bus_id];
    j.goal = target_tick;
    j.profile = profile;
    return matdog::actuator::BackendWriteOutcome::VERIFIED_APPLIED;
  }
  void advance(uint32_t now_ms, uint32_t dt_ms) {
    for (SimJoint& j : joint) j.advance(now_ms, dt_ms);
  }
  int goalWritesTo(uint8_t bus) const {
    int n = 0;
    for (const GoalWrite& w : writes) n += (w.bus == bus) ? 1 : 0;
    return n;
  }

  SimJoint joint[256];
  int torque_writes[256] = {0};
  int torque_limit_writes[256] = {0};
  bool torque_enable_holds_pose = true;
  std::vector<GoalWrite> writes;
};

}  // namespace simk

#endif  // MATDOG_TESTS_KINEMATIC_SERVO_SIM_H
