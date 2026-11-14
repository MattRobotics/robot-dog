// Derived from verification-artifacts/MATDOG_1_0_2_DEV6_ACC220_OPERATIONAL_20261110/
// mujoco_replay/export_walk_replay.cpp: the same production SimpleMotion on the
// same scripted rig of test_simple_motion.cpp (ideal tracking, level IMU), NOT a
// motor or ground model. For the MuJoCo replays it also records:
// - every release from STANDING 150 (before the walk command); the rows of the
//   walk proper carry the t_s of the delivered CSVs (t_s < 0 before them);
// - the sent RAW integers (rig map q = (raw - 2048) * kTickRad), torque state,
//   STOP verification and plan, the law's stance phases, body pose, foot
//   references and applied level correction of the release;
// - STAND HOLD poses at 50..150 mm and STOP scenarios at chosen instants.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main simple_motion_main
#include "tests/test_simple_motion.cpp"
#undef main
#pragma GCC diagnostic pop
#include "rig_record.h"
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>

namespace {
using simple::StopReferenceProbe;
struct Csv {
  std::ofstream f;
  std::vector<std::string> prefix;  // releases before the walk proper, timed at flush
  explicit Csv(const std::string& path) : f(path) { f << rig_record::header() << "\n"; }
  void before(Rig& r) { if (!r.p.sent.empty()) prefix.push_back(rig_record::fields(r)); }
  void row(Rig& r, double t) {
    if (r.p.sent.empty()) return;
    if (!prefix.empty()) {
      for (size_t k = 0; k < prefix.size(); ++k) f << -(double(prefix.size() - k) * .02) << "," << prefix[k] << "\n";
      prefix.clear();
    }
    f.precision(17);
    f << t << "," << rig_record::fields(r) << "\n";
  }
};
using Hook = std::function<void(Rig&, double)>;
// The rig's IMU is level whatever the commanded body attitude, so the support
// fit (stance anchors + encoder FK + gravity) has a neutral equilibrium and the
// applied level correction drifts. With imuFollows the IMU reports the attitude
// the commanded correction gives the body on flat ground (Ry(pitch) Rx(roll),
// the convention of servo_plant.inc tilt()): a flat-ground walk with ideal
// tracking. Variant for the MuJoCo replays only; the delivered CSVs use the
// level IMU.
bool imuFollows = false;
void beforeTick(Rig& r) {
  if (!imuFollows) return;
  const double hr = StopReferenceProbe::roll(r.m) / 2, hp = StopReferenceProbe::pitch(r.m) / 2;
  r.in.quaternion[0] = std::cos(hr) * std::cos(hp); r.in.quaternion[1] = std::sin(hr) * std::cos(hp);
  r.in.quaternion[2] = std::cos(hr) * std::sin(hp); r.in.quaternion[3] = -std::sin(hr) * std::sin(hp);
}
// export_walk_replay.cpp run(), with startIn() inlined and recorded from STANDING.
void run(const std::string& dir, const char* name, WalkMode m, double joy,
         std::initializer_list<std::pair<double, WalkMode>> changes, double stopAt, double endAt,
         const Hook& hook = nullptr) {
  Rig r;
  Csv c(dir + "/" + name + ".csv");
  assert(!r.m.stand(.15, r.p.now)); assert(r.reachStand());
  c.before(r);
  r.m.selectWalkMode(m);
  if (joy > 0) assert(!r.m.drive(joy * topSpeed(m), 0, 0, r.p.now)); else assert(!r.m.walk(2, r.p.now));
  for (unsigned n = 0; n < 6000 && !settled(r, m, (joy > 0 ? joy : 1) * topSpeed(m)); ++n) {
    if (joy > 0 && n % 12 == 0) r.m.drive(joy * topSpeed(m), 0, 0, r.p.now);
    beforeTick(r); r.tick(); assert(r.m.state() != simple::State::FAULT); c.before(r);
  }
  assert(r.m.state() == simple::State::WALKING && settled(r, m, (joy > 0 ? joy : 1) * topSpeed(m)));
  for (unsigned n = 0; n < unsigned(simple::walkModeSpec(m).periodS * 1.5 / .02); ++n) {
    if (joy > 0 && n % 12 == 0) r.m.drive(joy * topSpeed(m), 0, 0, r.p.now);
    beforeTick(r); r.tick(); assert(r.m.state() == simple::State::WALKING); c.before(r);
  }
  double t = 0; auto next = changes.begin();
  for (unsigned n = 0; t < endAt; ++n, t += .02) {
    if (next != changes.end() && t >= next->first) { r.m.selectWalkMode(next->second); ++next; }
    if (joy > 0 && n % 12 == 0 && r.m.state() == simple::State::WALKING && !r.m.stopActive())
      r.m.drive(joy * topSpeed(r.m.selectedWalkMode()), 0, 0, r.p.now);
    if (stopAt > 0 && std::abs(t - stopAt) < .01) r.m.stop();
    if (hook) hook(r, t);
    beforeTick(r); r.tick(); assert(r.m.state() != simple::State::FAULT); c.row(r, t);
  }
  std::printf("%s.csv: final %s/%s law %s\n", name, simple::name(r.m.state()), r.m.phase(),
              simple::walkModeSpec(r.m.walkMode()).name);
}
// STOP at the first front-leg swing progress >= `progress` after `after` s
// (0.5: mid-swing; 0.85: late swing, where the front UPPER nears its kHeight bound).
Hook frontSwingStop(double after, double progress) {
  auto done = std::make_shared<bool>(false);
  return [=](Rig& r, double t) {
    if (*done || t < after || r.m.state() != simple::State::WALKING) return;
    const auto& w = StopReferenceProbe::walkNominal(r.m);
    for (unsigned l = 0; l < 2; ++l)
      if (!w.cartesian.phases[l].stance && w.cartesian.phases[l].swingProgress >= progress) {
        std::printf("  STOP at t=%.2f s, %s swing progress %.3f\n", t, l ? "RF" : "LF", w.cartesian.phases[l].swingProgress);
        r.m.stop(); *done = true; return;
      }
  };
}
// Joystick on the selected mode (walk started at 25 %): stick 100 % from t = 0,
// 50 % from 10 s, 100 % from 16 s, released at 24 s. Release is walkStop(), as
// MotionService maps a zero stick (WALK STOP: C2 ramp of the law, then
// recentring with four contacts); drive() itself refuses a zero velocity.
Hook joystickSteps() {
  return [](Rig& r, double t) {
    if (r.m.state() != simple::State::WALKING || r.m.stopActive()) return;
    const unsigned n = unsigned(std::lround(t / .02));  // release index: t accumulates rounding
    const double stick = n < 500 ? 1. : n < 800 ? .5 : n < 1200 ? 1. : 0.;
    if (stick > 0 && n % 12 == 0) r.m.drive(stick * topSpeed(r.m.selectedWalkMode()), 0, 0, r.p.now);
    else if (n == 1200) r.m.walkStop();
  };
}
void standPoses(const std::string& dir) {
  std::ofstream f(dir + "/stand_poses.csv");
  f << "h_nominal_m,base_link_m,refused";
  for (unsigned i = 0; i < 12; ++i) f << ",raw" << i;
  for (unsigned l = 0; l < 4; ++l) f << ",contact" << l << "_x,contact" << l << "_y,contact" << l << "_z";
  f << "\n";
  f.precision(17);
  const auto& stand = motion::canonicalStandDefinition();
  for (unsigned mm = 50; mm <= 150; mm += 10) {
    const double h = mm / 1000.;
    Rig r;
    const char* refused = r.m.stand(h, r.p.now);
    if (!refused && !r.reachStand()) refused = "NOT_REACHED";
    for (unsigned n = 0; n < 50 && !refused; ++n) r.tick();
    f << h << "," << simple::baseLinkHeightFromNominalM(h) << "," << (refused ? refused : "");
    for (unsigned i = 0; i < 12; ++i) f << "," << (r.p.sent.empty() ? 0 : r.p.sent.back()[i]);
    for (unsigned l = 0; l < 4; ++l)
      f << "," << stand.contactsWorldM[l].x << "," << stand.contactsWorldM[l].y << "," << stand.contactsWorldM[l].z;
    f << "\n";
    std::printf("stand %3u mm: %s phase %s\n", mm, refused ? refused : "STANDING", r.m.phase());
  }
}
}  // namespace

int main(int argc, char** argv) {
  const std::string dir = argc > 1 ? argv[1] : ".";
  standPoses(dir);
  // The four delivered scenarios (export_walk_replay.cpp), recorded from STANDING.
  run(dir, "preset_slow_standard_fast_slow", WalkMode::SLOW, 0, {{2, WalkMode::STANDARD}, {14, WalkMode::FAST}, {24, WalkMode::SLOW}}, 0, 40);
  run(dir, "joystick_full_slow_fast_slow", WalkMode::SLOW, 1., {{2, WalkMode::FAST}, {20, WalkMode::SLOW}}, 0, 36);
  run(dir, "preset_fast_stop_midswing", WalkMode::FAST, 0, {}, 6.13, 12);
  run(dir, "preset_slow_to_fast_stop_during_change", WalkMode::SLOW, 0, {{2, WalkMode::FAST}}, 5.0, 12);
  // Derived STOP scenarios (MuJoCo handoff section 5): front leg mid-swing per mode,
  // STANDARD steady at eight instants of one cycle, inside SLOW -> STANDARD.
  run(dir, "stop_front_midswing_slow", WalkMode::SLOW, 0, {}, 0, 10, frontSwingStop(2, .5));
  run(dir, "stop_front_midswing_standard", WalkMode::STANDARD, 0, {}, 0, 10, frontSwingStop(2, .5));
  run(dir, "stop_front_midswing_fast", WalkMode::FAST, 0, {}, 0, 10, frontSwingStop(2, .5));
  run(dir, "stop_front_lateswing_slow", WalkMode::SLOW, 0, {}, 0, 10, frontSwingStop(2, .85));
  run(dir, "stop_front_lateswing_standard", WalkMode::STANDARD, 0, {}, 0, 10, frontSwingStop(2, .85));
  run(dir, "stop_front_lateswing_fast", WalkMode::FAST, 0, {}, 0, 10, frontSwingStop(2, .85));
  for (unsigned k = 0; k < 8; ++k) {
    const std::string name = "stop_standard_k" + std::to_string(k);  // on the 20 ms release grid
    run(dir, name.c_str(), WalkMode::STANDARD, 0, {}, std::round((2 + k * simple::walkModeSpec(WalkMode::STANDARD).periodS / 8) / .02) * .02, 8);
  }
  for (unsigned k = 0; k < 7; ++k) {
    const std::string name = "stop_slow_to_standard_t" + std::to_string(3 + k);
    run(dir, name.c_str(), WalkMode::SLOW, 0, {{2, WalkMode::STANDARD}}, 3 + k, 14);
  }
  // Joystick (handoff section 5.5): stride ramps up and down, then return to centre.
  run(dir, "joystick_steps_release_standard", WalkMode::STANDARD, .25, {}, 0, 34, joystickSteps());
  run(dir, "joystick_steps_release_fast", WalkMode::FAST, .25, {}, 0, 34, joystickSteps());
  // The delivered walks again, with the IMU following the commanded attitude.
  imuFollows = true;
  run(dir, "imufollows_preset_slow_standard_fast_slow", WalkMode::SLOW, 0, {{2, WalkMode::STANDARD}, {14, WalkMode::FAST}, {24, WalkMode::SLOW}}, 0, 40);
  run(dir, "imufollows_joystick_full_slow_fast_slow", WalkMode::SLOW, 1., {{2, WalkMode::FAST}, {20, WalkMode::SLOW}}, 0, 36);
  run(dir, "imufollows_stop_front_midswing_fast", WalkMode::FAST, 0, {}, 0, 10, frontSwingStop(2, .5));
  // Long steady walks on the top preset (dev.4 ground comparison: SLOW, WALK100).
  run(dir, "imufollows_walk_slow", WalkMode::SLOW, 0, {}, 0, 40);
  run(dir, "imufollows_walk_standard", WalkMode::STANDARD, 0, {}, 0, 30);
  run(dir, "imufollows_walk_fast", WalkMode::FAST, 0, {}, 0, 30);
  imuFollows = false;
}
