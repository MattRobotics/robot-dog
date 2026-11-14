// Closed-loop co-simulation bridge: the production SimpleMotion on the scripted
// rig of test_simple_motion.cpp (sources of this worktree, compiled unchanged),
// fed every release with the MuJoCo twin's encoders, speeds, loads and IMU
// instead of the commanded pose. One line per message on stdin/stdout:
//   STAND <h>          stand from the rig's PRONE start with ideal tracking
//                      (the twin starts at that STANDING pose); prints a record
//   SELECT <mode>      selectWalkMode;  WALK <preset>; DRIVE <vx> [<vy> <yaw>] (the joystick's
//                      three axes, m/s and rad/s); WALKSTOP; STOP
//   HEIGHT <h>         a STAND command while STANDING (HEIGHT_CHANGE), tracked by the twin
//   STOPFRONT <s> [p]  STOP at the first front-leg swing progress >= p (0.5) after <s> s of walk
//   TICK <raw x12> <speed x12> <load x12> <qw qx qy qz>
//                      feedback in the rig's RAW map (raw = 2048 + q / tick),
//                      speed step/s and load per mille signed; then one release
//   HEADER             the record columns (t_s first, absent from the records)
// Replies: "OK <refusal or ->" to commands, "R <record>" (rig_record.h columns
// from phase on) to STAND and TICK. Simulation only: no hardware is involved.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#define main simple_motion_main
#include "tests/test_simple_motion.cpp"
#undef main
#pragma GCC diagnostic pop
#include "rig_record.h"
#include <iostream>

namespace {
uint16_t signMagnitude(int v, unsigned bit) { return uint16_t(std::abs(v)) | (v < 0 ? (1u << bit) : 0u); }
}  // namespace

int main() {
  std::ios::sync_with_stdio(false);
  Rig r;
  double walkTime = 0, stopFrontAfter = -1, stopFrontProgress = .5;
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    std::string cmd;
    in >> cmd;
    if (cmd == "HEADER") {
      std::cout << rig_record::header() << std::endl;
    } else if (cmd == "STAND") {
      double h; in >> h;
      const char* refused = r.m.stand(h, r.p.now);
      if (!refused && !r.reachStand()) refused = "NOT_REACHED";
      std::cout << "R " << rig_record::fields(r) << " " << (refused ? refused : "-") << std::endl;
    } else if (cmd == "SELECT" || cmd == "WALK" || cmd == "DRIVE" || cmd == "WALKSTOP" || cmd == "STOP" || cmd == "STOPFRONT" || cmd == "HEIGHT") {
      const char* refused = nullptr;
      if (cmd == "HEIGHT") { double h; in >> h; refused = r.m.stand(h, r.p.now); }
      else if (cmd == "SELECT") {
        std::string m; in >> m; simple::WalkMode mode;
        refused = simple::parseWalkMode(m.c_str(), mode) ? r.m.selectWalkMode(mode) : "BAD_MODE";
      } else if (cmd == "WALK") { unsigned preset; in >> preset; refused = r.m.walk(preset, r.p.now); walkTime = 0; }
      else if (cmd == "DRIVE") { double vx, vy = 0, yaw = 0; in >> vx; in >> vy >> yaw; refused = r.m.drive(vx, vy, yaw, r.p.now); }
      else if (cmd == "WALKSTOP") r.m.walkStop();
      else if (cmd == "STOP") r.m.stop();
      else { in >> stopFrontAfter; if (!(in >> stopFrontProgress)) stopFrontProgress = .5; }
      std::cout << "OK " << (refused ? refused : "-") << std::endl;
    } else if (cmd == "TICK") {
      int raw[12], speed[12], load[12];
      for (int& v : raw) in >> v;
      for (int& v : speed) in >> v;
      for (int& v : load) in >> v;
      for (double& v : r.in.quaternion) in >> v;
      for (unsigned i = 0; i < 12; ++i) {
        auto& o = r.p.observed[i];
        o.raw = uint16_t(raw[i]);
        o.q = (raw[i] - 2048) * simple::kTickRad;
        o.speed = signMagnitude(speed[i], 15);
        o.load = signMagnitude(load[i], 10);
      }
      if (stopFrontAfter >= 0 && r.m.state() == simple::State::WALKING && walkTime >= stopFrontAfter) {
        const auto& w = simple::StopReferenceProbe::walkNominal(r.m);
        for (unsigned l = 0; l < 2; ++l)
          if (!w.cartesian.phases[l].stance && w.cartesian.phases[l].swingProgress >= stopFrontProgress) { r.m.stop(); stopFrontAfter = -1; break; }
      }
      r.tick(false);
      if (r.m.state() == simple::State::WALKING) walkTime += .02;
      std::cout << "R " << rig_record::fields(r) << std::endl;
    } else {
      std::cout << "OK UNKNOWN_COMMAND" << std::endl;
    }
  }
}
