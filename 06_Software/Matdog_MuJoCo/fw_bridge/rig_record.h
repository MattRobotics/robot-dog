// Shared by export_sequences.cpp and cosim.cpp (include after tests/test_simple_motion.cpp):
// one record per Motion release, the columns twin.sequence() reads.
#pragma once
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace rig_record {
using simple::StopReferenceProbe;
inline const char* header() {
  static std::string h = [] {
    std::ostringstream f;
    f << "t_s,phase,state,walk_cycles,law_mode,target_mode,period_s,height_m,vx_m_s,drive_speed_step_s,drive_acc_100step_s2,"
         "stop_active,stop_verified,torque_on";
    for (unsigned i = 0; i < 12; ++i) f << ",raw" << i;
    for (unsigned l = 0; l < 4; ++l) f << ",stance" << l;
    f << ",law_valid,body_x,body_y,body_z,body_yaw";
    for (unsigned l = 0; l < 4; ++l) f << ",foot" << l << "_x,foot" << l << "_y,foot" << l << "_z";
    f << ",imu_roll,imu_pitch,imu_reason,stop_plan_s";
    for (unsigned i = 0; i < 12; ++i) f << ",stop_end" << i;
    f << ",reason";
    return f.str();
  }();
  return h.c_str();
}
// Every field but t_s, for the latest sent frame.
inline std::string fields(Rig& r) {
  std::ostringstream o;
  o.precision(17);
  unsigned on = 0;
  for (unsigned i = 0; i < 12; ++i) on += r.p.observed[i].torque == 1;
  o << r.m.phase() << "," << simple::name(r.m.state()) << "," << r.m.walkCycles() << ","
    << simple::walkModeSpec(r.m.walkMode()).name << "," << simple::walkModeSpec(r.m.walkTarget()).name << ","
    << r.m.period() << "," << r.m.walkHeightM() << "," << r.m.walkVelocityX() << "," << r.p.drive.speed << ","
    << unsigned(r.p.drive.acc) << "," << int(r.m.stopActive()) << "," << int(r.m.stopVerified()) << "," << on;
  for (unsigned i = 0; i < 12; ++i) o << "," << r.p.sent.back()[i];
  const auto& w = StopReferenceProbe::walkNominal(r.m);
  const bool law = r.m.state() == simple::State::WALKING && w.cartesian.valid;
  for (unsigned l = 0; l < 4; ++l) o << "," << (law ? int(w.cartesian.phases[l].stance) : -1);
  const auto& b = w.cartesian.body;
  o << "," << int(law) << "," << b.translationM.x << "," << b.translationM.y << "," << b.translationM.z << ","
    << std::atan2(b.rotation[1][0], b.rotation[0][0]);
  for (unsigned l = 0; l < 4; ++l) o << "," << w.cartesian.feet[l].x << "," << w.cartesian.feet[l].y << "," << w.cartesian.feet[l].z;
  // Applied level correction (support fit of the stance anchors and the IMU):
  // the sent joints solve the law's feet for headed(body, roll, pitch).
  o << "," << StopReferenceProbe::roll(r.m) << "," << StopReferenceProbe::pitch(r.m) << "," << r.m.imuReason();
  const auto& plan = StopReferenceProbe::stopPlan(r.m);
  o << "," << (r.m.stopActive() ? plan.duration : 0.0);
  for (unsigned i = 0; i < 12; ++i) o << "," << (r.m.stopActive() ? plan.end[i] : 0.0);
  o << "," << r.m.reason();
  return o.str();
}
}  // namespace rig_record
