// Issue #52 gait envelope sweep (2026-10-15). Production gaitCartesian +
// solveGait (point-foot IK, G4 WALK), steady cycle sampled at 1 kHz. For every
// (stride, period, lift, duty): peak joint velocity / acceleration / jerk,
// per-joint target range, margin to the HISTORICAL gen-1 limits (reconstructed
// from hardware evidence, NOT a live NVS read) and to the URDF limits, and the
// ST3215 RAM drive pair the reference needs with a 1.25 margin
// (GoalSpeed steps/s, ACC x100 steps/s^2 - register semantics, not measured
// physical acceleration). Host only. Output: CSV on stdout; optional
// trajectory dump (q, qd, qdd per joint) for the inverse-dynamics estimate.
#include "../../src/motion/Gait.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
using namespace matdog::motion;
namespace {
constexpr double kStepRad = 2 * 3.14159265358979323846 / 4096;
// gen-1 reconstructed (audit gen1_reconstructed_limits.json, inset 5 ticks), slot leg*3+{hip,upper,lower}.
const double kGen1[12][2] = {{-0.7838641826095626,0.7838641826095626},{-0.9142525495798421,2.110757564130642},{-1.5984079809768381,0.5905826033359719},
  {-0.7838641826095626,0.7838641826095626},{-0.9142525495798421,2.100019698615443},{-1.5800002115222105,0.581378718608658},
  {-0.7838641826095626,0.7715923363064775},{-0.9142525495798421,2.136835237524698},{-1.5922720578252956,0.5905826033359719},
  {-0.7838641826095626,0.782330201821677},{-0.9142525495798421,2.12916533358527},{-1.59073807703741,0.5859806609723149}};
double slot(const LegJointAngles& q, unsigned j) { return j == 0 ? q.hip : j == 1 ? q.upper : q.lower; }
}
int main(int argc, char** argv) {
  const char* dump = argc > 1 ? argv[1] : nullptr;  // "stride,T,lift,duty" -> trajectory dump
  double ds = 0, dT = 0, dl = 0, dd = 0;
  if (dump && std::sscanf(dump, "%lf,%lf,%lf,%lf", &ds, &dT, &dl, &dd) != 4) return 2;
  if (!dump) std::printf("stride_mm,period_s,lift_mm,duty,v_equiv_mm_s,valid,vmax_rad_s,amax_rad_s2,jmax_rad_s3,step_20ms_rad,gen1_margin_rad,urdf_margin_rad,req_goal_speed,req_acc,range_min,range_max\n");
  for (double stride : {.03, .04, .05, .06}) for (double T : {2.4, 2.7, 3.0, 3.5, 4.0, 7.0}) for (double lift : {.010, .012, .015}) for (double duty : {.75, .78, .80}) {
    if (dump && (std::fabs(stride * 1000 - ds) > 1e-6 || std::fabs(T - dT) > 1e-6 || std::fabs(lift * 1000 - dl) > 1e-6 || std::fabs(duty - dd) > 1e-6)) continue;
    GaitParameters g; g.type = GaitType::WALK; g.heightM = .15; g.liftM = lift; g.duty = duty; g.advanceXM = stride;
    double vmax = 0, amax = 0, jmax = 0, step = 0, gm = 1e9, um = 1e9, lo[12], hi[12], pa[12], pq[12];
    for (unsigned i = 0; i < 12; ++i) { lo[i] = 1e9; hi[i] = -1e9; }
    bool valid = true, have = false; const unsigned N = static_cast<unsigned>(std::lround(T * 1000));
    LocomotionFrame prev; double q20[12]; bool have20 = false;
    for (unsigned n = 0; n <= N; ++n) {
      const double t = n / 1000.0;
      auto f = solveGait(g, gaitCartesian(g, 2 + t / T, 1 / T, 0), have ? &prev : nullptr);
      if (!f.target.valid) { valid = false; break; }
      for (unsigned i = 0; i < 12; ++i) {
        const double q = slot(f.target.legs[i / 3], i % 3), v = slot(f.velocityRadS[i / 3], i % 3), a = slot(f.accelerationRadS2[i / 3], i % 3);
        vmax = std::max(vmax, std::fabs(v)); amax = std::max(amax, std::fabs(a));
        if (have) jmax = std::max(jmax, std::fabs(a - pa[i]) * 1000);
        pa[i] = a; pq[i] = q; lo[i] = std::min(lo[i], q); hi[i] = std::max(hi[i], q);
        gm = std::min(gm, std::min(q - kGen1[i][0], kGen1[i][1] - q));
        if (n % 20 == 0 && have20) step = std::max(step, std::fabs(q - q20[i]));
        if (dump) std::printf("%u,%.4f,%.9f,%.9f,%.9f\n", i, t, q, v, a);
      }
      if (n % 20 == 0) { for (unsigned i = 0; i < 12; ++i) q20[i] = pq[i]; have20 = true; }
      prev = f; have = true;
    }
    if (dump) continue;
    // URDF revolute limits (matt_robodog_rev00.urdf): hip +-0.7854, upper -0.9163..2.1380, lower -1.6057..0.6545.
    const double ul[3][2] = {{-0.785398163397, 0.785398163397}, {-0.916297857297, 2.138028333693}, {-1.605702911835, 0.654498469498}};
    for (unsigned i = 0; i < 12 && valid; ++i) um = std::min(um, std::min(lo[i] - ul[i % 3][0], ul[i % 3][1] - hi[i]));
    std::printf("%.0f,%.1f,%.0f,%.2f,%.3f,%d,%.4f,%.3f,%.1f,%.5f,%.4f,%.4f,%.0f,%.0f,", stride * 1000, T, lift * 1000, duty, stride * 1000 / T, valid,
                vmax, amax, jmax, step, valid ? gm : -1, valid ? um : -1, std::ceil(1.25 * vmax / kStepRad), std::ceil(1.25 * amax / (100 * kStepRad)));
    for (unsigned i = 0; i < 12; ++i) std::printf("%s%.6f", i ? ";" : "", lo[i]);
    std::printf(",");
    for (unsigned i = 0; i < 12; ++i) std::printf("%s%.6f", i ? ";" : "", hi[i]);
    std::printf("\n");
  }
  return 0;
}
