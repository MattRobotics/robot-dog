// P0 WALK NOMINAL attribution bench (18/11/2026, offline). Production SimpleMotion of 30c894a on the shared qemu_motion
// Sim (folded entry, register 85 verified); only the bench clock in MotionTiming.h and the function probes differ.
// The fixture steps whose windows held the HW releases with a heavy NOMINAL and no compensation solve (30c894a chain):
//  B3  STANDARD, STAND 110, joystick yaw clockwise full scale 22 s (refresh 0.3 s), STOP;
//  D8  joystick yaw counter-clockwise 12 s, release; D10 yaw clockwise 12 s, release (settle tool);
//  H   WALK STANDARD preset 1, STOP, STAND 120 resume (stopped-walk RECOVERY) to STANDING;
//  E   WALK STANDARD preset 1 6 s, MODE SLOW 12 s, MODE STANDARD 12 s, WALK STOP.
// Per release: cost, the six timing parts and each probe's inclusive cost and calls in that release.
#include "motion_scenarios.h"
#include "MotionTiming.h"
#include "qprobe.h"
#include <cstring>
namespace qemu_motion {
namespace T = matdog::simple::timing;
struct Rel { uint32_t cost, part[6], idx; char phase[16]; uint32_t pc[qprobe::N]; uint16_t calls[qprobe::N]; };
// Compact per-release record (DRAM: the ESP32-S3 bench keeps only this and the heaviest releases in full).
struct Lite { uint32_t nom, cost; uint16_t sig; uint8_t walk, comp; };
static Lite gLite[5000]; static unsigned gN = 0;
static Rel gTopNom[8], gTopCost[5]; static unsigned gTopNomN = 0, gTopCostN = 0;
// Motion FP32 bench (18/11): heaviest release of each P0 class: C1 two or more compensation solves, C1P the same
// without a segment plan in the release (NOMINAL < 50k), C2 nominal gait solve + compensation solve, C3 WALK with
// more than four contact IK per gait solve (steered edge re-solves).
static Rel gClass[4]; static bool gClassN[4];
inline unsigned signature(const Rel& r) {
  return (r.calls[qprobe::STARTWALK] ? 1 : 0) | (r.calls[qprobe::HANDOVER] ? 2 : 0) | (r.calls[qprobe::ARREST] ? 4 : 0) |
         (r.calls[qprobe::SOLVEGAIT] > 1 ? 8 : 0) | (r.calls[qprobe::RETARGET] ? 16 : 0) | (r.calls[qprobe::BEGINNATIVE] ? 32 : 0) |
         (r.calls[qprobe::SOLVEGAIT] == 1 ? 64 : 0) | (r.calls[qprobe::LERP] ? 128 : 0); }
inline void keepTop(Rel* top, unsigned& n, unsigned cap, const Rel& r, bool byCost) {
  auto v = [&](const Rel& x) { return byCost ? x.cost : x.part[1]; };
  if (n < cap) { top[n++] = r; } else { unsigned lo = 0; for (unsigned i = 1; i < n; ++i) if (v(top[i]) < v(top[lo])) lo = i; if (v(r) > v(top[lo])) top[lo] = r; } }
inline void stepRec(Sim& s) {
  uint64_t c0[qprobe::N]; uint32_t n0[qprobe::N];
  for (unsigned i = 0; i < qprobe::N; ++i) { c0[i] = qprobe::acc[i].cc; n0[i] = qprobe::acc[i].calls; }
  const uint64_t c = s.cost.sum; s.tick();
  if (gN >= 5000) return; static Rel r; r.idx = gN; r.cost = uint32_t(s.cost.sum - c);
  for (unsigned i = 0; i < 6; ++i) r.part[i] = T::state.active ? 0 : T::state.working.us[i];
  std::snprintf(r.phase, sizeof r.phase, "%s", s.m.phase());
  for (unsigned i = 0; i < qprobe::N; ++i) { r.pc[i] = uint32_t(qprobe::acc[i].cc - c0[i]); r.calls[i] = uint16_t(qprobe::acc[i].calls - n0[i]); }
  gLite[gN] = Lite{r.part[1], r.cost, uint16_t(signature(r)), uint8_t(!std::strncmp(r.phase, "WALK", 4)), uint8_t(r.calls[qprobe::COMPTARGET])}; ++gN;
  keepTop(gTopNom, gTopNomN, 8, r, false); keepTop(gTopCost, gTopCostN, 5, r, true);
  { const unsigned ct = r.calls[qprobe::COMPTARGET], sg = r.calls[qprobe::SOLVEGAIT], ik = r.calls[qprobe::CIK];
    const bool in[4] = {ct >= 2, ct >= 2 && r.part[1] < 50000, sg >= 1 && ct >= 1, sg >= 1 && ik > 4 * sg};
    for (unsigned c = 0; c < 4; ++c) if (in[c] && (!gClassN[c] || r.cost > gClass[c].cost)) { gClass[c] = r; gClassN[c] = true; } }
}
inline void printRel(const char* sc, const char* why, const Rel& r) {
  std::printf("REL %-3s %-4s #%-5u %-14s cost=%lu READ=%lu NOMINAL=%lu COMPENSATE=%lu CONVERT=%lu FRAME=%lu REST=%lu |", sc, why, r.idx, r.phase,
    (unsigned long)r.cost, (unsigned long)r.part[0], (unsigned long)r.part[1], (unsigned long)r.part[2], (unsigned long)r.part[3],
    (unsigned long)r.part[4], (unsigned long)r.part[5]);
  for (unsigned i = 0; i < qprobe::N; ++i) if (r.calls[i]) std::printf(" %s=%lu/%u", qprobe::kName[i], (unsigned long)r.pc[i], unsigned(r.calls[i]));
  std::printf("\n");
}
// Summary of a scenario: the NOMINAL distribution of WALK releases and the heaviest releases with their probes.
inline void summary(const char* sc, Sim& s) {
  unsigned walkN = 0; uint64_t nomSum = 0; uint32_t nomMax = 0;
  for (unsigned k = 0; k < gN; ++k) if (gLite[k].walk) { ++walkN; nomSum += gLite[k].nom; nomMax = std::max(nomMax, gLite[k].nom); }
  std::printf("SCENARIO %-3s releases=%u walk_releases=%u nominal_mean=%lu nominal_max=%lu state=%s/%s reason=%s first_fault=%s frames=%u digest=%016llx\n",
    sc, gN, walkN, (unsigned long)(walkN ? nomSum / walkN : 0), (unsigned long)nomMax, simple::name(s.m.state()), s.m.phase(), s.m.reason(),
    s.firstFault ? s.firstFault : "-", s.p.frames, (unsigned long long)s.p.digest.h);
  for (unsigned i = 0; i < gTopNomN; ++i) printRel(sc, "NOM", gTopNom[i]);
  for (unsigned i = 0; i < gTopCostN; ++i) printRel(sc, "COST", gTopCost[i]);
  { const char* const name[4] = {"C1", "C1P", "C2", "C3"}; for (unsigned c = 0; c < 4; ++c) if (gClassN[c]) printRel(sc, name[c], gClass[c]); }
  // Every release whose NOMINAL exceeds 3x the WALK mean, counted by probe signature, and the WALK NOMINAL histogram
  // in multiples of the mean (bins <1, <2, <3, <4, <6, >=6).
  const uint32_t mean = uint32_t(walkN ? nomSum / walkN : 0), thr = 3 * mean; unsigned heavy = 0, sig[256] = {}, hist[6] = {};
  for (unsigned k = 0; k < gN; ++k) { if (gLite[k].walk && mean) { const uint32_t v = gLite[k].nom; ++hist[v < mean ? 0 : v < 2*mean ? 1 : v < 3*mean ? 2 : v < 4*mean ? 3 : v < 6*mean ? 4 : 5]; }
    if (gLite[k].nom > thr && thr) { ++heavy; ++sig[gLite[k].sig]; } }
  std::printf("HEAVY %-3s threshold=%lu releases=%u walk_hist=%u,%u,%u,%u,%u,%u", sc, (unsigned long)thr, heavy, hist[0], hist[1], hist[2], hist[3], hist[4], hist[5]);
  for (unsigned b = 0; b < 256; ++b) if (sig[b]) std::printf(" [%s%s%s%s%s%s%s%s]=%u", b & 1 ? "startWalk " : "", b & 2 ? "handover " : "", b & 4 ? "arrest " : "",
    b & 8 ? "solve>1 " : "", b & 16 ? "retarget " : "", b & 32 ? "beginNative " : "", b & 64 ? "solve1 " : "", b & 128 ? "lerp" : "", sig[b]);
  std::printf("\n");
  // Budget view (CCOUNT): releases above 280k/320k/360k/400k, and above 320k by class: WALK with a nominal solve
  // (solveGaitInto) and 0/1/2+ compensation solves, WALK without a nominal solve, other phases by compensation solves.
  unsigned over[4] = {}, cls[7] = {}; const uint32_t lim[4] = {280000, 320000, 360000, 400000};
  for (unsigned k = 0; k < gN; ++k) { const auto& x = gLite[k]; for (unsigned i = 0; i < 4; ++i) over[i] += x.cost > lim[i];
    if (x.cost > 320000) { const bool ns = x.sig & (8 | 64); const unsigned c = x.comp > 2 ? 2 : x.comp;
      ++cls[x.walk ? (ns ? c : 3) : 4 + c]; } }
  std::printf("BUDGET %-3s over280k=%u over320k=%u over360k=%u over400k=%u | >320k: walk+nominal_solve comp0=%u comp1=%u comp2+=%u walk_no_solve=%u other comp0=%u comp1=%u comp2+=%u\n",
    sc, over[0], over[1], over[2], over[3], cls[0], cls[1], cls[2], cls[3], cls[4], cls[5], cls[6]);
}
inline bool standTo(Sim& s, double h) { s.m.stand(h, gClock); for (unsigned k = 0; k < 4000; ++k) { stepRec(s); if (s.m.state() == simple::State::STANDING) return true;
  if (s.m.state() == simple::State::FAULT) return false; } return false; }
inline void hold(Sim& s, unsigned n) { for (unsigned k = 0; k < n; ++k) stepRec(s); }
inline void joystick(Sim& s, double vx, double vy, double yaw, double seconds) {
  const unsigned n = unsigned(seconds * 1e6 / simple::kPeriodUs); for (unsigned k = 0; k < n; ++k) { if (k % 15 == 0) s.m.drive(vx, vy, yaw, gClock); stepRec(s); } }
inline void release(Sim& s) { s.m.drive(0, 0, 0, gClock); for (unsigned k = 0; k < 1500; ++k) { stepRec(s); if (s.m.state() == simple::State::STANDING) break; } }
inline void stopTo(Sim& s) { s.m.stop(); for (unsigned k = 0; k < 400 && !s.m.stopVerified(); ++k) stepRec(s); }
inline void begin(Sim& s, double roll = .005, double pitch = .008) { s.begin(); s.tilt(roll, pitch); gN = 0; gTopNomN = gTopCostN = 0; for (bool& b : gClassN) b = false; T::arm(); }
// Scenario selection (Motion FP32 bench, 18/11): bit 0 B3, 1 D8, 2 H, 3 E, 4 S.
#ifndef QSCEN_MASK
#define QSCEN_MASK 31
#endif
inline void runWalk(Sim& s) {
  const auto& js = simple::joystickSpec(simple::WalkMode::STANDARD); const double yaw = js.yawRad / simple::walkModeSpec(simple::WalkMode::STANDARD).periodS;
  std::printf("START yaw_rate_max=%.6f period_us=%u\n", yaw, unsigned(simple::kPeriodUs));
  if (QSCEN_MASK & 1) { begin(s); standTo(s, .12); hold(s, 150); s.m.selectWalkMode(simple::WalkMode::STANDARD); standTo(s, .11); hold(s, 50);
    joystick(s, 0, 0, -yaw, 22); stopTo(s); hold(s, 200); summary("B3", s); }
  if (QSCEN_MASK & 2) { begin(s); standTo(s, .12); hold(s, 150); s.m.selectWalkMode(simple::WalkMode::STANDARD);
    joystick(s, 0, 0, yaw, 12); release(s); hold(s, 50); joystick(s, 0, 0, -yaw, 12); release(s); hold(s, 50); summary("D8", s); }
  if (QSCEN_MASK & 4) { begin(s); standTo(s, .12); hold(s, 150); s.m.selectWalkMode(simple::WalkMode::STANDARD); s.m.walk(1, gClock); hold(s, 300);
    stopTo(s); hold(s, 100); standTo(s, .12); hold(s, 100); summary("H", s); }
  if (QSCEN_MASK & 8) { begin(s); standTo(s, .12); hold(s, 150); s.m.selectWalkMode(simple::WalkMode::STANDARD); s.m.walk(1, gClock); hold(s, 300);
    s.m.selectWalkMode(simple::WalkMode::SLOW); hold(s, 600); s.m.selectWalkMode(simple::WalkMode::STANDARD); hold(s, 600); s.m.walkStop();
    for (unsigned k = 0; k < 1500; ++k) { stepRec(s); if (s.m.state() == simple::State::STANDING) break; } summary("E", s); }
  // S: the fixture attitude of the 30c894a HOLD samples (roll 34, pitch -46 mrad): STAND 120, 150, 80, 120, PRONE
  // (P0 class 1 windows: STAND_DEPLOY/LIFT, ALIGN, HEIGHT with probe and trial compensation solves, PRONE_*).
  if (QSCEN_MASK & 16) { begin(s, .034, -.046); standTo(s, .12); hold(s, 100); standTo(s, .15); hold(s, 100); standTo(s, .08); hold(s, 100);
    standTo(s, .12); hold(s, 100); s.m.prone(gClock); for (unsigned k = 0; k < 4000 && s.m.state() != simple::State::READY; ++k) stepRec(s);
    hold(s, 50); summary("S", s); }
}
}  // namespace qemu_motion
