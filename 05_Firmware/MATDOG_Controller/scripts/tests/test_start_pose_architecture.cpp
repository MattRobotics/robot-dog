// HW FAIL 2026-10-14, release-blocking item: start-pose RECOGNITION (A) is
// independent from path CLEARANCE (C) and actuator TRACKING (D), and the
// composition (B/E) is fail-closed. Real data: 13 Oct encoder-derived PRONE,
// the frozen fixture route head, the CAD LF UPPER-LOWER contact and the
// firmware's own planFixtureEntry with the 4 mrad entry certificate.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <type_traits>
#include "../../src/motion/StartPoseArchitecture.h"
#include "../../src/motion/FixtureEntryPlanner.h"
#include "../../src/motion/PairedProneRoute.h"

using namespace matdog::motion;
using namespace matdog::motion::startpose;
static int checks = 0, failures = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; std::printf("FAIL %d: %s\n", __LINE__, #c); } } while (0)
static constexpr double D = kPi / 180.0;

static double at(const JointTargetFrame& f, unsigned i) { const auto& q = f.legs[i / 3]; return i % 3 == 0 ? q.hip : i % 3 == 1 ? q.upper : q.lower; }
static void put(JointTargetFrame& f, unsigned i, double v) { auto& q = f.legs[i / 3]; (i % 3 == 0 ? q.hip : i % 3 == 1 ? q.upper : q.lower) = v; }

int main() {
  // ---- types cannot be confused (requirement 4) ----
  static_assert(!std::is_convertible<TrackingTolerance, RecognitionBand>::value, "D is not A");
  static_assert(!std::is_convertible<TrackingTolerance, PathClearance>::value, "D is not C");
  static_assert(!std::is_convertible<PathClearance, RecognitionBand>::value, "C is not A");
  static_assert(!std::is_convertible<RecognitionBand, TrackingTolerance>::value, "A is not D");

  // ---- data ----
  const auto head = pairedProneSample(0, 0, true).target;  // firmware fixture-limit head (commissioning goal)
  CHECK(head.valid);
  double nominal[12], goal[12];
  for (unsigned i = 0; i < 12; ++i) nominal[i] = goal[i] = at(head, i);
  // 2026-10-13 encoder-derived PRONE (same values as test_fixture_pose_classifier).
  const double real13[12] = {-2.29*D, 80.16*D, -91.76*D, 2.37*D, 79.54*D, -90.62*D, -2.55*D, 82.53*D, -91.41*D, 2.29*D, 82.35*D, -91.41*D};
  PoseFamily prone;
  for (unsigned i = 0; i < 12; ++i) {
    prone.nominal[i] = nominal[i];
    // Physical sign bands (hip near neutral, upper folded down, lower folded): broad, family-level.
    if (i % 3 == 0) { prone.band_lo[i] = -20 * D; prone.band_hi[i] = 20 * D; }
    if (i % 3 == 1) { prone.band_lo[i] = 50 * D; prone.band_hi[i] = 110 * D; }
    if (i % 3 == 2) { prone.band_lo[i] = -115 * D; prone.band_hi[i] = -70 * D; }
  }
  RecognitionBand band;
  for (unsigned i = 0; i < 12; ++i) band.per_joint_rad[i] = 10 * D;
  band.max_rms_rad = 7 * D; band.max_sample_spread_rad = 3 * kSt3215TickRad; band.max_age_ms = 300; band.min_samples = 5;
  auto snapshots = [&](const double* q, JointSnapshot* out, unsigned n, double jitter) {
    for (unsigned s = 0; s < n; ++s) for (unsigned i = 0; i < 12; ++i) {
      out[s].q[i] = q[i] + ((s + i) % 2 ? jitter : -jitter); out[s].mapped[i] = true; out[s].age_ms[i] = 40; }
  };
  JointSnapshot sn[5];
  snapshots(real13, sn, 5, kSt3215TickRad);  // +/- one tick of encoder noise

  // ---- (1) plausible PRONE several degrees from nominal: recognised, actuator stays OFF ----
  const auto c = classifyStart(sn, 5, prone, band);
  CHECK(c.cls == StartClass::PRONE_COMPATIBLE);
  CHECK(c.max_abs_deviation_rad > 3 * D);                          // several degrees, not a tube
  std::printf("real13 max_dev_deg=%.2f rms_dev_deg=%.2f\n", c.max_abs_deviation_rad / D, c.rms_deviation_rad / D);
  AdmissionInputs only_class; only_class.start_class = c.cls;     // everything else at its fail-closed default
  CHECK(admitSupervisedEntry(only_class).decision == AdmissionDecision::DENY);
  { AdmissionInputs in; in.stop_latched = false; in.estop_circuit_ok = in.feedback_fresh = in.actuator_bounds_ok = in.hard_limits_loaded = true;
    in.start_class = c.cls;  // classification + safe context, but no path / receipt / supervisor / consent
    CHECK(admitSupervisedEntry(in).decision == AdmissionDecision::DENY); }

  // ---- (3) the old certificate blocks it; a measured-start connection does not ----
  FeedbackPolicy policy;
  policy.samplePeriodS = .05; policy.maxJointStepRad = .006; policy.maxTrackingErrorRad = .0035; policy.segmentTimeoutS = 120;
  for (unsigned l = 0; l < 4; ++l) { policy.maxVelocityRadS[l] = {.2, .2, .2}; policy.maxAccelerationRadS2[l] = {.5, .5, .5};
    for (unsigned j = 0; j < 3; ++j) policy.jointLimits[l][j] = {-2.5, 2.5}; }
  FixtureEntryCertificate cert; cert.present = true; cert.tubeRad = .004; cert.goal = head; cert.start.valid = true;
  for (unsigned i = 0; i < 12; ++i) put(cert.start, i, real13[i]);  // certified around the 13 Oct pose
  JointTargetFrame today; today.valid = true;
  double measured14[12];
  for (unsigned i = 0; i < 12; ++i) { measured14[i] = real13[i] + (i % 3 == 1 ? .8 * D : 0); put(today, i, measured14[i]); }  // re-folded 0.8 deg off
  const auto old = planFixtureEntry(today, head, policy, cert, 4);
  CHECK(old.status == EntryStatus::START_OUTSIDE_CERTIFIED_REGION);  // the 14 Oct architecture defect
  std::printf("old_planner=%s start_dev_mrad=%.2f\n", entryStatusName(old.status), old.startDeviationRad * 1e3);
  for (unsigned s = 0; s < 5; ++s) for (unsigned i = 0; i < 12; ++i) sn[s].q[i] = measured14[i];
  CHECK(classifyStart(sn, 5, prone, band).cls == StartClass::PRONE_COMPATIBLE);

  EncoderModel enc; enc.backlash_rad = .0015; enc.latency_s = .02;  // HYPOTHETICAL until bench-measured
  const TrackingTolerance tol = deriveTrackingTolerance(enc, .2, .5);
  CHECK(tol.rad >= 2 * kSt3215TickRad);
  JointLimits12 urdf;
  for (unsigned i = 0; i < 12; ++i) {
    urdf.lower[i] = i % 3 == 0 ? -.785398163397 : i % 3 == 1 ? -.916297857297 : -1.605702911835;
    urdf.upper[i] = i % 3 == 0 ? .785398163397 : i % 3 == 1 ? 2.138028333693 : .654498469498;
  }
  // CAD/FCL (GEOMETRY_ENTRY_REPORT 2026-10-14): LF LOWER first zero separation ~ -92.070442 deg.
  JointKeepOut lf; lf.id = "LF_UPPER_LOWER_CONTACT"; lf.a = lf.b = 2; lf.ca = 1; lf.cb = 0; lf.bound = -92.070442 * D;
  const PathClearance margin{.001};
  double path[kEntryPathSamples][12];
  buildTwoStageEntry(measured14, goal, path);
  const auto newer = checkPath(path, kEntryPathSamples, urdf, &lf, 1, tol, margin);
  CHECK(newer.admissible);                                       // measured start connects, old tube irrelevant
  std::printf("new_path=%s tol_mrad=%.3f required_mrad=%.3f\n", toString(newer.reason), tol.rad * 1e3, newer.required_rad * 1e3);

  // ---- (2) recognised q, but THIS path drives LF LOWER into the CAD contact: precise denial ----
  double canonical[12];
  const auto frozen = pairedProneSample(0, 0, false).target;     // non-fixture head: LOWER at the URDF limit
  for (unsigned i = 0; i < 12; ++i) canonical[i] = at(frozen, i);
  CHECK(std::fabs(canonical[2] - (-1.6057029118349999)) < 1e-12);
  buildTwoStageEntry(measured14, canonical, path);
  const auto denied = checkPath(path, kEntryPathSamples, urdf, &lf, 1, tol, margin);
  CHECK(!denied.admissible && denied.reason == PathReason::KEEPOUT && denied.joint == 2 && denied.keepout == 0);
  CHECK(denied.clearance_rad > 0 && denied.clearance_rad < denied.required_rad);  // 1.2 mrad left vs ~5.8 required
  std::printf("canonical_head=%s joint=%u sample=%u clearance_mrad=%.3f required_mrad=%.3f\n", toString(denied.reason),
              denied.joint, denied.sample, denied.clearance_rad * 1e3, denied.required_rad * 1e3);
  // PRIME inward by 10 mrad from the resting LF pose is likewise refused (moving TOWARD the contact).
  { double prime[kEntryPathSamples][12]; double target[12]; for (unsigned i = 0; i < 12; ++i) target[i] = measured14[i];
    target[2] = measured14[2] - .010; buildTwoStageEntry(measured14, target, prime);
    const auto p = checkPath(prime, kEntryPathSamples, urdf, &lf, 1, tol, margin);
    CHECK(!p.admissible && p.reason == PathReason::KEEPOUT); }
  // Departure rule: a start inside the clearance band may only move AWAY.
  { double near[12]; for (unsigned i = 0; i < 12; ++i) near[i] = measured14[i]; near[2] = -92.0704 * D + .001;
    buildTwoStageEntry(near, goal, path);
    CHECK(checkPath(path, kEntryPathSamples, urdf, &lf, 1, tol, margin).admissible); }
  // A start deep inside the obstacle means wrong mapping/geometry: refused as such.
  { double deep[12]; for (unsigned i = 0; i < 12; ++i) deep[i] = measured14[i]; deep[2] = -92.0704 * D - 2 * tol.rad;
    buildTwoStageEntry(deep, goal, path);
    CHECK(checkPath(path, kEntryPathSamples, urdf, &lf, 1, tol, margin).reason == PathReason::START_IN_KEEPOUT); }

  // ---- (4) tracking is physical, separate and audited ----
  const auto residual = auditTubeResidual(.004, .0035, enc);
  CHECK(residual.verdict == ResidualVerdict::BELOW_ONE_TICK);
  std::printf("tube4_track3.5 residual_mrad=%.3f ticks=%.3f\n", residual.residual_rad * 1e3, residual.ticks);
  const auto r10 = auditTubeResidual(.010, tol.rad, enc);  // even 10 mrad is marginal with realistic tracking
  std::printf("tube10_track%.2f residual_mrad=%.3f ticks=%.3f verdict=%s\n", tol.rad * 1e3, r10.residual_rad * 1e3, r10.ticks,
              r10.verdict == ResidualVerdict::RESOLVABLE ? "RESOLVABLE" : "BELOW_ONE_TICK");
  CHECK(auditTubeResidual(.020, tol.rad, enc).verdict == ResidualVerdict::RESOLVABLE);
  CHECK(validateLayering(band, tol, margin, enc) == LayeringReason::OK);
  { RecognitionBand confused = band; for (auto& b : confused.per_joint_rad) b = .006;  // tube used as classifier
    CHECK(validateLayering(confused, tol, margin, enc) == LayeringReason::RECOGNITION_NOT_BROADER_THAN_TRACKING); }
  { TrackingTolerance tight{.0005, .5}; CHECK(validateLayering(band, tight, margin, enc) == LayeringReason::TRACKING_BELOW_TWO_TICKS); }
  CHECK(classifyStart(sn, 5, prone, [&] { RecognitionBand b = band; for (auto& x : b.per_joint_rad) x = .006; return b; }()).cls == StartClass::UNKNOWN);

  // ---- (5) noisy / incomplete / stale / unmapped / non-finite / mirrored -> UNKNOWN ----
  { snapshots(real13, sn, 5, 4 * kSt3215TickRad); CHECK(classifyStart(sn, 5, prone, band).reason == StartReason::NOISY); }
  snapshots(real13, sn, 5, 0);
  CHECK(classifyStart(sn, 4, prone, band).reason == StartReason::TOO_FEW_SAMPLES);
  { auto x = sn[0]; JointSnapshot s5[5] = {sn[0], sn[1], sn[2], sn[3], x}; s5[4].mapped[7] = false;
    const auto r = classifyStart(s5, 5, prone, band); CHECK(r.cls == StartClass::UNKNOWN && r.reason == StartReason::UNMAPPED && r.joint == 7); }
  { JointSnapshot s5[5] = {sn[0], sn[1], sn[2], sn[3], sn[4]}; s5[2].age_ms[4] = 301;
    CHECK(classifyStart(s5, 5, prone, band).reason == StartReason::STALE); }
  { JointSnapshot s5[5] = {sn[0], sn[1], sn[2], sn[3], sn[4]}; s5[0].q[9] = NAN;
    CHECK(classifyStart(s5, 5, prone, band).reason == StartReason::NONFINITE); }
  { JointSnapshot s5[5]; double flipped[12]; for (unsigned i = 0; i < 12; ++i) flipped[i] = real13[i]; flipped[7] = -flipped[7];
    snapshots(flipped, s5, 5, 0); const auto r = classifyStart(s5, 5, prone, band);
    CHECK(r.cls == StartClass::UNKNOWN && r.reason == StartReason::FAMILY_BAND && r.joint == 7); }
  { JointSnapshot s5[5]; double off[12]; for (unsigned i = 0; i < 12; ++i) off[i] = real13[i]; off[4] -= 12 * D;
    snapshots(off, s5, 5, 0); const auto r = classifyStart(s5, 5, prone, band);
    CHECK(r.reason == StartReason::JOINT_BAND && r.joint == 4); }

  // ---- (6) STOP and physical safety are fail-closed regardless of classification ----
  AdmissionInputs all; all.stop_latched = false; all.estop_circuit_ok = all.feedback_fresh = all.actuator_bounds_ok = true;
  all.hard_limits_loaded = true; all.start_class = StartClass::PRONE_COMPATIBLE; all.path_admissible = true;
  all.mesh_clearance_receipt = all.supervisor_present = all.operator_consent = true;
  CHECK(admitSupervisedEntry(all).decision == AdmissionDecision::ALLOW_SUPERVISED_ENTRY);
  unsigned allows = 0;
  for (unsigned m = 0; m < (1u << 10); ++m) {      // exhaustive: every input must be required
    AdmissionInputs x; x.stop_latched = !(m & 1); x.estop_circuit_ok = m & 2; x.feedback_fresh = m & 4; x.actuator_bounds_ok = m & 8;
    x.hard_limits_loaded = m & 16; x.start_class = (m & 32) ? StartClass::PRONE_COMPATIBLE : StartClass::UNKNOWN;
    x.path_admissible = m & 64; x.mesh_clearance_receipt = m & 128; x.supervisor_present = m & 256; x.operator_consent = m & 512;
    if (admitSupervisedEntry(x).decision == AdmissionDecision::ALLOW_SUPERVISED_ENTRY) { ++allows; CHECK(m == 1023u); }
  }
  CHECK(allows == 1);
  { auto x = all; x.stop_latched = true; CHECK(std::strcmp(admitSupervisedEntry(x).reason, "STOP_LATCHED") == 0); }
  // Tracking monitor stops on its own, even with a PRONE-compatible start.
  TrackingMonitor mon(tol);
  double cmd[12], meas[12]; bool fresh[12];
  for (unsigned i = 0; i < 12; ++i) { cmd[i] = meas[i] = goal[i]; fresh[i] = true; }
  CHECK(mon.update(cmd, meas, fresh, 0) == TrackingMonitor::Verdict::OK);
  meas[5] += 1.5 * tol.rad; CHECK(mon.update(cmd, meas, fresh, .1) == TrackingMonitor::Verdict::SETTLING);
  CHECK(mon.update(cmd, meas, fresh, .5) == TrackingMonitor::Verdict::SETTLING);
  CHECK(mon.update(cmd, meas, fresh, .7) == TrackingMonitor::Verdict::FAULT_STOP && mon.faultJoint() == 5);
  meas[5] = cmd[5]; CHECK(mon.update(cmd, meas, fresh, .8) == TrackingMonitor::Verdict::FAULT_STOP);  // latched
  { TrackingMonitor m2(tol); meas[1] = cmd[1] + 4 * tol.rad; CHECK(m2.update(cmd, meas, fresh, 0) == TrackingMonitor::Verdict::FAULT_STOP);
    CHECK(std::strcmp(m2.faultReason(), "GROSS_TRACKING_ERROR") == 0); meas[1] = cmd[1]; }
  { TrackingMonitor m3(tol); fresh[3] = false; CHECK(m3.update(cmd, meas, fresh, 0) == TrackingMonitor::Verdict::FAULT_STOP); fresh[3] = true; }

  std::printf("START_POSE_ARCHITECTURE: %d checks, %d failures (offline; no actuator path)\n", checks, failures);
  return failures ? 1 : 0;
}
