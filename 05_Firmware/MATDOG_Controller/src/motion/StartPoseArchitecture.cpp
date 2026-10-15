#include "StartPoseArchitecture.h"
#include <cmath>

namespace matdog {
namespace motion {
namespace startpose {
namespace {
bool positive(double v) { return std::isfinite(v) && v > 0; }
}

StartClassification classifyStart(const JointSnapshot* samples, unsigned count, const PoseFamily& family,
                                  const RecognitionBand& band) {
  StartClassification out;
  auto fail = [&](StartReason reason, unsigned joint) { out.reason = reason; out.joint = joint; return out; };
  for (unsigned j = 0; j < kJoints; ++j)
    if (!positive(band.per_joint_rad[j]) || !std::isfinite(family.nominal[j]) || !(family.band_lo[j] < family.band_hi[j]))
      return fail(StartReason::INVALID_BAND, j);
  if (!positive(band.max_rms_rad) || !positive(band.max_sample_spread_rad) || band.max_age_ms == 0 || band.min_samples == 0)
    return fail(StartReason::INVALID_BAND, kJoints);
  if (samples == nullptr || count < band.min_samples) return fail(StartReason::TOO_FEW_SAMPLES, kJoints);
  double lo[kJoints], hi[kJoints];
  for (unsigned j = 0; j < kJoints; ++j) { lo[j] = INFINITY; hi[j] = -INFINITY; out.mean_q[j] = 0; }
  for (unsigned s = 0; s < count; ++s)
    for (unsigned j = 0; j < kJoints; ++j) {
      const auto& x = samples[s];
      if (!x.mapped[j]) return fail(StartReason::UNMAPPED, j);
      if (x.age_ms[j] > band.max_age_ms) return fail(StartReason::STALE, j);
      if (!std::isfinite(x.q[j])) return fail(StartReason::NONFINITE, j);
      out.mean_q[j] += x.q[j] / count;
      lo[j] = std::fmin(lo[j], x.q[j]); hi[j] = std::fmax(hi[j], x.q[j]);
    }
  for (unsigned j = 0; j < kJoints; ++j)
    if (hi[j] - lo[j] > band.max_sample_spread_rad) return fail(StartReason::NOISY, j);
  double squares = 0;
  for (unsigned j = 0; j < kJoints; ++j) {
    const double q = out.mean_q[j], d = q - family.nominal[j];
    out.deviation_rad[j] = d;
    if (std::fabs(d) > out.max_abs_deviation_rad) out.max_abs_deviation_rad = std::fabs(d);
    squares += d * d;
  }
  out.rms_deviation_rad = std::sqrt(squares / kJoints);
  for (unsigned j = 0; j < kJoints; ++j)
    if (out.mean_q[j] < family.band_lo[j] || out.mean_q[j] > family.band_hi[j]) return fail(StartReason::FAMILY_BAND, j);
  for (unsigned j = 0; j < kJoints; ++j)
    if (std::fabs(out.deviation_rad[j]) > band.per_joint_rad[j]) return fail(StartReason::JOINT_BAND, j);
  if (out.rms_deviation_rad > band.max_rms_rad) return fail(StartReason::RMS, kJoints);
  out.cls = StartClass::PRONE_COMPATIBLE;
  out.reason = StartReason::OK; out.joint = kJoints;
  return out;
}

TrackingTolerance deriveTrackingTolerance(const EncoderModel& e, double v, double settle_s) {
  TrackingTolerance t;
  if (!positive(e.tick_rad) || !std::isfinite(e.backlash_rad) || e.backlash_rad < 0 || !std::isfinite(e.latency_s) ||
      e.latency_s < 0 || !std::isfinite(v) || v < 0 || !positive(settle_s)) return t;
  t.rad = 2 * e.tick_rad + e.backlash_rad + v * e.latency_s;
  t.settle_s = settle_s;
  return t;
}

ResidualAudit auditTubeResidual(double tube, double tracking, const EncoderModel& e) {
  ResidualAudit a;
  a.residual_rad = tube - tracking;
  a.ticks = positive(e.tick_rad) ? a.residual_rad / e.tick_rad : 0;
  a.verdict = !(a.residual_rad > 0) ? ResidualVerdict::NEGATIVE : a.ticks < 1 ? ResidualVerdict::BELOW_ONE_TICK
                                                                              : ResidualVerdict::RESOLVABLE;
  return a;
}

TrackingMonitor::Verdict TrackingMonitor::update(const double cmd[kJoints], const double meas[kJoints],
                                                 const bool fresh[kJoints], double t) {
  if (latched_) return Verdict::FAULT_STOP;
  auto stop = [&](unsigned j, const char* why) { latched_ = true; fault_joint_ = j; reason_ = why; return Verdict::FAULT_STOP; };
  if (!positive(tol_.rad) || !positive(tol_.settle_s) || !std::isfinite(t)) return stop(kJoints, "INVALID_TOLERANCE");
  bool settling = false;
  for (unsigned j = 0; j < kJoints; ++j) {
    if (!fresh[j] || !std::isfinite(cmd[j]) || !std::isfinite(meas[j])) return stop(j, "FEEDBACK_STALE_OR_INVALID");
    const double err = std::fabs(cmd[j] - meas[j]);
    if (err > 3 * tol_.rad) return stop(j, "GROSS_TRACKING_ERROR");
    if (err > tol_.rad) {
      if (!over_[j]) { over_[j] = true; over_since_s_[j] = t; }
      else if (t - over_since_s_[j] > tol_.settle_s) return stop(j, "TRACKING_SETTLE_TIMEOUT");
      settling = true;
    } else {
      over_[j] = false;
    }
  }
  return settling ? Verdict::SETTLING : Verdict::OK;
}

PathVerdict checkPath(const double (*path)[kJoints], unsigned count, const JointLimits12& limits,
                      const JointKeepOut* keepouts, unsigned nk, const TrackingTolerance& tracking,
                      const PathClearance& clearance) {
  PathVerdict v;
  v.required_rad = tracking.rad + clearance.margin_rad;
  if (path == nullptr || count < 2 || !positive(tracking.rad) || !positive(clearance.margin_rad) || (nk && !keepouts) || nk > 32)
    return v;
  for (unsigned s = 0; s < count; ++s)
    for (unsigned j = 0; j < kJoints; ++j)
      if (!std::isfinite(path[s][j])) { v.sample = s; v.joint = j; return v; }
  for (unsigned j = 0; j < kJoints; ++j)
    if (!(limits.lower[j] < limits.upper[j])) { v.joint = j; return v; }
  double prev_k[32] = {}, prev_lo[kJoints] = {}, prev_hi[kJoints] = {};
  auto keepClearance = [&](const JointKeepOut& k, const double* q) {
    const double norm = k.a == k.b ? std::fabs(k.ca + k.cb) : std::sqrt(k.ca * k.ca + k.cb * k.cb);
    const double value = k.a == k.b ? (k.ca + k.cb) * q[k.a] : k.ca * q[k.a] + k.cb * q[k.b];
    return norm > 0 ? (value - k.bound) / norm : -INFINITY;
  };
  for (unsigned s = 0; s < count; ++s) {
    const double* q = path[s];
    for (unsigned k = 0; k < nk; ++k) {
      if (keepouts[k].a >= kJoints || keepouts[k].b >= kJoints) { v.reason = PathReason::INVALID_INPUT; return v; }
      const double c = keepClearance(keepouts[k], q);
      if (s == 0) {
        // The measured start is a fact. Deeper than the tracking tolerance
        // inside a known obstacle means the mapping or the geometry is wrong.
        if (c < -tracking.rad) { v.reason = PathReason::START_IN_KEEPOUT; v.keepout = static_cast<int>(k); v.clearance_rad = c; return v; }
      } else if (c < v.required_rad && c < prev_k[k] - 1e-12) {
        v.reason = PathReason::KEEPOUT; v.sample = s; v.keepout = static_cast<int>(k);
        v.joint = keepouts[k].a; v.clearance_rad = c; return v;
      }
      prev_k[k] = c;
    }
    for (unsigned j = 0; j < kJoints; ++j) {
      const double lo = q[j] - limits.lower[j], hi = limits.upper[j] - q[j];
      if (s == 0) {
        if (lo < -tracking.rad || hi < -tracking.rad) { v.reason = PathReason::START_OUTSIDE_LIMITS; v.joint = j; v.clearance_rad = std::fmin(lo, hi); return v; }
      } else if ((lo < v.required_rad && lo < prev_lo[j] - 1e-12) || (hi < v.required_rad && hi < prev_hi[j] - 1e-12)) {
        v.reason = PathReason::LIMIT; v.sample = s; v.joint = j; v.clearance_rad = std::fmin(lo, hi); return v;
      }
      prev_lo[j] = lo; prev_hi[j] = hi;
    }
  }
  v.admissible = true; v.reason = PathReason::OK;
  return v;
}

void buildTwoStageEntry(const double start[kJoints], const double goal[kJoints], double (*out)[kJoints]) {
  double middle[kJoints];
  for (unsigned j = 0; j < kJoints; ++j) middle[j] = j % 3 == 2 ? goal[j] : start[j];
  for (unsigned i = 0; i <= kEntrySamplesPerStage; ++i) {
    const double u = static_cast<double>(i) / kEntrySamplesPerStage;
    for (unsigned j = 0; j < kJoints; ++j) out[i][j] = start[j] + (middle[j] - start[j]) * u;
  }
  for (unsigned i = 1; i <= kEntrySamplesPerStage; ++i) {
    const double u = static_cast<double>(i) / kEntrySamplesPerStage;
    for (unsigned j = 0; j < kJoints; ++j) out[kEntrySamplesPerStage + i][j] = middle[j] + (goal[j] - middle[j]) * u;
  }
}

Admission admitSupervisedEntry(const AdmissionInputs& in) {
  Admission a;
  // Physical safety first, independent of every classification below.
  if (in.stop_latched) { a.reason = "STOP_LATCHED"; return a; }
  if (!in.estop_circuit_ok) { a.reason = "ESTOP_NOT_ARMED"; return a; }
  if (!in.feedback_fresh) { a.reason = "FEEDBACK_STALE"; return a; }
  if (!in.actuator_bounds_ok) { a.reason = "ACTUATOR_BOUNDS"; return a; }
  if (!in.hard_limits_loaded) { a.reason = "HARD_LIMITS_ABSENT"; return a; }
  if (in.start_class != StartClass::PRONE_COMPATIBLE) { a.reason = "START_POSE_UNKNOWN"; return a; }
  if (!in.path_admissible) { a.reason = "PATH_NOT_ADMISSIBLE"; return a; }
  if (!in.mesh_clearance_receipt) { a.reason = "MESH_CLEARANCE_RECEIPT_REQUIRED"; return a; }
  if (!in.supervisor_present) { a.reason = "SUPERVISOR_ABSENT"; return a; }
  if (!in.operator_consent) { a.reason = "OPERATOR_CONSENT_REQUIRED"; return a; }
  a.decision = AdmissionDecision::ALLOW_SUPERVISED_ENTRY;
  a.reason = "OK";
  return a;
}

LayeringReason validateLayering(const RecognitionBand& band, const TrackingTolerance& tracking,
                                const PathClearance& clearance, const EncoderModel& encoder) {
  if (!(tracking.rad >= 2 * encoder.tick_rad)) return LayeringReason::TRACKING_BELOW_TWO_TICKS;
  for (unsigned j = 0; j < kJoints; ++j)
    if (!(band.per_joint_rad[j] >= 3 * tracking.rad)) return LayeringReason::RECOGNITION_NOT_BROADER_THAN_TRACKING;
  if (!(clearance.margin_rad > 0)) return LayeringReason::PATH_MARGIN_NONPOSITIVE;
  return LayeringReason::OK;
}

const char* toString(StartReason r) {
  static const char* n[] = {"OK", "INVALID_BAND", "TOO_FEW_SAMPLES", "UNMAPPED", "STALE", "NONFINITE", "NOISY",
                            "FAMILY_BAND", "JOINT_BAND", "RMS"};
  const unsigned i = static_cast<unsigned>(r);
  return i < sizeof(n) / sizeof(n[0]) ? n[i] : "UNKNOWN";
}
const char* toString(PathReason r) {
  static const char* n[] = {"OK", "INVALID_INPUT", "START_OUTSIDE_LIMITS", "START_IN_KEEPOUT", "KEEPOUT", "LIMIT"};
  const unsigned i = static_cast<unsigned>(r);
  return i < sizeof(n) / sizeof(n[0]) ? n[i] : "UNKNOWN";
}

}  // namespace startpose
}  // namespace motion
}  // namespace matdog
