#ifndef MATDOG_MOTION_START_POSE_ARCHITECTURE_H
#define MATDOG_MOTION_START_POSE_ARCHITECTURE_H
// Release-blocking architecture item (HW FAIL 2026-10-14): four layers that
// must never share a threshold. Pure, RAM-only, no hardware access, NOT yet
// wired into the actuator path (next-candidate design; see
// START_POSE_VS_TRACKING_ARCHITECTURE.md in the audit folder).
//
//   A  RecognitionBand    "is the measured robot in a PRONE-family pose?"
//                         Broad, per-joint, degrees. A LABEL. Grants nothing.
//   C  PathClearance      "is THIS commanded path, from THIS measured start,
//                         clear of limits and known contacts?" millimetres /
//                         milliradians of geometric clearance.
//   D  TrackingTolerance  "is the actuator following its command?" Derived
//                         from encoder resolution, backlash and latency;
//                         never below two encoder ticks.
//   B/E admitSupervisedEntry: composes the above with STOP, E-stop, fresh
//                         feedback, actuator bounds, supervisor and consent.
//                         Fail-closed; no single layer can allow by itself.
//
// The three tolerance types are distinct structs on purpose: a TrackingTolerance
// cannot be passed where a RecognitionBand or PathClearance is expected.
// Joint order everywhere: LF(H,U,L) RF(H,U,L) RH(H,U,L) LH(H,U,L), radians.
#include <stdint.h>

namespace matdog {
namespace motion {
namespace startpose {

constexpr unsigned kJoints = 12;
constexpr double kPi = 3.14159265358979323846;
constexpr double kSt3215TickRad = 2.0 * kPi / 4096.0;  // 1.534 mrad, 0.088 deg

// ------------------------------------------------------------------ A ------
struct JointSnapshot {
  double q[kJoints] = {};
  bool mapped[kJoints] = {};     // a RESTORED transform produced q[i]
  uint32_t age_ms[kJoints] = {}; // feedback age when the snapshot was taken
};
struct RecognitionBand {
  double per_joint_rad[kJoints] = {};  // e.g. ~10 deg; NOT a tracking or path value
  double max_rms_rad = 0;
  double max_sample_spread_rad = 0;    // repeated-sample noise / stationarity
  uint32_t max_age_ms = 0;
  uint8_t min_samples = 0;
};
// A physically meaningful pose family: nominal plus hard sign/range bands that
// keep a mirrored or flipped leg from being recognised as PRONE.
struct PoseFamily {
  double nominal[kJoints] = {};
  double band_lo[kJoints] = {};
  double band_hi[kJoints] = {};
};
enum class StartClass : uint8_t { UNKNOWN, PRONE_COMPATIBLE };
enum class StartReason : uint8_t {
  OK, INVALID_BAND, TOO_FEW_SAMPLES, UNMAPPED, STALE, NONFINITE, NOISY, FAMILY_BAND, JOINT_BAND, RMS,
};
struct StartClassification {
  StartClass cls = StartClass::UNKNOWN;
  StartReason reason = StartReason::INVALID_BAND;
  unsigned joint = kJoints;               // first failing joint, kJoints = none
  double mean_q[kJoints] = {};            // the measured start used downstream
  double deviation_rad[kJoints] = {};     // signed mean_q - nominal
  double max_abs_deviation_rad = 0, rms_deviation_rad = 0;
};
StartClassification classifyStart(const JointSnapshot* samples, unsigned count, const PoseFamily& family,
                                  const RecognitionBand& band);

// ------------------------------------------------------------------ D ------
struct EncoderModel {
  double tick_rad = kSt3215TickRad;
  double backlash_rad = 0;   // MUST come from a bench measurement; 0 = unmeasured
  double latency_s = 0;      // command-to-feedback latency, measured
};
struct TrackingTolerance {
  double rad = 0;
  double settle_s = 0;
};
// tol = 2 ticks (command + feedback quantization) + backlash + v_max * latency.
TrackingTolerance deriveTrackingTolerance(const EncoderModel& encoder, double max_velocity_rad_s, double settle_s);
enum class ResidualVerdict : uint8_t { RESOLVABLE, BELOW_ONE_TICK, NEGATIVE };
struct ResidualAudit {
  double residual_rad = 0;
  double ticks = 0;
  ResidualVerdict verdict = ResidualVerdict::NEGATIVE;
};
// What is left of a route tube after tracking: a start-allowance below one
// encoder tick cannot be measured, so it cannot be a pass/fail criterion.
ResidualAudit auditTubeResidual(double tube_rad, double tracking_rad, const EncoderModel& encoder);

// Checked WHILE executing, independent of any classification.
class TrackingMonitor {
 public:
  enum class Verdict : uint8_t { OK, SETTLING, FAULT_STOP };
  explicit TrackingMonitor(const TrackingTolerance& tolerance) : tol_(tolerance) {}
  Verdict update(const double commanded[kJoints], const double measured[kJoints], const bool fresh[kJoints], double t_s);
  unsigned faultJoint() const { return fault_joint_; }
  const char* faultReason() const { return reason_; }
 private:
  TrackingTolerance tol_;
  double over_since_s_[kJoints] = {};
  bool over_[kJoints] = {};
  bool latched_ = false;
  unsigned fault_joint_ = kJoints;
  const char* reason_ = "NONE";
};

// ------------------------------------------------------------------ C ------
struct JointLimits12 {
  double lower[kJoints] = {};
  double upper[kJoints] = {};
};
// Free region: ca*q[a] + cb*q[b] >= bound. A reviewed configuration-space
// obstacle derived OFFLINE from CAD/FCL (e.g. LF UPPER-LOWER contact).
struct JointKeepOut {
  const char* id = "";
  unsigned a = 0, b = 0;
  double ca = 1, cb = 0, bound = 0;
};
struct PathClearance {
  double margin_rad = 0;  // geometric margin ON TOP of tracking tolerance
};
enum class PathReason : uint8_t { OK, INVALID_INPUT, START_OUTSIDE_LIMITS, START_IN_KEEPOUT, KEEPOUT, LIMIT };
struct PathVerdict {
  bool admissible = false;
  PathReason reason = PathReason::INVALID_INPUT;
  unsigned sample = 0, joint = kJoints;
  int keepout = -1;
  double clearance_rad = 0;      // at the failing sample
  double required_rad = 0;       // tracking + margin
};
// path: count x 12, sample 0 is the MEASURED start. Every commanded sample
// must keep (tracking + margin) from limits and keep-outs, except that a
// sample may sit inside that band only while the path is still moving AWAY
// from the constraint the measured start already sits close to.
PathVerdict checkPath(const double (*path)[kJoints], unsigned count, const JointLimits12& limits,
                      const JointKeepOut* keepouts, unsigned keepout_count,
                      const TrackingTolerance& tracking, const PathClearance& clearance);
// Two-stage entry like the firmware: LOWER joints first, then HIP/UPPER.
constexpr unsigned kEntrySamplesPerStage = 40;
constexpr unsigned kEntryPathSamples = 2 * kEntrySamplesPerStage + 1;
void buildTwoStageEntry(const double start[kJoints], const double goal[kJoints], double (*out)[kJoints]);

// ---------------------------------------------------------------- B/E ------
struct AdmissionInputs {
  bool stop_latched = true;
  bool estop_circuit_ok = false;
  bool feedback_fresh = false;
  bool actuator_bounds_ok = false;     // temperature, voltage, current, BMS
  bool hard_limits_loaded = false;
  StartClass start_class = StartClass::UNKNOWN;
  bool path_admissible = false;        // checkPath() on the measured start
  bool mesh_clearance_receipt = false; // host FCL/MoveIt check of THIS path
  bool supervisor_present = false;
  bool operator_consent = false;
};
enum class AdmissionDecision : uint8_t { DENY, ALLOW_SUPERVISED_ENTRY };
struct Admission {
  AdmissionDecision decision = AdmissionDecision::DENY;
  const char* reason = "STOP_LATCHED";
};
Admission admitSupervisedEntry(const AdmissionInputs& in);

// Layer sanity: refuses configurations that betray a confused threshold.
enum class LayeringReason : uint8_t { OK, TRACKING_BELOW_TWO_TICKS, RECOGNITION_NOT_BROADER_THAN_TRACKING, PATH_MARGIN_NONPOSITIVE };
LayeringReason validateLayering(const RecognitionBand& band, const TrackingTolerance& tracking,
                                const PathClearance& clearance, const EncoderModel& encoder);

const char* toString(StartReason);
const char* toString(PathReason);

}  // namespace startpose
}  // namespace motion
}  // namespace matdog
#endif
