#include "OperationalWalkProfile.h"
namespace matdog { namespace motion { namespace operational_walk {
// Pinned 2026-10-15 (Issue #52 re-pin): union of OPERATIONAL_WALK_RANGES (50 ms
// feedback-paced targets) and OPERATIONAL_WALK_DENSE_RANGES (the time-driven
// reference sampled at 1 kHz, i.e. any release instant; 20/30/40 mm presets,
// ramp-in, stop in the ramp and after 3 cycles), rounded outward to 1 urad.
// Certificate: 09_Logs/Validation_Reports/Preflight_Review_2026-10-15/
// operational_walk_region_certificate_issue52_final.json (identity = sha256 of this file).
const double kTargetMinRad[12] = {-0.000002,0.292039,0.039823,-0.000002,0.260089,0.050855,-0.000002,0.743188,-0.255829,-0.000002,0.822855,-0.236307};
const double kTargetMaxRad[12] = {0.000002,0.671830,0.402696,0.000002,0.600151,0.421641,0.000002,1.067428,0.067472,0.000002,1.083141,0.099892};
// FAST (EXPERIMENTAL) box: OPERATIONAL_WALK_DENSE_RANGES of the 60 mm / 3 s
// profile (1 kHz, stop in the ramp and after 3 cycles), rounded outward 1 urad.
// Certificate: 09_Logs/Validation_Reports/Issue52_Realtime_2026-10-15/
// operational_walk_fast_region_certificate.json (identity = sha256 of this file).
const double kFastTargetMinRad[12] = {-0.000002,0.236168,0.127628,-0.000002,0.171181,0.139967,-0.000002,0.642031,-0.194010,-0.000002,0.785256,-0.168392};
const double kFastTargetMaxRad[12] = {0.000002,0.682630,0.436231,0.000002,0.587009,0.482302,0.000002,1.032084,0.104657,0.000002,1.049820,0.152278};
#ifdef MATDOG_HOST_REGION_SURVEY
// HOST-ONLY survey build (scripts/tests/run_operational_motion_tests.py
// --region-survey): the boxes are opened so the dense sampler can MEASURE the
// union of targets that gets pinned above. Never defined by scripts/build.sh.
static const double kSurveyMin[12] = {-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10};
static const double kSurveyMax[12] = {10,10,10,10,10,10,10,10,10,10,10,10};
#define MATDOG_REGION(min, max) kSurveyMin, kSurveyMax
#else
#define MATDOG_REGION(min, max) min, max
#endif
const WalkProfile kStandardProfile{"STANDARD", false, kPeriodS, kRampS, kLiftM, kDuty, kStrideM, kStrideCount,
  kMaxVelocityRadS, kMaxAccelerationRadS2, kMaxJointStepRad, kStandardDrive, MATDOG_REGION(kTargetMinRad, kTargetMaxRad)};
const WalkProfile kFastProfile{"FAST", true, 3.0, 3.0, 0.010, 0.75, kFastStrideM, 1,
  1.55, 13.0, 0.08, kFastDrive, MATDOG_REGION(kFastTargetMinRad, kFastTargetMaxRad)};
const WalkProfile* const kProfiles[kProfileCount] = {&kStandardProfile, &kFastProfile};
const WalkProfile* profileForVelocity(double vx) {
  for (const WalkProfile* p : kProfiles)
    for (unsigned i = 0; i < p->strideCount; ++i) if (std::fabs(vx - profileVelocity(*p, i)) <= 1e-12) return p;
  return nullptr;
}
WalkSpecification specification(const WalkProfile& p) {
  WalkSpecification w;
  w.periodS = p.periodS; w.rampDurationS = p.rampS; w.liftM = p.liftM; w.duty = p.duty;
  w.swayXM = 0; w.standShiftDurationS = 0;
  w.firstStrideOnly = false; w.compiledFirstStride = false; w.smoothStlReference = false; w.stlContactReference = false;
  w.firstStrideTimeScale = 1; w.maxJointStepRad = p.maxJointStepRad;
  w.maxVelocityRadS = p.maxVelocityRadS; w.maxAccelerationRadS2 = p.maxAccelerationRadS2;
  double vmax = 0; for (unsigned i = 0; i < p.strideCount; ++i) vmax = std::fmax(vmax, profileVelocity(p, i));
  w.maxVxMps = vmax; w.maxVyMps = 0; w.maxYawRateRadS = 0;
  w.maxCommandTtlS = kCommandTtlS;
  // Time-driven streaming (Issue #52): phase from the monotonic clock, 50 Hz
  // frames, asynchronous tracking. Every frame inside the profile's certified box.
  w.timeDriven = true; w.streamPeriodS = kStreamPeriodS;
#ifdef MATDOG_HOST_REGION_SURVEY
  w.streamPeriodS = 0.001;   // survey: every 1 ms release instant
#endif
  w.streamMaxGapS = kStreamMaxGapS;
  w.streamMaxObservationAgeS = kStreamObservationAgeS; w.streamFeedbackMaxAgeS = kStreamFeedbackAgeS;
  w.streamMaxLagS = kStreamMaxLagS; w.streamPersistSamples = kStreamPersistSamples;
  w.streamSettleSpeedRadS = kStreamSettleSpeedRadS; w.streamHoldTimeoutS = kStreamHoldTimeoutS;
  w.regionMinRad = p.regionMinRad; w.regionMaxRad = p.regionMaxRad;
  return w;
}
bool fitsLimits(const WalkProfile& prof, const FeedbackPolicy& p, double tube, unsigned* failed) {
  if (!std::isfinite(tube) || tube <= 0) return false;
#ifdef MATDOG_HOST_REGION_SURVEY
  (void)prof; (void)p; (void)failed; return true;
#endif
  for (unsigned i = 0; i < 12; ++i) {
    const auto& b = p.jointLimits[i / 3][i % 3];
    if (!std::isfinite(b.lower) || !std::isfinite(b.upper) || b.lower >= b.upper || !(prof.regionMinRad[i] < prof.regionMaxRad[i]) ||
        prof.regionMinRad[i] - tube < b.lower || prof.regionMaxRad[i] + tube > b.upper) { if (failed) *failed = i; return false; }
  }
  return true;
}
} } }
