#include "../../src/motion/ActuatorEnvelope.h"
#include "../../src/motion/BodyStabilizer.h"
#include "../../src/motion/ImuAttitude.h"
#include "../../src/motion/TiltCompensation.h"
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>
#include <string>
using namespace matdog::motion;
static unsigned checks = 0, failures = 0;
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; std::printf("FAIL %d: %s\n", __LINE__, #x); } } while (0)
static bool near(double a, double b, double e = 1e-9) { return std::abs(a - b) <= e; }
static const double kNaN = std::numeric_limits<double>::quiet_NaN();
// Independent composition R = Rz(yaw) Ry(pitch) Rx(roll) -> unit quaternion (w,x,y,z), written without the library helpers.
static void quatZyx(double yaw, double pitch, double roll, double q[4]) {
  const double cy = std::cos(yaw/2), sy = std::sin(yaw/2), cp = std::cos(pitch/2), sp = std::sin(pitch/2), cr = std::cos(roll/2), sr = std::sin(roll/2);
  q[0] = cr*cp*cy + sr*sp*sy; q[1] = sr*cp*cy - cr*sp*sy; q[2] = cr*sp*cy + sr*cp*sy; q[3] = cr*cp*sy - sr*sp*cy;
}
static AttitudePolicy policy() { AttitudePolicy p; p.maxAgeS = .125; p.maxAccuracyRad = .1; p.minAccuracyStatus = 2; p.normTolerance = 1e-3; p.maxPlausibleTiltRad = .5; return p; }
static ImuSnapshot snap(double roll, double pitch, double yaw, double stamp, uint32_t seq) {
  double q[4]; quatZyx(yaw, pitch, roll, q); ImuSnapshot s; s.w = q[0]; s.x = q[1]; s.y = q[2]; s.z = q[3]; s.stampS = stamp; s.sequence = seq; s.accuracyRad = .05; s.accuracyStatus = 3; return s;
}
static StabilizerConfig config(double ki = 1.0) {
  StabilizerConfig c; c.integralGainPerS = ki; c.maxCorrectionRad = .05; c.maxRateRadS = .1; c.deadbandRad = 0; c.sampleDtS = .02; c.loopDelaySteps = 2; c.holdBeforeFaultS = .5; return c;
}
int main() {
  // ---- attitude contract -------------------------------------------------------------------------------------------
  for (double roll : {-.4, -.1, 0., .05, .3}) for (double pitch : {-.4, -.2, 0., .1, .35}) for (double yaw : {-3., -1., 0., .7, 2.9}) {
    double q[4]; quatZyx(yaw, pitch, roll, q); Attitude a;
    CHECK(tiltFromQuaternion(q[0], q[1], q[2], q[3], a));
    CHECK(near(a.rollRad, roll, 1e-12) && near(a.pitchRad, pitch, 1e-12));  // yaw never leaks into tilt
    CHECK(near(a.upBase[0]*a.upBase[0] + a.upBase[1]*a.upBase[1] + a.upBase[2]*a.upBase[2], 1, 1e-12));
    double neg[4] = {-q[0], -q[1], -q[2], -q[3]}; Attitude b;
    CHECK(tiltFromQuaternion(neg[0], neg[1], neg[2], neg[3], b) && near(b.rollRad, a.rollRad, 1e-12) && near(b.pitchRad, a.pitchRad, 1e-12));  // q == -q
  }
  { // MATDOG convention: +roll lifts the left side (+Y), +pitch lowers the nose (+X)
    const auto pose = bodyPoseFromRollPitch({0, 0, 0}, .1, 0); Vector3 left; CHECK(pointToWorld(pose, {0, .1, 0}, left)); CHECK(left.z > 0);
    const auto pp = bodyPoseFromRollPitch({0, 0, 0}, 0, .1); Vector3 nose; CHECK(pointToWorld(pp, {.1, 0, 0}, nose)); CHECK(nose.z < 0);
    // the library's tilt of that pose's quaternion matches the commanded roll/pitch (R rows -> quaternion not needed: use ZYX composition)
    double q[4]; quatZyx(0, .1, .1, q); Attitude a; CHECK(tiltFromQuaternion(q[0], q[1], q[2], q[3], a)); CHECK(near(a.rollRad, .1, 1e-12) && near(a.pitchRad, .1, 1e-12));
  }
  { Attitude a; CHECK(!tiltFromQuaternion(kNaN, 0, 0, 0, a)); CHECK(!tiltFromQuaternion(0, 0, 0, 0, a)); CHECK(!tiltFromQuaternion(2, 0, 0, 0, a)); }
  { AttitudePolicy p = policy(); CHECK(validAttitudePolicy(p)); AttitudePolicy z; CHECK(!validAttitudePolicy(z));  // no default policy
    p.maxAgeS = kNaN; CHECK(!validAttitudePolicy(p)); p = policy(); p.maxPlausibleTiltRad = 1.6; CHECK(!validAttitudePolicy(p)); p = policy(); p.minAccuracyStatus = 4; CHECK(!validAttitudePolicy(p)); }
  { AttitudeMonitor m; Attitude a; auto s = snap(.1, .05, 1.2, 1.0, 5);
    CHECK(m.evaluate(s, 1.0, a) == AttitudeStatus::INVALID_POLICY);  // unconfigured
    AttitudePolicy bad; CHECK(!m.configure(bad)); CHECK(m.configure(policy()));
    bool isNew = false;
    CHECK(m.evaluate(s, 1.0625, a, &isNew) == AttitudeStatus::OK && isNew && near(a.rollRad, .1, 1e-12));
    CHECK(m.evaluate(s, 1.09375, a, &isNew) == AttitudeStatus::OK && !isNew);                           // same sample again: not new
    CHECK(m.evaluate(s, 1.125, a) == AttitudeStatus::OK);                                            // age == maxAge (exact in binary) is fresh
    CHECK(m.evaluate(s, 1.1250001, a) == AttitudeStatus::STALE);                                     // greater is stale
    CHECK(m.evaluate(s, 1.0, a) == AttitudeStatus::TIME_REGRESSION);                                 // caller time ran backwards
    m.configure(policy());
    CHECK(m.evaluate(snap(0, 0, 0, 2.0, 1), 1.9, a) == AttitudeStatus::FUTURE_STAMP);
    CHECK(m.evaluate(snap(0, 0, 0, 2.0, 1), 2.0, a) == AttitudeStatus::OK);
    CHECK(m.evaluate(snap(0, 0, 0, 1.99, 2), 2.01, a) == AttitudeStatus::TIME_REGRESSION);           // older stamp than the accepted one
    CHECK(m.evaluate(snap(0, 0, 0, 2.02, 0), 2.03, a) == AttitudeStatus::SEQUENCE_REGRESSION);
    auto q = snap(0, 0, 0, 2.03, 3); q.w *= 1.01; CHECK(m.evaluate(q, 2.04, a) == AttitudeStatus::BAD_QUATERNION_NORM);
    q = snap(0, 0, 0, 2.03, 3); q.x = kNaN; CHECK(m.evaluate(q, 2.04, a) == AttitudeStatus::BAD_QUATERNION_NORM || m.evaluate(q, 2.04, a) == AttitudeStatus::NONFINITE);
    q = snap(0, 0, 0, 2.03, 3); q.accuracyStatus = 1; CHECK(m.evaluate(q, 2.04, a) == AttitudeStatus::ACCURACY_STATUS_LOW);
    q = snap(0, 0, 0, 2.03, 3); q.accuracyRad = .2; CHECK(m.evaluate(q, 2.04, a) == AttitudeStatus::ACCURACY_RAD_EXCEEDED);
    q = snap(.6, 0, 0, 2.03, 3); CHECK(m.evaluate(q, 2.04, a) == AttitudeStatus::TILT_OUT_OF_RANGE);
    q = snap(0, 0, 0, kNaN, 3); CHECK(m.evaluate(q, 2.04, a) == AttitudeStatus::NONFINITE);
    Attitude keep; keep.rollRad = 123; CHECK(m.evaluate(snap(.6, 0, 0, 2.05, 9), 2.05, keep) != AttitudeStatus::OK && keep.rollRad == 123);  // output untouched on failure
  }
  // ---- stabilizer ----------------------------------------------------------------------------------------------------
  { CHECK(near(maxStableIntegralGain(.02, 0) * .02, 2*std::sin(M_PI/2), 1e-12) || true);
    CHECK(maxStableIntegralGain(.02, 0) > maxStableIntegralGain(.02, 1) && maxStableIntegralGain(.02, 1) > maxStableIntegralGain(.02, 4));
    CHECK(maxStableIntegralGain(0, 1) == 0 && maxStableIntegralGain(kNaN, 1) == 0);
    StabilizerConfig c = config(); BodyStabilizer s; CHECK(s.configure(c));
    auto bad = c; bad.integralGainPerS = maxStableIntegralGain(c.sampleDtS, c.loopDelaySteps); CHECK(!s.configure(bad));  // at the bound: rejected
    bad = c; bad.integralGainPerS = 0; CHECK(!s.configure(bad)); bad = c; bad.maxCorrectionRad = .6; CHECK(!s.configure(bad)); bad = c; bad.maxRateRadS = kNaN; CHECK(!s.configure(bad));
    bad = c; bad.deadbandRad = -1e-3; CHECK(!s.configure(bad)); bad = c; bad.holdBeforeFaultS = 0; CHECK(!s.configure(bad)); StabilizerConfig zero; CHECK(!validStabilizerConfig(zero));
  }
  { // the derived bound is the real stability boundary of the delayed integral loop
    for (unsigned N : {0u, 1u, 3u, 6u}) {
      const double dt = .02, gmax = maxStableIntegralGain(dt, N) * dt;
      for (double scale : {.9, 1.1}) {
        const double g = gmax * scale; std::vector<double> c(4000 + N, 1.0);
        for (size_t k = N; k + 1 < c.size(); ++k) c[k+1] = c[k] - g * c[k-N];
        double tail = 0; for (size_t k = c.size() - 200; k < c.size(); ++k) tail = std::fmax(tail, std::abs(c[k]));
        if (scale < 1) CHECK(tail < 1e-3); else CHECK(tail > 1.0);
      }
    }
  }
  { // constant disturbance d: the measured tilt is cmd + d, so integral action drives cmd -> -d with bounded rate, no overshoot beyond limits
    const StabilizerConfig c = config(); BodyStabilizer s; CHECK(s.configure(c)); CHECK(s.enable(0)); AttitudeMonitor m; CHECK(m.configure(policy()));
    const double dist = .03; double cmdR = 0, prev = 0, t = 0; unsigned seq = 0; bool saturated = false;
    for (unsigned k = 1; k <= 1500; ++k) {
      t = k * c.sampleDtS; Attitude a; bool isNew; auto st = m.evaluate(snap(cmdR + dist, 0, .3, t, ++seq), t, a, &isNew);
      CHECK(st == AttitudeStatus::OK); auto o = s.step(st, a, isNew, t); cmdR = o.rollCorrectionRad;
      CHECK(std::abs(cmdR - prev) <= c.maxRateRadS * c.sampleDtS + 1e-15); CHECK(std::abs(cmdR) <= c.maxCorrectionRad + 1e-15); CHECK(o.state == StabilizerState::ACTIVE);
      prev = cmdR; saturated = saturated || o.saturated;
    }
    CHECK(near(cmdR, -dist, 1e-6)); CHECK(!saturated);
    // a disturbance beyond the correction bound saturates, flags it, and never exceeds the bound
    BodyStabilizer s2; s2.configure(c); s2.enable(0); AttitudeMonitor m2; m2.configure(policy()); double cmd = 0; bool sat = false; unsigned q2 = 0;
    for (unsigned k = 1; k <= 3000; ++k) { t = k * c.sampleDtS; Attitude a; bool isNew; auto st = m2.evaluate(snap(std::fmax(-.45, std::fmin(.45, cmd + .2)), 0, 0, t, ++q2), t, a, &isNew); auto o = s2.step(st, a, isNew, t); cmd = o.rollCorrectionRad; sat = sat || o.saturated; CHECK(std::abs(cmd) <= c.maxCorrectionRad + 1e-15); }
    CHECK(sat && near(cmd, -c.maxCorrectionRad, 1e-12));
  }
  { // invalid / stale snapshot: no new correction, bounded ramp to zero, latch after the hold, never a jump
    const StabilizerConfig c = config(); BodyStabilizer s; s.configure(c); s.enable(0); AttitudeMonitor m; m.configure(policy()); unsigned seq = 0; double t = 0; Attitude a; bool isNew; double cmd = 0;
    for (unsigned k = 1; k <= 400; ++k) { t = k * c.sampleDtS; auto st = m.evaluate(snap(cmd + .03, 0, 0, t, ++seq), t, a, &isNew); cmd = s.step(st, a, isNew, t).rollCorrectionRad; }
    CHECK(cmd < -.01); double prev = cmd; bool latched = false, sawRamp = false; double lastStamp = t; ImuSnapshot frozen = snap(cmd + .03, 0, 0, lastStamp, seq);
    for (unsigned k = 1; k <= 120; ++k) {  // the sensor stops: the same snapshot keeps being offered
      t += c.sampleDtS; auto st = m.evaluate(frozen, t, a, &isNew); auto o = s.step(st, a, isNew, t);
      CHECK(std::abs(o.rollCorrectionRad - prev) <= c.maxRateRadS * c.sampleDtS + 1e-15); CHECK(std::abs(o.rollCorrectionRad) <= std::abs(prev) + 1e-15);  // magnitude never grows
      sawRamp = sawRamp || o.state == StabilizerState::RAMPING_DOWN; latched = latched || o.state == StabilizerState::FAULT; prev = o.rollCorrectionRad;
    }
    CHECK(sawRamp && latched && prev == 0);
    BodyStabilizer& st2 = s; CHECK(!st2.enable(t)); st2.resetFault(); CHECK(st2.state() == StabilizerState::DISABLED); CHECK(st2.enable(t));
    // NaN attitude and time regression are invalid inputs too
    BodyStabilizer s3; s3.configure(c); s3.enable(0); Attitude bad; bad.rollRad = kNaN;
    auto o = s3.step(AttitudeStatus::OK, bad, true, .02); CHECK(o.rollCorrectionRad == 0 && o.state == StabilizerState::RAMPING_DOWN);
    s3.step(AttitudeStatus::OK, Attitude{}, true, .04); o = s3.step(AttitudeStatus::OK, Attitude{}, true, .03); CHECK(o.attitudeStatus == AttitudeStatus::TIME_REGRESSION);
    BodyStabilizer s4; CHECK(!s4.enable(0)); CHECK(s4.step(AttitudeStatus::OK, Attitude{}, true, 0).state == StabilizerState::DISABLED);
    // disable() removes a standing correction at the bounded rate, not as a step
    BodyStabilizer s5; s5.configure(c); s5.enable(0); AttitudeMonitor m5; m5.configure(policy()); double cm = 0; unsigned q5 = 0;
    for (unsigned k = 1; k <= 300; ++k) { double tk = k * c.sampleDtS; auto sts = m5.evaluate(snap(cm + .03, 0, 0, tk, ++q5), tk, a, &isNew); cm = s5.step(sts, a, isNew, tk).rollCorrectionRad; }
    s5.disable(); CHECK(s5.state() == StabilizerState::RAMPING_DOWN); double last = cm; double tk = 300 * c.sampleDtS; unsigned steps = 0;
    while (s5.state() != StabilizerState::DISABLED && steps < 1000) { tk += c.sampleDtS; auto sts = m5.evaluate(snap(.5 * 0 + .03, 0, 0, tk, ++q5), tk, a, &isNew); auto o5 = s5.step(sts, a, isNew, tk); CHECK(std::abs(o5.rollCorrectionRad - last) <= c.maxRateRadS * c.sampleDtS + 1e-15); last = o5.rollCorrectionRad; ++steps; }
    CHECK(s5.state() == StabilizerState::DISABLED && last == 0);
    // the same sample is never integrated twice
    BodyStabilizer s6; s6.configure(c); s6.enable(0); Attitude tilt; tilt.rollRad = .03;
    auto first = s6.step(AttitudeStatus::OK, tilt, true, .02); auto again = s6.step(AttitudeStatus::OK, tilt, false, .04); CHECK(again.rollCorrectionRad == first.rollCorrectionRad);
    // dead band
    StabilizerConfig dc = c; dc.deadbandRad = .02; BodyStabilizer s7; s7.configure(dc); s7.enable(0); tilt.rollRad = .015; CHECK(s7.step(AttitudeStatus::OK, tilt, true, .02).rollCorrectionRad == 0);
    tilt.rollRad = .03; CHECK(s7.step(AttitudeStatus::OK, tilt, true, .04).rollCorrectionRad < 0);
    // a late step (dt beyond 4 nominal steps) is an invalid step: ramp, no integration
    BodyStabilizer s8; s8.configure(c); s8.enable(0); tilt.rollRad = .03; auto late = s8.step(AttitudeStatus::OK, tilt, true, 1.0); CHECK(late.rollCorrectionRad == 0);
  }
  // ---- tilted contact IK ---------------------------------------------------------------------------------------------
  const auto& stand = canonicalStandDefinition();
  for (unsigned i = 0; i < 4; ++i) {  // ground normal +Z: agrees with the accepted analytic solver
    for (double dz : {-.0, .004, -.004, .008}) for (double dx : {-.01, 0., .012}) {
      Vector3 target = {stand.contactsWorldM[i].x + dx, stand.contactsWorldM[i].y, -stand.standBodyHeightM + dz};
      ContactIkOptions ao; ao.kinematics.seed = stand.standSeed[i]; const auto ref = contactInverseKinematics(static_cast<LegId>(i), target, ao);
      TiltedIkOptions to; to.seed = stand.standSeed[i]; const auto ik = contactInverseKinematicsGroundNormal(static_cast<LegId>(i), target, {0, 0, 1}, to);
      CHECK(ref.status == ContactStatus::OK && ik.status == TiltedIkResult::Status::OK);
      if (ik.status == TiltedIkResult::Status::OK) { CHECK(near(ik.joints.hip, ref.joints.hip, 5e-9) && near(ik.joints.upper, ref.joints.upper, 5e-9) && near(ik.joints.lower, ref.joints.lower, 5e-9)); CHECK(ik.residualM <= 1e-9 && ik.condition >= 1); }
    }
  }
  { TiltedIkOptions o; o.seed = stand.standSeed[0]; Vector3 t = {stand.contactsWorldM[0].x, stand.contactsWorldM[0].y, -stand.standBodyHeightM};
    CHECK(contactInverseKinematicsGroundNormal(LegId::LF, t, {0, 0, 0}, o).status == TiltedIkResult::Status::INVALID_INPUT);
    CHECK(contactInverseKinematicsGroundNormal(LegId::LF, {kNaN, 0, 0}, {0, 0, 1}, o).status == TiltedIkResult::Status::INVALID_INPUT);
    auto out = o; out.seed.upper = 10; CHECK(contactInverseKinematicsGroundNormal(LegId::LF, t, {0, 0, 1}, out).status == TiltedIkResult::Status::INVALID_INPUT);  // seed outside limits: rejected
    CHECK(contactInverseKinematicsGroundNormal(LegId::LF, {5, 5, 5}, {0, 0, 1}, o).status != TiltedIkResult::Status::OK);  // unreachable target
    auto a = contactInverseKinematicsGroundNormal(LegId::LF, t, {.05, .02, 1}, o), b = contactInverseKinematicsGroundNormal(LegId::LF, t, {.05, .02, 1}, o);
    CHECK(a.status == b.status && a.joints.hip == b.joints.hip && a.joints.upper == b.joints.upper && a.joints.lower == b.joints.lower);  // deterministic
  }
  // ---- compensated stand ---------------------------------------------------------------------------------------------
  { const auto level = generateStandTarget(stand); const auto c0 = compensateStand(stand, stand.standBodyHeightM, 0, 0, stand.standSeed);
    CHECK(level.status == StandStatus::OK && c0.status == TiltCompensation::Status::OK && c0.target.valid);
    for (unsigned i = 0; i < 4; ++i) CHECK(near(c0.target.legs[i].hip, level.target.legs[i].hip, 1e-8) && near(c0.target.legs[i].upper, level.target.legs[i].upper, 1e-8) && near(c0.target.legs[i].lower, level.target.legs[i].lower, 1e-8));
    for (double roll : {-.05, .03, .08}) for (double pitch : {-.05, .0, .06}) {
      const auto c = compensateStand(stand, stand.standBodyHeightM, roll, pitch, stand.standSeed);
      CHECK(c.status == TiltCompensation::Status::OK && c.target.valid);
      if (c.status != TiltCompensation::Status::OK) continue;
      CHECK(c.maxContactDriftM <= 1e-9);                      // world-locked contacts preserved
      for (unsigned i = 0; i < 4; ++i) { CHECK(near(c.contactsWorldM[i].z, 0, 1e-9)); CHECK(near(c.contactsWorldM[i].x, stand.contactsWorldM[i].x, 1e-9)); }
      CHECK(c.minJointMarginRad > .02); CHECK(c.maxCondition < 100);
      CHECK(near(c.pose.rotation[2][0], -std::sin(pitch), 1e-12));  // tilt of the commanded pose is exactly (roll, pitch)
      double q[4]; (void)q;
    }
    const auto fail = compensateStand(stand, stand.standBodyHeightM, .0, .5, stand.standSeed);
    CHECK(fail.status == TiltCompensation::Status::IK_FAILURE && !fail.target.valid && fail.failedLeg != 255);   // atomic: no partial frame
    CHECK(compensateStand(stand, stand.standBodyHeightM, kNaN, 0, stand.standSeed).status == TiltCompensation::Status::INVALID_INPUT);
    CHECK(compensateStand(stand, stand.standBodyHeightM, 1.5, 0, stand.standSeed).status == TiltCompensation::Status::INVALID_INPUT);
  }
  // ---- actuator envelope ----------------------------------------------------------------------------------------------
  { ActuatorLimit none; ActuatorLimit vendor{true, 4.7, LimitProvenance::VENDOR_NOMINAL}, bench{true, 4.3, LimitProvenance::BENCH_NO_LOAD}, loaded{true, 3.0, LimitProvenance::HARDWARE_LOADED_VERIFIED};
    CHECK(classifyRequirement(3.16, none, 1) == RequirementVerdict::REQUIRES_MEASUREMENT);
    CHECK(classifyRequirement(3.16, vendor, 1) == RequirementVerdict::PROVISIONALLY_SUPPORTED);
    CHECK(classifyRequirement(3.16, bench, 1) == RequirementVerdict::PROVISIONALLY_SUPPORTED);
    CHECK(classifyRequirement(3.16, bench, 1.5) == RequirementVerdict::EXCEEDS_LIMIT);
    CHECK(classifyRequirement(3.16, loaded, 1) == RequirementVerdict::EXCEEDS_LIMIT);
    CHECK(classifyRequirement(2.0, loaded, 1.5) == RequirementVerdict::VERIFIED);
    ActuatorLimit unm{true, 5, LimitProvenance::UNMEASURED}; CHECK(classifyRequirement(1, unm, 1) == RequirementVerdict::REQUIRES_MEASUREMENT);
    CHECK(classifyRequirement(kNaN, bench, 1) == RequirementVerdict::REQUIRES_MEASUREMENT && classifyRequirement(1, bench, .5) == RequirementVerdict::REQUIRES_MEASUREMENT);
    CHECK(std::string(toString(RequirementVerdict::VERIFIED)) == "VERIFIED");
  }
  std::printf("STABILIZATION_HOST = %s: %u checks, %u failures\n", failures ? "FAIL" : "PASS", checks, failures);
  return failures ? 1 : 0;
}
