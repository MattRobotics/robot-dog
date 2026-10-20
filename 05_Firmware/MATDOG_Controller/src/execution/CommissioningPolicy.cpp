#include "CommissioningPolicy.h"
#include "../motion/OperationalWalkProfile.h"
#include "../motion/FirstStrideReference.h"
#include "../motion/PairedProneRoute.h"
#include "../motion/OperationalTolerance.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
namespace matdog { namespace execution {
namespace {
bool real(const char* s,double& v){if(!s||!*s)return false;char* e=nullptr;v=std::strtod(s,&e);return e!=s&&!*e&&std::isfinite(v);}
bool whole(const char* s,uint32_t& v){if(!s||!*s||*s=='-')return false;char* e=nullptr;errno=0;const unsigned long x=std::strtoul(s,&e,10);if(*e||errno==ERANGE||x>0xffffffffUL)return false;v=static_cast<uint32_t>(x);return true;}
bool in(double v,double lo,double hi){return std::isfinite(v)&&v>lo&&v<=hi;}
}
const char* toString(RuntimeTestMode m){switch(m){case RuntimeTestMode::OPERATIONAL:return "OPERATIONAL";case RuntimeTestMode::FIXTURE_COMMISSIONING:return "FIXTURE_COMMISSIONING";case RuntimeTestMode::GROUND_VALIDATION:return "GROUND_VALIDATION";case RuntimeTestMode::AUTONOMOUS:return "AUTONOMOUS";default:return "DIAGNOSTIC";}}
bool parseRuntimeTestMode(const char* s,RuntimeTestMode* m){
  if(!s||!m)return false;
  const RuntimeTestMode all[]={RuntimeTestMode::DIAGNOSTIC,RuntimeTestMode::FIXTURE_COMMISSIONING,RuntimeTestMode::GROUND_VALIDATION,RuntimeTestMode::AUTONOMOUS,RuntimeTestMode::OPERATIONAL};
  for(auto x:all)if(!std::strcmp(s,toString(x))){*m=x;return true;}
  return false;
}
bool runtimeModeAllowsScope(RuntimeTestMode mode,TrialScope scope){
  if(mode==RuntimeTestMode::FIXTURE_COMMISSIONING||mode==RuntimeTestMode::OPERATIONAL)return scope==TrialScope::FIXTURE_COMMISSIONING;
  if(mode==RuntimeTestMode::AUTONOMOUS)return scope==TrialScope::AUTONOMOUS_STAND||scope==TrialScope::AUTONOMOUS_WALK;
  if(mode==RuntimeTestMode::GROUND_VALIDATION)return scope==TrialScope::FIXTURE_MICRO||
    scope==TrialScope::SUPERVISED_FIXTURE_STAND||scope==TrialScope::SUPERVISED_FIXTURE_WALK||
    scope==TrialScope::AUTONOMOUS_STAND||scope==TrialScope::AUTONOMOUS_WALK;
  return false;
}
const char* toString(SegmentApproval a){switch(a){case SegmentApproval::PHASE:return "PHASE";case SegmentApproval::CONTINUOUS:return "CONTINUOUS";default:return "SEGMENT";}}
bool validCommissioningParameters(const CommissioningParameters& p,const char** reason){
  auto bad=[&](const char* r){if(reason)*reason=r;return false;};
  using C=CommissioningCaps;
  if(p.inset_ticks<2||p.inset_ticks>64)return bad("INSET_TICKS");
  if(p.prime_clamp_ticks>C::kMaxPrimeClampTicks)return bad("PRIME_CLAMP_TICKS");
  if(!in(p.max_velocity_rad_s,0,C::kMaxVelocityRadS))return bad("MAX_VELOCITY");
  if(!in(p.max_acceleration_rad_s2,0,C::kMaxAccelerationRadS2))return bad("MAX_ACCELERATION");
  if(!in(p.tracking_tolerance_rad,0,C::kMaxTrackingRad))return bad("TRACKING_TOLERANCE");
  if(!in(p.max_step_rad,0,C::kMaxStepRad))return bad("MAX_STEP");
  if(p.tracking_tolerance_rad+p.max_step_rad+motion::tolerance::kQuantizationRad+motion::tolerance::kModelUncertaintyRad>=motion::tolerance::kGeometryTubeRad)return bad("GEOMETRY_TUBE_BUDGET");
  if(!std::isfinite(p.sample_period_s)||p.sample_period_s<C::kMinSamplePeriodS||p.sample_period_s>C::kMaxSamplePeriodS)return bad("SAMPLE_PERIOD");
  if(!std::isfinite(p.entry_min_stage_s)||p.entry_min_stage_s<C::kMinStageS||p.entry_min_stage_s>C::kMaxStageS)return bad("ENTRY_MIN_STAGE");
  if(!std::isfinite(p.route_time_scale)||p.route_time_scale<C::kMinRouteScale||p.route_time_scale>C::kMaxRouteScale)return bad("ROUTE_TIME_SCALE");
  if(!in(p.prone_max_deviation_rad,0,C::kMaxProneDeviationRad)||!in(p.prone_max_rms_rad,0,p.prone_max_deviation_rad))return bad("PRONE_REGION");
  if(!p.feedback_max_age_ms||p.feedback_max_age_ms>C::kMaxFeedbackAgeMs)return bad("FEEDBACK_MAX_AGE");
  if(!p.batch_timeout_ms||p.batch_timeout_ms>C::kMaxBatchMs)return bad("BATCH_TIMEOUT");
  if(!p.tracking_timeout_ms||p.tracking_timeout_ms>C::kMaxTrackingMs)return bad("TRACKING_TIMEOUT");
  if(p.segment_timeout_ms<1000||p.segment_timeout_ms>C::kMaxSegmentMs)return bad("SEGMENT_TIMEOUT");
  if(!p.max_imu_age_ms||p.max_imu_age_ms>C::kMaxImuAgeMs)return bad("IMU_MAX_AGE");
  if(p.max_temperature_c<30||p.max_temperature_c>C::kMaxTemperatureC)return bad("MAX_TEMPERATURE");
  if(p.min_voltage_dv<60||p.max_voltage_dv>140||p.min_voltage_dv>=p.max_voltage_dv)return bad("VOLTAGE_WINDOW");
  if(p.drive_speed<1||p.drive_speed>40||p.drive_acceleration<1||p.drive_acceleration>10)return bad("DRIVE");
  if(p.transaction_timeout_ms<2||p.transaction_timeout_ms>20)return bad("TRANSACTION_TIMEOUT");
  if(!std::isfinite(p.walk_clock)||p.walk_clock<9||p.walk_clock>100)return bad("WALK_CLOCK");
  if(reason)*reason="OK";
  return true;
}
bool setCommissioningParameter(CommissioningParameters& p,const char* key,const char* value,const char** reason){
  if(reason)*reason="UNKNOWN_PARAMETER";
  if(!key||!value)return false;
  auto next=p;uint32_t u=0;bool parsed=true;
  if(!std::strcmp(key,"inset_ticks"))parsed=whole(value,u)&&u<=0xffff&&(next.inset_ticks=static_cast<uint16_t>(u),true);
  else if(!std::strcmp(key,"prime_clamp_ticks"))parsed=whole(value,u)&&u<=0xffff&&(next.prime_clamp_ticks=static_cast<uint16_t>(u),true);
  else if(!std::strcmp(key,"max_velocity_rad_s"))parsed=real(value,next.max_velocity_rad_s);
  else if(!std::strcmp(key,"max_acceleration_rad_s2"))parsed=real(value,next.max_acceleration_rad_s2);
  else if(!std::strcmp(key,"tracking_tolerance_rad"))parsed=real(value,next.tracking_tolerance_rad);
  else if(!std::strcmp(key,"max_step_rad"))parsed=real(value,next.max_step_rad);
  else if(!std::strcmp(key,"sample_period_s"))parsed=real(value,next.sample_period_s);
  else if(!std::strcmp(key,"entry_min_stage_s"))parsed=real(value,next.entry_min_stage_s);
  else if(!std::strcmp(key,"route_time_scale"))parsed=real(value,next.route_time_scale);
  else if(!std::strcmp(key,"prone_max_deviation_rad"))parsed=real(value,next.prone_max_deviation_rad);
  else if(!std::strcmp(key,"prone_max_rms_rad"))parsed=real(value,next.prone_max_rms_rad);
  else if(!std::strcmp(key,"feedback_max_age_ms"))parsed=whole(value,next.feedback_max_age_ms);
  else if(!std::strcmp(key,"batch_timeout_ms"))parsed=whole(value,next.batch_timeout_ms);
  else if(!std::strcmp(key,"tracking_timeout_ms"))parsed=whole(value,next.tracking_timeout_ms);
  else if(!std::strcmp(key,"segment_timeout_ms"))parsed=whole(value,next.segment_timeout_ms);
  else if(!std::strcmp(key,"max_imu_age_ms"))parsed=whole(value,next.max_imu_age_ms);
  else if(!std::strcmp(key,"max_temperature_c"))parsed=whole(value,u)&&u<=200&&(next.max_temperature_c=static_cast<int>(u),true);
  else if(!std::strcmp(key,"min_voltage_dv"))parsed=whole(value,u)&&u<=1000&&(next.min_voltage_dv=static_cast<int>(u),true);
  else if(!std::strcmp(key,"max_voltage_dv"))parsed=whole(value,u)&&u<=1000&&(next.max_voltage_dv=static_cast<int>(u),true);
  else if(!std::strcmp(key,"drive_speed"))parsed=whole(value,u)&&u<=0xffff&&(next.drive_speed=static_cast<uint16_t>(u),true);
  else if(!std::strcmp(key,"drive_acceleration"))parsed=whole(value,u)&&u<=0xff&&(next.drive_acceleration=static_cast<uint8_t>(u),true);
  else if(!std::strcmp(key,"transaction_timeout_ms"))parsed=whole(value,u)&&u<=0xff&&(next.transaction_timeout_ms=static_cast<uint8_t>(u),true);
  else if(!std::strcmp(key,"approval")){
    if(!std::strcmp(value,"SEGMENT"))next.approval=SegmentApproval::SEGMENT;
    else if(!std::strcmp(value,"PHASE"))next.approval=SegmentApproval::PHASE;
    else if(!std::strcmp(value,"CONTINUOUS"))next.approval=SegmentApproval::CONTINUOUS;
    else parsed=false;
  }
  else if(!std::strcmp(key,"walk_enabled"))parsed=whole(value,u)&&u<=1&&(next.walk_enabled=u==1,true);
  else if(!std::strcmp(key,"walk_clock"))parsed=real(value,next.walk_clock);
  else return false;
  if(!parsed){if(reason)*reason="VALUE_SYNTAX";return false;}
  if(!validCommissioningParameters(next,reason))return false;
  p=next;
  if(reason)*reason="PARAMETER_SET_RAM_ONLY";
  return true;
}
bool formatCommissioningParameters(const CommissioningParameters& p,char* out,size_t size){
  if(!out||!size)return false;
  const int n=std::snprintf(out,size,"inset_ticks=%u prime_clamp_ticks=%u max_velocity_rad_s=%.6g max_acceleration_rad_s2=%.6g tracking_tolerance_rad=%.6g max_step_rad=%.6g sample_period_s=%.6g entry_min_stage_s=%.6g route_time_scale=%.6g prone_max_deviation_rad=%.6g prone_max_rms_rad=%.6g feedback_max_age_ms=%lu batch_timeout_ms=%lu tracking_timeout_ms=%lu segment_timeout_ms=%lu max_imu_age_ms=%lu max_temperature_c=%d min_voltage_dv=%d max_voltage_dv=%d drive_speed=%u drive_acceleration=%u transaction_timeout_ms=%u approval=%s walk_enabled=%u walk_clock=%.6g",
    p.inset_ticks,p.prime_clamp_ticks,p.max_velocity_rad_s,p.max_acceleration_rad_s2,p.tracking_tolerance_rad,p.max_step_rad,p.sample_period_s,p.entry_min_stage_s,p.route_time_scale,
    p.prone_max_deviation_rad,p.prone_max_rms_rad,(unsigned long)p.feedback_max_age_ms,(unsigned long)p.batch_timeout_ms,(unsigned long)p.tracking_timeout_ms,(unsigned long)p.segment_timeout_ms,(unsigned long)p.max_imu_age_ms,
    p.max_temperature_c,p.min_voltage_dv,p.max_voltage_dv,p.drive_speed,p.drive_acceleration,p.transaction_timeout_ms,toString(p.approval),p.walk_enabled?1u:0u,p.walk_clock);
  return n>0&&static_cast<size_t>(n)<size;
}
bool applyOperationalStandProfile(CommissioningParameters& p,const char** reason){
  const auto& s=motion::operational_walk::kStandProfile;
  CommissioningParameters next=p;
  next.max_velocity_rad_s=s.velocityRadS;next.max_acceleration_rad_s2=s.accelerationRadS2;next.sample_period_s=s.samplePeriodS;
  next.max_step_rad=s.maxStepRad;next.entry_min_stage_s=s.minEntryStageS;
  next.tracking_tolerance_rad=motion::tolerance::kTrackingRad;
  if(!validCommissioningParameters(next,reason))return false;
  if(!motion::operational_walk::dynamicProfilesCoherent(reason))return false;   // STAND + WALK profiles: step cap, drive pair, period
  p=next;return true;
}
bool buildCommissioningRuntime(const CommissioningParameters& p,const double limits[12][2],uint32_t generation,uint32_t epoch,ReviewedMotionRuntime* out){
  if(!out||!validCommissioningParameters(p,nullptr)||!generation||!epoch)return false;
  ReviewedMotionRuntime r;r.scope=TrialScope::FIXTURE_COMMISSIONING;r.calibration_generation=generation;r.policy_epoch=epoch;
  std::strncpy(r.route_sha256,motion::pairedProneDigest(true),sizeof(r.route_sha256)-1);
  auto& f=r.feedback;f.mode=motion::FeedbackMode::FIXTURE_REVIEW;f.envelopesApproved=true;f.fixtureTrialRatesReviewed=true;
  f.loadedRatesVerified=false;f.fixtureLimitVariant=true;
  // The certified fixture route keeps its pinned proof; the entry is covered
  // by the runtime-loaded entry certificate. No cursor tube is used.
  f.routeTubeReviewed=true;f.routeFirstKnot=0;f.routeLastKnot=motion::kPairedProneSegments;f.routeJointTubeRad=.006;f.routePositionTubeM=.006;f.routeRotationTube=.006;
  f.maxJointStepRad=p.max_step_rad;f.maxTrackingErrorRad=p.tracking_tolerance_rad;f.observationMaxAgeS=2;f.assessmentMaxAgeS=2;
  f.segmentTimeoutS=p.segment_timeout_ms/1000.0;f.samplePeriodS=p.sample_period_s;f.heightToleranceM=.001;f.contactToleranceM=.001;
  for(unsigned i=0;i<12;++i){if(!std::isfinite(limits[i][0])||!std::isfinite(limits[i][1])||limits[i][0]>=limits[i][1])return false;f.jointLimits[i/3][i%3]={limits[i][0],limits[i][1]};}
  for(unsigned i=0;i<4;++i){f.maxVelocityRadS[i]={p.max_velocity_rad_s,p.max_velocity_rad_s,p.max_velocity_rad_s};f.maxAccelerationRadS2[i]={p.max_acceleration_rad_s2,p.max_acceleration_rad_s2,p.max_acceleration_rad_s2};}
  if(!motion::validFeedbackPolicy(f))return false;
  r.execution={p.max_velocity_rad_s,p.max_acceleration_rad_s2,p.tracking_tolerance_rad,p.feedback_max_age_ms,p.batch_timeout_ms,p.tracking_timeout_ms,true};
  r.stand_time_scale=p.route_time_scale;r.max_imu_age_ms=p.max_imu_age_ms;
  r.max_servo_temperature_c=p.max_temperature_c;r.min_servo_voltage_dv=p.min_voltage_dv;r.max_servo_voltage_dv=p.max_voltage_dv;
  // Bus timing is enforced live (freshness, transaction and batch timeouts),
  // not proven as a WCET; that is the commissioning policy, stated in the log.
  r.fixture_bus_timing_qualified=true;r.autonomous_bus_timing_qualified=false;
  if(p.walk_enabled){
    if(!motion::compiledFirstStrideIntegrityPinned())return false;
    // Exactly the compiled, certified first stride; only its clock is a parameter.
    auto& w=r.walk;w.compiledFirstStride=w.smoothStlReference=w.firstStrideOnly=w.stlContactReference=true;
    w.firstStrideTimeScale=p.walk_clock;w.maxJointStepRad=p.max_step_rad;w.periodS=90;w.rampDurationS=90;w.liftM=.005;w.duty=.8;
    w.swayXM=.03;w.standShiftDurationS=8;w.maxVxMps=.005/(90*p.walk_clock);w.maxVyMps=0;w.maxYawRateRadS=0;w.maxCommandTtlS=.25;
  }
  *out=r;return true;
}
} }
