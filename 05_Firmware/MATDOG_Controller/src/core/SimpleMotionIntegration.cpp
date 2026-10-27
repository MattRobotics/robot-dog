#include "Controller.h"
#include "../actuator/CalibrationTargetResolver.h"
#include "../config/BuildConfig.h"
#include <esp_timer.h>
#include <cmath>
#include <algorithm>
#include "../motion/ImuAttitude.h"
#include "../power/MotionPowerPolicy.h"
namespace matdog { namespace core {
namespace {
constexpr uint16_t kContactInsetTicks=5; // unchanged gen1 mechanical inset
uint8_t slotId(unsigned i){const auto* s=actuator::slotServo(i);return s?s->bus_id:0;}
}
bool Controller::SimpleServoPort::read(unsigned i,bool control,simple::Feedback& out){
  if(!owner||i>=12)return false;
  auto& c=*owner;auto& bus=c.servo_bus_;
  if(control){servo::ServoBus::ControlFeedbackSnapshot r;
    if(!bus.readControlFeedbackBudgeted(slotId(i),&r,simple::kReadTimeoutMs))return false;
    out.raw=r.present_position;out.speed=r.present_speed;out.load=r.present_load;out.current=r.present_current;
    out.temp=r.present_temperature;out.voltage=r.present_voltage;out.status=r.status;
    out.torque=r.torque_enable;out.goal=r.goal_position;out.goalSpeed=r.goal_speed;out.acc=r.goal_acceleration;
  }else{servo::ServoBus::FastFeedback r;
    if(!bus.readFastFeedback(slotId(i),&r,simple::kReadTimeoutMs))return false;
    out.raw=r.present_position;out.speed=r.present_speed;out.load=r.present_load;out.current=r.present_current;
    out.temp=r.present_temperature;out.voltage=r.present_voltage;out.status=r.status;
  }
  // Full Calibration recorded isolated corrupt bulk temperatures. Confirm
  // the physical temperature directly, within the same 2 ms reply budget.
  if(out.temp>=70){int direct=-1;if(!bus.readPresentTemperatureDirect(slotId(i),&direct,2))return false;out.temp=direct;}
  calibration::JointIdentity id;actuator::slotIdentity(i,&id);
  const auto* tr=c.actuator_policy_.transforms().find(id,c.geometry_profile_.provenanceTag());
  if(!tr||out.raw>=4096)return false;
  actuator::MicroRad urad=0;
  if(actuator::resolveRawObservation(c.geometry_profile_,actuator::geometry_data::kProvenance,*tr,out.raw,&urad)!=actuator::TargetResolveStatus::OK)return false;
  // Acquiring a folded pose can observe a contact outside the URDF. This
  // never expands command limits: only the unchanged recorded contact span
  // may be observed, then DISENGAGE moves LOWER inward before the route.
  const int contactMargin=c.simple_record_.repeatability[i]+calibration::LivePositionEvidence{}.extra_margin_ticks;
  if(static_cast<int>(out.raw)<static_cast<int>(c.simple_record_.contact_min[i])-contactMargin||
     static_cast<int>(out.raw)>static_cast<int>(c.simple_record_.contact_max[i])+contactMargin)return false;
  out.q=actuator::microRadToRadians(urad);out.stampUs=esp_timer_get_time();out.valid=true;if(control)out.controlStampUs=out.stampUs;
  return true;
}
bool Controller::SimpleServoPort::readDeferred() const {
  // A two-block control read can answer block40 then defer block56. Its TX
  // count is nonzero, but RX_PENDING is still no new unanswered request.
  return owner&&owner->servo_bus_.lastBusOutcome()==servo::BusOutcome::RX_PENDING;
}
bool Controller::SimpleServoPort::resolve(unsigned i,double q,uint16_t& raw){
  if(!owner||i>=12)return false;auto& c=*owner;
  calibration::JointIdentity id;actuator::slotIdentity(i,&id);
  const auto* tr=c.actuator_policy_.transforms().find(id,c.geometry_profile_.provenanceTag());
  const auto* lim=c.actuator_policy_.limits().find(id,c.geometry_profile_.provenanceTag());
  return tr&&lim&&actuator::resolveRadiansToRaw(c.geometry_profile_,actuator::geometry_data::kProvenance,*tr,q,&raw)==actuator::TargetResolveStatus::OK&&lim->contains(raw);
}
bool Controller::SimpleServoPort::frame(const uint16_t raw[12],simple::Drive drive){
  if(!owner||owner->simple_motion_.stopping())return false;
  uint8_t ids[12];for(unsigned i=0;i<12;++i)ids[i]=slotId(i);
  if(!owner->servo_bus_.setOperationalDrive(drive.speed,drive.acc))return false;
  servo::ServoBus::FrameTx tx;return owner->servo_bus_.syncWriteGoalPositions(ids,raw,&tx)&&tx.complete;
}
bool Controller::SimpleServoPort::enable(unsigned i){
  if(!owner||i>=12||owner->simple_motion_.stopping())return false;
  owner->servo_bus_.setValidationTimeout(2);
  const auto r=owner->servo_bus_.enableTorqueOn(slotId(i));
  owner->servo_bus_.setValidationTimeout(0);
  return r==servo::ServoWriteVerifyResult::VERIFIED_APPLIED;
}
bool Controller::SimpleServoPort::off(unsigned first){
  if(!owner||!build::kServoPowerAvailable||first>=13)return false;
  uint8_t ids[13];for(unsigned i=0;i<12;++i)ids[i]=slotId(i);ids[12]=51;
  return owner->servo_bus_.syncTorqueOff(ids+first,13-first)==8+2*(13-first);
}
bool Controller::SimpleServoPort::verifyOff(unsigned i){
  return owner&&i<13&&owner->servo_bus_.verifyTorqueOff(i<12?slotId(i):51,2)==servo::SafeOffResult::VERIFIED_OFF;
}
void Controller::SimpleServoPort::referenceFault(unsigned i,const simple::Feedback& f,double reference,uint16_t raw){
  Serial.printf("MOTION_REFERENCE_FAULT slot=%u id=%u torque=%d torque_bad=%u delta_bad=%u raw=%u ref_raw=%u q=%.9f ref_q=%.9f delta_ticks=%.3f valid=%u stamp_us=%llu control_stamp_us=%llu goal=%d speed=%d acc=%d\n",
    i,slotId(i),f.torque,unsigned(f.torque!=0),unsigned(std::abs(f.q-reference)>12*simple::kTickRad),
    unsigned(f.raw),unsigned(raw),f.q,reference,(f.q-reference)/simple::kTickRad,unsigned(f.valid),
    static_cast<unsigned long long>(f.stampUs),static_cast<unsigned long long>(f.controlStampUs),f.goal,f.goalSpeed,f.acc);
}
const char* Controller::prepareSimpleMotion(){
  if(!simple_limits_ready_)return "CALIBRATION_OR_LIMITS_NOT_READY";
  if(wifi_.configBusy()||authority_.inhibited())return "SYSTEM_CONFIGURATION_BUSY";
  if(calibration_.sessionLive()||first_motion_.active()||full_leg_calibration_.active()||q0_capture_.active())return "MAINTENANCE_BUSY";
  if(!imu_.taskRunning())return "IMU_ACQUISITION_TASK_REQUIRED";
  if(authority_.current()!=ActuatorAuthority::NONE&&authority_.current()!=ActuatorAuthority::MOTION)return "ACTUATOR_OWNER_BUSY";
  if(authority_.current()==ActuatorAuthority::NONE){
    operating_mode_.enterRunAutomatic();authority_.onOperatingModeChanged(OperatingMode::RUN);
    if(authority_.request(ActuatorAuthority::MOTION,OperatingMode::RUN,&motion_authority_lease_)!=AuthorityResult::GRANTED)return "ACTUATOR_OWNER_BUSY";
  }
  simple_inputs_.authorityOk=true;simple_motion_.inputs(simple_inputs_);return nullptr;
}
void Controller::updateSimpleMotion(uint64_t now){
  if(simple_motion_.stopping()){
    motion_permit_.revoke(calibration::CalibrationPermitRevokeReason::EXPLICIT);
    motion_authorization_.revoke();
    if(first_motion_.active())first_motion_.abort();
    if(full_leg_calibration_.active())full_leg_calibration_.abort();
  }
  const auto& restore=runtime_restore_.snapshot();
  if(build::kServoPowerAvailable&&simple_motion_.offVerified()&&servo_census_.state()==servo::ServoCensus::State::IDLE)
    servo_census_.start();
  const bool idle=!simple_motion_.engaged()&&authority_.current()==ActuatorAuthority::NONE&&
    !calibration_.sessionLive()&&!q0_capture_.active()&&!first_motion_.active()&&!full_leg_calibration_.active();
  if(build::kServoPowerAvailable&&idle&&simple_motion_.offVerified()&&
     servo_census_.state()==servo::ServoCensus::State::COMPLETE&&servo_census_.result().verdict==servo::CensusVerdict::PASS){
    if(!preflight_started_){servo_preflight_.start();preflight_started_=true;}
    if(servo_preflight_.state()==servo::ServoPreflight::State::COMPLETE&&servo_preflight_.result().allPass()){
      if(!restore.restored&&!restore_refused_permanent_&&!auto_restore_suspended_){
        calibration::RestoreRequestFacts request;request.trigger=calibration::RestoreTrigger::BOOT_AUTOMATIC;requestCalibrationRestore(request);
      }
      if(restore.restored&&restore.calibration_accepted&&!simple_limits_ready_){
        runtime_restore_.inspect(persistence_,geometry_profile_,actuator::geometry_data::kProvenance,&simple_record_);
        if(simple_record_.ok&&simple_record_.generation==1&&simple_record_.fingerprint==0xbe8d5902bb4e37daULL){
          calibration::RuntimeRestoreRequest facts;facts.expected_generation=restore.restored_generation;
          facts.current_installation_confirmed=true;facts.executor_idle=true;facts.torque_off_confirmed=true;facts.motion_permit_revoked=true;
          simple_limits_ready_=runtime_restore_.qualifyCommissioningLimits(geometry_profile_,actuator::geometry_data::kProvenance,kContactInsetTicks,facts,actuator_policy_.transforms(),actuator_policy_.limits())==calibration::OperationalAdmissionStatus::APPROVED;
        }
        if(simple_limits_ready_){for(unsigned i=0;i<12;++i){calibration::JointIdentity id;actuator::slotIdentity(i,&id);
          const auto* tr=actuator_policy_.transforms().find(id,geometry_profile_.provenanceTag());
          const auto* lim=actuator_policy_.limits().find(id,geometry_profile_.provenanceTag());double a=0,b=0;
          if(!tr||!lim||actuator::resolveRawToRadians(geometry_profile_,actuator::geometry_data::kProvenance,*tr,lim->min_tick,&a)!=actuator::TargetResolveStatus::OK||
             actuator::resolveRawToRadians(geometry_profile_,actuator::geometry_data::kProvenance,*tr,lim->max_tick,&b)!=actuator::TargetResolveStatus::OK){simple_limits_ready_=false;break;}
          simple_inputs_.minRad[i]=std::fmin(a,b);simple_inputs_.maxRad[i]=std::fmax(a,b);
        }}
      }
    }
  }
  if(actuator_policy_.limits().size()!=12||actuator_policy_.transforms().size()!=12)simple_limits_ready_=false;
  const bool maintenance=calibration_.sessionLive()||q0_capture_.active()||first_motion_.active()||full_leg_calibration_.active();
  const bool profile=servo_preflight_.state()==servo::ServoPreflight::State::COMPLETE&&servo_preflight_.result().allPass();
  const bool mapping=restore.restored&&restore.calibration_accepted&&actuator_policy_.transforms().size()==12&&actuator_policy_.limits().size()==12;
  const auto& b=daly_.sample();
  const auto electrical=power::motionPower(b,millis(),power_state_.state()==PowerState::RUN);
  const bool power=electrical==power::MotionPower::OK;
  simple_inputs_.powerReason=power::name(electrical);
  simple_inputs_.bmsAgeMs=b.valid?static_cast<uint32_t>(millis()-b.sampled_at_ms):UINT32_MAX;
  const auto imu=imu_.orientationSnapshot();
  now=esp_timer_get_time(); // after the atomic IMU copy: no future-age underflow
  const uint32_t rxAge=static_cast<uint32_t>(static_cast<uint32_t>(now)-imu.rx_us);
  const uint64_t sampleAge=static_cast<uint64_t>(rxAge)+(imu.sensor_age_us>0?static_cast<uint32_t>(imu.sensor_age_us):0);
  simple_inputs_.imuStampUs=sampleAge<=now?now-sampleAge:0;
  // BNO085 rotation-vector components are quantized; normalize once before
  // the existing unit-quaternion tilt extraction, as AttitudeMonitor did.
  double norm=0;for(double q:imu.wxyz)norm+=q*q;norm=std::sqrt(norm);
  const bool unit=std::isfinite(norm)&&std::abs(norm-1)<=.001;
  for(unsigned i=0;i<4;++i)simple_inputs_.quaternion[i]=unit?imu.wxyz[i]/norm:(i==0?1:0);
  motion::Attitude attitude;
  simple_inputs_.imuValid=imu.valid&&sampleAge<=200000&&imu_.taskRunning()&&unit&&
    motion::tiltFromQuaternion(simple_inputs_.quaternion[0],simple_inputs_.quaternion[1],simple_inputs_.quaternion[2],simple_inputs_.quaternion[3],attitude);
  simple_inputs_.powerOk=power;
  simple_inputs_.authorityOk=authority_.current()==ActuatorAuthority::MOTION&&!authority_.inhibited();
  simple_inputs_.ready=build::kServoPowerAvailable&&profile&&mapping&&simple_limits_ready_&&!maintenance&&!authority_.inhibited()&&!wifi_.configBusy();
  // Motion admission also needs the 12 servo feedbacks the idle polling keeps
  // fresh; that polling depends on simple_inputs_.ready only, never on this.
  const bool feedback=simple_motion_.fresh(now);
  const char* reason=!build::kServoPowerAvailable?"USB_ONLY":!simple_motion_.offVerified()&&!simple_motion_.energized()?"SAFE_OFF_IN_PROGRESS":
    !profile?"SERVO_PREFLIGHT_REQUIRED":!mapping?"CALIBRATION_RESTORE_REQUIRED":!simple_limits_ready_?"GEN1_LIMITS_OR_FINGERPRINT":maintenance?"MAINTENANCE_BUSY":
    authority_.inhibited()||wifi_.configBusy()?"SYSTEM_CONFIGURATION_BUSY":!power?simple_inputs_.powerReason:!simple_inputs_.imuValid?"IMU_UNAVAILABLE":!feedback?"SERVO_FEEDBACK_STALE":"READY";
  simple_inputs_.refusal=reason;
  simple_motion_.inputs(simple_inputs_);
  if(build::kServoPowerAvailable)simple_motion_.update(now);
  if(!simple_motion_.engaged()&&authority_.current()==ActuatorAuthority::MOTION){
    authority_.release(motion_authority_lease_);motion_authority_lease_={};servo_bus_.setValidationDrive(0,0);servo_bus_.setValidationTimeout(0);
  }
  motion_service_.setReady(simple_inputs_.ready&&power&&simple_inputs_.imuValid&&feedback,reason);motion_service_.update();
  ControllerService::ReadinessView view;view.system_health=toString(system_state_.systemHealth());
  view.calibration=mapping?"RESTORED_GEN1":"NOT_RESTORED";view.joint_feedback=feedback?"AVAILABLE":"UNAVAILABLE";
  view.motion_authority=simple_motion_.energized()?"MOTION":"IDLE";view.phase=simple::name(simple_motion_.state());view.reason=reason;service_.setReadinessView(view);
}
} }
