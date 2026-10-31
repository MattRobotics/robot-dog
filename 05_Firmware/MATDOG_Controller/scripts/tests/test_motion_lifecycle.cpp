// Command lifecycle regression over the real service, executor, Safe Actuator
// runtime/policy, authority and verified geometry mapping. Qualification facts
// and raw intervals below are synthetic test data; they grant no physical GO.
#include <cstdio>
#include <cstring>
#include <string>
#include "calibration_record_golden.h"
#include "../../src/core/MotionService.h"

using namespace matdog;
static int checks=0, failures=0;
#define CHECK(c) do { ++checks; if(!(c)){++failures;std::printf("FAIL %d: %s\n",__LINE__,#c);} } while(0)

namespace {
class Backend final:public actuator::ActuatorBackend {
 public:
  unsigned positions=0,torque=0;
  actuator::BackendWriteOutcome enableTorque(uint8_t) override {++torque;return actuator::BackendWriteOutcome::VERIFIED_APPLIED;}
  actuator::BackendWriteOutcome writeCalibrationTorqueLimit(uint8_t) override {++torque;return actuator::BackendWriteOutcome::VERIFIED_APPLIED;}
  actuator::BackendWriteOutcome writeGoalPosition(uint8_t,uint16_t,actuator::MotionProfile) override {++positions;return actuator::BackendWriteOutcome::VERIFIED_APPLIED;}
};
class Planner final:public core::SemanticMotionPlanner {
 public:
  const char* current="IDLE";
  bool coordinated=false;
  unsigned accepts=0,stops=0;
  const char* accept(const motion::MotionCommand& c,uint32_t) override {
    ++accepts;
    if(c.intent==motion::Intent::STAND)current="OBSERVE";
    else if(c.intent==motion::Intent::BASE_VELOCITY)current="WALK_STARTING";
    return nullptr;
  }
  void stop(uint32_t) override {++stops;current=coordinated?"STOPPING":"STOPPED";}
  const char* state() const override {return current;}
  bool coordinatedStopping() const override {return coordinated;}
};
struct Fixture {
  actuator::CalibrationGeometryProfile geometry=golden::boundProfile();
  core::ActuatorAuthorityArbiter authority;
  actuator::SafeActuatorPolicy policy;
  Backend backend;
  actuator::ActuatorRuntime runtime;
  execution::MotionExecutor executor;
  execution::MotionExecutionFacts facts;
  execution::ExecutionEnvelope envelope;
  execution::JointFeedback feedback;
  Planner planner;
  core::MotionService service;
  core::MotionReadiness ready;
  Fixture(){
    authority.reset(core::AuthorityClearReason::BOOT);
    policy.begin(&authority);policy.bindGeometry(&geometry,&actuator::geometry_data::kProvenance);
    const auto record=golden::goldenRecord();
    for(unsigned i=0;i<12;i++){
      const auto& j=record.joint[i];calibration::JointIdentity identity;
      identity.leg=static_cast<calibration::Leg>(j.leg);identity.joint=static_cast<calibration::JointKind>(j.joint);
      calibration::setPhysicalUnit(&identity,j.unit);
      actuator::JointTransform transform;transform.identity=identity;transform.geometry=geometry.provenanceTag();
      transform.q0_tick=j.q0_tick;transform.present=true;transform.origin=calibration::CalibrationOrigin::LIVE_SESSION;
      transform.state=calibration::EvidenceState::PROMOTED;CHECK(policy.transforms().admit(transform));
      actuator::JointLimit limit;limit.identity=identity;limit.geometry=transform.geometry;limit.present=true;
      limit.origin=transform.origin;limit.state=transform.state;limit.min_tick=j.q0_tick-100;limit.max_tick=j.q0_tick+100;
      CHECK(policy.limits().admit(limit));feedback.read_ok[i]=feedback.torque_on[i]=true;feedback.sampled_at_ms[i]=100;
    }
    CHECK(authority.request(core::ActuatorAuthority::MOTION,core::OperatingMode::RUN,&facts.lease)==core::AuthorityResult::GRANTED);
    facts.mode=core::OperatingMode::RUN;
    facts.calibration_accepted=facts.envelopes_approved=facts.hardware_qualified=true;
    facts.operator_permit=facts.bms_ok=facts.power_ok=facts.sequential_bus_qualified=true;
    facts.permit_generation=1;facts.permit_expires_at_ms=10000;
    envelope.approved=true;envelope.max_velocity_rad_s=1;envelope.max_acceleration_rad_s2=10;
    envelope.tracking_tolerance_rad=.02;envelope.feedback_max_age_ms=100;envelope.batch_timeout_ms=100;
    envelope.tracking_timeout_ms=100;
    runtime.begin(&policy,&backend);executor.begin(&policy,&runtime,&geometry,&actuator::geometry_data::kProvenance,&authority);
    CHECK(executor.arm(facts,envelope,feedback,100));
    ready.calibration_restored=ready.calibration_accepted=ready.limits_approved=true;
    ready.hardware_approved=ready.operational_permit=ready.support_observable=true;
    ready.walk_qualified=true;ready.max_vx=.01;ready.max_vy=.01;ready.max_yaw_rate=.1; // synthetic scope approval
    ready.power_ok=ready.bms_ok=ready.authority_live=true;ready.mode=core::OperatingMode::RUN;
    service.begin(&executor,&planner);service.refresh(ready,100);
  }
};
motion::MotionCommand command(motion::Intent intent,uint32_t sequence=1,motion::ClientOwner owner=motion::ClientOwner::DASHBOARD){
  motion::MotionCommand c;c.intent=intent;c.sequence=sequence;c.owner=owner;c.issued_at_ms=100;c.ttl_ms=500;
  std::snprintf(c.id,sizeof(c.id),"lifecycle-%u-%lu",static_cast<unsigned>(owner),static_cast<unsigned long>(sequence));return c;
}
bool outcome(const motion::MotionAck& a,motion::CommandOutcome expected){return a.outcome==expected;}
void standSurvivesIndependentQueriesAndRejections(){
  Fixture f;const auto stand=command(motion::Intent::STAND);
  CHECK(outcome(f.service.command(stand,100),motion::CommandOutcome::ACCEPTED));
  const auto status=command(motion::Intent::STATUS,1,motion::ClientOwner::LOCAL);
  CHECK(outcome(f.service.command(status,101),motion::CommandOutcome::COMPLETED));
  CHECK(std::strcmp(f.service.lastAck().id,status.id)==0);
  CHECK(std::strcmp(f.service.activeAck().id,stand.id)==0);
  const auto rejected=command(motion::Intent::LIE_DOWN,2,motion::ClientOwner::LOCAL);
  CHECK(outcome(f.service.command(rejected,102),motion::CommandOutcome::REJECTED));
  CHECK(std::strcmp(f.service.lastAck().id,rejected.id)==0);
  CHECK(std::strcmp(f.service.activeAck().id,stand.id)==0);
  f.planner.current="STAND";f.service.refresh(f.ready,110);
  CHECK(outcome(f.service.activeAck(),motion::CommandOutcome::COMPLETED));
  CHECK(std::strcmp(f.service.activeAck().reason,"STAND_VERIFIED")==0);
  CHECK(outcome(f.service.command(stand,110),motion::CommandOutcome::COMPLETED));
  char json[4096]{};CHECK(f.service.statusJson(json,sizeof(json),110));
  CHECK(std::strstr(json,"\"active_command\"")!=nullptr);
  CHECK(std::strstr(json,stand.id)!=nullptr);
  auto next=command(motion::Intent::STAND,3,motion::ClientOwner::LOCAL);next.issued_at_ms=110;
  CHECK(outcome(f.service.command(next,110),motion::CommandOutcome::ACCEPTED)); // terminal command released ownership
  CHECK(f.backend.positions==0&&f.backend.torque==0);
}
void statusFromSameOwnerDoesNotCancelActiveMotion(){
  Fixture f;const auto stand=command(motion::Intent::STAND);
  CHECK(outcome(f.service.command(stand,100),motion::CommandOutcome::ACCEPTED));
  CHECK(outcome(f.service.command(command(motion::Intent::STATUS,2),101),motion::CommandOutcome::COMPLETED));
  f.planner.current="STAND";f.service.refresh(f.ready,110);
  CHECK(std::strcmp(f.service.activeAck().id,stand.id)==0);
  CHECK(outcome(f.service.activeAck(),motion::CommandOutcome::COMPLETED));
  auto other=command(motion::Intent::STAND,1,motion::ClientOwner::LOCAL);other.issued_at_ms=110;
  CHECK(outcome(f.service.command(other,110),motion::CommandOutcome::ACCEPTED));
}
void verifiedSafeOffTerminatesStop(){
  Fixture f;CHECK(outcome(f.service.command(command(motion::Intent::STAND),100),motion::CommandOutcome::ACCEPTED));
  const auto stop=command(motion::Intent::STOP,1,motion::ClientOwner::LOCAL);
  CHECK(outcome(f.service.command(stop,101),motion::CommandOutcome::ACCEPTED));
  CHECK(f.executor.status().safe_off_required);
  f.executor.confirmSafeOff(false);f.service.refresh(f.ready,102);
  CHECK(outcome(f.service.activeAck(),motion::CommandOutcome::ACCEPTED));
  f.executor.confirmSafeOff(true);f.service.refresh(f.ready,103);
  CHECK(f.executor.status().state==execution::ExecutorState::OFF);
  CHECK(outcome(f.service.activeAck(),motion::CommandOutcome::COMPLETED));
  CHECK(std::strcmp(f.service.activeAck().executor_state,"OFF")==0);
  CHECK(f.backend.positions==0&&f.backend.torque==0);
}
void coordinatedStopTerminatesAtStandWithoutTorqueWrites(){
  Fixture f;f.planner.current="STAND";
  auto velocity=command(motion::Intent::BASE_VELOCITY);velocity.vx=.001;
  CHECK(outcome(f.service.command(velocity,100),motion::CommandOutcome::ACCEPTED));
  f.planner.current="WALK";f.planner.coordinated=true;
  CHECK(outcome(f.service.command(command(motion::Intent::STOP,1,motion::ClientOwner::LOCAL),101),motion::CommandOutcome::ACCEPTED));
  f.service.refresh(f.ready,102);CHECK(outcome(f.service.activeAck(),motion::CommandOutcome::ACCEPTED));
  CHECK(!f.executor.status().safe_off_required);
  f.planner.current="STAND";f.planner.coordinated=false;f.service.refresh(f.ready,103);
  CHECK(outcome(f.service.activeAck(),motion::CommandOutcome::COMPLETED));
  CHECK(f.executor.status().state==execution::ExecutorState::READY);
  CHECK(f.backend.positions==0&&f.backend.torque==0);
}
void intentSpecificCompletionAndExpiry(){
  Fixture gait;gait.planner.current="STAND";
  auto select=command(motion::Intent::GAIT_SELECT);std::strcpy(select.gait,"WALK");
  const auto selected=gait.service.command(select,100);
  CHECK(outcome(selected,motion::CommandOutcome::COMPLETED));
  CHECK(std::strcmp(selected.reason,"GAIT_SELECTED")==0);
  Fixture walking;walking.planner.current="STAND";
  auto velocity=command(motion::Intent::BASE_VELOCITY);velocity.vx=.001;
  CHECK(outcome(walking.service.command(velocity,100),motion::CommandOutcome::ACCEPTED));
  walking.planner.current="STAND";walking.service.refresh(walking.ready,101);
  CHECK(outcome(walking.service.activeAck(),motion::CommandOutcome::ACCEPTED));
  walking.service.refresh(walking.ready,600);
  CHECK(outcome(walking.service.activeAck(),motion::CommandOutcome::ABORTED));
  CHECK(walking.executor.status().safe_off_required);
  walking.executor.confirmSafeOff(true);walking.service.refresh(walking.ready,601);
  CHECK(outcome(walking.service.activeAck(),motion::CommandOutcome::ABORTED));
}
void revocationWinsOverNominalCompletion(){
  Fixture f;CHECK(outcome(f.service.command(command(motion::Intent::STAND),100),motion::CommandOutcome::ACCEPTED));
  f.planner.current="STAND";f.ready.fault=true;f.service.refresh(f.ready,110);
  CHECK(outcome(f.service.activeAck(),motion::CommandOutcome::ABORTED));
  CHECK(f.executor.status().safe_off_required);
}
void executorTrackingFailureTerminatesCommand(){
  Fixture f;const auto stand=command(motion::Intent::STAND);
  CHECK(outcome(f.service.command(stand,100),motion::CommandOutcome::ACCEPTED));
  motion::JointTargetFrame target;target.valid=true;target.sequence=1;
  for(auto& leg:target.legs)leg.hip=leg.upper=leg.lower=.03;
  execution::FrameValidation proof;proof.sequence=1;proof.policy_epoch=f.policy.epoch();proof.geometry=f.geometry.provenanceTag();
  proof.collision_clear=proof.support_valid=proof.path_continuous=proof.configuration_valid=true;
  CHECK(f.executor.submit(target,proof,100,1000,1,f.facts,f.feedback,100));
  for(uint32_t now=101;now<=112;now++)f.executor.update(f.facts,f.feedback,now);
  CHECK(f.executor.status().state==execution::ExecutorState::TRACKING);
  for(unsigned i=0;i<12;i++)f.feedback.sampled_at_ms[i]=212;
  f.executor.update(f.facts,f.feedback,212);CHECK(f.executor.status().safe_off_required);
  CHECK(std::strcmp(f.executor.status().reason,"TRACKING_TIMEOUT")==0);
  f.service.refresh(f.ready,212);CHECK(outcome(f.service.activeAck(),motion::CommandOutcome::ABORTED));
  CHECK(outcome(f.service.command(stand,212),motion::CommandOutcome::ABORTED));
  CHECK(f.backend.positions==12&&f.backend.torque==0);
}
void rearmCannotCompleteOrRetainPriorCommand(){
  for(const auto intent:{motion::Intent::STAND,motion::Intent::BASE_VELOCITY,motion::Intent::STOP}){
    Fixture f;auto request=command(intent);if(intent==motion::Intent::BASE_VELOCITY)request.vx=.001;
    CHECK(outcome(f.service.command(request,100),motion::CommandOutcome::ACCEPTED));
    const uint32_t generation=f.executor.status().arm_generation;
    // Simulate confirmed SAFE_OFF, a distinct operational grant and separately
    // supervised prime feedback between Controller passes. No bus is written.
    f.executor.stop("SESSION_TEST");f.executor.confirmSafeOff(true);++f.facts.permit_generation;
    for(unsigned i=0;i<12;i++)f.feedback.sampled_at_ms[i]=150;
    CHECK(f.executor.arm(f.facts,f.envelope,f.feedback,150));
    CHECK(f.executor.status().arm_generation>generation);
    f.planner.current="STAND"; // stale prior-session state, before coordinator synchronization
    f.service.refresh(f.ready,150);
    CHECK(outcome(f.service.activeAck(),motion::CommandOutcome::ABORTED));
    CHECK(std::strcmp(f.service.activeAck().reason,"SESSION_CHANGED")==0);
    CHECK(f.executor.status().state==execution::ExecutorState::READY); // old command cannot stop the new arm
    CHECK(f.backend.positions==0&&f.backend.torque==0);
  }
}
void measuredFeedbackOutsideApprovedEnvelopeStopsEveryPhase(){
  for(unsigned phase=0;phase<4;phase++){
    Fixture f;f.executor.stop("ENVELOPE_TEST_SETUP");f.executor.confirmSafeOff(true);
    const auto& joint=golden::kJoints[0];calibration::JointIdentity identity;
    identity.leg=static_cast<calibration::Leg>(joint.leg);identity.joint=static_cast<calibration::JointKind>(joint.joint);
    calibration::setPhysicalUnit(&identity,joint.unit);
    const auto* transform=f.policy.transforms().find(identity,f.geometry.provenanceTag());
    auto limit=*f.policy.limits().find(identity,f.geometry.provenanceTag());
    limit.min_tick=transform->q0_tick;limit.max_tick=transform->q0_tick+100;CHECK(f.policy.limits().admit(limit));
    CHECK(f.executor.arm(f.facts,f.envelope,f.feedback,100));
    double outside=0;
    for(const auto q:{-.01,.01}){
      uint16_t raw=0;
      CHECK(actuator::resolveUrdfQToRaw(f.geometry,actuator::geometry_data::kProvenance,*transform,
            static_cast<actuator::MicroRad>(q*1e6),&raw)==actuator::TargetResolveStatus::OK);
      if(!limit.contains(raw))outside=q;
    }
    CHECK(outside!=0); // excursion is inside tracking tolerance but beyond the approved envelope
    if(phase){
      motion::JointTargetFrame target;target.valid=true;target.sequence=1;
      execution::FrameValidation proof;proof.sequence=1;proof.policy_epoch=f.policy.epoch();proof.geometry=f.geometry.provenanceTag();
      proof.collision_clear=proof.support_valid=proof.path_continuous=proof.configuration_valid=true;
      if(phase==3){
        f.feedback.actual_rad[0]=outside;
        CHECK(!f.executor.submit(target,proof,100,1000,1,f.facts,f.feedback,100));
        CHECK(f.executor.status().safe_off_required&&f.executor.status().completed_sequence==0);
        CHECK(f.backend.positions==0&&f.backend.torque==0);
        continue;
      }
      CHECK(f.executor.submit(target,proof,100,1000,1,f.facts,f.feedback,100));
      if(phase==2)for(uint32_t now=101;now<=112;now++)f.executor.update(f.facts,f.feedback,now);
    }
    const unsigned writes=f.backend.positions;const uint32_t now=phase==2?113:101;
    for(unsigned i=0;i<12;i++)f.feedback.sampled_at_ms[i]=now;
    f.feedback.actual_rad[0]=outside;f.executor.update(f.facts,f.feedback,now);
    CHECK(f.executor.status().safe_off_required);
    CHECK(f.executor.status().completed_sequence==0);
    CHECK(f.backend.positions==writes&&f.backend.torque==0);
  }
}
}
int main(){
  standSurvivesIndependentQueriesAndRejections();statusFromSameOwnerDoesNotCancelActiveMotion();
  verifiedSafeOffTerminatesStop();coordinatedStopTerminatesAtStandWithoutTorqueWrites();
  intentSpecificCompletionAndExpiry();revocationWinsOverNominalCompletion();executorTrackingFailureTerminatesCommand();
  rearmCannotCompleteOrRetainPriorCommand();
  measuredFeedbackOutsideApprovedEnvelopeStopsEveryPhase();
  std::printf("MotionLifecycle: %d checks, %d failures (offline, synthetic qualifications only)\n",checks,failures);
  return failures?1:0;
}
