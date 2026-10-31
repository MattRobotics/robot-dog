// Controller composition seam: production service, coordinator, planner,
// executor, mapping and Safe Actuator layer. All hardware facts are declared
// synthetic doubles; no device/authority endpoint is used.
#define main prior_motion_boundary_main
#include "test_motion_service_executor.cpp"
#undef main
#include "../../src/execution/OperationalFeedback.h"
#include <fstream>

motion::FeedbackPolicy pairedPolicy(){
 motion::FeedbackPolicy p;p.mode=motion::FeedbackMode::PHYSICAL_REVIEW;
 p.envelopesApproved=p.loadedRatesVerified=true;p.maxJointStepRad=.05;p.maxTrackingErrorRad=.001;
 p.observationMaxAgeS=.5;p.assessmentMaxAgeS=.5;p.segmentTimeoutS=2;
 p.heightToleranceM=1e-5;p.contactToleranceM=1e-5;p.samplePeriodS=.02;
 for(unsigned i=0;i<4;++i){const auto* m=motion::legModel(static_cast<motion::LegId>(i));
  for(unsigned j=0;j<3;++j)p.jointLimits[i][j]=m->limits[j];
  p.maxVelocityRadS[i]={.25,.25,.25};p.maxAccelerationRadS2[i]={2,2,2};}
 return p;
}
void observations(motion::AdaptiveObservation& o,Rig& rig,const motion::LocomotionFrame& f,uint32_t now){
 o.actual=f.target;o.body=f.cartesian.body;o.stampS=now/1000.;o.stationary=true;o.flatPlaneConfirmed=true;
 o.heightSource=o.supportSource=motion::EvidenceSource::EXTERNAL_MEASURED; // simulated instruments
 for(unsigned i=0;i<12;++i){const auto q=f.target.legs[i/3];rig.feedback.actual_rad[i]=i%3==0?q.hip:i%3==1?q.upper:q.lower;rig.feedback.sampled_at_ms[i]=now;}
}
void producerGates(){
 Rig r;OperationalFeedback producer;
 for(unsigned i=0;i<12;++i){RawJointObservation s;s.read_ok=true;s.position=golden::goldenRecord().joint[i].q0_tick;
  s.torque=0;s.status=0;s.temperature=30;s.voltage=120;s.sampled_at_ms=100;
  producer.ingest(i,s,r.geometry,geometry_data::kProvenance,r.policy.transforms());}
 CHECK(producer.rawFresh(101,500));auto f=producer.snapshot(101,500);
 for(unsigned i=0;i<12;++i)CHECK(f.read_ok[i]&&!f.torque_on[i]&&std::abs(f.actual_rad[i])<1e-12);
 CHECK(!producer.snapshot(601,500).read_ok[0]);CHECK(!producer.snapshot(99,500).read_ok[0]);
 auto s=producer.raw(11);s.read_ok=false;s.sampled_at_ms=102;
 producer.ingest(11,s,r.geometry,geometry_data::kProvenance,r.policy.transforms());
 CHECK(!producer.snapshot(102,500).read_ok[11]&&!producer.rawFresh(102,500));
 s.read_ok=true;s.temperature=80;producer.ingest(11,s,r.geometry,geometry_data::kProvenance,r.policy.transforms());CHECK(!producer.snapshot(102,500).read_ok[11]);
}
void sessionLifecycle(){
 Rig r;CHECK(r.arbiter.release(r.facts.lease)==AuthorityResult::RELEASED);r.facts.lease={};
 for(auto& on:r.feedback.torque_on)on=false;
 MotionSession session;session.begin(&r.policy,&r.runtime,&r.executor,&r.arbiter,&r.geometry,&geometry_data::kProvenance);
 LocalTrialEvidence e;e.generation=1;e.expires_at_ms=10000;e.max_fixture_excursion_rad=.02;
 e.local_operator_present=e.physical_power_cut_verified=e.fixture_support_verified=e.installation_verified=e.servo_output_ceiling_verified=e.imu_fresh=true;
 CHECK(!session.request(e,r.facts,r.envelope,r.feedback,false,100));CHECK(r.backend.writes.empty()&&r.arbiter.current()==ActuatorAuthority::NONE);
 auto missing=e;missing.fixture_support_verified=false;CHECK(!session.request(missing,r.facts,r.envelope,r.feedback,true,100));
 CHECK(session.request(e,r.facts,r.envelope,r.feedback,true,100));
 uint32_t now=100;
 for(unsigned i=0;i<24;++i){r.fresh(++now);session.update(e,r.facts,r.feedback,now);}
 CHECK(r.backend.writes.size()==12&&r.backend.torque_calls==12);
 for(auto& on:r.feedback.torque_on)on=true;
 r.facts.prime_profiles_verified=false;
 r.fresh(++now);session.update(e,r.facts,r.feedback,now);
 CHECK(session.state()==MotionSessionState::ARMING&&!r.executor.active());
 r.facts.prime_profiles_verified=true;
 r.fresh(++now);session.update(e,r.facts,r.feedback,now);
 CHECK(session.state()==MotionSessionState::ARMED&&r.executor.active());r.facts.lease=session.lease();
 MotionCoordinator coordinator;coordinator.begin(&r.executor,&r.policy);coordinator.bindSession(&session);
 MotionService fixtureService;fixtureService.begin(&r.executor,&coordinator);auto fixtureReady=ready();
 fixtureReady.local_fixture_session=true;fixtureReady.support_observable=false;
 fixtureService.refresh(fixtureReady,now);CHECK(r.executor.active());REASON(fixtureService.refusal(),"LOCAL_FIXTURE_MICRO_ONLY");
 CHECK(!session.requestFixtureMicro(12,.01,1,r.validation(),r.facts,r.feedback,now));
 CHECK(!session.requestFixtureMicro(0,.03,1,r.validation(),r.facts,r.feedback,now));
 CHECK(session.requestFixtureMicro(0,.01,1,r.validation(),r.facts,r.feedback,now));
 for(unsigned k=0;k<21;++k){
  now+=50;for(unsigned i=0;i<12;++i)r.feedback.sampled_at_ms[i]=now;
  session.update(e,r.facts,r.feedback,now);
  if(r.executor.status().state==ExecutorState::WRITING){
   for(unsigned i=0;i<12;++i)r.executor.update(r.facts,r.feedback,++now);
   for(unsigned i=0;i<12;++i){r.feedback.actual_rad[i]=r.executor.targetRadians()[i];r.feedback.sampled_at_ms[i]=now+1;}
   r.executor.update(r.facts,r.feedback,++now);
  }
 }
 REASON(session.reason(),"FIXTURE_MICRO_TRACKED");CHECK(r.executor.status().completed_sequence==20);CHECK(r.backend.writes.size()==252);
 e.fixture_support_verified=false;session.update(e,r.facts,r.feedback,++now);CHECK(session.safeOffRequired()&&r.executor.status().safe_off_required);
 const auto writes=r.backend.writes.size();session.update(e,r.facts,r.feedback,++now);CHECK(writes==r.backend.writes.size());
 r.executor.confirmSafeOff(true);session.confirmSafeOff();CHECK(r.arbiter.current()==ActuatorAuthority::NONE);
}
void sessionDenials(){
 for(unsigned failure=0;failure<9;++failure){Rig r;r.arbiter.release(r.facts.lease);r.facts.lease={};
  for(auto& on:r.feedback.torque_on){on=false;}
  r.fresh(600);
  MotionSession session;session.begin(&r.policy,&r.runtime,&r.executor,&r.arbiter,&r.geometry,&geometry_data::kProvenance);
  LocalTrialEvidence e;e.generation=1;e.expires_at_ms=1000;
  e.local_operator_present=e.physical_power_cut_verified=e.fixture_support_verified=e.installation_verified=e.servo_output_ceiling_verified=e.imu_fresh=true;
  if(failure==0)e.local_operator_present=false;
  if(failure==1)e.physical_power_cut_verified=false;
  if(failure==2)e.imu_fresh=false;
  if(failure==3)r.facts.bms_ok=false;
  if(failure==4)r.facts.sequential_bus_qualified=false;
  if(failure==5)r.feedback.sampled_at_ms[11]=0;
  if(failure==6)r.feedback.torque_on[4]=true;
  if(failure==7){e.scope=TrialScope::AUTONOMOUS_STAND;e.support_load_observable=false;}
  if(failure==8){e.scope=TrialScope::AUTONOMOUS_WALK;e.support_load_observable=true;e.loaded_dynamics_qualified=false;}
  CHECK(!session.request(e,r.facts,r.envelope,r.feedback,true,600));
  CHECK(r.backend.writes.empty()&&r.backend.torque_calls==0&&r.arbiter.current()==ActuatorAuthority::NONE);
 }
}
int main(int argc,char** argv){
 sessionDenials();
 CHECK(argc==2);if(argc!=2)return 1;
 // Frozen host oracle includes corrected base Z, pitch, and all 12 targets.
 std::ifstream oracle(argv[1]);unsigned segment;double u,expected;
 while(oracle>>segment>>u){const auto f=motion::pairedProneSample(segment,u);CHECK(f.target.valid);
  for(unsigned i=0;i<12;++i){oracle>>expected;auto q=f.target.legs[i/3];double actual=i%3==0?q.hip:i%3==1?q.upper:q.lower;CHECK(std::abs(actual-expected)<2e-12);}
  oracle>>expected;CHECK(std::abs(f.cartesian.body.translationM.x-expected)<2e-12);
  oracle>>expected;CHECK(std::abs(f.cartesian.body.translationM.z-expected)<2e-12);
  oracle>>expected;CHECK(std::abs(std::atan2(f.cartesian.body.rotation[0][2],f.cartesian.body.rotation[0][0])-expected)<2e-12);
 }
 CHECK(!motion::pairedProneSample(251,0).target.valid);CHECK(!motion::pairedProneSample(0,-.1).target.valid);
 Rig rig;auto record=golden::goldenRecord();
 for(unsigned i=0;i<12;++i){JointIdentity id;id.leg=static_cast<Leg>(i/3);id.joint=static_cast<JointKind>(i%3);setPhysicalUnit(&id,record.joint[i].unit);
  const auto* t=rig.policy.transforms().find(id,rig.geometry.provenanceTag());const auto* g=rig.geometry.findJoint(id);
  uint16_t lo,hi;CHECK(resolveUrdfQToRaw(rig.geometry,geometry_data::kProvenance,*t,g->urdf_lower,&lo)==TargetResolveStatus::OK);
  CHECK(resolveUrdfQToRaw(rig.geometry,geometry_data::kProvenance,*t,g->urdf_upper,&hi)==TargetResolveStatus::OK);
  JointLimit l;l.identity=id;l.geometry=rig.geometry.provenanceTag();l.origin=CalibrationOrigin::LIVE_SESSION;l.state=EvidenceState::PROMOTED;l.present=true;l.min_tick=std::min(lo,hi);l.max_tick=std::max(lo,hi);CHECK(rig.policy.limits().admit(l));
 }
 rig.facts.permit_expires_at_ms=1000000;rig.envelope.max_velocity_rad_s=.3;rig.envelope.max_acceleration_rad_s2=5;
 MotionCoordinator coordinator;coordinator.begin(&rig.executor,&rig.policy);
 motion::AcquisitionRoute route;route.pairedProne=true;route.geometryIdentityVerified=true;route.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
 const auto policy=pairedPolicy();CHECK(coordinator.configure(policy,{},&route));
 MotionService service;service.begin(&rig.executor,&coordinator);service.refresh(ready(),100);
 auto initial=motion::pairedProneSample(0,0);motion::AdaptiveObservation o;observations(o,rig,initial,100);o.bellySupported=true;o.confirmedFeet=0;
 CHECK(rig.arm());coordinator.observe(o);
 auto c=command(motion::Intent::STAND);c.target_height_m=.150;CHECK(service.command(c,100).outcome==motion::CommandOutcome::ACCEPTED);
 uint32_t now=100;unsigned frames=0;
 while(std::strcmp(coordinator.state(),"STAND")&&now<500000){
  now+=20;observations(o,rig,initial,now);coordinator.observe(o);coordinator.update(rig.facts,rig.feedback,now);
  auto frame=coordinator.frame();
  if(frame.target.valid && frame.target.sequence>rig.executor.status().completed_sequence){
   motion::SegmentAssessment a;a.sequence=frame.target.sequence;a.stampS=now/1000.;a.source=motion::EvidenceSource::EXTERNAL_MEASURED;
   a.geometryIdentityVerified=a.sweptCollisionClear=a.supportCompatible=a.continuousBoundsConfirmed=a.loadedTrackingQualified=true;
   CHECK(coordinator.assess(a,now));coordinator.update(rig.facts,rig.feedback,now);
   CHECK(rig.executor.status().state==ExecutorState::WRITING);
   for(unsigned i=0;i<12;++i)rig.executor.update(rig.facts,rig.feedback,++now);
   initial.target=frame.target;initial.cartesian.body=frame.body;
   o.bellySupported=frame.bellySupportRequired;o.confirmedFeet=frame.requiredFeet;
   observations(o,rig,initial,++now);rig.executor.update(rig.facts,rig.feedback,now);
   CHECK(rig.executor.status().completed_sequence==frame.target.sequence);++frames;
  }
  if(!std::strcmp(coordinator.state(),"REJECTED")){std::printf("route rejected: %s\n",motion::feedbackReasonName(coordinator.frame().reason));break;}
 }
 CHECK(!std::strcmp(coordinator.state(),"STAND"));CHECK(frames>3000);CHECK(rig.backend.writes.size()==frames*12);CHECK(rig.backend.torque_calls==0);
 producerGates();sessionLifecycle();
 std::printf("PAIRED_RUNTIME %s checks=%d frames=%u mapped_writes=%zu hardware=false\n",failures?"FAIL":"PASS",checks,frames,rig.backend.writes.size());return failures?1:0;
}
