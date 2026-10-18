// Targeted production composition: no device transport, no physical claims.
#define MATDOG_OPERATIONAL_NO_MAIN
#include "test_operational_motion.cpp"
#include "../../src/motion/OperationalTolerance.h"
#include "frame61_replay_fixture.h"

namespace {
void measuredFrame61(bool old){
 Rig r;r.policy.transforms().clear();r.policy.limits().clear();
 for(unsigned i=0;i<12;++i){JointTransform t;t.identity=slotIdentity(i);t.geometry=r.geometry.provenanceTag();t.origin=CalibrationOrigin::LIVE_SESSION;t.state=EvidenceState::PROMOTED;t.present=true;t.q0_tick=kReplayQ0[i];CHECK(r.policy.transforms().admit(t));
  JointLimit l;l.identity=t.identity;l.geometry=t.geometry;l.origin=t.origin;l.state=t.state;l.present=true;l.min_tick=kReplayMin[i];l.max_tick=kReplayMax[i];CHECK(r.policy.limits().admit(l));
  r.feedback.actual_rad[i]=kReplayTarget[i];}
 r.envelope.frame_writes=true;r.envelope.timed_segments=!old;r.envelope.arrival_tolerance_rad=motion::tolerance::kArrivalRad;
 r.envelope.tracking_tolerance_rad=old?.006:motion::tolerance::kTrackingRad;r.envelope.tracking_timeout_ms=4000;r.envelope.feedback_max_age_ms=400;
 CHECK(r.arm());auto f=r.frame(61,0);for(unsigned i=0;i<12;++i){auto& q=f.legs[i/3];(i%3==0?q.hip:i%3==1?q.upper:q.lower)=kReplayTarget[i];}
 auto v=r.validation(61);v.final=false;CHECK(r.executor.submit(f,v,100,20000,.020,r.facts,r.feedback,100));r.executor.update(r.facts,r.feedback,101);
 for(unsigned i=0;i<12;++i){r.feedback.actual_rad[i]=qOf(r,i,kReplayRaw[i]);CHECK(std::abs(r.feedback.actual_rad[i]-kReplayActual[i])<=1e-6);r.feedback.sampled_at_ms[i]=130;
  // The USB target/goal fit was observed within one tick, not exact equality.
  CHECK(std::abs(static_cast<int>(tickOf(r,i,kReplayTarget[i]))-static_cast<int>(kReplayGoal[i]))<=1);}
 r.executor.update(r.facts,r.feedback,130);
 CHECK(r.executor.status().completed_sequence==(old?0u:61u));
 std::printf("MEASURED_FRAME61 old=%d max_error_mrad=%.3f completed=%u reason=%s\n",old,r.executor.status().max_tracking_error_rad*1000,r.executor.status().completed_sequence,r.executor.status().reason);
}
void quantizedFrame(unsigned ticks,bool final,bool old=false){
 Bench b;CHECK(b.arm());auto& r=b.r;
 // Re-arm the real executor with the candidate envelope, preserving mapping.
 r.executor.stop();r.executor.confirmSafeOff(true);
 r.envelope.frame_writes=!old;r.envelope.timed_segments=!old;
 r.envelope.tracking_tolerance_rad=old?.006:motion::tolerance::kTrackingRad;
 r.envelope.arrival_tolerance_rad=motion::tolerance::kArrivalRad;
 r.envelope.tracking_timeout_ms=4000;r.envelope.feedback_max_age_ms=400;
 uint32_t now=b.now;for(auto& at:r.feedback.sampled_at_ms)at=now;
 CHECK(r.executor.arm(r.facts,r.envelope,r.feedback,now));
 motion::JointTargetFrame f;f.valid=true;f.sequence=61;
 for(unsigned i=0;i<12;++i){auto& q=f.legs[i/3];(i%3==0?q.hip:i%3==1?q.upper:q.lower)=r.feedback.actual_rad[i];}
 auto v=r.validation(61);v.final=final;
 CHECK(r.executor.submit(f,v,now,20000,.020,r.facts,r.feedback,now));
 for(unsigned k=0;k<(old?12u:1u);++k)r.executor.update(r.facts,r.feedback,++now);
 const uint32_t sent=now;
 for(unsigned i=0;i<12;++i){
  uint16_t raw=tickOf(r,i,slot(f,i));
  // RH_UP/LH_UP: the observed five-tick semantic shortfall. Other joints
  // use their actual rounded command, rather than assigning ideal radians.
  if(i==7||i==10){const auto* g=r.geometry.findJoint(slotIdentity(i));const int sign=actuator::jointDirection(r.geometry,g->identity);raw=static_cast<uint16_t>(raw-sign*static_cast<int>(ticks));}
  r.feedback.actual_rad[i]=qOf(r,i,raw);r.feedback.speed_rad_s[i]=0;r.feedback.sampled_at_ms[i]=sent+19;
 }
 r.executor.update(r.facts,r.feedback,sent+19);
 if(!old)CHECK(r.executor.status().completed_sequence==0); // temporal release
 if(!old){
  // All samples are fresh by age but predate the last UART byte: no ACK.
  for(auto& at:r.feedback.sampled_at_ms){at=sent+1;}
  r.executor.update(r.facts,r.feedback,sent+20);CHECK(r.executor.status().completed_sequence==((!final&&ticks<=12)?61u:0u));
 }
 for(auto& at:r.feedback.sampled_at_ms)at=sent+20;
 r.executor.update(r.facts,r.feedback,sent+20);
 const bool expected=!old&&ticks<=12&&(!final||ticks<=10); // ten raw counts plus conversion rounding
 if(expected)CHECK(r.executor.status().completed_sequence==61);
 if(old){CHECK(r.executor.status().state==ExecutorState::TRACKING);
  for(auto& at:r.feedback.sampled_at_ms){at=sent+2001;}r.executor.update(r.facts,r.feedback,sent+2001);
  CHECK(r.executor.status().completed_sequence==0);}
 if(!old&&final&&ticks==12){
  CHECK(r.executor.status().state==ExecutorState::TRACKING);
  for(auto& at:r.feedback.sampled_at_ms){at=sent+2001;}r.executor.update(r.facts,r.feedback,sent+2001);
  CHECK(r.executor.status().state==ExecutorState::TRACKING);
  for(auto& at:r.feedback.sampled_at_ms){at=sent+4000;}r.executor.update(r.facts,r.feedback,sent+4000);
  REASON(r.executor.status().reason,"ARRIVAL_TIMEOUT");
 }
 if(!old&&!final&&ticks>=14){CHECK(r.executor.status().safe_off_required);REASON(r.executor.status().reason,"TRACKING_LAG_LIMIT");}
 std::printf("QUANTIZED_FRAME61 ticks=%u final=%d old=%d completed=%u reason=%s\n",ticks,final,old,r.executor.status().completed_sequence,r.executor.status().reason);
}
void assessmentLifetime(){
 motion::FeedbackPolicy p; p.assessmentMaxAgeS=2;p.mode=motion::FeedbackMode::FIXTURE_REVIEW;p.routeTubeReviewed=true;
 motion::SegmentAssessment a;a.sequence=61;a.stampS=1;a.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
 a.geometryIdentityVerified=a.sweptCollisionClear=a.supportCompatible=a.continuousBoundsConfirmed=a.fixtureTrialEnvelopeReviewed=true;
 CHECK(motion::checkAssessment(a,61,p,3.01)==motion::FeedbackReason::ASSESSMENT_STALE);
 CHECK(motion::checkAssessment(a,61,p,3.01,true)==motion::FeedbackReason::OK);
 a.sweptCollisionClear=false;CHECK(motion::checkAssessment(a,61,p,3.01,true)==motion::FeedbackReason::COLLISION);
 a.sweptCollisionClear=true;a.source=motion::EvidenceSource::EXTERNAL_MEASURED;
 CHECK(motion::checkAssessment(a,61,p,3.01,true)==motion::FeedbackReason::ASSESSMENT_STALE);
}
void stopAndStale(){
 for(unsigned kind=0;kind<6;++kind){Rig r;r.envelope.frame_writes=true;r.envelope.timed_segments=true;
  r.envelope.tracking_tolerance_rad=motion::tolerance::kTrackingRad;r.envelope.arrival_tolerance_rad=motion::tolerance::kArrivalRad;
  CHECK(r.arm());auto f=r.frame(61,0);auto v=r.validation(61);v.final=false;CHECK(r.executor.submit(f,v,100,1000,.020,r.facts,r.feedback,100));
  r.executor.update(r.facts,r.feedback,101);const auto writes=r.backend.writes.size();
  if(kind==0)r.executor.update(r.facts,r.feedback,201);
  if(kind==1){r.fresh(102);r.facts.power_ok=false;r.executor.update(r.facts,r.feedback,102);}
  if(kind==2)r.executor.stop("OPERATOR_STOP");
  if(kind==3){r.feedback.torque_on[0]=false;r.executor.update(r.facts,r.feedback,102);}
  if(kind==4){r.feedback.read_ok[0]=false;r.executor.update(r.facts,r.feedback,102);}
  if(kind==5){r.feedback.sampled_at_ms[0]=103;r.executor.update(r.facts,r.feedback,102);}
  CHECK(r.executor.status().safe_off_required);const char* reasons[]={"STALE_FEEDBACK","POWER_OR_BMS","OPERATOR_STOP","TORQUE_LOST","FEEDBACK_INVALID","FEEDBACK_CLOCK_INVALID"};
  REASON(r.executor.status().reason,reasons[kind]);
  r.executor.update(r.facts,r.feedback,202);CHECK(writes==r.backend.writes.size());
 }
 // The service must report the executor cause before the permit consequence.
 Rig r;Planner planner;MotionService service;service.begin(&r.executor,&planner);service.refresh(ready(),100);CHECK(r.arm());
 CHECK(service.command(command(motion::Intent::STAND),100).outcome==motion::CommandOutcome::ACCEPTED);
 r.executor.stop("STALE_FEEDBACK");auto rd=ready();rd.operational_permit=false;service.refresh(rd,101);REASON(service.activeAck().reason,"STALE_FEEDBACK");
}
void finiteClockWrap(){
 Rig r;const uint32_t start=UINT32_MAX-10;r.envelope.frame_writes=r.envelope.timed_segments=true;
 r.envelope.arrival_tolerance_rad=motion::tolerance::kArrivalRad;r.facts.permit_expires_at_ms=1000;
 for(auto& at:r.feedback.sampled_at_ms){at=start;}
 CHECK(r.executor.arm(r.facts,r.envelope,r.feedback,start));auto f=r.frame(61,0);auto v=r.validation(61);v.final=false;
 CHECK(r.executor.submit(f,v,start,1000,.020,r.facts,r.feedback,start));r.executor.update(r.facts,r.feedback,start+1);
 for(unsigned i=0;i<12;++i){r.feedback.sampled_at_ms[i]=12;r.feedback.sampled_at_us[i]=(static_cast<uint64_t>(UINT32_MAX)+13)*1000;}
 r.executor.update(r.facts,r.feedback,12);CHECK(r.executor.status().completed_sequence==61);
}
void numericalIk(){
 for(unsigned leg=0;leg<4;++leg)for(double upper:{.2,.7,1.2}){
  motion::LegJointAngles q{.02,upper,-.6};auto fk=motion::forwardKinematics(static_cast<motion::LegId>(leg),q);CHECK(fk.status==motion::KinematicsStatus::OK);
  motion::IkOptions options;options.seed=q;CHECK(options.toleranceM==1e-9);
  auto ik=motion::inverseKinematics(static_cast<motion::LegId>(leg),fk.footOriginM,options);CHECK(ik.status==motion::KinematicsStatus::OK);CHECK(ik.residualM<=1e-9);
  CHECK(std::abs(ik.joints.hip-q.hip)<1e-8&&std::abs(ik.joints.upper-q.upper)<1e-8&&std::abs(ik.joints.lower-q.lower)<1e-8);
 }
}
void heightRanges(){
 motion::StandDefinition d=motion::canonicalStandDefinition();d.lowBodyHeightM=.060;
 double lo[12],hi[12];for(unsigned i=0;i<12;++i){lo[i]=1e9;hi[i]=-1e9;}
 motion::LegJointAngles seed[4];for(unsigned i=0;i<4;++i)seed[i]=d.standSeed[i];
 for(int k=900;k>=0;--k){auto s=motion::sampleStandPath(d,k/900.,seed);CHECK(s.status==motion::StandStatus::OK);
  for(unsigned i=0;i<12;++i){const double q=slot(s.target,i);lo[i]=std::min(lo[i],q);hi[i]=std::max(hi[i],q);}for(unsigned i=0;i<4;++i)seed[i]=s.target.legs[i];}
 std::printf("HEIGHT_TARGET_BOX [");for(unsigned i=0;i<12;++i)std::printf("%s[%.12f,%.12f]",i?",":"",lo[i],hi[i]);std::printf("]\n");
}
}
int main(){
 stand_backend::run();postValidationAdmissionClock();operationalRefusals();
 measuredFrame61(true);measuredFrame61(false);
 quantizedFrame(5,false,true);
 for(unsigned n:{5u,8u,10u,12u}){quantizedFrame(n,false);quantizedFrame(n,true);}
 quantizedFrame(20,false);assessmentLifetime();stopAndStale();finiteClockWrap();numericalIk();heightRanges();
 StreamModel band;band.name="V1_STOP_SHORT_5_TICKS";band.stop_short_ticks=5;CHECK(standTiming(false,band,0)>0);
 // A loaded LOWER limit wider than the proved folded domain is refused.
 {Operational op;auto p=op.b.runtime.feedback;p.jointLimits[0][2].lower=-1.605;
  op.b.r.executor.stop();op.b.r.executor.confirmSafeOff(true);
  motion::AcquisitionRoute route;route.pairedProne=true;route.geometryIdentityVerified=true;route.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
  CHECK(op.b.coordinator.configure(p,op.b.runtime.walk,&route));auto cc=op.b.coordinator.commissioning();CHECK(!op.b.coordinator.configureCommissioning(cc));}
 variableStandHeight(); // 60/100/140 -> 150 -> WALK admission, STOP during adjust
 heartbeatCoupling();
 StreamModel m;m.name="V1_STANDARD_QUANTIZED";auto walk=streamWalk(1,m,1);CHECK(walk.ok);
 std::printf("TOLERANCE_V1 %s checks=%d failures=%d hardware=false\n",failures?"FAIL":"PASS",checks,failures);
 return failures?1:0;
}
