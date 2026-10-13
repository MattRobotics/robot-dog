#define main baseline_paired_main
#include "test_motion_service_executor.cpp"
#include "../../src/motion/TimedStand.h"
motion::FeedbackPolicy pairedPolicy(){motion::FeedbackPolicy p;p.mode=motion::FeedbackMode::PHYSICAL_REVIEW;p.envelopesApproved=p.loadedRatesVerified=true;p.maxJointStepRad=.05;p.maxTrackingErrorRad=.001;p.observationMaxAgeS=.5;p.assessmentMaxAgeS=.5;p.segmentTimeoutS=2;p.heightToleranceM=1e-5;p.contactToleranceM=1e-5;p.samplePeriodS=.02;for(unsigned i=0;i<4;++i){auto* m=motion::legModel(static_cast<motion::LegId>(i));for(unsigned j=0;j<3;++j)p.jointLimits[i][j]=m->limits[j];p.maxVelocityRadS[i]={.25,.25,.25};p.maxAccelerationRadS2[i]={2,2,2};}return p;}
#undef main
#include "stl_gait_adapter.cpp"
#include <fstream>
#include "../../src/motion/FirstStrideReference.h"
// Instruments/servo completion are simulated. Fixed fixture height never
// follows virtual route height. Compare the actual FeedbackWalk planner.
int main(){
 std::ofstream trace;const char* path=std::getenv("MATDOG_SUPERVISED_TRACE");if(path){trace.open(path);trace<<"phase,time_s,segment,sequence,actual_fixture_z_m,target_x_m,target_z_m,pitch_rad";for(unsigned i=0;i<12;++i)trace<<",q"<<i;trace<<"\n";}
 auto record=[&](const char* phase,double now,unsigned segment,const motion::FeedbackFrame& f){if(!trace)return;trace<<phase<<","<<now<<","<<segment<<","<<f.target.sequence<<",0.3,"<<f.body.translationM.x<<","<<f.body.translationM.z<<","<<atan2(f.body.rotation[0][2],f.body.rotation[0][0]);for(auto q:f.target.legs)trace<<","<<q.hip<<","<<q.upper<<","<<q.lower;trace<<"\n";};
 Rig rig;auto p=pairedPolicy();p.mode=motion::FeedbackMode::FIXTURE_REVIEW;p.fixtureTrialRatesReviewed=true;p.loadedRatesVerified=false;p.routeTubeReviewed=true;p.fixtureLimitVariant=true;p.routeFirstKnot=0;p.routeLastKnot=251;p.routeJointTubeRad=.0012;p.routePositionTubeM=.001;p.routeRotationTube=.001;
 p.samplePeriodS=.05;p.maxJointStepRad=.0015;p.maxTrackingErrorRad=.0009;
 unsigned failed=99;CHECK(motion::compiledFirstStrideFitsLimits(p,motion::compiledFirstStrideTubeRad(),&failed));
 auto narrow=p;narrow.jointLimits[1][2].upper=25*3.141592653589793/180;CHECK(!motion::compiledFirstStrideFitsLimits(narrow,motion::compiledFirstStrideTubeRad(),&failed)&&failed==5);CHECK(!motion::compiledFirstStrideFitsLimits(p,.006,&failed));
 motion::AcquisitionRoute route;route.pairedProne=true;route.supervisedSegments=true;route.timeScale=9;route.geometryIdentityVerified=true;route.source=motion::EvidenceSource::OFFLINE_CERTIFIED;
 motion::AdaptiveObservation o;o.actual=motion::pairedProneSample(0,0,true).target;o.body.translationM.z=.3;o.externalSupportConfirmed=o.flatPlaneConfirmed=o.stationary=true;o.heightSource=o.supportSource=motion::EvidenceSource::EXTERNAL_MEASURED;
 double now=1;o.stampS=now;motion::AdaptiveStand stand;motion::AdaptiveStandRequest request;request.targetHeightM=.15;request.acquisition=&route;CHECK(stand.request(request,p,now));
 // Recover measured intermediate positions only inside the common certified
 // progress corridor. Opposite pair perturbations must never select a route.
 for(unsigned k:{3u,18u,40u,120u,230u}){
  auto measured=o;measured.actual=motion::pairedProneSample(k,.37,true).target;
  const auto cursor=motion::pairedProneCursor(measured,p);CHECK(cursor.segment>=0&&cursor.progress>=0&&cursor.progress<1);
  if(cursor.segment>=0){const auto selected=motion::pairedProneSample(cursor.segment,0,true,cursor.progress);
   for(unsigned j=0;j<12;++j){auto x=measured.actual.legs[j/3],y=selected.target.legs[j/3];CHECK(std::abs((j%3==0?x.hip:j%3==1?x.upper:x.lower)-(j%3==0?y.hip:j%3==1?y.upper:y.lower))<=p.routeJointTubeRad+1e-12);}}
  measured.actual.legs[0].upper+=.02;measured.actual.legs[1].upper-=.02;CHECK(motion::pairedProneMatch(measured,p)<0);
 }
 auto unknown=o;unknown.actual.legs[2].hip=.2;CHECK(motion::pairedProneMatch(unknown,p)<0);
 unsigned frames=0,segments=0;uint32_t sequence=0;unsigned last=999;
 for(unsigned loop=0;loop<60000&&stand.state()!=motion::AdaptiveStandState::STAND;++loop){now+=.051;o.stampS=now;auto f=stand.update(o,now);
  if(f.target.valid&&!f.executable){motion::SegmentAssessment a;a.sequence=f.target.sequence;a.stampS=now;a.source=motion::EvidenceSource::OFFLINE_CERTIFIED;a.fixtureTrialEnvelopeReviewed=true;a.geometryIdentityVerified=a.sweptCollisionClear=a.continuousBoundsConfirmed=a.supportCompatible=a.loadedTrackingQualified=true;CHECK(stand.assess(a,now));f=stand.update(o,now);}
  if(f.target.valid&&f.executable&&f.target.sequence!=sequence){CHECK(f.requiredFeet==0&&f.externalSupportRequired);CHECK(o.body.translationM.z==.3);o.actual=f.target;o.acknowledged=true;o.acknowledgedSequence=f.target.sequence;sequence=f.target.sequence;++frames;record("STAND",now,stand.proneSegment(),f);}
  if(!f.target.valid&&f.reason==motion::FeedbackReason::ASSESSMENT_REQUIRED){const unsigned segment=stand.proneSegment();CHECK(!stand.approveProneSegment(segment+1));CHECK(stand.approveProneSegment(segment));if(segment!=last){++segments;last=segment;}}
  if(stand.state()==motion::AdaptiveStandState::REJECTED){printf("fixture stand rejected %s frame=%u segment=%u\n",motion::feedbackReasonName(f.reason),frames,stand.proneSegment());break;}
 }
 CHECK(stand.state()==motion::AdaptiveStandState::STAND);CHECK(segments==251&&frames>7000);CHECK(o.confirmedFeet==0&&o.body.translationM.z==.3);
 motion::WalkSpecification s;s.periodS=s.rampDurationS=90;s.liftM=.005;s.duty=.8;s.swayXM=.03;s.standShiftDurationS=8;s.compiledFirstStride=s.firstStrideOnly=s.smoothStlReference=s.stlContactReference=true;s.maxVxMps=.005/(90*9);s.maxVyMps=s.maxYawRateRadS=0;s.maxCommandTtlS=.25;s.firstStrideTimeScale=9;s.maxJointStepRad=.004;
 motion::PlanarVelocityCommand c;c.vxMps=s.maxVxMps;c.stampS=now;c.ttlS=.2;c.sequence=1;
 motion::FeedbackWalk walk;CHECK(walk.start(stand,o,c,s,p,now));unsigned walkFrames=0;double maxError=0;
 for(unsigned loop=0;loop<50000&&walk.state()!=motion::FeedbackWalkState::STAND;++loop){now+=.051;o.stampS=now;c.stampS=now;++c.sequence;CHECK(walk.command(c,now));auto f=walk.update(o,now);
  if(f.target.valid&&!f.executable){motion::SegmentAssessment a;a.sequence=f.target.sequence;a.stampS=now;a.source=motion::EvidenceSource::OFFLINE_CERTIFIED;a.fixtureTrialEnvelopeReviewed=true;a.geometryIdentityVerified=a.sweptCollisionClear=a.continuousBoundsConfirmed=a.supportCompatible=a.loadedTrackingQualified=true;CHECK(walk.assess(a,now));f=walk.update(o,now);}
  if(f.target.valid&&f.executable&&f.target.sequence!=sequence){++walkFrames;const double t=walkFrames*.05/9;double cycle=0,rate=0,acc=0,x=0,v=0,a=0;int hold=0;
   if(t<8){auto law=motion::quinticTimeLaw(t/8,8);x=-.03*law.progress;v=-.03*law.velocity;a=-.03*law.acceleration;hold=1;}
   else if(t>=98){auto law=motion::quinticTimeLaw(std::fmin(1.,(t-98)/8),8);cycle=1;x=-.03*(1-law.progress);v=.03*law.velocity;a=.03*law.acceleration;hold=1;}
   else{auto law=motion::quinticTimeLaw((t-8)/90,90);cycle=law.progress;rate=law.velocity;acc=law.acceleration;}
   double expected[40]{};CHECK(replay_soft_qualified(cycle,rate,acc,x,v,a,hold,0,.001,expected)==0);
   for(unsigned i=0;i<12;++i){auto q=f.target.legs[i/3];const double actual=i%3==0?q.hip:i%3==1?q.upper:q.lower;maxError=std::fmax(maxError,std::abs(actual-expected[i]));CHECK(std::abs(actual-expected[i])<1e-7);}
   CHECK(f.requiredFeet==0&&f.externalSupportRequired&&o.confirmedFeet==0);o.actual=f.target;o.acknowledgedSequence=f.target.sequence;sequence=f.target.sequence;record("WALK",now,251,f);
  }
  if(walk.state()==motion::FeedbackWalkState::REJECTED){printf("fixture walk rejected %s frame=%u\n",motion::feedbackReasonName(f.reason),walkFrames);break;}
 }
 CHECK(walk.state()==motion::FeedbackWalkState::STAND);CHECK(walkFrames==19080);CHECK(o.body.translationM.z==.3);
 motion::FeedbackWalk stopped; c.stampS=now;++c.sequence;CHECK(stopped.start(stand,o,c,s,p,now));stopped.requestStop();CHECK(stopped.state()==motion::FeedbackWalkState::REJECTED);
 motion::FeedbackWalk expired;CHECK(expired.start(stand,o,c,s,p,now));o.stampS=now+.21;expired.update(o,now+.21);CHECK(expired.state()==motion::FeedbackWalkState::REJECTED);
 printf("SUPERVISED_RUNTIME %s checks=%d stand_frames=%u segments=%u walk_frames=%u max_target_error=%.12g hardware=false\n",failures?"FAIL":"PASS",checks,frames,segments,walkFrames,maxError);return failures?1:0;
}
