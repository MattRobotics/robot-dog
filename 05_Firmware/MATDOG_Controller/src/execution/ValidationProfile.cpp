#include "ValidationProfile.h"
#include "../servo/ServoPopulation.h"
#include "../motion/CertifiedMotionEvidence.h"
#include "../motion/FirstStrideReference.h"
#include <cmath>
#include <cstring>
#include <cstdlib>
namespace matdog { namespace execution {
namespace {
bool number(const char* s,double& d){char* end=nullptr;d=std::strtod(s,&end);return end!=s&&!*end&&std::isfinite(d);}
bool integer(const char* s,uint32_t& u){double d;if(!number(s,d)||d<0||d>UINT32_MAX||std::floor(d)!=d)return false;u=static_cast<uint32_t>(d);return true;}
bool digest(const char* s){uint8_t b[32];if(std::strlen(s)!=64||!update::parseHex(s,64,b))return false;uint8_t any=0;for(auto x:b)any|=x;return any!=0;}
}
bool ValidationProfile::card(const char* line){
 reason_="PROFILE_CARD_INVALID";
 if(!line||std::strlen(line)>240)return false;
 char copy[241];std::strcpy(copy,line);const char* t[18]{};unsigned n=0;
 char* save=nullptr;for(char* p=strtok_r(copy," ",&save);p;p=strtok_r(nullptr," ",&save)){if(n==18)return false;t[n++]=p;}
 if(!n)return false;
 if(!std::strcmp(t[0],"BEGIN")){
  uint32_t gen,scope;if(n!=3||!integer(t[1],gen)||gen!=1||!integer(t[2],scope)||scope<1||scope>4)return false;
  clear();open_=true;workspace.calibration_generation=gen;runtime.calibration_generation=gen;
  runtime.scope=static_cast<TrialScope>(scope);
 }else{
  if(!open_||sealed_)return false;
  if(!std::strcmp(t[0],"REVIEW")){
   if(n!=2||review_||!digest(t[1]))return false;
   strcpy(review_sha256,t[1]);review_=true;
  }else if(!std::strcmp(t[0],"RECORD")){
   if(n!=2||record_||!digest(t[1]))return false;
   strcpy(record_sha256,t[1]);record_=true;
  }else if(!std::strcmp(t[0],"ROUTE")){
   if(n!=2||route_)return false;
   if(!strcmp(t[1],"CANONICAL"))runtime.feedback.fixtureLimitVariant=false;
   else if(!strcmp(t[1],"FIXTURE_LIMIT_COMPATIBLE")&&(runtime.scope==TrialScope::SUPERVISED_FIXTURE_STAND||runtime.scope==TrialScope::SUPERVISED_FIXTURE_WALK))runtime.feedback.fixtureLimitVariant=true;
   else return false;
   route_=true;
  }else if(!std::strcmp(t[0],"JOINT")){
   uint32_t i;double lo,hi,m;
   if(n!=5||!integer(t[1],i)||i>=12||(joint_mask_&(1<<i))||!number(t[2],lo)||!number(t[3],hi)||!number(t[4],m)||
      lo<-3200000||hi>3200000||lo>=hi||m<=0||m*2>=hi-lo||std::floor(lo)!=lo||std::floor(hi)!=hi||std::floor(m)!=m)return false;
   const auto* servo=servo::legServoAt((i/3)*3+2-i%3);if(!servo)return false;
   auto& j=workspace.joint[i];j.joint.leg=static_cast<calibration::Leg>(i/3);j.joint.joint=static_cast<calibration::JointKind>(i%3);
   std::strncpy(j.joint.physical_unit,servo->physical_unit,sizeof(j.joint.physical_unit)-1);
   j.required_min_urad=static_cast<actuator::MicroRad>(lo);j.required_max_urad=static_cast<actuator::MicroRad>(hi);j.safety_margin_urad=static_cast<actuator::MicroRad>(m);
   runtime.feedback.jointLimits[i/3][i%3]={(lo+m)*1e-6,(hi-m)*1e-6};joint_mask_|=1<<i;
  }else if(!std::strcmp(t[0],"EXEC")){
   if(n!=14||exec_)return false;
   double v,a,e;uint32_t x[10];if(!number(t[1],v)||!number(t[2],a)||!number(t[3],e)||v<=0||a<=0||e<=0)return false;
   for(unsigned i=0;i<10;++i)if(!integer(t[i+4],x[i]))return false;
   if(!x[0]||x[0]>1000||!x[1]||x[1]>3000||!x[2]||x[2]>30000||!x[3]||x[3]>3000||!x[4]||x[4]>70||!x[5]||x[6]<=x[5]||!x[7]||x[7]>1000||x[8]<2||x[8]>20||!x[9]||x[9]>128)return false;
   runtime.execution={v,a,e,x[0],x[1],x[2],true};runtime.max_imu_age_ms=x[3];runtime.max_servo_temperature_c=x[4];runtime.min_servo_voltage_dv=x[5];runtime.max_servo_voltage_dv=x[6];output_ceiling=x[7];transaction_timeout_ms=x[8];workspace.contact_inset_ticks=x[9];
   auto& f=runtime.feedback;f.mode=runtime.scope==TrialScope::SUPERVISED_FIXTURE_STAND||runtime.scope==TrialScope::SUPERVISED_FIXTURE_WALK?motion::FeedbackMode::FIXTURE_REVIEW:motion::FeedbackMode::PHYSICAL_REVIEW;f.envelopesApproved=true;f.loadedRatesVerified=f.mode==motion::FeedbackMode::PHYSICAL_REVIEW;f.fixtureTrialRatesReviewed=f.mode==motion::FeedbackMode::FIXTURE_REVIEW;
   f.maxTrackingErrorRad=e;f.observationMaxAgeS=2;f.assessmentMaxAgeS=2;f.segmentTimeoutS=x[2]/1000.0;
   f.samplePeriodS=.05;f.heightToleranceM=.001;f.contactToleranceM=.001;
   for(unsigned i=0;i<4;++i){f.maxVelocityRadS[i]={v,v,v};f.maxAccelerationRadS2[i]={a,a,a};}exec_=true;
  }else if(!std::strcmp(t[0],"DRIVE")){
   uint32_t v,a;double sv,sa;if(n!=6||drive_||!integer(t[1],v)||v<1||v>40||!integer(t[2],a)||a<1||a>10||!number(t[3],sv)||sv<=0||!number(t[4],sa)||sa<=0||!digest(t[5]))return false;
   drive_speed=v;drive_acceleration=a;speed_scale=sv;acceleration_scale=sa;strcpy(drive_proof_sha256,t[5]);drive_=true;
  }else if(!std::strcmp(t[0],"SUPPORT")){
   if(n!=3||support_||!digest(t[1])||!digest(t[2]))return false;
   strcpy(support_proof_sha256,t[1]);strcpy(loaded_tracking_proof_sha256,t[2]);support_=true;
  }else if(!std::strcmp(t[0],"DYNAMICS")){
   if(n!=2||dynamics_||!digest(t[1]))return false;
   strcpy(dynamics_proof_sha256,t[1]);dynamics_=true;
  }else if(!std::strcmp(t[0],"STEP")){
   double x;if(n!=2||step_||!number(t[1],x)||x<=0||x>.01)return false;
   runtime.feedback.maxJointStepRad=x;step_=true;
  }else if(!std::strcmp(t[0],"ERROR_BUDGET")){
   double e,m;if(n!=3||error_budget_||!number(t[1],e)||!number(t[2],m)||e<3.141592653589793/4096||m<=0)return false;
   encoder_uncertainty_rad=e;mapping_uncertainty_rad=m;error_budget_=true;
  }else if(!std::strcmp(t[0],"FIXTURE")){
   double v[6];if(n!=8||fixture_||!digest(t[7]))return false;
   for(unsigned i=0;i<6;++i)if(!number(t[i+1],v[i]))return false;
   fixture_pose.translationM={v[0],v[1],v[2]};const double cr=cos(v[3]),sr=sin(v[3]),cp=cos(v[4]),sp=sin(v[4]),cy=cos(v[5]),sy=sin(v[5]);
   const double rot[3][3]={{cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr},{sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr},{-sp,cp*sr,cp*cr}};memcpy(fixture_pose.rotation,rot,sizeof(rot));
   if(!motion::validBodyPose(fixture_pose)||v[2]<=0)return false;
   strcpy(fixture_proof_sha256,t[7]);fixture_=true;
  }else if(!std::strcmp(t[0],"CLOCK")){
   double x;if(n!=2||clock_||!number(t[1],x)||x<1||x>100)return false;
   runtime.stand_time_scale=x;clock_=true;
  }else if(!std::strcmp(t[0],"CORRIDOR")){
   uint32_t first,last;double q,p,r;
   if(n!=7||corridor_||!integer(t[1],first)||!integer(t[2],last)||first>last||last>motion::kPairedProneSegments||
      !number(t[3],q)||!number(t[4],p)||!number(t[5],r)||q<=0||q>.01||p<=0||p>.01||r<=0||r>.01||!digest(t[6]))return false;
   first_knot=first;last_knot=last;joint_tube_rad=q;position_tube_m=p;rotation_tube=r;std::strcpy(corridor_proof_sha256,t[6]);corridor_=true;
  }else if(!std::strcmp(t[0],"WALK_CLOCK")){
   double s,step;if(n!=3||walk_clock_||!number(t[1],s)||s<9||s>100||!number(t[2],step)||step<=0||step>.01)return false;
   runtime.walk.firstStrideTimeScale=s;runtime.walk.maxJointStepRad=step;walk_clock_=true;
  }else if(!std::strcmp(t[0],"WALK")){
   if(n!=12||walk_)return false;
   double x[9];for(unsigned i=0;i<9;++i)if(!number(t[i+1],x[i])||x[i]<0)return false;
   if(!digest(t[10])||x[0]<=0||x[1]<=0||x[2]<=0||x[3]<=.5||x[3]>=1||x[5]<=0||x[6]<=0)return false;
   if(!motion::compiledFirstStrideIntegrityPinned()||strcmp(t[10],motion::certified_evidence::kFirstStrideProof)||!number(t[11],walk_tube_rad)||walk_tube_rad>motion::compiledFirstStrideTubeRad()||walk_tube_rad<=0||walk_tube_rad>.01||x[0]!=90||x[1]!=90||x[2]!=.005||x[3]!=.8||x[4]!=.03||x[5]!=8||!walk_clock_||std::abs(x[6]-.005/(90*runtime.walk.firstStrideTimeScale))>1e-12||x[7]!=0||x[8]!=0)return false;
   strcpy(walk_proof_sha256,t[10]);runtime.walk.compiledFirstStride=true;runtime.walk.smoothStlReference=true;runtime.walk.firstStrideOnly=true;runtime.walk.periodS=x[0];runtime.walk.rampDurationS=x[1];runtime.walk.liftM=x[2];runtime.walk.duty=x[3];runtime.walk.swayXM=x[4];runtime.walk.standShiftDurationS=x[5];runtime.walk.maxVxMps=x[6];runtime.walk.maxVyMps=x[7];runtime.walk.maxYawRateRadS=x[8];runtime.walk.maxCommandTtlS=.25;runtime.walk.stlContactReference=true;walk_=true;
  }else if(!std::strcmp(t[0],"SEAL")){
   if(n!=3||!digest(t[1])||std::strcmp(t[2],"CONFIRM_INSTALLATION_WORKSPACE_MARGIN_SWEEP_TRACKING_TIMING_OUTPUT")||joint_mask_!=4095||!exec_||!review_||!record_||!route_||!corridor_||!clock_||!fixture_||!error_budget_||!step_||!drive_||drive_speed*speed_scale>runtime.execution.max_velocity_rad_s||drive_acceleration*acceleration_scale>runtime.execution.max_acceleration_rad_s2||runtime.execution.tracking_tolerance_rad+encoder_uncertainty_rad+mapping_uncertainty_rad+runtime.feedback.maxJointStepRad/2+drive_speed*speed_scale*runtime.execution.feedback_max_age_ms/1000>=joint_tube_rad||
      ((runtime.scope==TrialScope::SUPERVISED_FIXTURE_WALK||runtime.scope==TrialScope::AUTONOMOUS_WALK)&&(!walk_||!walk_clock_||runtime.execution.tracking_tolerance_rad+encoder_uncertainty_rad+mapping_uncertainty_rad+drive_speed*speed_scale*runtime.execution.feedback_max_age_ms/1000>=walk_tube_rad))||
      ((runtime.scope==TrialScope::AUTONOMOUS_STAND||runtime.scope==TrialScope::AUTONOMOUS_WALK)&&!support_)||(runtime.scope==TrialScope::AUTONOMOUS_WALK&&!dynamics_))return false;
   if(joint_tube_rad>motion::certified_evidence::kStandTubeRad||last_knot!=251||(!runtime.feedback.fixtureLimitVariant&&first_knot<motion::certified_evidence::kCanonicalFirstKnot)||strcmp(corridor_proof_sha256,runtime.feedback.fixtureLimitVariant?motion::certified_evidence::kFixtureRouteProof:motion::certified_evidence::kCanonicalTubeProof))return false;
   uint8_t result[32],expected[32];hash_.finish(result);update::parseHex(t[1],64,expected);
   if(!update::digestsEqual(result,expected)){clear();reason_="PROFILE_DIGEST_MISMATCH";return false;}
   update::toHex(result,workspace.artifact_sha256,65);workspace.workspace_reviewed=workspace.simultaneous_configuration_checked=workspace.collision_clearance_checked=workspace.margins_reviewed=true;
   sealed_=true;open_=false;reason_="SEALED_RAM_REVIEW_REQUIRES_LIVE_ADMISSION";return true;
  }else return false;
 }
 hash_.update(reinterpret_cast<const uint8_t*>(line),std::strlen(line));const uint8_t lf=10;hash_.update(&lf,1);reason_="STAGED_NO_AUTHORITY";return true;
}
} }
