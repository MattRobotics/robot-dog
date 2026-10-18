#include "MotionContract.h"
#include <cmath>
#include <cstring>
namespace matdog { namespace motion {
#define A(id,category,label,parameters,capability,implemented,intent) {id,category,label,parameters,"runtime_calibration;approved_limits;support_evidence;local_GO",capability,implemented,Intent::intent}
static const MotionAction catalog[kMotionActionCount] = {
 A("posture.stand","Posture","Stand adattivo","target_height_m:0..0.15","stand",true,STAND),
 A("posture.lie_down","Posture","Lie down","target_height_m","lie_down",false,LIE_DOWN),
 A("posture.sit","Posture","Sit","none","sit",false,ACTION_EXECUTE),
 A("posture.crouch","Posture","Crouch","target_height_m","crouch",false,ACTION_EXECUTE),
 A("posture.height","Posture","Body height","target_height_m:0..0.15","stand",true,STAND),
 A("posture.confirm","Posture","Confirm stand","target_height_m:0..0.15","stand",true,STAND),
 A("gait.walk","Gait","WALK","gait:WALK","walk",true,GAIT_SELECT),
 A("gait.trot","Gait","TROT","gait:TROT","trot",false,GAIT_SELECT),
 A("gait.pace","Gait","PACE","gait:PACE","pace",false,GAIT_SELECT),
 A("gait.bound","Gait","BOUND","gait:BOUND","bound",false,GAIT_SELECT),
 A("gait.amble","Gait","AMBLE","gait:AMBLE","amble",false,GAIT_SELECT),
 A("gait.crawl","Gait","CRAWL","gait:CRAWL","crawl",false,GAIT_SELECT),
 A("body.shift_forward","Body","Shift forward","x_m","body_pose",false,BODY_POSE),
 A("body.shift_backward","Body","Shift backward","x_m","body_pose",false,BODY_POSE),
 A("body.shift_left","Body","Shift left","y_m","body_pose",false,BODY_POSE),
 A("body.shift_right","Body","Shift right","y_m","body_pose",false,BODY_POSE),
 A("body.shift_up","Body","Shift up","z_m","body_pose",false,BODY_POSE),
 A("body.shift_down","Body","Shift down","z_m","body_pose",false,BODY_POSE),
 A("body.roll","Body","Roll","roll_rad","body_pose",false,BODY_POSE),
 A("body.pitch","Body","Pitch","pitch_rad","body_pose",false,BODY_POSE),
 A("body.yaw","Body","Yaw","yaw_rad","body_pose",false,BODY_POSE),
 A("body.neutral","Body","Neutral body","none","body_pose",false,BODY_POSE),
 A("action.bow","Actions","Bow","none","preset",false,ACTION_EXECUTE),
 A("action.stretch","Actions","Stretch","none","preset",false,ACTION_EXECUTE),
 A("action.shake","Actions","Shake","none","preset",false,ACTION_EXECUTE),
 A("action.wave","Actions","Wave","none","preset",false,ACTION_EXECUTE),
 A("action.salute","Actions","Salute","none","preset",false,ACTION_EXECUTE),
 A("action.look_left","Actions","Look left","none","preset",false,ACTION_EXECUTE),
 A("action.look_right","Actions","Look right","none","preset",false,ACTION_EXECUTE),
 A("action.scan","Actions","Scan","none","preset",false,ACTION_EXECUTE),
 A("action.dance","Actions","Dance","none","preset",false,ACTION_EXECUTE),
 A("action.turn_left","Actions","Turn left","yaw_rate","preset",false,ACTION_EXECUTE),
 A("action.turn_right","Actions","Turn right","yaw_rate","preset",false,ACTION_EXECUTE),
 A("action.step_over","Actions","Step over","none","preset",false,ACTION_EXECUTE),
 A("leg.lf.position","Legs","LF position","xyz_m","leg_control",false,ACTION_EXECUTE),
 A("leg.rf.position","Legs","RF position","xyz_m","leg_control",false,ACTION_EXECUTE),
 A("leg.lh.position","Legs","LH position","xyz_m","leg_control",false,ACTION_EXECUTE),
 A("leg.rh.position","Legs","RH position","xyz_m","leg_control",false,ACTION_EXECUTE),
 A("leg.lf.lift","Legs","LF lift","height_m","leg_control",false,ACTION_EXECUTE),
 A("leg.rf.lift","Legs","RF lift","height_m","leg_control",false,ACTION_EXECUTE),
 A("leg.lh.lift","Legs","LH lift","height_m","leg_control",false,ACTION_EXECUTE),
 A("leg.rh.lift","Legs","RH lift","height_m","leg_control",false,ACTION_EXECUTE),
 A("recovery.observe","Recovery","Observe pose","none","observe",false,ACTION_EXECUTE),
 A("recovery.classify","Recovery","Classify pose","none","classify",false,ACTION_EXECUTE),
 A("recovery.acquire_support","Recovery","Acquire support","none","acquire_support",false,ACTION_EXECUTE),
 A("recovery.stand","Recovery","Recover to stand","target_height_m:0..0.15","stand",true,STAND),
 A("motion.stop","Safety","STOP","none","stop",true,STOP),
 A("motion.status","Safety","Motion status","none","status",true,STATUS)
};
#undef A
const MotionAction* actionCatalog() { return catalog; }
const MotionAction* findAction(const char* id) { if(!id)return nullptr; for(const auto& a:catalog)if(!strcmp(id,a.id))return &a; return nullptr; }
const char* toString(Intent i) {
 static const char* names[]={"POSTURE.STAND","POSTURE.LIE_DOWN","GAIT.SELECT","BASE_VELOCITY","BODY_POSE","ACTION.EXECUTE","MOTION.STOP","MOTION.STATUS","MOTION.CAPABILITIES","UNKNOWN"};
 return names[static_cast<unsigned>(i)<=static_cast<unsigned>(Intent::UNKNOWN)?static_cast<unsigned>(i):9];
}
Intent parseIntent(const char* s) { if(s)for(unsigned i=0;i<9;i++)if(!strcmp(s,toString(static_cast<Intent>(i))))return static_cast<Intent>(i); return Intent::UNKNOWN; }
const char* toString(ClientOwner o) { switch(o){case ClientOwner::DASHBOARD:return "dashboard";case ClientOwner::JETSON:return "jetson";case ClientOwner::LOCAL:return "local";default:return "none";} }
ClientOwner parseOwner(const char* s) { if(s)for(unsigned i=0;i<3;i++)if(!strcmp(s,toString(static_cast<ClientOwner>(i))))return static_cast<ClientOwner>(i); return ClientOwner::UNKNOWN; }
const char* toString(CommandOutcome o) { switch(o){case CommandOutcome::ACCEPTED:return "ACCEPTED";case CommandOutcome::COMPLETED:return "COMPLETED";case CommandOutcome::ABORTED:return "ABORTED";default:return "REJECTED";} }
bool validCommand(const MotionCommand& c,uint32_t now,const char** reason) {
 auto reject=[&](const char* r){if(reason)*reason=r;return false;};
 if(c.version!=kMotionContractVersion)return reject("SCHEMA_VERSION");
 const size_t n=strnlen(c.id,sizeof(c.id)); if(!n||n>=sizeof(c.id))return reject("COMMAND_ID");
 for(size_t i=0;i<n;i++)if(!((c.id[i]>='a'&&c.id[i]<='z')||(c.id[i]>='A'&&c.id[i]<='Z')||(c.id[i]>='0'&&c.id[i]<='9')||c.id[i]=='_'||c.id[i]=='-'||c.id[i]=='.'))return reject("COMMAND_ID");
 if(!c.sequence)return reject("SEQUENCE");
 if(c.owner==ClientOwner::UNKNOWN||static_cast<unsigned>(c.owner)>3)return reject("OWNER");
 if(c.intent==Intent::UNKNOWN||static_cast<unsigned>(c.intent)>9)return reject("INTENT");
 // STOP de-escalates even with a stale dashboard clock or expired command TTL.
 if(c.intent==Intent::STOP){if(reason)*reason="OK";return true;}
 // signed age rejects future times and remains correct across uptime wrap.
 const int32_t age=static_cast<int32_t>(now-c.issued_at_ms);
 if(age<0)return reject("FUTURE_TIMESTAMP");
 if(!c.ttl_ms||c.ttl_ms>30000||(c.intent==Intent::BASE_VELOCITY&&c.ttl_ms>1000))return reject("TTL_RANGE");
 if(static_cast<uint32_t>(age)>=c.ttl_ms)return reject("EXPIRED");
 const double values[]={c.target_height_m,c.vx,c.vy,c.yaw_rate,c.x,c.y,c.z,c.roll,c.pitch,c.yaw};
 for(double v:values)if(!std::isfinite(v))return reject("NONFINITE_PARAMETER");
 if(strnlen(c.gait,sizeof(c.gait))==sizeof(c.gait)||strnlen(c.action_id,sizeof(c.action_id))==sizeof(c.action_id))return reject("PARAMETER_LENGTH");
 if(reason)*reason="OK";
 return true;
}
} }
