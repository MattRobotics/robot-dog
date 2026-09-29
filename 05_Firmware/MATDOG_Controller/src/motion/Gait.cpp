#include "Gait.h"
#include <cmath>
#include <limits>
namespace matdog { namespace motion {
namespace {
// Second-order scalar jet with respect to caller-supplied physical time.
struct D { double p=0,v=0,a=0; D()=default; D(double x,double y=0,double z=0):p(x),v(y),a(z){} };
D operator+(D x,D y){return {x.p+y.p,x.v+y.v,x.a+y.a};}
D operator-(D x,D y){return {x.p-y.p,x.v-y.v,x.a-y.a};}
D operator*(D x,D y){return {x.p*y.p,x.v*y.p+x.p*y.v,x.a*y.p+2*x.v*y.v+x.p*y.a};}
D sn(D x){return {std::sin(x.p),std::cos(x.p)*x.v,std::cos(x.p)*x.a-std::sin(x.p)*x.v*x.v};}
D cs(D x){return {std::cos(x.p),-std::sin(x.p)*x.v,-std::sin(x.p)*x.a-std::cos(x.p)*x.v*x.v};}
D smooth(D u){return u*u*u*(D(10)+u*(D(-15)+D(6)*u));}
struct Planar {D x,y,yaw;};
Planar nominal(const GaitParameters& p,D s) {
  D angle=D(p.yawRad)*s, si,co;
  // Stable analytic SE(2) exponential at zero angular displacement.
  if(std::abs(angle.p)<1e-4) {
    D a2=angle*angle;
    si=s*(D(1)-a2*D(1.0/6)+a2*a2*D(1.0/120));
    co=s*angle*(D(.5)-a2*D(1.0/24)+a2*a2*D(1.0/720));
  } else {si=sn(angle)*D(1/p.yawRad);co=(D(1)-cs(angle))*D(1/p.yawRad);}
  return {D(p.advanceXM)*si-D(p.advanceYM)*co,D(p.advanceXM)*co+D(p.advanceYM)*si,angle};
}
D sway(const GaitParameters& p,D s) {
  if(p.type!=GaitType::WALK||p.swayXM==0)return {};
  double quarter=std::floor(s.p*4), start=quarter*.25, gap=p.duty-.75;
  double sign=std::fmod(quarter,2)==0?1:-1;
  D u=(s-D(start))*D(1/gap);
  return D(sign*p.swayXM)*(u.p<1?D(-1)+D(2)*smooth(u):D(1));
}
void body(CartesianSample& out,Planar b,D height,D shift) {
  D x=b.x+cs(b.yaw)*shift,y=b.y+sn(b.yaw)*shift;
  out.body.translationM={x.p,y.p,height.p};
  out.bodyVelocity={x.v,y.v,height.v};out.bodyAcceleration={x.a,y.a,height.a};
  double c=std::cos(b.yaw.p),s=std::sin(b.yaw.p);
  out.body.rotation[0][0]=c;out.body.rotation[0][1]=-s;
  out.body.rotation[1][0]=s;out.body.rotation[1][1]=c;
  out.yawVelocity=b.yaw.v;out.yawAcceleration=b.yaw.a;
}
Vector3 anchor(const GaitParameters& p,unsigned leg,double td,double terminal) {
  const Vector3 f=canonicalStandDefinition().contactsWorldM[leg];
  if(td<=0)return f;
  double s=terminal>=0&&td>terminal-1?terminal:td+p.duty*.5;
  const auto b=nominal(p,D(s));double c=std::cos(b.yaw.p),si=std::sin(b.yaw.p);
  return {b.x.p+c*f.x-si*f.y,b.y.p+si*f.x+c*f.y,0};
}
bool finite(Vector3 v){return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);}
bool solve3(const double j[3][3],const double b[3],double x[3]) {
  double m[3][4];for(unsigned r=0;r<3;++r){for(unsigned c=0;c<3;++c)m[r][c]=j[r][c];m[r][3]=b[r];}
  for(unsigned c=0;c<3;++c){unsigned p=c;for(unsigned r=c+1;r<3;++r)if(std::abs(m[r][c])>std::abs(m[p][c]))p=r;
    if(std::abs(m[p][c])<1e-12||!std::isfinite(m[p][c]))return false;
    for(unsigned k=0;k<4;++k){double t=m[p][k];m[p][k]=m[c][k];m[c][k]=t;}
    double d=m[c][c];for(unsigned k=c;k<4;++k)m[c][k]/=d;
    for(unsigned r=0;r<3;++r)if(r!=c){double f=m[r][c];for(unsigned k=c;k<4;++k)m[r][k]-=f*m[c][k];}}
  for(unsigned r=0;r<3;++r){x[r]=m[r][3];if(!std::isfinite(x[r]))return false;}return true;
}
bool differential(LegId leg,LegJointAngles q,Vector3 velocity,Vector3 acceleration,
                  LegJointAngles& qv,LegJointAngles& qa,double& condition) {
  const auto& m=*legModel(leg);double a=-m.lowerOrigin.z,f=m.footOrigin.x;
  double r=footContactModel().radiusM,g=m.footOrigin.z+r,d=m.upperOrigin.y+m.footOrigin.y;
  double ch=std::cos(q.hip),sh=std::sin(q.hip),su=std::sin(q.upper),cu=std::cos(q.upper);
  double st=std::sin(q.upper+q.lower),ct=std::cos(q.upper+q.lower);
  double v=-a*cu-f*st+g*ct-r,xu=v+r,xl=-f*st+g*ct;
  double vu=a*su-f*ct-g*st,vl=-f*ct-g*st;
  double j[3][3]={{0,xu,xl},{-d*sh-v*ch,-vu*sh,-vl*sh},{d*ch-v*sh,vu*ch,vl*ch}};
  double inverse[3][3],norm=0,inorm=0;
  for(unsigned c=0;c<3;++c){double b[3]={0,0,0},x[3];b[c]=1;if(!solve3(j,b,x))return false;for(unsigned k=0;k<3;++k)inverse[k][c]=x[k];}
  for(unsigned k=0;k<3;++k){double n=0,in=0;for(unsigned c=0;c<3;++c){n+=std::abs(j[k][c]);in+=std::abs(inverse[k][c]);}norm=std::fmax(norm,n);inorm=std::fmax(inorm,in);}
  condition=norm*inorm;
  double bv[3]={velocity.x,velocity.y,velocity.z},qs[3],qss[3];if(!solve3(j,bv,qs))return false;
  double h=qs[0],u=qs[1],t=u+qs[2],vp=vu*u+vl*qs[2];
  double xpp=a*su*u*u-(f*ct+g*st)*t*t,vpp=a*cu*u*u+(f*st-g*ct)*t*t;
  double ba[3]={acceleration.x-xpp,acceleration.y-(-d*ch*h*h+v*sh*h*h-2*vp*ch*h-vpp*sh),
               acceleration.z-(-d*sh*h*h-v*ch*h*h-2*vp*sh*h+vpp*ch)};
  if(!solve3(j,ba,qss))return false;
  qv={qs[0],qs[1],qs[2]};qa={qss[0],qss[1],qss[2]};return true;
}
Vector3 rotateBack(const BodyPose& b,Vector3 v){return {b.rotation[0][0]*v.x+b.rotation[1][0]*v.y,b.rotation[0][1]*v.x+b.rotation[1][1]*v.y,v.z};}
Vector3 sub(Vector3 a,Vector3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
}
bool validGaitParameters(const GaitParameters& p) {
  const double values[]={p.heightM,p.liftM,p.duty,p.advanceXM,p.advanceYM,p.yawRad,p.swayXM,p.maxCondition};
  for(double v:values)if(!std::isfinite(v))return false;
  return (p.type==GaitType::WALK||p.type==GaitType::TROT)&&p.heightM>0&&p.liftM>0&&p.duty<1&&
    p.duty>=(p.type==GaitType::WALK?.75:.5)&&p.maxCondition>=1&&p.swayXM>=0&&
    (p.swayXM==0||(p.type==GaitType::WALK&&p.duty>.75));
}
bool validGaitPeriod(double t){double v=1/t;return std::isfinite(t)&&t>0&&std::isfinite(v*v)&&v*v>0;}
double gaitOffset(GaitType t,LegId l){
  const unsigned i=static_cast<unsigned>(l);if(i>3||(t!=GaitType::WALK&&t!=GaitType::TROT))return std::numeric_limits<double>::quiet_NaN();
  const double walk[4]={0,.5,.75,.25},trot[4]={0,.5,0,.5};return t==GaitType::WALK?walk[i]:trot[i];
}
LegPhase legPhase(const GaitParameters& p,LegId leg,double cycles){
  LegPhase r;double x=cycles+gaitOffset(p.type,leg);
  if(!validGaitParameters(p)||!std::isfinite(x)||cycles<0||cycles>1e6){r.phase=std::numeric_limits<double>::quiet_NaN();return r;}
  r.phase=x-std::floor(x);
  // Numerical equality only: restore phase boundaries within eight input ULPs.
  const double eps=8*std::numeric_limits<double>::epsilon()*std::fmax(1.,std::abs(x));
  if(r.phase<eps||1-r.phase<eps)r.phase=0;
  if(std::abs(r.phase-p.duty)<eps)r.phase=p.duty;
  r.stance=r.phase<p.duty;
  r.boundary=r.phase==0||r.phase==p.duty;r.swingProgress=r.stance?0:(r.phase-p.duty)/(1-p.duty);return r;
}
CartesianSample gaitCartesian(const GaitParameters& p,double cycles,double rate,double accel,double terminal){
  CartesianSample out;
  if(!validGaitParameters(p)||!std::isfinite(cycles)||cycles<0||cycles>1e6||!std::isfinite(rate)||!std::isfinite(accel)||
     !std::isfinite(terminal)||(terminal!=-1&&(terminal<1||terminal!=std::floor(terminal)||cycles>terminal)))return out;
  D s(cycles,rate,accel);body(out,nominal(p,s),D(p.heightM),sway(p,s));out.cycles=cycles;
  for(unsigned i=0;i<4;++i){auto phase=legPhase(p,static_cast<LegId>(i),cycles);out.phases[i]=phase;
    double k=std::floor(cycles+gaitOffset(p.type,static_cast<LegId>(i))),td=k-gaitOffset(p.type,static_cast<LegId>(i));
    if(phase.phase==0&&cycles+gaitOffset(p.type,static_cast<LegId>(i))-k>.5)td+=1;
    auto a=anchor(p,i,td,terminal);out.feet[i]=a;
    if(!phase.stance){auto b=anchor(p,i,td+1,terminal);D u=(s-D(td+p.duty))*D(1/(1-p.duty));if(phase.boundary)u.p=0;D f=smooth(u);
      D x=D(a.x)+D(b.x-a.x)*f,y=D(a.y)+D(b.y-a.y)*f,z=D(64*p.liftM)*u*u*u*(D(1)-u)*(D(1)-u)*(D(1)-u);
      out.feet[i]={x.p,y.p,z.p};out.velocity[i]={x.v,y.v,z.v};out.acceleration[i]={x.a,y.a,z.a};}}
  out.valid=true;return out;
}
CartesianSample gaitHold(const GaitParameters& p,double cycle,double height,double shift,double hv,double sv,double ha,double sa){
  CartesianSample out;
  if(!validGaitParameters(p)||!std::isfinite(cycle)||cycle<0||cycle>1e6)return out;
  body(out,nominal(p,D(cycle)),D(height,hv,ha),D(shift,sv,sa));out.cycles=cycle;
  for(unsigned i=0;i<4;++i){out.feet[i]=cycle==0?canonicalStandDefinition().contactsWorldM[i]:anchor(p,i,cycle,cycle);out.phases[i].stance=true;}
  out.valid=true;return out;
}
LocomotionFrame solveGait(const GaitParameters& p,const CartesianSample& s,const LocomotionFrame* prev){
  LocomotionFrame out;out.cartesian=s;
  if(!validGaitParameters(p)){out.status=GaitStatus::INVALID_PARAMETER;return out;}
  if(!s.valid||(prev&&!prev->target.valid)){out.status=GaitStatus::STATE_ERROR;return out;}
  if(!supportsFlatContactIk(s.body)||!finite(s.bodyVelocity)||!finite(s.bodyAcceleration)||
     !std::isfinite(s.yawVelocity)||!std::isfinite(s.yawAcceleration)){out.status=GaitStatus::NONFINITE;return out;}
  out.minJointMarginRad=std::numeric_limits<double>::infinity();
  for(unsigned i=0;i<4;++i){out.failedLeg=i;LegId leg=static_cast<LegId>(i);
    if(!finite(s.feet[i])||!finite(s.velocity[i])||!finite(s.acceleration[i])){out.status=GaitStatus::NONFINITE;return out;}
    ContactIkOptions opts;opts.kinematics.seed=prev?prev->target.legs[i]:canonicalStandDefinition().standSeed[i];
    opts.requireNominalStrip=s.phases[i].stance||s.phases[i].boundary;
    const auto ik=worldContactInverseKinematics(leg,s.feet[i],s.body,opts);const auto& b=ik.baseResult;
    if(ik.status!=WorldContactIkResult::Status::OK){out.status=b.status==ContactStatus::UNREACHABLE_LIMITS||b.status==ContactStatus::JOINT_LIMIT?GaitStatus::JOINT_LIMIT:
      b.status==ContactStatus::CONTACT_MODE?GaitStatus::CONTACT_INVALID:GaitStatus::IK_UNREACHABLE;return out;}
    if(prev&&(prev->branches[i].hip!=b.branch.hip||prev->branches[i].elbow!=b.branch.elbow)){out.status=GaitStatus::BRANCH_CHANGE;return out;}
    Vector3 r;pointToBase(s.body,s.feet[i],r);Vector3 vr=rotateBack(s.body,sub(s.velocity[i],s.bodyVelocity)),ar=rotateBack(s.body,sub(s.acceleration[i],s.bodyAcceleration));
    double w=s.yawVelocity,wa=s.yawAcceleration;
    Vector3 velocity={vr.x+w*r.y,vr.y-w*r.x,vr.z};
    Vector3 acceleration={ar.x+2*w*vr.y+wa*r.y-w*w*r.x,ar.y-2*w*vr.x-wa*r.x-w*w*r.y,ar.z};
    if(!differential(leg,b.joints,velocity,acceleration,out.velocityRadS[i],out.accelerationRadS2[i],out.condition[i])||out.condition[i]>p.maxCondition){out.status=GaitStatus::ILL_CONDITIONED;return out;}
    out.target.legs[i]=b.joints;out.branches[i]=b.branch;out.maxResidualM=std::fmax(out.maxResidualM,b.residualM);
    const auto& m=*legModel(leg);double q[3]={b.joints.hip,b.joints.upper,b.joints.lower};
    for(unsigned j=0;j<3;++j)out.minJointMarginRad=std::fmin(out.minJointMarginRad,std::fmin(q[j]-m.limits[j].lower,m.limits[j].upper-q[j]));
  }
  out.failedLeg=255;out.target.valid=true;out.status=GaitStatus::OK;return out;
}
GaitStatus assessGait(LocomotionFrame& f,GaitType t,const GaitAssessment& a){
  if(!f.target.valid)return f.status;
  if(!a.collisionFree)f.status=GaitStatus::COLLISION;
  else if(!a.contactsValid)f.status=GaitStatus::CONTACT_INVALID;
  else if(t==GaitType::WALK&&!a.positiveWalkSupport)f.status=GaitStatus::SUPPORT_INVALID;
  f.target.valid=f.status==GaitStatus::OK;return f.status;
}
} }
