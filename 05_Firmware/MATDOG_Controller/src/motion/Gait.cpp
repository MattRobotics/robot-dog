#include "Gait.h"
#include "LegKinematicsInternal.h"
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
// Position-only evaluation of the same law (motion-perf-next A1). A Motion
// release samples with zero rate and acceleration and solves the IK without
// derivatives, so every jet it would carry is identically zero: the p part is
// computed alone, term for term in the same order as the jet operators (bit
// for bit the same positions), at one trig call per sn/cs instead of two and
// one multiplication instead of three. The jet path is unchanged for callers
// that need joint velocities (envelope sweeps).
inline double pv(double x){return x;}
inline double pv(const D& x){return x.p;}
inline double sn(double x){return std::sin(x);}
inline double cs(double x){return std::cos(x);}
// cos/sin jets from already evaluated c=cos(x.p), s=sin(x.p): the same bits
// as cs(x)/sn(x), without re-evaluating the trig functions.
inline double csFrom(double,double c,double){return c;}
inline double snFrom(double,double,double s){return s;}
inline D csFrom(const D& x,double c,double s){return {c,-s*x.v,-s*x.a-c*x.v*x.v};}
inline D snFrom(const D& x,double c,double s){return {s,c*x.v,c*x.a-s*x.v*x.v};}
template<class T> T smooth(T u){return u*u*u*(T(10)+u*(T(-15)+T(6)*u));}
template<class T> struct PlanarT {T x,y,yaw;};
using Planar=PlanarT<D>;
template<class T> PlanarT<T> nominal(const GaitParameters& p,T s) {
  T angle=T(p.yawRad)*s, si,co;
  // Stable analytic SE(2) exponential at zero angular displacement.
  if(std::abs(pv(angle))<1e-4) {
    T a2=angle*angle;
    si=s*(T(1)-a2*T(1.0/6)+a2*a2*T(1.0/120));
    co=s*angle*(T(.5)-a2*T(1.0/24)+a2*a2*T(1.0/720));
  } else {si=sn(angle)*T(1/p.yawRad);co=(T(1)-cs(angle))*T(1/p.yawRad);}
  return {T(p.advanceXM)*si-T(p.advanceYM)*co,T(p.advanceXM)*co+T(p.advanceYM)*si,angle};
}
template<class T> T swayRegular(const GaitParameters& p,T s) {
  double quarter=std::floor(pv(s)*4), start=quarter*.25, gap=p.duty-.75;
  double sign=std::fmod(quarter,2)==0?1:-1;
  if(pv(s)<gap)return T(p.comShiftXM+p.swayXM)*smooth(s*T(1/gap));  // leave the centred stand
  T u=(s-T(start))*T(1/gap);
  return T(p.comShiftXM)+T(sign*p.swayXM)*(pv(u)<1?T(-1)+T(2)*smooth(u):T(1));
}
// Forward while a hind leg swings, backward while a front leg swings; changes
// only with four contacts. After the terminal touchdown the held offset
// recentres with four contacts.
template<class T> T sway(const GaitParameters& p,T s,double terminal) {
  if(p.type!=GaitType::WALK||(p.swayXM==0&&p.comShiftXM==0))return T();
  if(terminal>=0&&pv(s)>=terminal)return T(pv(swayRegular(p,T(terminal))))*(T(1)-smooth((s-T(terminal))*T(1/gaitSettleCycles(p))));
  return swayRegular(p,s);
}
template<class T> T lateralRegular(const GaitParameters& p,T s) {
  const double w=p.duty-.75;const T a(p.swayYM),inv(1/w);
  if(p.reverse){  // LH, RF, RH, LF: the forward switches one quarter later
    if(pv(s)<w)return T(-1)*a*smooth(s*inv);  // leave the centred stand
    if(pv(s)<.25)return T(-1)*a;
    s=s-T(.25);
  }else if(pv(s)<w)return a*smooth(s*inv);  // leave the centred stand
  const T f=s-T(std::floor(pv(s)));
  if(pv(f)<w)return a*(T(2)*smooth(f*inv)-T(1));
  if(pv(f)<.5)return a;
  if(pv(f)<.5+w)return a*(T(1)-T(2)*smooth((f-T(.5))*inv));
  return T(-1)*a;
}
// +Y while the right legs swing, -Y while the left legs swing; changes only
// with four contacts, recentres after the terminal touchdown.
template<class T> T lateral(const GaitParameters& p,T s,double terminal) {
  if(p.type!=GaitType::WALK||p.swayYM==0)return T();
  if(terminal>=0&&pv(s)>=terminal)return T(pv(lateralRegular(p,T(terminal))))*(T(1)-smooth((s-T(terminal))*T(1/gaitSettleCycles(p))));
  return lateralRegular(p,s);
}
inline void bodyRates(CartesianSample& out,double,double,double,double){out.bodyVelocity={0,0,0};out.bodyAcceleration={0,0,0};out.yawVelocity=out.yawAcceleration=0;}
inline void bodyRates(CartesianSample& out,const D& x,const D& y,const D& height,const D& yaw){
  out.bodyVelocity={x.v,y.v,height.v};out.bodyAcceleration={x.a,y.a,height.a};out.yawVelocity=yaw.v;out.yawAcceleration=yaw.a;}
template<class T> void body(CartesianSample& out,PlanarT<T> b,T height,T shift,T side=T()) {
  const double c=std::cos(pv(b.yaw)),s=std::sin(pv(b.yaw));
  const T cy=csFrom(b.yaw,c,s),sy=snFrom(b.yaw,c,s);
  T x=b.x+cy*shift-sy*side,y=b.y+sy*shift+cy*side;
  out.body.translationM={pv(x),pv(y),pv(height)};
  bodyRates(out,x,y,height,b.yaw);
  out.body.rotation[0][0]=c;out.body.rotation[0][1]=-s;
  out.body.rotation[1][0]=s;out.body.rotation[1][1]=c;
}
template<class T> T ramp(T u){return u*u*u*u*(T(2.5)+u*(T(-3)+u));}  // integral of smooth(u)
// Swing progress 0->1: smootherstep, or trapezoidal velocity with linear ramps r.
template<class T> T swingProfile(T u,double r){
  if(r<=0)return smooth(u);
  const double k=.5/(r*(1-r));
  if(pv(u)<=r)return T(k)*u*u;
  if(pv(u)>=1-r){const T w=T(1)-u;return T(1)-T(k)*w*w;}
  return T(1/(1-r))*(u-T(.5*r));
}
// Geometric stride law: body progress, in cycles of advance. Identity without
// ramps. The start raises the progress rate 0->1 over startCycles; after the
// decel cycle S=terminal-stopCycles the rate falls 1->0 at the terminal
// touchdown. Both are C2 in cycles, so a constant clock starts and stops the
// body with zero velocity and acceleration.
template<class T> T progress(const GaitParameters& p,T s,double terminal,bool decel) {
  const double a=p.startCycles;
  T r=a>0?(pv(s)<a?T(a)*ramp(s*T(1/a)):s-T(.5*a)):s;
  if(decel&&terminal>=0&&p.stopCycles>0&&pv(s)>terminal-p.stopCycles){
    const double S=terminal-p.stopCycles,L=p.stopCycles,base=a>0?S-.5*a:S;
    if(pv(s)>=terminal)return T(base+.5*L);
    const T v=(s-T(S))*T(1/L);r=T(base)+T(L)*(v-ramp(v));
  }
  return r;
}
Vector3 anchor(const GaitParameters& p,unsigned leg,double td,double terminal) {
  const Vector3 f=canonicalStandDefinition().contactsWorldM[leg];
  if(td<=0)return f;
  double s=terminal>=0&&td>terminal-1?terminal+gaitSettleCycles(p):td+p.duty*.5;
  // A foot that touches down by the decel cycle keeps the regular law: a STOP
  // request never moves a planted foot or an in-flight swing target.
  const bool decel=terminal>=0&&td>terminal-p.stopCycles;
  const auto b=nominal(p,progress(p,s,terminal,decel));double c=std::cos(b.yaw),si=std::sin(b.yaw);
  return {b.x+c*f.x-si*f.y,b.y+si*f.x+c*f.y,0};
}
double offset(const GaitParameters& p,unsigned leg){
  static const double reverse[4]={0,.5,.25,.75};
  return p.reverse&&p.type==GaitType::WALK&&leg<4?reverse[leg]:gaitOffset(p.type,static_cast<LegId>(leg));
}
// A planar law expressed in the world through a constant SE(2) frame.
template<class T> PlanarT<T> place(const double f[3],PlanarT<T> a){
  const T c(std::cos(f[2])),s(std::sin(f[2]));
  return {T(f[0])+c*a.x-s*a.y,T(f[1])+s*a.x+c*a.y,T(f[2])+a.yaw};
}
Vector3 place(const double f[3],Vector3 v){
  const double c=std::cos(f[2]),s=std::sin(f[2]);
  return {f[0]+c*v.x-s*v.y,f[1]+s*v.x+c*v.y,v.z};
}
inline void setP(double& u,double value){u=value;}
inline void setP(D& u,double value){u.p=value;}
inline void footRates(CartesianSample&,unsigned,double,double,double){}
inline void footRates(CartesianSample& out,unsigned i,const D& x,const D& y,const D& z){out.velocity[i]={x.v,y.v,z.v};out.acceleration[i]={x.a,y.a,z.a};}
// One sample of the law with the clock jet s (T=double: positions only).
template<class T> void sampleLaw(CartesianSample& out,const GaitParameters& p,T s,double terminal,const double* frame,GaitTrack* track){
  const double cycles=pv(s);
  body(out,place(frame,nominal(p,progress(p,s,terminal,true))),T(p.heightM),sway(p,s,terminal),lateral(p,s,terminal));out.cycles=cycles;
  for(unsigned i=0;i<4;++i){auto phase=legPhase(p,static_cast<LegId>(i),cycles);
    // After the terminal touchdown all four feet stay down (recentring window).
    if(terminal>=0&&cycles>=terminal&&!phase.stance){phase.stance=phase.boundary=true;phase.swingProgress=0;}
    out.phases[i]=phase;
    double k=std::floor(cycles+offset(p,i)),td=k-offset(p,i);
    if(phase.phase==0&&cycles+offset(p,i)-k>.5)td+=1;
    Vector3 a,b;
    if(track){
      // The next step lands on the target fixed at its lift-off; otherwise
      // (start, or a sampling gap) the anchor comes from the current law.
      if(track->touchdown[i]!=td){
        track->anchor[i]=track->targetValid[i]&&td==track->touchdown[i]+1?track->target[i]:place(frame,anchor(p,i,td,terminal));
        track->touchdown[i]=td;track->targetValid[i]=false;
      }
      if(!phase.stance&&!track->targetValid[i]){
        Vector3 b=place(frame,anchor(p,i,td+1,terminal));
        const double now[3]={p.advanceXM,p.advanceYM,p.yawRad};bool ramp=false;
        for(unsigned j=0;j<3;++j)ramp=ramp||(track->goalRate[j]>0&&track->goal[j]!=now[j]);
        if(ramp&&terminal<0){
          // Integrate the planned twist from now to that step's mid-stance.
          const double mid=td+1+p.duty*.5;double r0=progress(p,cycles,terminal,true);
          const auto w=place(frame,nominal(p,r0));double x=w.x,y=w.y,yaw=w.yaw;
          for(unsigned k=1;k<=16;++k){const double c=cycles+(mid-cycles)*k/16,r1=progress(p,c,terminal,true),t=c-cycles-(mid-cycles)/32,dr=r1-r0;r0=r1;
            double xi[3];for(unsigned j=0;j<3;++j){const double g=track->goal[j]-now[j];xi[j]=now[j]+std::copysign(std::fmin(std::abs(g),track->goalRate[j]*t),g);}
            const double a=xi[2]*dr,si=std::abs(xi[2])>1e-9?std::sin(a)/xi[2]:dr,co=std::abs(xi[2])>1e-9?(1-std::cos(a))/xi[2]:0;
            const double bx=xi[0]*si-xi[1]*co,by=xi[0]*co+xi[1]*si;
            x+=std::cos(yaw)*bx-std::sin(yaw)*by;y+=std::sin(yaw)*bx+std::cos(yaw)*by;yaw+=a;}
          const Vector3 f=canonicalStandDefinition().contactsWorldM[i];
          b={x+std::cos(yaw)*f.x-std::sin(yaw)*f.y,y+std::sin(yaw)*f.x+std::cos(yaw)*f.y,0};
        }
        track->target[i]=b;track->targetValid[i]=true;}
      a=track->anchor[i];b=track->target[i];
    }else{a=anchor(p,i,td,terminal);if(!phase.stance)b=anchor(p,i,td+1,terminal);}
    out.feet[i]=a;
    if(!phase.stance){T u=(s-T(td+p.duty))*T(1/(1-p.duty));if(phase.boundary)setP(u,0);T f=swingProfile(u,p.swingRamp);
      T x=T(a.x)+T(b.x-a.x)*f,y=T(a.y)+T(b.y-a.y)*f,z=T(64*p.liftM)*u*u*u*(T(1)-u)*(T(1)-u)*(T(1)-u);
      out.feet[i]={pv(x),pv(y),pv(z)};footRates(out,i,x,y,z);}}
}
CartesianSample sample(const GaitParameters& p,double cycles,double rate,double accel,double terminal,GaitTrack* track){
  CartesianSample out;
  if(!validGaitParameters(p)||!std::isfinite(cycles)||cycles<0||cycles>1e6||!std::isfinite(rate)||!std::isfinite(accel)||
     !std::isfinite(terminal)||(terminal!=-1&&(terminal<1||terminal*4!=std::floor(terminal*4)||cycles>terminal+gaitSettleCycles(p))))return out;
  if(terminal!=-1&&p.stopCycles>0&&terminal-p.stopCycles<p.startCycles)return out;
  const double identity[3]={0,0,0};const double* frame=track?track->frame:identity;
  if(rate==0&&accel==0)sampleLaw<double>(out,p,cycles,terminal,frame,track);
  else sampleLaw<D>(out,p,D(cycles,rate,accel),terminal,frame,track);
  out.valid=true;return out;
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
                  LegJointAngles& qv,LegJointAngles& qa,double& condition,bool derivatives) {
  const auto& m=*legModel(leg);double a=-m.lowerOrigin.z,f=m.footOrigin.x;
  double r=footContactModel().radiusM,g=m.footOrigin.z+r,d=m.upperOrigin.y+m.footOrigin.y;
  double ch=std::cos(q.hip),sh=std::sin(q.hip),su=std::sin(q.upper),cu=std::cos(q.upper);
  double st=std::sin(q.upper+q.lower),ct=std::cos(q.upper+q.lower);
  double v=-a*cu-f*st+g*ct-r,xu=v+r,xl=-f*st+g*ct;
  double vu=a*su-f*ct-g*st,vl=-f*ct-g*st;
  double j[3][3]={{0,xu,xl},{-d*sh-v*ch,-vu*sh,-vl*sh},{d*ch-v*sh,vu*ch,vl*ch}};
  double inverse[3][3],norm=0,inorm=0;
  if(!detail::invert3(j,inverse))return false;
  for(unsigned k=0;k<3;++k){double n=0,in=0;for(unsigned c=0;c<3;++c){n+=std::abs(j[k][c]);in+=std::abs(inverse[k][c]);}norm=std::fmax(norm,n);inorm=std::fmax(inorm,in);}
  condition=norm*inorm;
  if(!derivatives)return true;
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
  const double values[]={p.heightM,p.liftM,p.duty,p.advanceXM,p.advanceYM,p.yawRad,p.swayXM,p.swayYM,p.startCycles,p.stopCycles,p.maxCondition,p.comShiftXM,p.swingRamp,p.settleCycles};
  for(double v:values)if(!std::isfinite(v))return false;
  return (p.type==GaitType::WALK||p.type==GaitType::TROT)&&p.heightM>0&&p.liftM>0&&p.duty<1&&
    p.duty>=(p.type==GaitType::WALK?.75:.5)&&p.maxCondition>=1&&p.swayXM>=0&&
    (p.swayXM==0||(p.type==GaitType::WALK&&p.duty>.75))&&(p.comShiftXM==0||(p.type==GaitType::WALK&&p.duty>.75))&&
    p.swayYM>=0&&(p.swayYM==0||(p.type==GaitType::WALK&&p.duty>.75))&&
    p.startCycles>=0&&p.stopCycles>=0&&(!p.reverse||p.type==GaitType::WALK)&&
    std::isfinite(p.edgeTiltRad)&&p.edgeTiltRad>=0&&p.edgeTiltRad<=kEdgeContactMaxTiltRad&&p.swingRamp>=0&&p.swingRamp<.5&&p.settleCycles>=0&&p.settleCycles<=1;
}
double gaitSettleCycles(const GaitParameters& p){
  if(p.type!=GaitType::WALK||(p.swayXM==0&&p.swayYM==0&&p.comShiftXM==0))return 0;
  return p.settleCycles>0?p.settleCycles:p.duty-.75;
}
bool validGaitPeriod(double t){double v=1/t;return std::isfinite(t)&&t>0&&std::isfinite(v*v)&&v*v>0;}
double gaitOffset(GaitType t,LegId l){
  const unsigned i=static_cast<unsigned>(l);if(i>3||(t!=GaitType::WALK&&t!=GaitType::TROT))return std::numeric_limits<double>::quiet_NaN();
  const double walk[4]={0,.5,.75,.25},trot[4]={0,.5,0,.5};return t==GaitType::WALK?walk[i]:trot[i];
}
LegPhase legPhase(const GaitParameters& p,LegId leg,double cycles){
  LegPhase r;double x=cycles+offset(p,static_cast<unsigned>(leg));
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
  return sample(p,cycles,rate,accel,terminal,nullptr);
}
CartesianSample gaitTrackCartesian(GaitTrack& track,const GaitParameters& p,double cycles,double rate,double accel,double terminal){
  return sample(p,cycles,rate,accel,terminal,&track);
}
bool gaitRetarget(GaitTrack& track,const GaitParameters& from,const GaitParameters& to,double cycles,double terminal){
  if(!validGaitParameters(from)||!validGaitParameters(to)||!std::isfinite(cycles)||cycles<0||cycles>1e6||!std::isfinite(terminal)||
     from.type!=to.type||from.heightM!=to.heightM||from.liftM!=to.liftM||from.duty!=to.duty||from.swayXM!=to.swayXM||
     from.startCycles!=to.startCycles||from.stopCycles!=to.stopCycles||from.reverse!=to.reverse||from.edgeTiltRad!=to.edgeTiltRad||
     from.comShiftXM!=to.comShiftXM||from.swingRamp!=to.swingRamp||from.settleCycles!=to.settleCycles)return false;
  // Same world pose of the nominal body at this progress under both laws.
  const double r=progress(from,cycles,terminal,true);
  const auto world=place(track.frame,nominal(from,r)),next=nominal(to,r);
  const double yaw=world.yaw-next.yaw,c=std::cos(yaw),s=std::sin(yaw);
  const double f[3]={world.x-(c*next.x-s*next.y),world.y-(s*next.x+c*next.y),yaw};
  for(double v:f)if(!std::isfinite(v))return false;
  for(unsigned i=0;i<3;++i)track.frame[i]=f[i];
  return true;
}
CartesianSample gaitHold(const GaitParameters& p,double cycle,double height,double shift,double hv,double sv,double ha,double sa){
  CartesianSample out;
  if(!validGaitParameters(p)||!std::isfinite(cycle)||cycle<0||cycle>1e6)return out;
  body(out,nominal(p,D(cycle)),D(height,hv,ha),D(shift,sv,sa));out.cycles=cycle;
  for(unsigned i=0;i<4;++i){out.feet[i]=cycle==0?canonicalStandDefinition().contactsWorldM[i]:anchor(p,i,cycle,cycle);out.phases[i].stance=true;}
  out.valid=true;return out;
}
LocomotionFrame solveGait(const GaitParameters& p,const CartesianSample& s,const LocomotionFrame* prev,bool derivatives){
  LocomotionFrame out;solveGaitInto(out,p,s,prev,derivatives);return out;
}
void solveGaitInto(LocomotionFrame& out,const GaitParameters& p,const CartesianSample& s,const LocomotionFrame* prev,bool derivatives){
  // One body for both entry points. The reset copies a constant-initialized
  // frame instead of materializing a temporary on the caller's task stack.
  static const LocomotionFrame kEmpty{};
  out=kEmpty;out.cartesian=s;
  if(!validGaitParameters(p)){out.status=GaitStatus::INVALID_PARAMETER;return;}
  if(!s.valid||(prev&&!prev->target.valid)){out.status=GaitStatus::STATE_ERROR;return;}
  if(!std::isfinite(s.cycles)||!finite(s.body.translationM)||!finite(s.bodyVelocity)||!finite(s.bodyAcceleration)||
     !std::isfinite(s.yawVelocity)||!std::isfinite(s.yawAcceleration)){out.status=GaitStatus::NONFINITE;return;}
  for(const auto& row:s.body.rotation)for(double value:row)if(!std::isfinite(value)){out.status=GaitStatus::NONFINITE;return;}
  if(!supportsFlatContactIk(s.body)){out.status=GaitStatus::INVALID_PARAMETER;return;}
  out.minJointMarginRad=std::numeric_limits<double>::infinity();
  for(unsigned i=0;i<4;++i){out.failedLeg=i;LegId leg=static_cast<LegId>(i);
    if(!std::isfinite(s.phases[i].phase)||!std::isfinite(s.phases[i].swingProgress)){out.status=GaitStatus::NONFINITE;return;}
    if(s.phases[i].phase<0||s.phases[i].phase>=1||s.phases[i].swingProgress<0||s.phases[i].swingProgress>1){out.status=GaitStatus::INVALID_PARAMETER;return;}
    if(!finite(s.feet[i])||!finite(s.velocity[i])||!finite(s.acceleration[i])){out.status=GaitStatus::NONFINITE;return;}
    ContactIkOptions opts;opts.kinematics.seed=prev?prev->target.legs[i]:canonicalStandDefinition().standSeed[i];
    opts.requireNominalStrip=p.edgeTiltRad==0&&(s.phases[i].stance||s.phases[i].boundary);
    // Edge contact: the fillet point, not the strip reference, meets the
    // target. The flat body keeps the foot axis tilt equal to |HIP|; the seed's
    // lift is exact to ~1e-5 m, else one more solve with the solved tilt.
    double lift=p.edgeTiltRad>0?edgeContactLiftM(opts.kinematics.seed.hip):0;
    auto ik=worldContactInverseKinematics(leg,{s.feet[i].x,s.feet[i].y,s.feet[i].z+lift},s.body,opts);
    if(p.edgeTiltRad>0&&ik.status==WorldContactIkResult::Status::OK&&std::abs(edgeContactLiftM(ik.baseResult.joints.hip)-lift)>1e-5){
      lift=edgeContactLiftM(ik.baseResult.joints.hip);opts.kinematics.seed=ik.baseResult.joints;
      ik=worldContactInverseKinematics(leg,{s.feet[i].x,s.feet[i].y,s.feet[i].z+lift},s.body,opts);}
    const auto& b=ik.baseResult;
    if(ik.status==WorldContactIkResult::Status::OK&&p.edgeTiltRad>0&&b.contact.axisTiltRad>p.edgeTiltRad){out.status=GaitStatus::CONTACT_INVALID;return;}
    if(ik.status!=WorldContactIkResult::Status::OK){out.status=b.status==ContactStatus::UNREACHABLE_LIMITS||b.status==ContactStatus::JOINT_LIMIT?GaitStatus::JOINT_LIMIT:
      b.status==ContactStatus::CONTACT_MODE?GaitStatus::CONTACT_INVALID:GaitStatus::IK_UNREACHABLE;return;}
    if(prev&&(prev->branches[i].hip!=b.branch.hip||prev->branches[i].elbow!=b.branch.elbow)){out.status=GaitStatus::BRANCH_CHANGE;return;}
    Vector3 r;pointToBase(s.body,s.feet[i],r);Vector3 vr=rotateBack(s.body,sub(s.velocity[i],s.bodyVelocity)),ar=rotateBack(s.body,sub(s.acceleration[i],s.bodyAcceleration));
    double w=s.yawVelocity,wa=s.yawAcceleration;
    Vector3 velocity={vr.x+w*r.y,vr.y-w*r.x,vr.z};
    Vector3 acceleration={ar.x+2*w*vr.y+wa*r.y-w*w*r.x,ar.y-2*w*vr.x-wa*r.x-w*w*r.y,ar.z};
    if(!differential(leg,b.joints,velocity,acceleration,out.velocityRadS[i],out.accelerationRadS2[i],out.condition[i],derivatives)||out.condition[i]>p.maxCondition){out.status=GaitStatus::ILL_CONDITIONED;return;}
    out.target.legs[i]=b.joints;out.branches[i]=b.branch;out.maxResidualM=std::fmax(out.maxResidualM,b.residualM);
    const auto& m=*legModel(leg);double q[3]={b.joints.hip,b.joints.upper,b.joints.lower};
    for(unsigned j=0;j<3;++j)out.minJointMarginRad=std::fmin(out.minJointMarginRad,std::fmin(q[j]-m.limits[j].lower,m.limits[j].upper-q[j]));
  }
  out.failedLeg=255;out.target.valid=true;out.status=GaitStatus::OK;return;
}
GaitStatus assessGait(LocomotionFrame& f,GaitType t,const GaitAssessment& a){
  if(!f.target.valid)return f.status;
  if(t!=GaitType::WALK&&t!=GaitType::TROT)f.status=GaitStatus::INVALID_PARAMETER;
  else if(!a.collisionFree)f.status=GaitStatus::COLLISION;
  else if(!a.contactsValid)f.status=GaitStatus::CONTACT_INVALID;
  else if(t==GaitType::WALK&&!a.positiveWalkSupport)f.status=GaitStatus::SUPPORT_INVALID;
  f.target.valid=f.status==GaitStatus::OK;return f.status;
}
} }
