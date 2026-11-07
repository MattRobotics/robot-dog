#include "TiltCompensation.h"
#include <cmath>
namespace matdog { namespace motion {
namespace {
Vector3 difference(Vector3 a,Vector3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vector3 cross(Vector3 a,Vector3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double dot(Vector3 a,Vector3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
bool unit(Vector3& v){
  const double n=std::sqrt(dot(v,v));
  if(!std::isfinite(n)||n<=1e-12)return false;
  v={v.x/n,v.y/n,v.z/n};return true;
}
bool triad(Vector3 a,Vector3 b,Vector3 c,Vector3 out[3]){
  out[0]=difference(b,a);out[2]=cross(out[0],difference(c,a));
  if(!unit(out[0])||!unit(out[2]))return false;
  out[1]=cross(out[2],out[0]);return true;
}
}
bool supportLevelTarget(const Vector3 anchors[4],const JointTargetFrame& measured,
                        const bool stance[4],const Vector3& worldUpBase,
                        const BodyPose& previousBodyInSupport,double& roll,double& pitch){
  roll=pitch=0;
  if(!anchors||!stance||!measured.valid||!validBodyPose(previousBodyInSupport))return false;
  Vector3 up=worldUpBase;if(!unit(up))return false;
  // Select by the labelled gait anchors before doing only three FK evaluations.
  unsigned ids[3]{};Vector3 support[3];bool found=false;
  for(unsigned i=0;i<4&&!found;++i)if(stance[i])
    for(unsigned j=i+1;j<4&&!found;++j)if(stance[j])
      for(unsigned k=j+1;k<4&&!found;++k)if(stance[k]&&triad(anchors[i],anchors[j],anchors[k],support)){
        ids[0]=i;ids[1]=j;ids[2]=k;found=true;
      }
  if(!found)return false;
  const Vector3 normal={previousBodyInSupport.rotation[2][0],previousBodyInSupport.rotation[2][1],previousBodyInSupport.rotation[2][2]};
  Vector3 points[3];
  for(unsigned n=0;n<3;++n){
    const auto c=contactForwardKinematics(static_cast<LegId>(ids[n]),measured.legs[ids[n]],normal);
    if(c.status!=ContactStatus::OK)return false;
    points[n]=c.referenceM;
  }
  Vector3 body[3];if(!triad(points[0],points[1],points[2],body))return false;
  // R_BS = bodyTriad * supportTriad^T, hence upS = R_BS^T * upB.
  const double x=dot(body[0],up),y=dot(body[1],up),z=dot(body[2],up);
  Vector3 upSupport={support[0].x*x+support[1].x*y+support[2].x*z,
                    support[0].y*x+support[1].y*y+support[2].y*z,
                    support[0].z*x+support[1].z*y+support[2].z*z};
  if(!unit(upSupport)||upSupport.z<=0)return false;
  // Ry(pitch)*Rx(roll) has third column (sin(p)cos(r),-sin(r),cos(p)cos(r)).
  roll=-std::asin(std::fmax(-1.,std::fmin(1.,upSupport.y)));
  pitch=std::atan2(upSupport.x,upSupport.z);
  return std::isfinite(roll)&&std::isfinite(pitch);
}
BodyPose bodyPoseFromRollPitch(const Vector3& t, double roll, double pitch) {
  BodyPose p; p.translationM = t;
  const double cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch);
  // R = Ry(pitch) * Rx(roll)
  const double r[3][3] = {{cp, sp*sr, sp*cr}, {0, cr, -sr}, {-sp, cp*sr, cp*cr}};
  for (unsigned i = 0; i < 3; ++i) for (unsigned j = 0; j < 3; ++j) p.rotation[i][j] = r[i][j];
  return p;
}
TiltCompensation compensateStand(const StandDefinition& d, double height, double roll, double pitch, const LegJointAngles seeds[4], bool strip) {
  TiltCompensation out;
  if (!std::isfinite(height) || height <= 0 || !std::isfinite(roll) || !std::isfinite(pitch) || std::abs(roll) > 1.0 || std::abs(pitch) > 1.0) return out;
  out.pose = bodyPoseFromRollPitch({0, 0, height}, roll, pitch);
  if (!validBodyPose(out.pose)) return out;
  out.minJointMarginRad = 1e9;
  for (unsigned i = 0; i < 4; ++i) {
    TiltedIkOptions o; o.seed = seeds[i]; o.requireNominalStrip = strip;
    const auto ik = worldContactInverseKinematicsTilted(static_cast<LegId>(i), d.contactsWorldM[i], out.pose, o);
    if (ik.status != TiltedIkResult::Status::OK) {
      TiltCompensation failed; failed.status = TiltCompensation::Status::IK_FAILURE; failed.failedLeg = static_cast<uint8_t>(i);
      failed.legStatus = ik.status; failed.pose = out.pose;
      return failed;  // never leak a partially solved 12-joint frame
    }
    out.target.legs[i] = ik.base.joints;
    out.contactsWorldM[i] = ik.achievedWorldM;
    const double dx = ik.achievedWorldM.x - d.contactsWorldM[i].x, dy = ik.achievedWorldM.y - d.contactsWorldM[i].y, dz = ik.achievedWorldM.z - d.contactsWorldM[i].z;
    out.maxContactDriftM = std::fmax(out.maxContactDriftM, std::sqrt(dx*dx + dy*dy + dz*dz));
    out.maxCondition = std::fmax(out.maxCondition, ik.base.condition);
    out.maxIterations = ik.base.iterations > out.maxIterations ? ik.base.iterations : out.maxIterations;
    const auto* m = legModel(static_cast<LegId>(i));
    const double q[3] = {ik.base.joints.hip, ik.base.joints.upper, ik.base.joints.lower};
    for (unsigned j = 0; j < 3; ++j) out.minJointMarginRad = std::fmin(out.minJointMarginRad, std::fmin(q[j] - m->limits[j].lower, m->limits[j].upper - q[j]));
  }
  out.target.valid = true; out.status = TiltCompensation::Status::OK;
  return out;
}
} }
