#!/usr/bin/env python3
"""Locomotion vNext (2026-11-13): a candidate law as ONE patch on the dev.7 Motion (git archive export), for
the twin and the firmware's own offline gates. Nothing in any repository is modified: the dev.7 sources are read,
F40T_JCB66.patch (FAST 40 law, mode transition, FAST joystick) is applied to a scratch copy, the vNext edits go
on top, and the diff against dev.7 is written.

vNext law edits (each 0 / default keeps the F40T_JCB66 law bit for bit):
- heave=slow,std,fast (m): body heave during each swing. The stance feet's targets go heaveM below their anchors
  with a plateau over the swing of the leg in flight (smootherstep rise over the first heave_ramp of that swing,
  fall over the last), zero in the four-contact windows. The law body stays at the nominal height, so the
  swing leg's joint trajectory is unchanged; the stance legs extend and raise the real body, and with it the
  swing foot, by about heaveM against the sag of the loaded tripod (RCA 01: -2.7 mm, 11 mrad towards the
  swinging corner). The support fit sees the same uniform offset on its three anchors (orientation unchanged).
- heave_ramp=r (fraction of the swing, default .25).
- compliance=C (rad per N*m): stance compliance feedforward. The position servos are P only; under load a stance
  joint settles tau/K away from its reference (twin: K = k_duty / tick * tau_full / eta = 29.8 N*m/rad; the twin
  sags a little more than the robot, LOWER hind -12 against -8 ticks). Each stance leg's reference is moved by
  -C J^T F: F the static vertical reaction of that foot (weighted minimum-norm distribution of weight=W N over the
  stance feet, COM at the law body plus com_x m along its heading), J the contact Jacobian at the solved joints, in
  the body frame. The load weights ramp 0->1 over the four-contact window after touchdown and 1->0 over the one
  before lift-off, so the reference is continuous. Applied in solveGait and in the levelled solve
  (SimpleMotion::compensationTarget in WALK / WALK_HOLD, walk_tilt_envelope_check's compensated()), i.e. wherever
  a WALK reference is formed, before every box and STOP proof. weight default: the URDF mass 2.928 kg; com_x
  default -0.0094 (SimpleMotion.h: URDF COM 9.4 mm behind the canonical contacts' centre).

CRAWL prototype (crawl=period,duty,height,sway_x,ramp,stride0/stride1/stride2,lift,sway_y): the SLOW row of the
mode table becomes the CRAWL law (same generator, same proofs, the SLOW slot and its envelopes; a 4th mode is
integration work). The modes' common fore/aft sway assert is relaxed for it.

TROT prototype (trot=period,duty,height,ramp,ramp_end,stride0/stride1/stride2,lift): the FAST row becomes a
diagonal trot (GaitType::TROT of the same generator: pairs LF+RH and RF+LH, no sway, no surge, no lead ramps),
fore/aft only; the constant COM shift is allowed for TROT so that the COM sits on the diagonals.

Usage: make_vnext_patch.py <dev.7 firmware tree> <F40T_JCB66.patch> <out.patch> [heave=0,0,0] [heave_ramp=.25]
       [compliance=0] [weight=28.714] [com_x=-0.0094] [compliance_hip=1]
"""
import difflib
import pathlib
import shutil
import subprocess
import sys
import tempfile

ROOT = "05_Firmware/MATDOG_Controller"
FILES = ("src/motion/Gait.h", "src/motion/Gait.cpp", "src/simple/SimpleMotion.h", "src/simple/SimpleMotion.cpp",
         "scripts/offline_motion/walk_tilt_envelope_check.cpp")


def edit(text, old, new, count=1):
    if text.count(old) != count:
        raise SystemExit(f"anchor found {text.count(old)}x, expected {count}: {old[:80]!r}")
    return text.replace(old, new)


def main():
    tree, base_patch, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2]), pathlib.Path(sys.argv[3])
    v = {"heave": "0,0,0", "heave_ramp": ".25", "compliance": "0", "weight": "28.714", "com_x": "-0.0094", "compliance_hip": "1", **dict(a.split("=", 1) for a in sys.argv[4:])}
    heave = v["heave"].split(",")
    with tempfile.TemporaryDirectory() as tmp:
        work = pathlib.Path(tmp) / ROOT
        for rel in FILES:
            (work / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(tree / rel, work / rel)
        subprocess.run(["patch", "-s", "-d", tmp, "-p1", "--no-backup-if-mismatch", "-i", str(base_patch.resolve())], check=True)
        new = {rel: (work / rel).read_text() for rel in FILES}

    def change(rel, fn):
        new[rel] = fn(new[rel])

    change("src/motion/Gait.h", lambda s: edit(s, "  double surgeXM = 0, surgePhase = 0, surgeStrideM = 0;\n", """  double surgeXM = 0, surgePhase = 0, surgeStrideM = 0;
  // Body heave during each swing (Locomotion vNext, 2026-11-13): the stance feet's
  // targets go heaveM below their anchors with a plateau over the flying leg's
  // swing (smootherstep rise over its first heaveRamp, fall over its last), zero
  // in the four-contact windows. The law body keeps the nominal height, so the
  // swing leg's joints are those of the law without heave; the stance legs extend
  // and raise the real body, and the swing foot with it, against the sag of the
  // loaded tripod. 0 keeps the law bit for bit.
  double heaveM = 0, heaveRamp = .25;
  // Stance compliance feedforward (Locomotion vNext, 2026-11-13): each stance
  // leg's reference moves by -complianceRadPerNm J^T F, F the static vertical
  // reaction of its foot (weighted minimum-norm share of weightN over the
  // stance feet, COM at the body plus comOffsetXM along the heading), so the
  // loaded P-only servo settles on the law's pose instead of tau/K away from
  // it. Load weights ramp over the four-contact windows (continuous reference).
  // 0 keeps the law bit for bit.
  // complianceHip scales the HIP term (the HIP's kHeight room is shared with the
  // lateral sway, the IMU roll correction and the STOP arrest).
  double complianceRadPerNm = 0, weightN = 0, comOffsetXM = 0, complianceHip = 1;
  // Steered regime: the compliance feedforward and the heave fade out linearly as the
  // lateral sway amplitude swayYM rises from vnextSwayBaseM to vnextSwayFadeM (0: no
  // fade). The sway changes only with four feet down and rate limited, so the fade
  // inherits that continuity; a fore/aft walk keeps its base sway and the full terms.
  double vnextSwayBaseM = 0, vnextSwayFadeM = 0;
"""))
    if v.get("compliance_steer_fade") == "0":
        change("src/motion/Gait.h", lambda s: edit(s, """  // Steered regime: the compliance feedforward and the heave fade out linearly as the
  // lateral sway amplitude swayYM rises from vnextSwayBaseM to vnextSwayFadeM (0: no
  // fade). The sway changes only with four feet down and rate limited, so the fade
  // inherits that continuity; a fore/aft walk keeps its base sway and the full terms.""",
            """  // Steered regime: the heave fades out linearly as the lateral sway amplitude swayYM
  // rises from vnextSwayBaseM to vnextSwayFadeM (0: no fade); the compliance
  // feedforward keeps its full share (its steered STOP proofs pass without a fade).
  // The sway changes only with four feet down and rate limited, so the fade inherits
  // that continuity; a fore/aft walk keeps its base sway and the full terms."""))
    if v.get("lift_hind"):
        change("src/motion/Gait.h", lambda s: edit(s, "  double vnextSwayBaseM = 0, vnextSwayFadeM = 0;\n", """  double vnextSwayBaseM = 0, vnextSwayFadeM = 0;
  // Swing lift of the hind legs (RH, LH); 0: liftM like the front legs.
  double liftHindM = 0;
"""))
    change("src/motion/Gait.h", lambda s: edit(s, """  double stlSmoothing=0,stlSmoothingVelocity=0,stlSmoothingAcceleration=0;
""", """  double stlSmoothing=0,stlSmoothingVelocity=0,stlSmoothingAcceleration=0;
  // Locomotion vNext: share of the stance compliance feedforward in this sample, 0 -> 1 over the start
  // cycles, 1 -> 0 from the WALK STOP decel to the end of the recentring (0 outside the walking law).
  double complianceScale=0;
"""))
    change("src/motion/Gait.h", lambda s: edit(s, """GaitStatus assessGait(LocomotionFrame& frame, GaitType type, const GaitAssessment& assessment);""",
                                              """GaitStatus assessGait(LocomotionFrame& frame, GaitType type, const GaitAssessment& assessment);
// Stance compliance feedforward of GaitParameters::complianceRadPerNm on joints solved for the body pose
// `body` (the law's, or the levelled one): no effect when it is 0 or the frame is invalid.
void complianceOffset(const GaitParameters& p, const CartesianSample& s, const BodyPose& body, JointTargetFrame& q);
// Share (0..1) of the compliance feedforward and heave at the law's current lateral sway amplitude.
double vnextSwayShare(const GaitParameters& p);"""))

    def gait_cpp(s):
        s = edit(s, """    out.feet[i]=a;
    if(!phase.stance){""", """    out.feet[i]=a;
    if(!phase.stance&&p.heaveM>0){  // heave plateau of this swing (one leg flies at a time in WALK)
      T u=(s-T(td+p.duty))*T(1/(1-p.duty));if(phase.boundary)setP(u,0);
      const T up=pv(u)<p.heaveRamp?smooth(u*T(1/p.heaveRamp)):T(1);
      const T down=pv(u)>1-p.heaveRamp?smooth((T(1)-u)*T(1/p.heaveRamp)):T(1);
      heave=T(p.heaveM*vnextSwayShare(p))*up*down;}
    if(!phase.stance){""")
        # the stance feet take the heave after the loop (the flying leg may come after them)
        s = edit(s, """      out.feet[i]={pv(x),pv(y),pv(z)};footRates(out,i,x,y,z);}}
}""", """      out.feet[i]={pv(x),pv(y),pv(z)};footRates(out,i,x,y,z);}}
  if(pv(heave)>0)for(unsigned i=0;i<4;++i)if(out.phases[i].stance){
    const T z=T(out.feet[i].z)-heave;out.feet[i].z=pv(z);footRates(out,i,T(out.feet[i].x),T(out.feet[i].y),z);}
}""")
        s = edit(s, """  body(out,place(frame,nominal(p,progress(p,s,terminal,true))),T(p.heightM),sway(p,s,terminal),lateral(p,s,terminal));out.cycles=cycles;
""", """  body(out,place(frame,nominal(p,progress(p,s,terminal,true))),T(p.heightM),sway(p,s,terminal),lateral(p,s,terminal));out.cycles=cycles;
  T heave=T(0);
  if(p.complianceRadPerNm>0){  // stance compliance feedforward fades in with the start and out with a WALK STOP
    double k=p.startCycles>0&&cycles<p.startCycles?smooth(cycles/p.startCycles):1;
    if(terminal>=0&&cycles>terminal-p.stopCycles)k*=1-smooth(std::fmin(1.,(cycles-(terminal-p.stopCycles))/(p.stopCycles+gaitSettleCycles(p))));
    out.complianceScale=k;}
""")
        s = edit(s, "p.surgeXM,p.surgePhase,p.surgeStrideM};", "p.surgeXM,p.surgePhase,p.surgeStrideM,p.heaveM,p.heaveRamp,p.complianceRadPerNm,p.weightN,p.comOffsetXM,p.complianceHip,p.vnextSwayBaseM,p.vnextSwayFadeM};")
        # contact Jacobian shared by differential() and the compliance feedforward (same expressions)
        s = edit(s, """bool differential(LegId leg,LegJointAngles q,Vector3 velocity,Vector3 acceleration,
                  LegJointAngles& qv,LegJointAngles& qa,double& condition,bool derivatives) {
  const auto& m=*legModel(leg);double a=-m.lowerOrigin.z,f=m.footOrigin.x;""", """// Contact-point Jacobian in the body frame, rows x y z, columns HIP UPPER LOWER.
void contactJacobian(LegId leg,LegJointAngles q,double j[3][3]){
  const auto& m=*legModel(leg);double a=-m.lowerOrigin.z,f=m.footOrigin.x;
  double r=footContactModel().radiusM,g=m.footOrigin.z+r,d=m.upperOrigin.y+m.footOrigin.y;
  double ch=std::cos(q.hip),sh=std::sin(q.hip),su=std::sin(q.upper),cu=std::cos(q.upper);
  double st=std::sin(q.upper+q.lower),ct=std::cos(q.upper+q.lower);
  double v=-a*cu-f*st+g*ct-r,xu=v+r,xl=-f*st+g*ct;
  double vu=a*su-f*ct-g*st,vl=-f*ct-g*st;
  const double k[3][3]={{0,xu,xl},{-d*sh-v*ch,-vu*sh,-vl*sh},{d*ch-v*sh,vu*ch,vl*ch}};
  for(unsigned r_=0;r_<3;++r_)for(unsigned c=0;c<3;++c)j[r_][c]=k[r_][c];
}
bool differential(LegId leg,LegJointAngles q,Vector3 velocity,Vector3 acceleration,
                  LegJointAngles& qv,LegJointAngles& qa,double& condition,bool derivatives) {
  const auto& m=*legModel(leg);double a=-m.lowerOrigin.z,f=m.footOrigin.x;""")
        s = edit(s, """    out.target.legs[i]=b.joints;out.branches[i]=b.branch;out.maxResidualM=std::fmax(out.maxResidualM,b.residualM);""",
                 """    out.target.legs[i]=b.joints;out.branches[i]=b.branch;out.maxResidualM=std::fmax(out.maxResidualM,b.residualM);""")
        s = edit(s, """  out.failedLeg=255;out.target.valid=true;out.status=GaitStatus::OK;return;
}""", """  out.failedLeg=255;out.target.valid=true;out.status=GaitStatus::OK;
  complianceOffset(p,s,s.body,out.target);return;
}
double vnextSwayShare(const GaitParameters& p){
  if(p.vnextSwayFadeM<=p.vnextSwayBaseM)return 1;
  return std::fmax(0.,std::fmin(1.,(p.vnextSwayFadeM-p.swayYM)/(p.vnextSwayFadeM-p.vnextSwayBaseM)));
}
void complianceOffset(const GaitParameters& p,const CartesianSample& s,const BodyPose& body,JointTargetFrame& q){
  if(p.complianceRadPerNm<=0||p.weightN<=0||!q.valid||p.type!=GaitType::WALK||!s.valid||s.complianceScale<=0)return;
  const double C=p.complianceRadPerNm*s.complianceScale*vnextSwayShare(p);
  if(C<=0)return;
  // Load weights: 1 in stance, 0->1 over the four-contact window after touchdown, 1->0 over the one before
  // lift-off, 0 in swing; a leg held down after the terminal touchdown keeps 1.
  const double gap=p.duty-.75;double w[4];
  for(unsigned i=0;i<4;++i){const auto& ph=s.phases[i];const double x=ph.phase;
    w[i]=!ph.stance?0:x>=p.duty?1:x<gap?smooth(x/gap):x>=.75?1-smooth(std::fmin(1.,(x-.75)/gap)):1;}
  const double c=body.rotation[0][0],sn=body.rotation[1][0];
  const double cx=body.translationM.x+c*p.comOffsetXM,cy=body.translationM.y+sn*p.comOffsetXM;
  // Weighted minimum-norm vertical reactions; no pull: a negative share is clipped to zero and the rest scaled
  // back to the weight. Continuous in the COM and the weights, so the reference never jumps (a COM a few mm
  // outside the law's support triangle only loses the moment balance, not the offset).
  double F[4]{},M[3][3]{},rhs[3]={p.weightN,p.weightN*cx,p.weightN*cy},lam[3];
  for(unsigned i=0;i<4;++i){const double a[3]={1,s.feet[i].x,s.feet[i].y};
    for(unsigned r=0;r<3;++r)for(unsigned k=0;k<3;++k)M[r][k]+=w[i]*a[r]*a[k];}
  if(!solve3(M,rhs,lam))return;
  double total=0;
  for(unsigned i=0;i<4;++i){F[i]=std::fmax(0.,w[i]*(lam[0]+lam[1]*s.feet[i].x+lam[2]*s.feet[i].y));total+=F[i];}
  if(total<=0)return;
  for(double& x:F)x*=p.weightN/total;
  // Body-frame reaction: R^T (0, 0, F); joints settle at q + C J^T f, so the reference moves by -C J^T f.
  for(unsigned i=0;i<4;++i){if(F[i]<=0)continue;
    const double f[3]={body.rotation[2][0]*F[i],body.rotation[2][1]*F[i],body.rotation[2][2]*F[i]};
    double j[3][3];contactJacobian(static_cast<LegId>(i),q.legs[i],j);
    double tau[3];for(unsigned k=0;k<3;++k)tau[k]=j[0][k]*f[0]+j[1][k]*f[1]+j[2][k]*f[2];
    q.legs[i].hip-=C*p.complianceHip*tau[0];q.legs[i].upper-=C*tau[1];q.legs[i].lower-=C*tau[2];}
}""")
        s = edit(s, "p.surgeStrideM>0&&p.startCycles>0&&p.stopCycles>0));\n}",
                 "p.surgeStrideM>0&&p.startCycles>0&&p.stopCycles>0))&&\n    p.heaveM>=0&&p.heaveRamp>0&&p.heaveRamp<=.5&&(p.heaveM==0||p.type==GaitType::WALK);\n}")
        s = edit(s, "from.surgePhase!=to.surgePhase||from.surgeStrideM!=to.surgeStrideM)return false;",
                 "from.surgePhase!=to.surgePhase||from.surgeStrideM!=to.surgeStrideM||\n     from.heaveM!=to.heaveM||from.heaveRamp!=to.heaveRamp)return false;")
        s = edit(s, "p.heaveM>=0&&p.heaveRamp>0&&p.heaveRamp<=.5&&(p.heaveM==0||p.type==GaitType::WALK);\n}",
                 "p.heaveM>=0&&p.heaveRamp>0&&p.heaveRamp<=.5&&(p.heaveM==0||p.type==GaitType::WALK)&&\n    p.complianceRadPerNm>=0&&p.complianceRadPerNm<=.1&&p.weightN>=0&&std::abs(p.comOffsetXM)<=.05&&p.complianceHip>=0&&p.complianceHip<=1&&p.vnextSwayBaseM>=0&&p.vnextSwayFadeM>=0&&(p.complianceRadPerNm==0||(p.type==GaitType::WALK&&p.weightN>0));\n}")
        s = edit(s, "     from.heaveM!=to.heaveM||from.heaveRamp!=to.heaveRamp)return false;",
                 "     from.heaveM!=to.heaveM||from.heaveRamp!=to.heaveRamp||from.complianceRadPerNm!=to.complianceRadPerNm||\n     from.weightN!=to.weightN||from.comOffsetXM!=to.comOffsetXM||from.complianceHip!=to.complianceHip||\n     from.vnextSwayBaseM!=to.vnextSwayBaseM||from.vnextSwayFadeM!=to.vnextSwayFadeM)return false;")
        if v.get("compliance_steer_fade") == "0":  # follow-up F2: the steered STOP_PLAN refusals come from the heave alone
            s = edit(s, "  const double C=p.complianceRadPerNm*s.complianceScale*vnextSwayShare(p);", "  const double C=p.complianceRadPerNm*s.complianceScale;")
        if v.get("lift_hind"):  # follow-up F2: hind legs' own swing lift (0: liftM)
            s = edit(s, "z=T(64*p.liftM)*u*u*u*(T(1)-u)*(T(1)-u)*(T(1)-u);", "z=T(64*(p.liftHindM>0&&i>=2?p.liftHindM:p.liftM))*u*u*u*(T(1)-u)*(T(1)-u)*(T(1)-u);")
            s = edit(s, "p.vnextSwayBaseM,p.vnextSwayFadeM};", "p.vnextSwayBaseM,p.vnextSwayFadeM,p.liftHindM};")
            s = edit(s, "p.heaveM>=0&&p.heaveRamp>0&&", "p.heaveM>=0&&p.liftHindM>=0&&p.heaveRamp>0&&")
            s = edit(s, "from.vnextSwayBaseM!=to.vnextSwayBaseM||from.vnextSwayFadeM!=to.vnextSwayFadeM)return false;",
                     "from.vnextSwayBaseM!=to.vnextSwayBaseM||from.vnextSwayFadeM!=to.vnextSwayFadeM||from.liftHindM!=to.liftHindM)return false;")
        return s
    change("src/motion/Gait.cpp", gait_cpp)
    if v.get("trot"):
        change("src/motion/Gait.cpp", lambda s: edit(s, "(p.comShiftXM==0||(p.type==GaitType::WALK&&p.duty>.75))",
                                                    "(p.comShiftXM==0||p.type==GaitType::TROT||(p.type==GaitType::WALK&&p.duty>.75))"))

    def sm_cpp(s):
        if v.get("trot"):
            P, D, Hh, R, RE, ST, L = v["trot"].split(",")
            st = ",".join(ST.split("/"))
            s = edit(s, '  {"FAST",2.75,.80,.105,.0045,.35,.30,{1950,220},{.080,.095,.110},.005,.00075,&kFastTilt,&kFastTilt,false,.19,.70,.0012,.265}};',
                     f'  // Locomotion vNext TROT prototype in the FAST slot (diagonal pairs, no sway)\n'
                     f'  {{"FAST",{P},{D},{Hh},0,{R},{RE},{{1950,220}},{{{st}}},{L},0,&kFastTilt,&kFastTilt,false,-1,0,0,0}}}};')
            s = edit(s, "kWalkModes[0].swayXM==kWalkModes[1].swayXM&&kWalkModes[0].surgeXM==0", "kWalkModes[0].surgeXM==0") if "kWalkModes[0].swayXM==kWalkModes[1].swayXM&&kWalkModes[0].surgeXM==0" in s else s
            s = edit(s, "  GaitParameters g;g.heightM=baseLinkHeightFromNominalM(w.heightM);",
                     "  GaitParameters g;if(mode==WalkMode::FAST)g.type=GaitType::TROT;  // vNext TROT prototype\n  g.heightM=baseLinkHeightFromNominalM(w.heightM);")
        if v.get("crawl"):
            P, D, Hh, SX, R, ST, L, SY = v["crawl"].split(",")
            st = ",".join(ST.split("/"))
            s = edit(s, '  {"SLOW",kWalkPeriodS,kWalkDuty,kWalkHeightM,kWalkSwayXM,kWalkSwingRamp,0,kWalkDrive,{kStrideM[0],kStrideM[1],kStrideM[2]},kWalkLiftM,kWalkSwayYM,&kWalkTilt75,&kWalkTilt100,true},',
                     f'  // Locomotion vNext CRAWL prototype in the SLOW slot (statically stable crawl, slow transfer windows)\n'
                     f'  {{"SLOW",{P},{D},{Hh},{SX},{R},0,kWalkDrive,{{{st}}},{L},{SY},&kWalkTilt75,&kWalkTilt100,false}},')
            s = edit(s, "kWalkModes[0].swayXM==kWalkModes[1].swayXM&&kWalkModes[0].surgeXM==0", "kWalkModes[0].surgeXM==0")
        # per-mode heave: aggregate initialisation of the table rows cannot reach the new member after the
        # FAST 40 defaults, so it is set by mode index in walkModeSpec's consumers
        s = edit(s, "const WalkModeSpec& walkModeSpec(WalkMode m){return kWalkModes[static_cast<unsigned>(m)<3?static_cast<unsigned>(m):0];}",
                 f"const WalkModeSpec& walkModeSpec(WalkMode m){{return kWalkModes[static_cast<unsigned>(m)<3?static_cast<unsigned>(m):0];}}\n"
                 f"// Locomotion vNext: body heave during each swing, per WALK mode (SLOW, STANDARD, FAST).\n"
                 f"constexpr double kWalkHeaveM[3]={{{heave[0]},{heave[1]},{heave[2]}}};\n"
                 f"static double walkHeaveM(WalkMode m){{return kWalkHeaveM[static_cast<unsigned>(m)<3?static_cast<unsigned>(m):0];}}")
        s = edit(s, "g.leadSwingRamp=w.leadSwingRamp;g.leadSwingRampEnd=w.leadSwingRampEnd;",
                 f"g.heaveM=walkHeaveM(mode);g.heaveRamp={v['heave_ramp']};g.complianceRadPerNm=kWalkComplianceRadPerNm;g.weightN=kWalkWeightN;"
                 f"g.comOffsetXM=kWalkComOffsetXM;g.complianceHip=kWalkComplianceHip;\n  g.vnextSwayBaseM=w.swayYM;g.vnextSwayFadeM=joystickSpec(mode).swayLowM;\n  g.leadSwingRamp=w.leadSwingRamp;g.leadSwingRampEnd=w.leadSwingRampEnd;")
        s = edit(s, "static double walkHeaveM(WalkMode m){",
                 f"// Locomotion vNext: stance compliance feedforward (GaitParameters), the same for every WALK mode.\n"
                 f"constexpr double kWalkComplianceRadPerNm={v['compliance']},kWalkWeightN={v['weight']},kWalkComOffsetXM={v['com_x']},kWalkComplianceHip={v['compliance_hip']};\n"
                 f"static double walkHeaveM(WalkMode m){{")
        # the T2 interpolated frame carries the feedforward's share too (its levelled solve re-adds the offset)
        s = edit(s, "  auto& s=f.cartesian;s.valid=true;s.cycles=cycles_;",
                 "  auto& s=f.cartesian;s.valid=true;s.cycles=cycles_;\n  s.complianceScale=a.cartesian.complianceScale+(b.cartesian.complianceScale-a.cartesian.complianceScale)*u;")
        # the levelled solve of a WALK reference carries the same feedforward (before compensationSafe)
        s = edit(s, """    if(ik.status!=TiltedIkResult::Status::OK||((nominalStrip||edgeTilt>0)&&ik.base.contact.axisTiltRad>tiltLimit-.5*kTickRad)){q.valid=false;break;}q.legs[l]=ik.base.joints;}
  return q;
}""", """    if(ik.status!=TiltedIkResult::Status::OK||((nominalStrip||edgeTilt>0)&&ik.base.contact.axisTiltRad>tiltLimit-.5*kTickRad)){q.valid=false;break;}q.legs[l]=ik.base.joints;}
  if(phase_==Phase::WALK||phase_==Phase::WALK_HOLD)complianceOffset(gait_,f.cartesian,body,q);
  return q;
}""")
        s = edit(s, "gait_.leadSwingRamp=shape.leadSwingRamp;gait_.leadSwingRampEnd=shape.leadSwingRampEnd;",
                 "gait_.heaveM=shape.heaveM;gait_.heaveRamp=shape.heaveRamp;gait_.leadSwingRamp=shape.leadSwingRamp;gait_.leadSwingRampEnd=shape.leadSwingRampEnd;")
        s = edit(s, "a.heaveM==b.heaveM&&a.heaveRamp==b.heaveRamp&&" if False else "a.leadSwingRamp==b.leadSwingRamp&&a.leadSwingRampEnd==b.leadSwingRampEnd&&",
                 "a.leadSwingRamp==b.leadSwingRamp&&a.leadSwingRampEnd==b.leadSwingRampEnd&&a.complianceRadPerNm==b.complianceRadPerNm&&")
        s = edit(s, "a.leadSwingRamp==b.leadSwingRamp&&a.leadSwingRampEnd==b.leadSwingRampEnd&&",
                 "a.heaveM==b.heaveM&&a.heaveRamp==b.heaveRamp&&a.leadSwingRamp==b.leadSwingRamp&&a.leadSwingRampEnd==b.leadSwingRampEnd&&")
        if v.get("trot"):  # the feedforward is WALK-only (validGaitParameters): the TROT slot carries none
            s = edit(s, "g.complianceRadPerNm=kWalkComplianceRadPerNm;", "g.complianceRadPerNm=g.type==GaitType::WALK?kWalkComplianceRadPerNm:0;")
        if v.get("lift_hind"):
            s = edit(s, "gait_.heaveM=shape.heaveM;gait_.heaveRamp=shape.heaveRamp;", "gait_.heaveM=shape.heaveM;gait_.heaveRamp=shape.heaveRamp;gait_.liftHindM=shape.liftHindM;")
            s = edit(s, "a.heaveM==b.heaveM&&a.heaveRamp==b.heaveRamp&&", "a.heaveM==b.heaveM&&a.heaveRamp==b.heaveRamp&&a.liftHindM==b.liftHindM&&")
            s = edit(s, "g.heaveM=walkHeaveM(mode);", f"g.liftHindM=mode==WalkMode::FAST?{v['lift_hind']}:0;  // vNext: FAST hind swing lift\n  g.heaveM=walkHeaveM(mode);")
        if v.get("fast_sway_x"):
            s = edit(s, '  {"FAST",2.75,.80,.105,.0045,', f'  {{"FAST",2.75,.80,.105,{v["fast_sway_x"]},')
        return s
    change("src/simple/SimpleMotion.cpp", sm_cpp)

    def tilt_gate(s):  # the gate's levelled solve is compensationTarget's: same feedforward
        s = edit(s, "bool compensated(const LocomotionFrame& f,double roll,double pitch,JointTargetFrame& q){",
                 "bool compensated(const LocomotionFrame& f,double roll,double pitch,JointTargetFrame& q,const GaitParameters& g){")
        s = edit(s, """    q.legs[l]=ik.base.joints;}
  return true;
}""", """    q.legs[l]=ik.base.joints;}
  complianceOffset(g,f.cartesian,body,q);
  return true;
}""")
        return edit(s, "if(!compensated(f,rays[r].roll,rays[r].pitch,q))", "if(!compensated(f,rays[r].roll,rays[r].pitch,q,L.g))")
    change("scripts/offline_motion/walk_tilt_envelope_check.cpp", tilt_gate)
    diff = []
    for rel in FILES:
        a, b = (tree / rel).read_text().splitlines(keepends=True), new[rel].splitlines(keepends=True)
        diff += difflib.unified_diff(a, b, f"a/{ROOT}/{rel}", f"b/{ROOT}/{rel}")
    out.write_text("".join(diff))
    print(f"{out}: {sum(1 for l in diff if l.startswith('+') and not l.startswith('+++'))} lines added")


if __name__ == "__main__":
    main()
