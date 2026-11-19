#!/usr/bin/env python3
"""FAST 40 study: a candidate as a patch on the dev.7 Motion (git archive export), for the twin and the
firmware's own offline checks. Nothing in any repository is modified.

Three law changes besides the FAST row of kWalkModes:
- leading-leg swing ramps (GaitParameters/WalkModeSpec leadSwingRamp[End]; front legs forward, hind legs
  with reverse; negative = the law's own ramps on every leg, i.e. every other mode bit for bit);
- FAST's own fore/aft sway swayXM (WalkModeSpec already carries it per mode; dev.7 static_asserts that
  the three modes share it);
- body fore/aft speed modulation (GaitParameters/WalkModeSpec surgeXM, surgePhase): surgeXM
  sin(4 pi (s - surgePhase)) added to the fore/aft offset, scaled by the forward stride per cycle over
  the mode's top stride (surgeStrideM; 0 with no forward part: lateral, yaw, backward), faded in over the
  start ramp and out over the decel cycle of a WALK STOP (C2), so the terminal hold never carries it;
  0 keeps every other mode bit for bit.
transition=1 (2026-11-13): the WALK mode change also moves swayXM (four feet down, as swayYM) and surgeXM
(rate-limited) to the target's, with the faster law's shape at the slower law's clock: before the shape step
of a slower target, after the shape step of a faster one; the clock takes the target's rate only with both
done; gaitRetarget accepts both, sameLawShape ignores them (rate-limited amplitudes, as swayYM). Without it
(transition=0, F40) the shape step keeps the old swayXM and switches the modulation at once.
NOT implemented here: the FAST envelope, steer box and JoystickSpec re-derivation.

Usage: make_fast40_patch.py <dev.7 firmware tree> <out.patch> period=2.75 duty=.80 height=.110 ramp=.40
       ramp_end=.30 lead=.15,.65 strides=.060,.080,.110 lift=.005 sway_x=.0045 sway_y=.00075 [surge=.0012,.265]
       [drive=1950,220] [joy=lateral,yaw,sway_low,sway_full[,back]]  (FAST JoystickSpec steered values per cycle)
       [transition=0|1] [lift_profile=rise,fall]  (FAST vertical swing profile; default 0,0 = the 64 u^3 (1-u)^3 bell)
       [lift_legs=all|trail]  (trail: the legs with the lead ramps keep the bell; their UPPER sets FAST's RAW step)
"""
import difflib
import pathlib
import sys

ROOT = "05_Firmware/MATDOG_Controller"


def edit(text, old, new, count=1):
    if text.count(old) != count:
        raise SystemExit(f"anchor found {text.count(old)}x, expected {count}: {old[:80]!r}")
    return text.replace(old, new)


def main():
    tree, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    v = {"period": "2.5", "duty": ".825", "height": ".105", "ramp": ".35", "ramp_end": ".55", "lead": "-1,0", "strides": ".060,.070,.080",
         "lift": ".006", "sway_x": "kWalkSwayXM", "sway_y": ".0010", "drive": "1950,220", "surge": "0,0", "joy": "", "transition": "0", "lift_profile": "0,0", "lift_legs": "all", **dict(a.split("=", 1) for a in sys.argv[3:])}
    lead, lead_end = v["lead"].split(",")
    tr = v["transition"] == "1"
    rise, fall = v["lift_profile"].split(",")
    lp = float(rise) > 0
    trail = v["lift_legs"] == "trail"
    surge, surge_phase = v["surge"].split(",")
    files = {}

    def change(rel, fn):
        p = tree / rel
        files.setdefault(rel, p.read_text())
        files[rel + "#new"] = fn(files.get(rel + "#new", files[rel]))

    change("src/motion/Gait.h", lambda s: edit(s, '''  double swingRampEnd = 0;
''', '''  double swingRampEnd = 0;
  // Leading-leg swing ramps (FAST 40 study, 2026-11-12): the legs that reach
  // forward at touchdown (the front legs; the hind legs with reverse) take
  // their own trapezoid, the others swingRamp/swingRampEnd. The leading UPPER
  // rate grows through the swing (it extends towards kHeight), the trailing
  // LOWER peaks early (lift-off): one shape cannot serve both. Negative keeps
  // swingRamp/swingRampEnd on all four legs, bit for bit.
  double leadSwingRamp = -1, leadSwingRampEnd = 0;
  // WALK body fore/aft speed modulation (FAST 40 study, 2026-11-12): surgeXM
  // sin(4 pi (cycles - surgePhase)) on the fore/aft offset, zero mean, two
  // periods per cycle (one per front swing); faded in over startCycles and out
  // over the decel cycle of a WALK STOP; off for reverse. It slows the
  // body at the end of each front swing, where the STOP capture point meets
  // the diagonal support edge, without moving the body back (front reach).
  // The amplitude scales with the forward stride per cycle (advanceXM over
  // surgeStrideM, at most 1): no modulation without a forward part.
  double surgeXM = 0, surgePhase = 0, surgeStrideM = 0;
''' + ('''  // Vertical swing profile (FAST 40 study, 2026-11-13; M6 A6): with liftRise > 0
  // the foot rises with a smootherstep over the first liftRise and lands with one
  // over the last liftFall of the swing, at liftM between (C2 at lift-off and
  // touchdown; liftRise + liftFall <= 1). The 64 u^3 (1-u)^3 bell (0, the default,
  // bit for bit) is at 10 % of liftM at u = .12 and 26 % at u = .2: in the twin
  // and on the dev.7 robot alike (joint lag in swing) the foot leaves the floor late.
  double liftRise = 0, liftFall = 0;
''' if lp else "")))
    def gait_cpp(s):
        # surge: added in sway(), faded in over the start ramp and out over the decel cycle of a WALK STOP
        # (C2 both ends), so the terminal hold and recentring of swayRegular see none of it
        s = edit(s, """template<class T> T sway(const GaitParameters& p,T s,double terminal) {
  if(p.type!=GaitType::WALK||(p.swayXM==0&&p.comShiftXM==0))return T();
  if(terminal>=0&&pv(s)>=terminal)return T(pv(swayRegular(p,T(terminal))))*(T(1)-smooth((s-T(terminal))*T(1/gaitSettleCycles(p))));
  return swayRegular(p,s);
}""", """// Body fore/aft speed modulation (GaitParameters::surgeXM): scaled by the forward
// stride, faded in over the start ramp and out over the decel cycle of a WALK STOP
// (smootherstep, C2 at both ends), so the terminal hold and recentring see none of it.
template<class T> T surge(const GaitParameters& p,T s,double terminal) {
  if(p.surgeXM==0||p.reverse||p.advanceXM<=0)return T();
  if(terminal>=0&&pv(s)>=terminal)return T();
  const double scale=std::fmin(1.,p.advanceXM/p.surgeStrideM);
  T fade=pv(s)<p.startCycles?smooth(s*T(1/p.startCycles)):T(1);
  if(terminal>=0&&pv(s)>terminal-p.stopCycles)fade=fade*(T(1)-smooth((s-T(terminal-p.stopCycles))*T(1/p.stopCycles)));
  return T(p.surgeXM*scale)*sn(T(4*3.14159265358979323846)*(s-T(p.surgePhase)))*fade;
}
template<class T> T sway(const GaitParameters& p,T s,double terminal) {
  if(p.type!=GaitType::WALK||(p.swayXM==0&&p.comShiftXM==0))return T();
  if(terminal>=0&&pv(s)>=terminal)return T(pv(swayRegular(p,T(terminal))))*(T(1)-smooth((s-T(terminal))*T(1/gaitSettleCycles(p))));
  return swayRegular(p,s)+surge(p,s,terminal);
}""")
        s = edit(s, "T f=swingProfile(u,p.swingRamp,p.swingRampEnd);",
                 "const bool lead=p.leadSwingRamp>=0&&(p.reverse?i>=2:i<2);\n      T f=lead?swingProfile(u,p.leadSwingRamp,p.leadSwingRampEnd):swingProfile(u,p.swingRamp,p.swingRampEnd);")
        if tr:
            s = edit(s, "from.duty!=to.duty||from.swayXM!=to.swayXM||", "from.duty!=to.duty||")
        s = edit(s, "p.comShiftXM,p.swingRamp,p.swingRampEnd,p.settleCycles};", "p.comShiftXM,p.swingRamp,p.swingRampEnd,p.settleCycles,p.leadSwingRamp,p.leadSwingRampEnd,p.surgeXM,p.surgePhase,p.surgeStrideM};")
        s = edit(s, "&&p.settleCycles>=0&&p.settleCycles<=1;\n}",
                 "&&p.settleCycles>=0&&p.settleCycles<=1&&\n    (p.leadSwingRamp<0||(p.leadSwingRamp<.5&&p.leadSwingRampEnd>=0&&(p.leadSwingRampEnd==0||(p.leadSwingRamp>0&&p.leadSwingRamp+p.leadSwingRampEnd<1))))&&\n    (p.surgeXM==0||(p.type==GaitType::WALK&&p.surgeXM>0&&p.surgeStrideM>0&&p.startCycles>0&&p.stopCycles>0));\n}")
        s = edit(s, "from.swingRampEnd!=to.swingRampEnd||from.settleCycles!=to.settleCycles)return false;",
                 "from.swingRampEnd!=to.swingRampEnd||from.settleCycles!=to.settleCycles||\n     from.leadSwingRamp!=to.leadSwingRamp||from.leadSwingRampEnd!=to.leadSwingRampEnd||\n     " +
                 ("" if tr else "from.surgeXM!=to.surgeXM||") + "from.surgePhase!=to.surgePhase||from.surgeStrideM!=to.surgeStrideM)return false;")
        if lp:
            s = edit(s, """// Geometric stride law: body progress, in cycles of advance.""", """// Vertical swing profile (GaitParameters::liftRise/liftFall): a smootherstep rise
// over the first liftRise and fall over the last liftFall of the swing, liftM
// between; 0 keeps the 64 u^3 (1-u)^3 bell bit for bit.
""" + ("""// The legs with the lead ramps (GaitParameters::leadSwingRamp) keep the bell:
// their UPPER sets the RAW step of the FAST 40 law (32 ticks per release is a
// 0.30 s StopPlan arrest), and a faster rise moves it too.
template<class T> T liftProfile(const GaitParameters& p,T u,bool lead){
  if(p.liftRise<=0||lead)return T(64*p.liftM)*u*u*u*(T(1)-u)*(T(1)-u)*(T(1)-u);""" if trail else """template<class T> T liftProfile(const GaitParameters& p,T u){
  if(p.liftRise<=0)return T(64*p.liftM)*u*u*u*(T(1)-u)*(T(1)-u)*(T(1)-u);""") + """
  const T up=pv(u)<p.liftRise?smooth(u*T(1/p.liftRise)):T(1);
  const T down=pv(u)>1-p.liftFall?smooth((T(1)-u)*T(1/p.liftFall)):T(1);
  return T(p.liftM)*up*down;
}
// Geometric stride law: body progress, in cycles of advance.""")
            s = edit(s, "z=T(64*p.liftM)*u*u*u*(T(1)-u)*(T(1)-u)*(T(1)-u);", "z=liftProfile(p,u,lead);" if trail else "z=liftProfile(p,u);")
            s = edit(s, "p.surgeXM,p.surgePhase,p.surgeStrideM};", "p.surgeXM,p.surgePhase,p.surgeStrideM,p.liftRise,p.liftFall};")
            s = edit(s, "p.surgeStrideM>0&&p.startCycles>0&&p.stopCycles>0));\n}", "p.surgeStrideM>0&&p.startCycles>0&&p.stopCycles>0))&&\n    (p.liftRise==0?p.liftFall==0:p.liftRise>0&&p.liftFall>0&&p.liftRise+p.liftFall<=1);\n}")
            s = edit(s, "from.surgePhase!=to.surgePhase||from.surgeStrideM!=to.surgeStrideM)return false;", "from.surgePhase!=to.surgePhase||from.surgeStrideM!=to.surgeStrideM||\n     from.liftRise!=to.liftRise||from.liftFall!=to.liftFall)return false;")
        return s
    change("src/motion/Gait.cpp", gait_cpp)
    if tr:
        change("src/motion/Gait.h", lambda s: edit(s, """// Only advanceXM/advanceYM/yawRad and swayYM may differ (the lateral offset is
// applied in the heading frame after the law frame, so a sway change moves the
// body by the amplitude difference times the current lateral law value; the
// caller rate-limits it). False leaves the track unchanged.""", """// Only advanceXM/advanceYM/yawRad, swayYM and (FAST 40 study) swayXM and surgeXM
// may differ (the lateral and fore/aft offsets are applied in the heading frame
// after the law frame, so a sway change moves the body by the amplitude
// difference times the current law value; the caller rate-limits it, the
// fore/aft one with four contacts). False leaves the track unchanged."""))
    change("src/simple/SimpleMotion.h", lambda s: edit(s, "  bool hardwareQualified;\n};", "  bool hardwareQualified;\n  double leadSwingRamp=-1,leadSwingRampEnd=0,surgeXM=0,surgePhase=0" + (",liftRise=0,liftFall=0" if lp else "") + ";  // GaitParameters (FAST 40 study)\n};"))
    def sm_cpp(s):
        row = (f'  {{"FAST",{v["period"]},{v["duty"]},{v["height"]},{v["sway_x"]},{v["ramp"]},{v["ramp_end"]},{{{v["drive"]}}},'
               f'{{{v["strides"]}}},{v["lift"]},{v["sway_y"]},&kFastTilt,&kFastTilt,false,{lead},{lead_end},{surge},{surge_phase}' + (f',{rise},{fall}' if lp else '') + '}};')
        s = edit(s, '  {"FAST",2.5,.825,.105,kWalkSwayXM,.35,.55,{1950,220},{.060,.070,.080},.006,.0010,&kFastTilt,&kFastTilt,false}};', row)
        if v["joy"]:
            lat, yaw, low, full, *back = v["joy"].split(",")
            s = edit(s, "  {kJoystickBackM,kJoystickStrideM,.012,.08,kWalkModes[2].swayYM,kJoystickSwayLowM,.0075,.0005,3}};",
                     f"  {{{back[0] if back else 'kJoystickBackM'},kJoystickStrideM,{lat},{yaw},kWalkModes[2].swayYM,{low},{full},.0005,3}}}};")
        s = edit(s, "kWalkModes[0].periodS>kWalkModes[1].periodS&&kWalkModes[1].periodS>kWalkModes[2].periodS&&\n              kWalkModes[0].swayXM==kWalkModes[1].swayXM&&kWalkModes[1].swayXM==kWalkModes[2].swayXM,",
                 "kWalkModes[0].periodS>kWalkModes[1].periodS&&kWalkModes[1].periodS>kWalkModes[2].periodS&&\n              kWalkModes[0].swayXM==kWalkModes[1].swayXM,")
        s = edit(s, "g.swingRamp=w.swingRamp;g.swingRampEnd=w.swingRampEnd;g.settleCycles=kWalkSettleCycles;return g;",
                 "g.swingRamp=w.swingRamp;g.swingRampEnd=w.swingRampEnd;g.settleCycles=kWalkSettleCycles;\n  g.leadSwingRamp=w.leadSwingRamp;g.leadSwingRampEnd=w.leadSwingRampEnd;g.surgeXM=w.surgeXM;g.surgePhase=w.surgePhase;g.surgeStrideM=w.strideM[2];" + ("g.liftRise=w.liftRise;g.liftFall=w.liftFall;" if lp else "") + "return g;")
        s = edit(s, "gait_.swingRamp=shape.swingRamp;gait_.swingRampEnd=shape.swingRampEnd;gait_.edgeTiltRad=shape.edgeTiltRad;",
                 "gait_.swingRamp=shape.swingRamp;gait_.swingRampEnd=shape.swingRampEnd;gait_.edgeTiltRad=shape.edgeTiltRad;\n  gait_.leadSwingRamp=shape.leadSwingRamp;gait_.leadSwingRampEnd=shape.leadSwingRampEnd;" +
                 ("" if tr else "gait_.surgeXM=shape.surgeXM;") + "gait_.surgePhase=shape.surgePhase;gait_.surgeStrideM=shape.surgeStrideM;" + ("gait_.liftRise=shape.liftRise;gait_.liftFall=shape.liftFall;" if lp else ""))
        s = edit(s, "a.swingRamp==b.swingRamp&&a.swingRampEnd==b.swingRampEnd&&a.settleCycles==b.settleCycles;",
                 "a.swingRamp==b.swingRamp&&a.swingRampEnd==b.swingRampEnd&&a.settleCycles==b.settleCycles&&\n    a.leadSwingRamp==b.leadSwingRamp&&a.leadSwingRampEnd==b.leadSwingRampEnd&&" +
                 ("" if tr else "a.surgeXM==b.surgeXM&&") + "a.surgePhase==b.surgePhase&&a.surgeStrideM==b.surgeStrideM" + ("&&a.liftRise==b.liftRise&&a.liftFall==b.liftFall" if lp else "") + ";")
        if tr:
            s = edit(s, "  return a.type==b.type&&a.liftM==b.liftM&&a.duty==b.duty&&a.swayXM==b.swayXM&&\n",
                     "  return a.type==b.type&&a.liftM==b.liftM&&a.duty==b.duty&&\n")
            s = edit(s, """// Fields of the law other than the steered displacement per cycle and the
// steered sway: a change of these is structural""", """// Fields of the law other than the steered displacement per cycle and the
// sway amplitudes (lateral; FAST 40 study: fore/aft and the body speed
// modulation, rate-limited by stepWalkMode): a change of these is structural""")
            s = edit(s, """// stepWalkMode: the modes are ordered (a faster mode has the higher drive, so
// the faster of two drives is one of the table's) and share the fore/aft COM
// offsets, which therefore never change inside a walk.""", """// stepWalkMode: the modes are ordered (a faster mode has the higher drive, so
// the faster of two drives is one of the table's) and share the fore/aft COM
// shift. The fore/aft sway and the body speed modulation move inside a walk
// (FAST 40 study), rate-limited; only the fastest mode carries the modulation,
// so its phase and stride scale change at a shape step only with it at zero.""")
            s = edit(s, """              kWalkModes[0].swayXM==kWalkModes[1].swayXM,"WALK modes ordered by speed");""",
                     """              kWalkModes[0].swayXM==kWalkModes[1].swayXM&&kWalkModes[0].surgeXM==0&&kWalkModes[1].surgeXM==0,"WALK modes ordered by speed");""")
            s = edit(s, """  const bool shaped=walkMode_==walkTarget_;
  if(shaped&&walkPeriod_==to.periodS&&walkHeight_==to.heightM&&gait_.swayYM==to.swayYM&&""", """  const bool shaped=walkMode_==walkTarget_;
  // FAST 40 study: the fore/aft sway and the body speed modulation are the target's own too.
  const bool own=gait_.swayYM==to.swayYM&&gait_.swayXM==to.swayXM&&gait_.surgeXM==to.surgeXM;
  if(shaped&&walkPeriod_==to.periodS&&walkHeight_==to.heightM&&own&&""")
            s = edit(s, "const double goalPeriod=shaped&&walkHeight_==to.heightM&&gait_.swayYM==to.swayYM?to.periodS:",
                     "const double goalPeriod=shaped&&walkHeight_==to.heightM&&own?to.periodS:")
            s = edit(s, """  if((next.advanceXM!=gait_.advanceXM||next.swayYM!=gait_.swayYM)&&gaitRetarget(track_,gait_,next,cycles_))gait_=next;
  const double h=geometry""", """  // Fore/aft sway (four feet down, as the lateral one) and body speed modulation
  // (FAST 40 study): the target's, with the faster law's shape at the slower
  // law's clock: before the shape step of a slower target, after the shape step
  // of a faster one.
  if(geometry&&(shaped||to.periodS>from.periodS)){
    if(fourDown)next.swayXM=approach(gait_.swayXM,to.swayXM,kWalkModeSwayRate*dc);
    next.surgeXM=approach(gait_.surgeXM,to.surgeXM,kWalkModeSwayRate*dc);}
  if((next.advanceXM!=gait_.advanceXM||next.swayYM!=gait_.swayYM||next.swayXM!=gait_.swayXM||next.surgeXM!=gait_.surgeXM)&&
     gaitRetarget(track_,gait_,next,cycles_))gait_=next;
  const double h=geometry""")
            s = edit(s, "  if(to.periodS>from.periodS&&walkHeight_!=to.heightM)return;",
                     "  if(to.periodS>from.periodS&&(walkHeight_!=to.heightM||gait_.swayXM!=to.swayXM||gait_.surgeXM!=to.surgeXM))return;")
        return s
    change("src/simple/SimpleMotion.cpp", sm_cpp)
    diff = []
    for rel in sorted(k for k in files if not k.endswith("#new")):
        a, b = files[rel].splitlines(keepends=True), files[rel + "#new"].splitlines(keepends=True)
        diff += difflib.unified_diff(a, b, f"a/{ROOT}/{rel}", f"b/{ROOT}/{rel}")
    out.write_text("".join(diff))
    print(f"{out}: {sum(1 for l in diff if l.startswith('+') and not l.startswith('+++'))} lines added")


if __name__ == "__main__":
    main()
