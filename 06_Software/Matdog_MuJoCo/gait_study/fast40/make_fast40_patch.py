#!/usr/bin/env python3
"""FAST 40 study: a candidate as a patch on the dev.7 Motion (git archive export), for the twin and the
firmware's own offline checks. Nothing in any repository is modified.

Two law changes besides the FAST row of kWalkModes:
- leading-leg swing ramps (GaitParameters/WalkModeSpec leadSwingRamp[End]; front legs forward, hind legs
  with reverse; negative = the law's own ramps on every leg, i.e. every other mode bit for bit);
- FAST's own fore/aft sway swayXM (WalkModeSpec already carries it per mode; dev.7 static_asserts that
  the three modes share it).
NOT implemented here (firmware work, see REPORT_FAST40.md): the rate-limited swayXM change of a WALK mode
change inside a walk (the shape step keeps the old swayXM); the FAST envelope, steer box and JoystickSpec
re-derivation.

Usage: make_fast40_patch.py <dev.7 firmware tree> <out.patch> period=2.75 duty=.80 height=.110 ramp=.40
       ramp_end=.30 lead=.15,.65 strides=.060,.080,.110 lift=.005 sway_x=.0045 sway_y=.00075 [drive=1950,220]
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
         "lift": ".006", "sway_x": "kWalkSwayXM", "sway_y": ".0010", "drive": "1950,220", **dict(a.split("=", 1) for a in sys.argv[3:])}
    lead, lead_end = v["lead"].split(",")
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
'''))
    def gait_cpp(s):
        s = edit(s, "T f=swingProfile(u,p.swingRamp,p.swingRampEnd);",
                 "const bool lead=p.leadSwingRamp>=0&&(p.reverse?i>=2:i<2);\n      T f=lead?swingProfile(u,p.leadSwingRamp,p.leadSwingRampEnd):swingProfile(u,p.swingRamp,p.swingRampEnd);")
        s = edit(s, "p.comShiftXM,p.swingRamp,p.swingRampEnd,p.settleCycles};", "p.comShiftXM,p.swingRamp,p.swingRampEnd,p.settleCycles,p.leadSwingRamp,p.leadSwingRampEnd};")
        s = edit(s, "&&p.settleCycles>=0&&p.settleCycles<=1;\n}",
                 "&&p.settleCycles>=0&&p.settleCycles<=1&&\n    (p.leadSwingRamp<0||(p.leadSwingRamp<.5&&p.leadSwingRampEnd>=0&&(p.leadSwingRampEnd==0||(p.leadSwingRamp>0&&p.leadSwingRamp+p.leadSwingRampEnd<1))));\n}")
        s = edit(s, "from.swingRampEnd!=to.swingRampEnd||from.settleCycles!=to.settleCycles)return false;",
                 "from.swingRampEnd!=to.swingRampEnd||from.settleCycles!=to.settleCycles||\n     from.leadSwingRamp!=to.leadSwingRamp||from.leadSwingRampEnd!=to.leadSwingRampEnd)return false;")
        return s
    change("src/motion/Gait.cpp", gait_cpp)
    change("src/simple/SimpleMotion.h", lambda s: edit(s, "  bool hardwareQualified;\n};", "  bool hardwareQualified;\n  double leadSwingRamp=-1,leadSwingRampEnd=0;  // GaitParameters (FAST 40 study)\n};"))
    def sm_cpp(s):
        row = (f'  {{"FAST",{v["period"]},{v["duty"]},{v["height"]},{v["sway_x"]},{v["ramp"]},{v["ramp_end"]},{{{v["drive"]}}},'
               f'{{{v["strides"]}}},{v["lift"]},{v["sway_y"]},&kFastTilt,&kFastTilt,false,{lead},{lead_end}}}}};')
        s = edit(s, '  {"FAST",2.5,.825,.105,kWalkSwayXM,.35,.55,{1950,220},{.060,.070,.080},.006,.0010,&kFastTilt,&kFastTilt,false}};', row)
        s = edit(s, "kWalkModes[0].periodS>kWalkModes[1].periodS&&kWalkModes[1].periodS>kWalkModes[2].periodS&&\n              kWalkModes[0].swayXM==kWalkModes[1].swayXM&&kWalkModes[1].swayXM==kWalkModes[2].swayXM,",
                 "kWalkModes[0].periodS>kWalkModes[1].periodS&&kWalkModes[1].periodS>kWalkModes[2].periodS&&\n              kWalkModes[0].swayXM==kWalkModes[1].swayXM,")
        s = edit(s, "g.swingRamp=w.swingRamp;g.swingRampEnd=w.swingRampEnd;g.settleCycles=kWalkSettleCycles;return g;",
                 "g.swingRamp=w.swingRamp;g.swingRampEnd=w.swingRampEnd;g.settleCycles=kWalkSettleCycles;\n  g.leadSwingRamp=w.leadSwingRamp;g.leadSwingRampEnd=w.leadSwingRampEnd;return g;")
        s = edit(s, "gait_.swingRamp=shape.swingRamp;gait_.swingRampEnd=shape.swingRampEnd;gait_.edgeTiltRad=shape.edgeTiltRad;",
                 "gait_.swingRamp=shape.swingRamp;gait_.swingRampEnd=shape.swingRampEnd;gait_.edgeTiltRad=shape.edgeTiltRad;\n  gait_.leadSwingRamp=shape.leadSwingRamp;gait_.leadSwingRampEnd=shape.leadSwingRampEnd;")
        s = edit(s, "a.swingRamp==b.swingRamp&&a.swingRampEnd==b.swingRampEnd&&a.settleCycles==b.settleCycles;",
                 "a.swingRamp==b.swingRamp&&a.swingRampEnd==b.swingRampEnd&&a.settleCycles==b.settleCycles&&\n    a.leadSwingRamp==b.leadSwingRamp&&a.leadSwingRampEnd==b.leadSwingRampEnd;")
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
