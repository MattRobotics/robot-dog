#!/usr/bin/env python3
"""FAST 40 study: a candidate FAST law as a patch of the FAST row of kWalkModes (SimpleMotion.cpp),
nothing else (envelope, boxes, StopPlan, drive table ceiling and every other mode unchanged).

Usage: make_patch.py <firmware tree> <out.patch> period=2.5 duty=.825 height=.105 ramp=.35 ramp_end=.55
                     strides=.060,.070,.080 lift=.006 sway_y=.0010 [drive=1950,220]
"""
import difflib
import pathlib
import sys

ROW = '  {"FAST",2.5,.825,.105,kWalkSwayXM,.35,.55,{1950,220},{.060,.070,.080},.006,.0010,&kFastTilt,&kFastTilt,false}};'
REL = "05_Firmware/MATDOG_Controller/src/simple/SimpleMotion.cpp"


def main():
    tree, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    kv = dict(a.split("=", 1) for a in sys.argv[3:])
    v = {"period": "2.5", "duty": ".825", "height": ".105", "ramp": ".35", "ramp_end": ".55", "strides": ".060,.070,.080",
         "lift": ".006", "sway_y": ".0010", "drive": "1950,220", **kv}
    row = (f'  {{"FAST",{v["period"]},{v["duty"]},{v["height"]},kWalkSwayXM,{v["ramp"]},{v["ramp_end"]},{{{v["drive"]}}},'
           f'{{{v["strides"]}}},{v["lift"]},{v["sway_y"]},&kFastTilt,&kFastTilt,false}}}};')
    src = (tree / "src/simple/SimpleMotion.cpp").read_text().splitlines(keepends=True)
    i = [k for k, l in enumerate(src) if l.rstrip("\n") == ROW]
    if len(i) != 1:
        raise SystemExit("FAST row not found exactly once (tree is not dev.7?)")
    new = src[:i[0]] + [row + "\n"] + src[i[0] + 1:]
    out.write_text("".join(difflib.unified_diff(src, new, f"a/{REL}", f"b/{REL}")))
    print(row)


if __name__ == "__main__":
    main()
