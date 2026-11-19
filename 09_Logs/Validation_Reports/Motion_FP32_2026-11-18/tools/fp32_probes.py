#!/usr/bin/env python3
"""Motion FP32 host bench (18/11/2026): the ONLY source change on the COPIED tree is the capture of the nominal target
before the IMU compensation (frames2_*.bin), the same line the tolerance bench used. Usage: fp32_probes.py <copied src>"""
import sys, pathlib
S = pathlib.Path(sys.argv[1]); p = S / 'simple/SimpleMotion.cpp'; t = p.read_text()
head = 'void SimpleMotion::compensate(LocomotionFrame& f,uint64_t now){\n'
if t.count(head) != 1: raise SystemExit(f"{p}: {t.count(head)} x compensate head")
p.write_text('#include "tolprobe.h"\n' + t.replace(head, head + '  for(unsigned i=0;i<12;++i)tolprobe::nom[i]=jointSlot(f.target,i);tolprobe::nomValid=f.target.valid;\n'))
