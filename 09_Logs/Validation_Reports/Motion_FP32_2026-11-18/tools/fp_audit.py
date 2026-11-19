#!/usr/bin/env python3
"""Motion FP32 (18/11/2026): FP usage of the Motion functions in an ESP32-S3 ELF (objdump -d -C). For each function of
the motion library and SimpleMotion: calls to soft-double helpers (libgcc __*df*), to double libm and to float libm,
and the count of FPU (.s) instructions. Usage: fp_audit.py <objdump> <elf> [regex of function names]"""
import re, subprocess, sys, collections
objdump, elf = sys.argv[1], sys.argv[2]
sel = re.compile(sys.argv[3] if len(sys.argv) > 3 else r'matdog::(motion|simple)::')
soft = re.compile(r'<(__(?:add|sub|mul|div|neg|eq|ne|lt|gt|le|ge|unord|cmp)df[23]|__extendsfdf2|__truncdfsf2|__fix(?:uns)?dfsi|__fix(?:uns)?dfdi|__float(?:un)?sidf|__float(?:un)?didf)>')
dlib = re.compile(r'<(sin|cos|tan|atan2|atan|acos|asin|sqrt|hypot|remainder|floor|ceil|fmin|fmax|fmod|exp|log|pow|copysign|sincos|ldexp)>')
flib = re.compile(r'<(sinf|cosf|tanf|atan2f|atanf|acosf|asinf|sqrtf|hypotf|remainderf|floorf|ceilf|fminf|fmaxf|fmodf|copysignf|sincosf|ldexpf|nextafterf)>')
fpu = re.compile(r'\t[a-z0-9]+\.s\b')
sfl = re.compile(r'<(__(?:add|sub|mul|div)sf3|__(?:eq|ne|lt|gt|le|ge|unord)sf2|__fix(?:uns)?sfsi|__float(?:un)?sisf)>')
out = subprocess.run([objdump, '-d', '-C', '--no-show-raw-insn', elf], capture_output=True, text=True).stdout
funcs = collections.OrderedDict(); cur = None
for line in out.splitlines():
    m = re.match(r'^[0-9a-f]+ <(.*)>:$', line)
    if m: cur = m.group(1); funcs[cur] = collections.Counter(); continue
    if cur is None or not sel.search(cur): continue
    c = funcs[cur]
    # Direct calls (call8 ADDR <f>) and far calls through a literal (l32r aN, LIT (ADDR <f>); callx8 aN).
    if 'call' in line or ('l32r' in line and '(' in line):
        for r, k in ((soft, 'soft'), (dlib, 'dlibm'), (flib, 'flibm'), (sfl, 'sfloat')):
            for x in r.findall(line): c[k] += 1; c[k + ':' + x] += 1
    if fpu.search(line): c['fpu'] += 1
    c['insn'] += 1
tot = collections.Counter()
for f, c in funcs.items():
    if not sel.search(f) or not c['insn']: continue
    tot.update({k: v for k, v in c.items() if ':' not in k})
    det = ' '.join(f'{k.split(":")[1]}={v}' for k, v in sorted(c.items()) if ':' in k)
    print(f"{c['insn']:6d} insn  fpu={c['fpu']:4d} soft={c['soft']:4d} dlibm={c['dlibm']:3d} flibm={c['flibm']:3d} sfloat={c['sfloat']:3d}  {f[:110]}  [{det}]")
print(f"TOTAL insn={tot['insn']} fpu={tot['fpu']} soft={tot['soft']} dlibm={tot['dlibm']} flibm={tot['flibm']} sfloat={tot['sfloat']}")
