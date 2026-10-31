#!/usr/bin/env python3
"""Targeted offline original transport / PRIME lifecycle tests. No device API."""
import os,subprocess,tempfile,sys,shlex
from pathlib import Path
fw=Path(__file__).resolve().parents[2];tests=fw/'scripts/tests'
lib=Path(os.environ.get('SCSERVO_DIR',str(Path.home()/'Arduino/libraries/SCServo/src')))
bus_only='--bus-only' in sys.argv
with tempfile.TemporaryDirectory(prefix='matdog-prime-diagnostic-') as out:
    if bus_only:
        cmd=['g++','-std=c++17','-O1','-g','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-fno-omit-frame-pointer','-DMATDOG_PRIME_BUS_ONLY',str(tests/'test_prime_diagnostic.cpp')]
    else:
        s=(tests/'run_motion_service_executor_tests.sh').read_text()
        c=s[s.index('"$CXX" -std='):s.index('"$OUT/test_motion_service_executor"\n')]
        c=c.replace('test_motion_service_executor.cpp','test_prime_diagnostic.cpp').replace('-Werror','-Werror -Wno-unused-function')
        c=c.replace('"$SKETCH_DIR/src/execution/MotionExecutor.cpp"','"$SKETCH_DIR/src/execution/CommissioningPolicy.cpp" "$SKETCH_DIR/src/execution/MotionExecutor.cpp"')
        for k,v in {'CXX':'g++','OUT':out,'SCRIPT_DIR':str(tests),'SKETCH_DIR':str(fw)}.items():c=c.replace('$'+k,v)
        cmd=shlex.split(c.replace('\\\n',' '))
    cmd+=['-DARDUINO=100','-Wno-unused-parameter','-Wno-array-bounds','-I',str(tests/'prime_diagnostic_stub'),'-I',str(lib)]
    cmd += [str(fw/'src/servo/ServoBus.cpp'),str(fw/'src/servo/ServoProfile.cpp'),str(fw/'src/core/Availability.cpp'),str(fw/'src/diagnostics/PrimeDiagnostic.cpp')]
    cmd += [str(lib/n) for n in ('SCS.cpp','SCSerial.cpp','SMS_STS.cpp')]
    if bus_only:cmd+=['-o',out+'/test_motion_service_executor']
    subprocess.run(cmd,check=True)
    subprocess.run([out+'/test_motion_service_executor'],check=True)
