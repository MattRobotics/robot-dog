#!/usr/bin/env python3
"""Core Motion V5: Motion cadence with slow/blocking services (host model, production Motion core)."""
import os,subprocess,tempfile
from pathlib import Path
FW=Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='matdog-isolation-') as d:
 script=(FW/'scripts/tests/run_motion_service_executor_tests.sh').read_text();command=script[script.index('"$CXX" -std='):script.index('"$OUT/test_motion_service_executor"\n')]
 command=command.replace('\"$SKETCH_DIR/src/execution/MotionExecutor.cpp\"','\"$SKETCH_DIR/src/execution/OperationalFeedback.cpp\" \"$SKETCH_DIR/src/execution/MotionExecutor.cpp\"')
 command=command.replace('-Werror','-Werror -Wno-unused-function').replace('test_motion_service_executor.cpp','test_motion_isolation.cpp').replace('"$SKETCH_DIR/src/execution/MotionExecutor.cpp"','"$SKETCH_DIR/src/execution/CommissioningPolicy.cpp" "$SKETCH_DIR/src/execution/MotionExecutor.cpp"')
 subprocess.run(['bash','-c',command],check=True,env={**os.environ,'CXX':'g++','OUT':d,'SCRIPT_DIR':str(FW/'scripts/tests'),'SKETCH_DIR':str(FW)})
 subprocess.run([d+'/test_motion_service_executor'],check=True)
