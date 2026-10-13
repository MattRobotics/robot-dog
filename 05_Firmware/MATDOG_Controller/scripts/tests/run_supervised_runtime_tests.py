#!/usr/bin/env python3
import subprocess,tempfile,os
from pathlib import Path
FW=Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='matdog-supervised-') as d:
 script=(FW/'scripts/tests/run_motion_service_executor_tests.sh').read_text();command=script[script.index('"$CXX" -std='):script.index('"$OUT/test_motion_service_executor"\n')]
 command=command.replace('test_motion_service_executor.cpp',os.environ.get('MATDOG_SUPERVISED_TEST_UNIT','test_supervised_runtime.cpp')).replace('"$SKETCH_DIR/src/execution/MotionExecutor.cpp"','"$SKETCH_DIR/src/execution/OperationalFeedback.cpp" "$SKETCH_DIR/src/execution/MotionExecutor.cpp"')
 subprocess.run(['bash','-c',command],check=True,env={**os.environ,'CXX':'g++','OUT':d,'SCRIPT_DIR':str(FW/'scripts/tests'),'SKETCH_DIR':str(FW)})
 subprocess.run([d+'/test_motion_service_executor'],check=True)
