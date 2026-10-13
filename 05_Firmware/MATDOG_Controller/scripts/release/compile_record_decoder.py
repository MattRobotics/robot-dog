#!/usr/bin/env python3
"""Compile the production decoder; no device access, outputs outside Git."""
import subprocess,sys
from pathlib import Path
FW=Path(__file__).resolve().parents[2]
def compile_decoder(out,unit=None):
 script=(FW/'scripts/tests/run_calibration_restore_tests.sh').read_text();names=script.split('SOURCES=(')[1].split(')')[0].split();files=[FW/'src/calibration'/f'{x}.cpp' for x in names]
 files += [FW/'src/actuator'/f'{x}.cpp' for x in 'OperationalEnvelope MotionDeadman ActuatorRuntime ActuatorWritePolicy CalibrationSequencePlan CalibrationTargetResolver CalibrationGeometryProfile'.split()]
 files += [FW/'src/servo/ServoPopulation.cpp',*[FW/'src/core'/f'{x}.cpp' for x in 'SystemState ActuatorAuthority OperatingMode'.split()]]
 subprocess.run(['g++','-std=c++17','-O1','-Wall','-Wextra','-Werror','-DDISABLED=0',str(unit or FW/'scripts/release/validate_record_offline.cpp'),*map(str,files),'-o',str(out)],check=True)
if __name__=='__main__':compile_decoder(Path(sys.argv[1]))
