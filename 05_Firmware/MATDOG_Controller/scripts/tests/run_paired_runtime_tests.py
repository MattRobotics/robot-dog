#!/usr/bin/env python3
"""Frozen profile oracle and full production executor composition, offline only."""
import json, math, subprocess, tempfile, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4];FW=ROOT/'05_Firmware/MATDOG_Controller'
sys.path.insert(0,str(ROOT/'06_Software/Matdog_Core/motion/paired_prone'))
from paired_prone_model import PairModel,r_y
import numpy as np

def main():
    d=json.loads((ROOT/'09_Logs/Validation_Reports/Motion_Night_Run_2026-10-12/continuous_pair_path.json').read_text());scene=PairModel().scene
    with tempfile.TemporaryDirectory(prefix='matdog-paired-runtime-') as temp:
        temp=Path(temp);oracle=temp/'oracle.txt'
        with oracle.open('w') as out:
            for i,s in enumerate(d['segments']):
                for t in [0,.125,.25,.5,.75,.875,1]:
                    progress=10*t**3-15*t**4+6*t**5
                    p={k:s['start_parameters'][k]+progress*(s['end_parameters'][k]-s['start_parameters'][k]) for k in s['start_parameters']}
                    q={}
                    for leg in ('lf','rf','rh','lh'):
                        q.update({leg+'_hip_joint':p['h']*(-1 if leg in ('lf','lh') else 1),leg+'_upper_leg_joint':p['uf' if leg in ('lf','rf') else 'ur'],leg+'_lower_leg_joint':p['lf' if leg in ('lf','rf') else 'lr']})
                    z=p['z'];minimum=0
                    for n in scene.model.collision_link_names:
                        tf=scene.collision_transform(n,q);points=scene.mesh(n).hull_vertices_local@tf.rotation.T+tf.translation
                        minimum=min(minimum,float((points@r_y(p['p'])[2]+z).min()))
                    out.write(' '.join(map(str,[i,t,*q.values(),p['x'],z-minimum,p['p']]))+'\n')
        script=(FW/'scripts/tests/run_motion_service_executor_tests.sh').read_text()
        # Reuse the reviewed production linkage; replace only the test unit and output.
        command=script[script.index('"$CXX" -std='):script.index('"$OUT/test_motion_service_executor"\n')]
        command=command.replace('test_motion_service_executor.cpp','test_paired_runtime.cpp').replace('"$SKETCH_DIR/src/execution/MotionExecutor.cpp"','"$SKETCH_DIR/src/execution/OperationalFeedback.cpp" "$SKETCH_DIR/src/execution/MotionExecutor.cpp"')
        subprocess.run(['bash','-c',command],check=True,env={**__import__('os').environ,'CXX':'g++','OUT':str(temp),'SCRIPT_DIR':str(FW/'scripts/tests'),'SKETCH_DIR':str(FW)})
        subprocess.run([str(temp/'test_motion_service_executor'),str(oracle)],check=True)
if __name__=='__main__':main()
