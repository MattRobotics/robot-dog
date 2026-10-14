#!/usr/bin/env python3
"""Run G5-B and the pinned historical G1-G5-A regressions, wholly offline.

Historical test tools are materialized only in TemporaryDirectory, linked to CURRENT
unchanged G5-A sources. No git checkout/fetch, device I/O, firmware build or installation.
If the historical object/dependencies are missing, report BLOCKED rather than PASS.
"""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
MOTION=ROOT/'05_Firmware/MATDOG_Controller/src/motion'
HISTORICAL='e1704719979789cd9c9f18741e4546725558797e'
BASE='fdd04345c39b50137cc7cdc50fb08cd592b8c2af'
DEFAULT_OUT=ROOT/'09_Logs/Validation_Reports/G5B_IMU_Offline_2026-10-14'
FLAGS=['-std=c++17','-Wall','-Wextra','-Werror','-pedantic','-fno-exceptions','-fno-rtti']
CORE=['LegKinematics','LegInverseKinematics','FootContact','BodyPose','StandTrajectory',
      'ImuAttitude','BodyStabilizer','TiltedContactIk','TiltCompensation']


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()


def source_scope():
    changed=subprocess.check_output(['git','diff','--name-only',BASE],cwd=ROOT,text=True).splitlines()
    changed+=subprocess.check_output(['git','ls-files','--others','--exclude-standard'],cwd=ROOT,text=True).splitlines()
    allowed=('06_Software/Matdog_Core/stabilization_g5b/','09_Logs/Validation_Reports/G5B_IMU_Offline_2026-10-14/')
    bad=[p for p in changed if not p.startswith(allowed)]
    code='\n'.join(p.read_text() for p in HERE.glob('*') if p.suffix in ('.h','.cpp'))
    stripped=re.sub(r'/\*.*?\*/|//[^\n]*','',code,flags=re.S)
    forbidden=re.findall(r'\b(?:Arduino|HardwareSerial|ServoBus|Preferences|new|delete|malloc|calloc|realloc|free|vector|millis|micros|delay)\b',stripped)
    return dict(name='scope_and_core_purity',status='PASS' if not bad and not forbidden else 'FAIL',
                unexpected_paths=bad,forbidden_tokens=forbidden,main_base=BASE)


def stage_historical(directory):
    stage=directory/'reference'; stage.mkdir()
    tests=stage/'05_Firmware/MATDOG_Controller/scripts/tests'; tests.mkdir(parents=True)
    src=stage/'05_Firmware/MATDOG_Controller/src'; src.mkdir(); (src/'motion').symlink_to(MOTION,target_is_directory=True)
    (stage/'03_CAD').symlink_to(ROOT/'03_CAD',target_is_directory=True)
    for part in ('kinematics','calibration'):
        shutil.copytree(ROOT/'06_Software/Matdog_Core'/part,stage/'06_Software/Matdog_Core'/part)
    shutil.copytree(ROOT/'09_Logs/Validation_Reports',stage/'09_Logs/Validation_Reports')
    manifest=json.loads((ROOT/'05_Firmware/MATDOG_Controller/scripts/tests/motion_integration_manifest_pr2.json').read_text())
    paths=[row['path'] for row in manifest['deferred_sources'] if row['path'].startswith('05_Firmware/MATDOG_Controller/scripts/tests/')]
    paths += ['06_Software/Matdog_Core/'+p for p in ('stabilization_audit','pose_audit','gait_audit','contact_audit')]
    paths += ['06_Software/Matdog_Core/kinematics/'+p for p in ('matdog_motion_geometry_export.py','matdog_contact_stand_export.py')]
    blob=subprocess.check_output(['git','archive',HISTORICAL,*paths],cwd=ROOT)
    with tarfile.open(fileobj=io.BytesIO(blob)) as archive: archive.extractall(stage,filter='data')
    pins=[]
    for row in manifest['deferred_sources']:
        p=stage/row['path']
        if p.is_file():
            actual=sha(p)
            if actual!=row['sha256']: raise RuntimeError('Historical hash mismatch: '+row['path'])
            pins.append(dict(path=row['path'],sha256=actual))
    output=stage/'09_Logs/Validation_Reports/G5A_Stabilization_Feasibility'; output.mkdir(exist_ok=True)
    return stage,tests,pins


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--output',type=Path,default=DEFAULT_OUT)
    args=parser.parse_args(); out=args.output.resolve(); out.mkdir(parents=True,exist_ok=True)
    env={**os.environ,'OPENBLAS_NUM_THREADS':'1','PYTHONDONTWRITEBYTECODE':'1',
         'PYTHONPATH':os.pathsep.join([str(ROOT/'06_Software/Matdog_Core/kinematics'),str(ROOT/'06_Software/Matdog_Core/calibration'),os.environ.get('PYTHONPATH','')]),
         'ASAN_OPTIONS':'detect_leaks=1:halt_on_error=1','UBSAN_OPTIONS':'halt_on_error=1:print_stacktrace=1'}
    gates=[source_scope()]; cxx=os.environ.get('CXX','g++'); historical_pins=[]

    def run(name,command,cwd=ROOT):
        p=subprocess.run(list(map(str,command)),cwd=cwd,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        (out/(name+'.log')).write_text(p.stdout)
        gate=dict(name=name,status='PASS' if p.returncode==0 else 'FAIL',returncode=p.returncode,
                  command=[str(s).replace(str(ROOT),'$REPO') for s in command],output_sha256=hashlib.sha256(p.stdout.encode()).hexdigest())
        gates.append(gate); print(gate['status'],name,flush=True)
        if p.returncode: print(p.stdout[-3000:],flush=True)
        return p.returncode==0

    with tempfile.TemporaryDirectory(prefix='matdog-g5b-offline-') as temporary:
        tmp=Path(temporary); sources=[MOTION/(n+'.cpp') for n in CORE]+[HERE/'StabilizationPipeline.cpp']
        includes=['-I',str(MOTION),'-I',str(HERE)]
        if run('g5b_strict_build',[cxx,*FLAGS,'-O2',*includes,HERE/'tests/test_pipeline.cpp',*sources,'-o',tmp/'test']):
            run('g5b_cpp',[tmp/'test'])
        sanitizer=shutil.which('clang++') or cxx
        for name,test,extra in [('g5b',HERE/'tests/test_pipeline.cpp',[HERE/'StabilizationPipeline.cpp'])]:
            if run(name+'_sanitizer_build',[sanitizer,*FLAGS,'-g','-O1','-fsanitize=address,undefined','-fno-omit-frame-pointer',
                    *includes,test,*[MOTION/(n+'.cpp') for n in CORE],*extra,'-o',tmp/(name+'_san')]):
                run(name+'_asan_ubsan',[tmp/(name+'_san')])
        if run('g5b_bridge_build',[cxx,*FLAGS,'-O2','-shared','-fPIC',*includes,HERE/'tests/bridge.cpp',*sources,'-o',tmp/'bridge.so']):
            run('g5b_independent_oracle_and_simulations',[sys.executable,HERE/'tests/offline_oracle.py',tmp/'bridge.so',out])
            repeated=tmp/'repeated'; repeated.mkdir()
            if run('g5b_deterministic_replay',[sys.executable,HERE/'tests/offline_oracle.py',tmp/'bridge.so',repeated]):
                names=('offline_results.json','simulation_trace.csv')
                same=all((out/n).is_file() and sha(out/n)==sha(repeated/n) for n in names)
                gates.append(dict(name='g5b_byte_identical_replay',status='PASS' if same else 'FAIL',files=list(names)))
        # Compiler-reported per-function stack frames, not an embedded whole-call-chain bound.
        if run('g5b_stack_build',[cxx,*FLAGS,'-O2','-fstack-usage',*includes,'-c',HERE/'StabilizationPipeline.cpp','-o',tmp/'pipeline.o']):
            shutil.copyfile(tmp/'pipeline.su',out/'host_stack_usage.txt')
        run('main_motion_source_provenance',[sys.executable,ROOT/'05_Firmware/MATDOG_Controller/scripts/tests/run_motion_convergence_tests.py','--source-only'])
        run('existing_kinematics_python',[sys.executable,'-m','unittest','discover','-s',ROOT/'06_Software/Matdog_Core/kinematics/tests'],cwd=ROOT)
        # Historical visual-zero tests consume an obsolete calibration-status enum on main.
        # Preserve the real FAIL and verify identical baseline files; do not edit calibration.
        log=(out/'existing_kinematics_python.log').read_text()
        baseline_files=[ROOT/'06_Software/Matdog_Core/kinematics/tests/test_matdog_leg_fk_live.py',
                        ROOT/'06_Software/Matdog_Core/kinematics/matdog_leg_fk_live.py',
                        ROOT/'06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml']
        baseline_identical=all(p.read_bytes()==subprocess.check_output(['git','show',BASE+':'+str(p.relative_to(ROOT))],cwd=ROOT) for p in baseline_files)
        known_baseline_failure=(baseline_identical and 'FAILED (errors=16)' in log and
            log.count("RuntimeError: calibration_status inatteso: 'DIGITAL_ZERO_CALIBRATED_AND_VERIFIED'; atteso 'VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION'")==16)
        if known_baseline_failure:
            gates[-1].update(required_for_g5b=False,baseline_identical=True,
                baseline_reason='16 errors from obsolete visual-zero calibration-status expectation; protected files unchanged',
                baseline_sha256={str(p.relative_to(ROOT)):sha(p) for p in baseline_files})
        modules=[p.stem for p in sorted((ROOT/'06_Software/Matdog_Core/kinematics/tests').glob('test_*.py')) if p.stem!='test_matdog_leg_fk_live']
        run('pertinent_kinematics_python',[sys.executable,'-m','unittest',*modules],cwd=ROOT/'06_Software/Matdog_Core/kinematics/tests')
        try:
            stage,tests,historical_pins=stage_historical(tmp)
        except (subprocess.CalledProcessError,RuntimeError) as e:
            gates.append(dict(name='historical_G1_G5A_materialization',status='BLOCKED',reason=str(e)))
        else:
            # The legacy test suites use all pure units, including G3/G4. Never production runtime files.
            motion=sorted(MOTION.glob('*.cpp')); legacy=['test_motion_kinematics','test_contact_stand','test_startup_timed_stand',
                                                      'test_pose_support','test_gait','test_contact_mode','test_stabilization']
            objects=tmp/'objects'; objects.mkdir()
            compiled=run('legacy_core_build',[cxx,*FLAGS,'-O1','-c',*motion],cwd=objects)
            objs=sorted(objects.glob('*.o'))
            if compiled:
                for name in legacy:
                    if run(name+'_build',[cxx,*FLAGS,'-O1',tests/(name+'.cpp'),*objs,'-o',tmp/name]): run(name,[tmp/name])
                for name in ('motion','contact_stand','startup_timed'):
                    driver=name+'_oracle_driver'
                    if run(driver+'_build',[cxx,*FLAGS,'-O1',tests/(driver+'.cpp'),*objs,'-o',tmp/driver]):
                        run(name+'_historical_python_oracle',[sys.executable,tests/('test_'+name+'_oracle.py'),tmp/driver],cwd=stage)
            if run('g5a_sanitizer_build',[sanitizer,*FLAGS,'-g','-O1','-fsanitize=address,undefined','-fno-omit-frame-pointer',
                tests/'test_stabilization.cpp',*motion,'-o',tmp/'g5a_san']): run('g5a_asan_ubsan',[tmp/'g5a_san'])
            audit=stage/'06_Software/Matdog_Core/stabilization_audit'
            dependency_check=subprocess.run([sys.executable,'-c','import numpy,scipy,yaml,trimesh,fcl,rtree,networkx'],env=env,capture_output=True,text=True)
            if dependency_check.returncode:
                gates.append(dict(name='historical_mesh_oracles',status='BLOCKED',reason='Missing existing offline audit dependencies; set PYTHONPATH to the historical dependency directory.'))
            else:
                run('g5a_independent_checks',[sys.executable,audit/'independent_checks.py'],cwd=audit)
                run('g5a_loop_simulations',[sys.executable,audit/'perturbation_sim.py'],cwd=audit)
                run('g4_independent_oracle',[sys.executable,audit/'run_g4_oracle.py'],cwd=audit)
                for file in ('frames_oracle.json','ik_oracle.json','perturbation_results.json','oracle_results_g5a.json'):
                    produced=stage/'09_Logs/Validation_Reports/G5A_Stabilization_Feasibility'/file
                    if produced.is_file(): shutil.copyfile(produced,out/('legacy_'+file))
    source_files=sorted([p for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts]+[p for p in MOTION.glob('*') if p.is_file()])
    result=dict(scope='offline only; not a firmware release',gates=gates,historical_test_commit=HISTORICAL,
                historical_test_sources=historical_pins,source_sha256={str(p.relative_to(ROOT)):sha(p) for p in source_files},
                all_passed=all(g['status']=='PASS' for g in gates),
                required_gates_passed=all(g['status']=='PASS' for g in gates if g.get('required_for_g5b',True)),
                hardware_commands_executed=False)
    (out/'validation_results.json').write_text(json.dumps(result,indent=2)+'\n')
    print('G5B_OFFLINE_REQUIRED = '+('PASS' if result['required_gates_passed'] else 'FAIL/BLOCKED'))
    print('ALL_OBSERVED_SUITES = '+('PASS' if result['all_passed'] else 'FAIL/BLOCKED (see preserved baseline failure)'))
    return 0 if result['required_gates_passed'] else 1


if __name__=='__main__': sys.exit(main())
