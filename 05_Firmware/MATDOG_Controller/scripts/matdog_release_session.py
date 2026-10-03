#!/usr/bin/env python3
"""Three guarded release stages. Importing/offline-check never opens a port.

The application provenance is pinned separately from host tools/docs. A failed
stage is consumed: automatic retries/reflash/repeated movement are forbidden.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import calibration_hw_session as native


class ReleaseFailure(Exception):
    pass


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


def write_new(path, value):
    with Path(path).open('x') as out:
        json.dump(value, out, indent=2)
        out.write('\n')


def require(condition, reason):
    if not condition:
        raise ReleaseFailure(reason)


def canonical_receipt(config, receipt):
    require(receipt.get('hardware_observed') is True, 'no actual hardware observation receipt')
    require(receipt.get('build_id') == config['build_id'], 'receipt build mismatch')
    require(receipt.get('application_sha256') == config['application_sha256'], 'receipt binary mismatch')


class Operations:
    def command(self, command, cwd, log, env=None):
        runtime_env = dict(os.environ, GIT_TERMINAL_PROMPT='0', GIT_SSH_COMMAND='ssh -o BatchMode=yes')
        if env: runtime_env.update(env)
        with Path(log).open('a') as out:
            out.write('COMMAND=' + repr(command) + '\n')
            process = subprocess.Popen(command, cwd=cwd, env=runtime_env, stdout=out,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            try:
                code = process.wait(timeout=3600)
            except (KeyboardInterrupt, subprocess.TimeoutExpired):
                # The native runner handles SIGTERM with ABORT/SAFE_OFF. Give
                # it time to finish before closing its sole serial connection.
                os.killpg(process.pid, signal.SIGTERM)
                # Worst case: 1 s settle + 13 x 3 s SAFE_OFF timeouts,
                # followed by the native evidence export. Do not kill early.
                try: process.wait(timeout=55)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                raise
        require(code == 0, f'command failed ({code}); inspect {log}')

    def text(self, command, cwd):
        result = subprocess.run(command, cwd=cwd, capture_output=True, text=True, timeout=30)
        require(result.returncode == 0, result.stderr.strip() or repr(command))
        return result.stdout.strip()

    def inspect(self, config, directory, safe_off=True):
        log = native.Log(str(directory/'controller-inspection.log'))
        link = native.SerialLink(config['port'], log)
        session = native.Session(link, log)
        try:
            link.open()
            link.send('')
            session.verify_signature(config['build_id'])
            session.verify_maintenance()
            if safe_off: session.safe_off_all()
            session.authority_none()
            positions = session.read_positions('actual torque-off pose') if safe_off else {}
            uptime = session.uptime()
            return {'hardware_observed': True, 'build_id': config['build_id'],
                    'application_sha256': config['application_sha256'],
                    'uptime_ms': uptime, 'observed_epoch': time.time(),
                    'mode':'MAINTENANCE','authority':'NONE',
                    'safe_off_ids': native.INSTALLED if safe_off else [],
                    'positions': {str(bus): list(value) for bus,value in positions.items()}}
        except Exception:
            if safe_off and link.fd is not None and not link.lost: session.emergency_stop()
            raise
        finally:
            link.close()
            log.close()

    def attestation(self, text):
        require(sys.stdin.isatty(), 'operator presence/pose attestation needs an interactive terminal')
        print(text, flush=True)
        return input('Digitare GO_Q0_SOSTENUTO: ').strip() == 'GO_Q0_SOSTENUTO'

    def wait_power_cycle(self, port, directory, timeout):
        print('Eseguire ora il ciclo reale di alimentazione del robot, con supporti e disgiuntore. Nessun RESET viene inviato.', flush=True)
        deadline = time.monotonic() + timeout
        while Path(port).exists():
            require(time.monotonic() < deadline, 'power-off not observed before deadline')
            time.sleep(.2)
        off = time.time()
        while not Path(port).exists():
            require(time.monotonic() < deadline, 'power-on/USB return not observed before deadline')
            time.sleep(.2)
        time.sleep(3)
        write_new(directory/'power-cycle.json', {'usb_absent_epoch': off,
                   'usb_return_epoch': time.time(), 'operator_power_cycle_requested': True})

    def run_native(self, config, directory, phase, extra):
        args = ['--phase', phase, '--no-flash', '--evidence-dir', str(directory),
                '--port', config['port'], '--firmware-sketch-dir', config['firmware_sketch'],
                '--result-json', str(directory/f'{phase}-result.json')] + extra
        self.command([sys.executable, config['runner'], *args],
                     config['tools_repo'], directory/f'{phase}-process.log')
        return load(directory/f'{phase}-result.json')

    def regressions(self, config, directory):
        root = Path(config['tools_repo'])
        scripts = root/'05_Firmware/MATDOG_Controller/scripts'
        for name in ['test_daly_calibration_guard.py', 'test_calibration_hw_session.py',
                     'test_matdog_release_session.py']:
            self.command([sys.executable,str(scripts/'tests'/name)],root,directory/'regressions.log')
        # Native recovery/thermal/NVS regressions; no generic audit rerun here.
        self.command(['bash',str(scripts/'tests/run_host_tests.sh')],root,directory/'regressions.log')


def preflight(package, config, ops):
    require(ops.text(['git','rev-parse','HEAD'],config['tools_repo'])==config['tools_commit'],
            'host tools commit moved; repack the reviewed automation')
    require(ops.text(['git','status','--porcelain'],config['tools_repo'])=='',
            'host tools worktree is dirty; preserve it and stop')
    require(ops.text(['git','status','--porcelain'], Path(config['firmware_sketch']).parents[1]) == '',
            'pinned flash worktree is dirty')
    require(ops.text(['git','rev-parse','HEAD'], Path(config['firmware_sketch']).parents[1]) == config['firmware_commit'],
            'pinned flash worktree moved')
    manifest = native.read_manifest(package/'application/matdog_build_manifest.txt')
    checks = {'SOURCE_COMMIT':config['firmware_commit'], 'BUILD_ID':config['build_id'],
              'SOURCE_STATE':'CLEAN','HARDWARE_PROFILE':'ROBOT_POWERED','OTA_INGEST_ENABLED':'0',
              'LAYOUT_ID':'MATDOG_16M_2x5M_NVS_V1', 'APPLICATION_SHA256':config['application_sha256'],
              'PARTITION_TABLE_SHA256':config['partition_sha256'], 'APP_PARTITION_SIZE':'5242880'}
    for key,value in checks.items(): require(manifest.get(key)==value,'manifest mismatch: '+key)
    binary = package/'application/MATDOG_Controller.ino.bin'
    require(digest(binary)==config['application_sha256'],'release application SHA mismatch')
    require(binary.stat().st_size==int(manifest['APPLICATION_SIZE']),'release application size mismatch')
    require(config['build_id'].encode() in binary.read_bytes(),'embedded build ID missing')
    canonical=Path(config['firmware_sketch'])/'build/esp32.esp32.esp32s3'
    require(digest(canonical/'MATDOG_Controller.ino.bin')==config['application_sha256'],'canonical binary mismatch')
    require((canonical/'matdog_build_manifest.txt').read_bytes()==(package/'application/matdog_build_manifest.txt').read_bytes(),
            'canonical manifest differs from release')
    require(digest(canonical/'MATDOG_Controller.ino.partitions.bin')==config['partition_sha256'],'canonical partition artifact differs')
    require((canonical/'sdkconfig').read_bytes()==(package/'application/sdkconfig').read_bytes(),
            'canonical SDK configuration differs from release')
    for relative, sha in config['tool_hashes'].items():
        require(digest(package/relative)==sha,'packaged host tool changed: '+relative)
    for relative, sha in config['artifact_hashes'].items():
        require(digest(package/relative)==sha,'artifact changed: '+relative)
    backup=Path(config['backup'])
    require(backup.stat().st_size==16777216 and digest(backup)==config['backup_sha256'],'historical backup integrity failed')
    require(digest(package/'rollback/MATDOG_Controller.ino.bin')==config['rollback_sha256'],'rollback image changed')
    old=native.read_manifest(package/'rollback/matdog_build_manifest.txt')
    require(old.get('APPLICATION_SHA256')==config['rollback_sha256'] and old.get('HARDWARE_PROFILE')=='ROBOT_POWERED',
            'rollback profile/manifest mismatch')


def pose_gate(package):
    plan=load(package/'initial-pose-plan.json')
    require(plan.get('qualified_path') is True and bool(plan.get('qualification_evidence')),
            'INITIAL_POSE_BLOCKED: '+plan.get('block_reason',
                'no qualified startup path and reference for all twelve joints'))
    if plan.get('startup_recovery_enabled') is True:
        from matdog_startup_reference import Q0,UNITS,GEOMETRY
        reference=load(package/'evidence/startup-reference.json')
        geometry=load(package/'evidence/startup-geometry-v5.json')
        require(reference.get('reference_status')=='VERIFIED_12_OF_12' and reference.get('same_boot') is True and
                reference.get('q0')=={str(k):v for k,v in Q0.items()} and
                reference.get('units')=={str(k):v for k,v in UNITS.items()} and reference.get('geometry')==GEOMETRY,
                'STARTUP_REFERENCE_MISMATCH')
        require(geometry.get('status')=='PASS' and geometry.get('support_tolerance_ticks')==10 and
                geometry.get('rf_lower_direction')==1 and len(geometry.get('outcomes',[]))==396 and
                all(item.get('status','').startswith('PASS') for item in geometry['outcomes']),
                'STARTUP_GEOMETRY_NOT_QUALIFIED')


def check_safe_off_positions(config, record):
    canonical_receipt(config,record)
    require(record.get('mode')=='MAINTENANCE' and record.get('authority')=='NONE',
            'MAINTENANCE or authority NONE not verified')
    require(record.get('safe_off_ids')==native.INSTALLED,'SAFE_OFF not verified for all 13 servos')
    positions=record.get('positions',{})
    require(set(positions)==set(map(str,native.CR2C)),'missing/duplicate 12-joint encoder evidence')
    for bus,reference in native.CR2C.items():
        position,torque=positions[str(bus)]
        require(isinstance(position,int) and not isinstance(position,bool) and
                0<=position<4096 and torque==0,'encoder invalid or torque not OFF: '+str(bus))


def check_positions(config, record):
    check_safe_off_positions(config,record)
    positions=record['positions']
    for bus,reference in native.CR2C.items():
        position,_=positions[str(bus)]
        require(abs(position-reference)<82,'pose outside existing Q0 plausibility screen: '+str(bus))
    # Encoders are only a plausibility screen. A separate physical jig/square
    # attestation is indispensable; logs alone cannot establish nominal Q0.


def phase_flash(package, config, directory, ops):
    write_new(directory/'flash-started.json', {'attempt':1})
    env={'MATDOG_FLASH_PROFILE':'ROBOT_POWERED','MATDOG_FLASH_OTA_INGEST':'0',
         'MATDOG_FLASH_BACKUP':config['backup'],'MATDOG_FLASH_BACKUP_SHA256':config['backup_sha256'],
         'MATDOG_ESP32_PORT':config['port'],'MATDOG_ESP32_MAC':config['mac']}
    sketch=Path(config['firmware_sketch'])
    ops.command(['bash',str(sketch/'scripts/flash_app_only.sh')],sketch,directory/'flash-process.log',env)
    output=(directory/'flash-process.log').read_text()
    for line in ['APPLICATION_ONLY_FLASH = PASS','FLASHED_HARDWARE_PROFILE = ROBOT_POWERED','FLASHED_OTA_INGEST_ENABLED = 0']:
        require(line in output,'canonical flash success evidence missing: '+line)
    # Independent connection after esptool exits; never concurrent with it.
    deadline=time.monotonic()+30
    while not Path(config['port']).exists():
        require(time.monotonic()<deadline,'USB did not re-enumerate after application write')
        time.sleep(.2)
    result=ops.inspect(config,directory)
    check_safe_off_positions(config,result)
    result.update(nominal_pose_verified=False, status='FLASH_OK')
    write_new(directory/'FLASH_OK.json',result)
    print('FLASH_OK')


def phase_calibrate(package, config, directory, ops):
    pose_gate(package)
    flash=load(directory/'FLASH_OK.json'); check_safe_off_positions(config,flash)
    fresh=ops.inspect(config,directory)
    startup=load(package/'initial-pose-plan.json').get('startup_recovery_enabled') is True
    if startup:
        from matdog_startup_reference import classify_positions
        check_safe_off_positions(config,fresh)
        try: classify_positions({int(b):value[0] for b,value in fresh['positions'].items()})
        except ValueError as error: raise ReleaseFailure(str(error)) from error
    else: check_positions(config,fresh)
    anchor=flash['observed_epoch']-flash['uptime_ms']/1000
    require(abs(fresh['observed_epoch']-fresh['uptime_ms']/1000-anchor)<=3,'reboot or host-clock change after FLASH_OK')
    require(ops.attestation('GO per l’intera procedura: installazione e riferimenti invariati; operatore presente; corpo e giunti passivi sostenuti contro gravità; zona libera; disgiuntore accessibile; caricatore scollegato.' if startup else
                            'Operatore presente; Q0 a dime; robot sostenuto; zona libera; disgiuntore accessibile; caricatore scollegato.'),
            'hardware execution prerequisites not attested')
    write_new(directory/'calibration-started.json', {'attempt':1,'boot_anchor':anchor})
    result=ops.run_native(config,directory,'all',['--confirm-q0-pose','--confirm-operator-go','--require-daly',
                              '--expected-boot-anchor',str(anchor)]+(['--qualified-startup-recovery'] if startup else []))
    require(result.get('hardware_observed') is True and result.get('contacts_accepted')==24 and result.get('fresh_q0') is True,
            'full calibration is not actual fresh 24/24')
    require(result.get('build_id')==config['build_id'] and isinstance(result.get('daly'),dict) and
            result['daly'].get('blocks',0)>0,'calibration signature or DALY evidence missing')
    persisted=ops.run_native(config,directory,'persist',['--require-daly','--expected-boot-anchor',str(anchor)])
    receipt=load(directory/'persistence_ack.json')
    require(persisted.get('hardware_observed') is True and persisted.get('save_ok') is True and persisted.get('ack_ok') is True,
            'SAVE/ACK not verified on hardware')
    require(persisted.get('build_id')==config['build_id'] and isinstance(persisted.get('daly'),dict) and
            persisted['daly'].get('blocks',0)>0,'SAVE/ACK signature or DALY evidence missing')
    require(persisted.get('generation')==receipt.get('generation') and persisted['generation']>0 and receipt.get('build_id')==config['build_id'],
            'ACK generation/build mismatch')
    persisted.update(application_sha256=config['application_sha256'],status='CALIBRATION_SAVE_ACK_OK')
    write_new(directory/'CALIBRATION_SAVE_ACK_OK.json',persisted)
    print(f"24/24 SAVE_OK ACK_OK generation={persisted['generation']}")


def finalize_git(config, directory, ops):
    root=Path(config['tools_repo'])
    require(ops.text(['git','status','--porcelain'],root)=='','tools branch has pre-existing changes; no integration')
    feature=ops.text(['git','rev-parse','HEAD'],root)
    require(ops.text(['git','branch','--show-current'],root)=='fix/calibration-post-abort-thermal','unexpected feature branch')
    suffix=directory.name
    ops.command(['git','fetch','origin','main'],root,directory/'integration.log')
    remote=ops.text(['git','rev-parse','origin/main'],root)
    main=ops.text(['git','rev-parse','main'],root)
    ops.command(['git','merge-base','--is-ancestor',main,remote],root,directory/'integration.log')
    worktrees=ops.text(['git','worktree','list','--porcelain'],root)
    require('branch refs/heads/main' not in worktrees,'main checked out elsewhere; preserve it')
    integration=directory/'integration-worktree'
    ops.command(['git','worktree','add','-b','release/post-abort-'+suffix,str(integration),remote],root,directory/'integration.log')
    # Conflicts leave this isolated checkout for review; main remains untouched.
    ops.command(['git','merge','--no-ff','--no-commit',feature],integration,directory/'integration.log')
    require(not ops.text(['git','diff','--name-only','--diff-filter=U'],integration),'merge conflicts; retain isolated result')
    # Main must not acquire firmware that differs from the hardware-tested image.
    firmware_paths=['05_Firmware/MATDOG_Controller/src','05_Firmware/MATDOG_Controller/MATDOG_Controller.ino',
                    '05_Firmware/MATDOG_Controller/partitions.csv']
    require(ops.text(['git','diff',config['firmware_commit'],'--',*firmware_paths],integration)=='',
            'merged firmware differs from the installed binary; block integration')
    gait=ops.text(['git','rev-parse','feat/g5a-stabilization-feasibility'],root)
    for incoming in [feature,remote]:
        bases=ops.text(['git','merge-base',gait,incoming],root)
        require(bases!=gait,'gait branch ancestry would be incorporated')
    ops.regressions(dict(config,tools_repo=str(integration)),directory)
    ops.command(['git','commit','-m','Merge verified MATDOG post-abort calibration release'],integration,directory/'integration.log')
    merged=ops.text(['git','rev-parse','HEAD'],integration)
    # Compare-and-swap, no history rewrite. A rejected push retains the local result.
    ops.command(['git','update-ref','refs/heads/main',merged,main],root,directory/'integration.log')
    ops.command(['git','push','origin',merged+':refs/heads/main'],root,directory/'integration.log')
    return merged


def phase_finalize(package, config, directory, ops, timeout):
    saved=load(directory/'CALIBRATION_SAVE_ACK_OK.json'); canonical_receipt(config,saved)
    write_new(directory/'finalize-started.json',{'attempt':1})
    ops.wait_power_cycle(config['port'],directory,timeout)
    result=ops.run_native(config,directory,'verify-persistence',[])
    require(result.get('hardware_observed') is True and result.get('generation')==saved['generation'] and
            result.get('build_id')==config['build_id'] and result.get('acknowledged_record_intact') is True and
            result.get('motion_authorized') is False and result.get('authority')=='NONE','post-power-cycle verification incomplete')
    ops.regressions(config,directory)
    write_new(directory/'HARDWARE_AND_PERSISTENCE_PASS.json',dict(result,application_sha256=config['application_sha256']))
    root=Path(config['tools_repo'])
    require(ops.text(['git','status','--porcelain'],root)=='','Git state not clean before documentation; preserve changes')
    summary=(f"\n\n## Hardware session {directory.name}\n\n"
             f"Firmware `{config['firmware_commit']}`, SHA256 `{config['application_sha256']}`.\n"
             f"Observed 24/24, SAVE/ACK generation {saved['generation']}, real operator power cycle,\n"
             "acknowledged record intact, MOTION_AUTHORIZED=0; relevant regressions PASS.\n"
             f"Evidence: `{directory}`. Binary provenance stays pinned; this is a documentation commit.\n")
    docs=['05_Firmware/MATDOG_Controller/VALIDATION.md','05_Firmware/MATDOG_Controller/CHANGELOG.md',
          '05_Firmware/MATDOG_Controller/POST_ABORT_RECOVERY_RUNBOOK.md','01_Docs/02_Architecture/ARCHITECTURE.md']
    for name in docs:
        with (root/name).open('a') as out: out.write(summary)
    ops.command(['git','add',*docs],root,directory/'documentation.log')
    ops.command(['git','commit','-m','docs: record actual MATDOG calibration SAVE ACK and reboot evidence'],root,directory/'documentation.log')
    documentation_commit=ops.text(['git','rev-parse','HEAD'],root)
    merged=finalize_git(config,directory,ops)
    write_new(directory/'FINALIZED.json',dict(result,firmware_commit=config['firmware_commit'],main_commit=merged,
                                            host_tools_commit=config['tools_commit'],documentation_commit=documentation_commit,
                                            application_sha256=config['application_sha256']))
    print('FINALIZED_MAIN='+merged)


def export_evidence(package, config, directory, status, reason=None):
    shutil.copy2(package/'application/matdog_build_manifest.txt',directory/'installed-build-manifest.txt')
    report={'status':status,'reason':reason,'firmware_commit':config['firmware_commit'],
            'host_tools_commit':config['tools_commit'],
            'application_sha256':config['application_sha256'],'hardware_validation_inferred_from_simulation':False}
    (directory/'OPERATIONAL_REPORT.json').write_text(json.dumps(report,indent=2)+'\n')
    files=[p for p in directory.rglob('*') if p.is_file() and '.git' not in p.parts and
           'integration-worktree' not in p.parts and p.name!='EVIDENCE_SHA256SUMS']
    (directory/'EVIDENCE_SHA256SUMS').write_text(''.join(digest(p)+'  '+str(p.relative_to(directory))+'\n' for p in sorted(files)))


def main(argv=None, ops=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('phase',choices=['flash','calibrate','finalize'])
    parser.add_argument('--package',required=True,type=Path)
    parser.add_argument('--session-dir',type=Path)
    parser.add_argument('--offline-check',action='store_true')
    parser.add_argument('--hardware-session',action='store_true')
    parser.add_argument('--power-cycle-timeout',type=int,default=900)
    args=parser.parse_args(argv); ops=ops or Operations()
    package=args.package.resolve(); directory=args.session_dir
    try:
        config=load(package/'session-config.json')
        preflight(package,config,ops)
        if args.offline_check:
            print('OFFLINE_ARTIFACTS=PASS; HARDWARE_IO=NO')
            if args.phase=='flash':
                print('APPLICATION_ONLY_FLASH_PREPARATION=PASS; POSE_ADMISSION=CALIBRATION_ONLY')
                return 0
            try: pose_gate(package)
            except ReleaseFailure as e:
                print(str(e))
                print('OFFLINE_READY=NO')
            return 0
        require(args.hardware_session,'hardware stage requires explicit --hardware-session; offline by default')
        require(directory is not None,'--session-dir is required; keep one directory across the three stages')
        directory=directory.resolve(); directory.mkdir(parents=True,exist_ok=True)
        require(not (directory/'FAILED.json').exists(),'previous failure retained; no automatic retry in this session')
        if args.phase=='flash': phase_flash(package,config,directory,ops)
        elif args.phase=='calibrate': phase_calibrate(package,config,directory,ops)
        else: phase_finalize(package,config,directory,ops,args.power_cycle_timeout)
        export_evidence(package,config,directory,'PASS_'+args.phase.upper())
        return 0
    except (ReleaseFailure,native.SessionFailure,OSError,ValueError,KeyError,TypeError,subprocess.SubprocessError,KeyboardInterrupt) as e:
        print('BLOCKED: '+str(e),file=sys.stderr)
        if directory is not None and directory.exists() and 'config' in locals():
            if not (directory/'FAILED.json').exists(): write_new(directory/'FAILED.json',{'phase':args.phase,'reason':str(e)})
            export_evidence(package,config,directory,'BLOCKED',str(e))
        return 1


if __name__=='__main__': sys.exit(main())
