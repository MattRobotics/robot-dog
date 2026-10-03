#!/usr/bin/env python3
"""Synthetic stage I/O and local temporary Git repositories. No device access."""
import json
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import matdog_release_session as release


class SimulatedOperations:
    def __init__(self,config):
        self.config=config; self.calls=[]; self.fail=None; self.bad=None
    def inspect(self,config,directory,safe_off=True):
        self.calls.append('inspect')
        result=dict(hardware_observed=True,build_id=config['build_id'],application_sha256=config['application_sha256'],
                    uptime_ms=100000,observed_epoch=time.time(),safe_off_ids=release.native.INSTALLED,
                    positions={str(b):[p,0] for b,p in release.native.CR2C.items()})
        if self.bad=='pose': result['positions']['21'][0]=2348
        if self.bad=='torque': result['positions']['21'][1]=1
        if self.bad=='signature': result['build_id']='old'
        if self.bad=='safe_off': result['safe_off_ids']=[]
        if self.bad=='boot': result['uptime_ms']=1
        return result
    def command(self,command,cwd,log,env=None):
        self.calls.append(command)
        if self.fail=='command': raise release.ReleaseFailure('simulated command failure')
        Path(log).write_text('APPLICATION_ONLY_FLASH = PASS\nFLASHED_HARDWARE_PROFILE = ROBOT_POWERED\nFLASHED_OTA_INGEST_ENABLED = 0\n'
                             if self.bad!='flash_verification' else 'write failed\n')
    def attestation(self,text): return self.bad!='attestation'
    def run_native(self,config,directory,phase,extra):
        self.calls.append((phase,extra))
        if self.fail==phase: raise release.ReleaseFailure('simulated native '+phase+' failure')
        result=dict(hardware_observed=True,build_id=config['build_id'],uptime_ms=100000,observed_epoch=time.time())
        if phase=='all': result.update(contacts_accepted=23 if self.bad=='23' else 24,fresh_q0=True,
                                      daly=None if self.bad=='daly' else {'blocks':4})
        if phase=='persist':
            release.write_new(directory/'persistence_ack.json',dict(build_id=config['build_id'],generation=1,uptime_ms=100000))
            result.update(generation=2 if self.bad=='generation' else 1,save_ok=True,ack_ok=True)
        if phase=='verify-persistence': result.update(generation=1,acknowledged_record_intact=self.bad!='record',
                    motion_authorized=self.bad=='motion',authority='NONE')
        return result
    def wait_power_cycle(self,port,directory,timeout):
        self.calls.append('power-cycle')
        if self.fail=='power': raise release.ReleaseFailure('simulated missing power cycle')
        release.write_new(directory/'power-cycle.json',dict(usb_absent_epoch=1,usb_return_epoch=2))
    def regressions(self,config,directory):
        self.calls.append('regressions')
        if self.fail=='tests': raise release.ReleaseFailure('simulated regression failure')
    def text(self,command,cwd): return ''


class StageTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.root=Path(self.tmp.name)
        self.package=self.root/'package';self.package.mkdir()
        self.directory=self.root/'session';self.directory.mkdir()
        self.repo=self.root/'repo';self.repo.mkdir()
        self.port=self.root/'fake-device';self.port.touch()
        self.config=dict(build_id='125d981b02a3',application_sha256='a'*64,firmware_commit='b'*40,
                         port=str(self.port),firmware_sketch=str(self.repo),tools_repo=str(self.repo),backup='fake',
                         backup_sha256='c'*64,mac='test')
        self.ops=SimulatedOperations(self.config)
        release.write_new(self.package/'initial-pose-plan.json',dict(qualified_path=True,qualification_evidence='SYNTHETIC_TEST_ONLY'))
        for name in ['05_Firmware/MATDOG_Controller/VALIDATION.md','05_Firmware/MATDOG_Controller/CHANGELOG.md',
                     '05_Firmware/MATDOG_Controller/POST_ABORT_RECOVERY_RUNBOOK.md','01_Docs/02_Architecture/ARCHITECTURE.md']:
            path=self.repo/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('baseline\n')
    def tearDown(self): self.tmp.cleanup()
    def flash(self): release.phase_flash(self.package,self.config,self.directory,self.ops)
    def calibrate(self): release.phase_calibrate(self.package,self.config,self.directory,self.ops)
    def test_three_stages_positive_simulation(self):
        self.flash();self.calibrate()
        with patch.object(release,'finalize_git',return_value='d'*40) as merge:
            release.phase_finalize(self.package,self.config,self.directory,self.ops,1);merge.assert_called_once()
        self.assertEqual(release.load(self.directory/'FINALIZED.json')['generation'],1)
        all_args=next(v[1] for v in self.ops.calls if isinstance(v,tuple) and v[0]=='all')
        self.assertIn('--require-daly',all_args);self.assertIn('--confirm-q0-pose',all_args)
        verification=next(v[1] for v in self.ops.calls if isinstance(v,tuple) and v[0]=='verify-persistence')
        self.assertEqual(verification,[])
    def test_unqualified_pose_blocks_before_any_hardware(self):
        (self.package/'initial-pose-plan.json').write_text(json.dumps(dict(qualified_path=False)))
        with self.assertRaisesRegex(release.ReleaseFailure,'INITIAL_POSE_BLOCKED'):self.flash()
        self.assertEqual(self.ops.calls,[])
    def test_flash_errors_never_emit_flash_ok(self):
        for fault in ['pose','torque','signature','safe_off','flash_verification','attestation']:
            with self.subTest(fault=fault):
                self.ops.bad=fault
                (self.directory/'flash-started.json').unlink(missing_ok=True)
                with self.assertRaises(release.ReleaseFailure):self.flash()
                self.assertFalse((self.directory/'FLASH_OK.json').exists())
    def test_flash_repeat_is_refused(self):
        self.flash();count=len(self.ops.calls)
        with self.assertRaises(FileExistsError):self.flash()
        self.assertEqual(len(self.ops.calls),count)
    def test_calibration_boot_fault_stops_before_movement(self):
        self.flash();self.ops.bad='boot'
        with self.assertRaisesRegex(release.ReleaseFailure,'reboot'):self.calibrate()
        self.assertFalse(any(isinstance(v,tuple) for v in self.ops.calls))
    def test_bad_24_or_daly_never_saves(self):
        self.flash()
        for fault in ['23','daly']:
            self.ops.bad=fault;(self.directory/'calibration-started.json').unlink(missing_ok=True)
            with self.assertRaises(release.ReleaseFailure):self.calibrate()
            self.assertFalse(any(isinstance(v,tuple) and v[0]=='persist' for v in self.ops.calls))
    def test_ack_generation_error_not_accepted(self):
        self.flash();self.ops.bad='generation'
        with self.assertRaisesRegex(release.ReleaseFailure,'generation'):self.calibrate()
        self.assertFalse((self.directory/'CALIBRATION_SAVE_ACK_OK.json').exists())
    def test_finalization_errors_never_merge_or_update_docs(self):
        self.flash();self.calibrate()
        baseline=(self.repo/'05_Firmware/MATDOG_Controller/VALIDATION.md').read_text()
        for field,value in [('fail','power'),('bad','record'),('bad','motion'),('fail','tests')]:
            with self.subTest(value=value),patch.object(release,'finalize_git') as merge:
                self.ops.fail=None;self.ops.bad=None;setattr(self.ops,field,value)
                for name in ['finalize-started.json','power-cycle.json']:(self.directory/name).unlink(missing_ok=True)
                with self.assertRaises(release.ReleaseFailure):release.phase_finalize(self.package,self.config,self.directory,self.ops,1)
                merge.assert_not_called()
                self.assertEqual((self.repo/'05_Firmware/MATDOG_Controller/VALIDATION.md').read_text(),baseline)


class LocalGitOps(release.Operations):
    def regressions(self,config,directory): self.checked=True


class OfflineOps:
    def __init__(self, config): self.config=config; self.bad=None; self.calls=[]
    def text(self, command, cwd):
        self.calls.append(command)
        host=str(cwd)==self.config['tools_repo']
        if command[1]=='status': return ' M dirty' if self.bad==('dirty_host' if host else 'dirty_firmware') else ''
        return 'wrong' if self.bad==('moved_host' if host else 'moved_firmware') else self.config['tools_commit' if host else 'firmware_commit']
    def inspect(self,*args): raise AssertionError('offline mode accessed hardware')
    def command(self,*args): raise AssertionError('offline mode ran an external action')


class PreflightTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.package=self.root/'package';self.package.mkdir()
        sketch=self.root/'frozen/05_Firmware/MATDOG_Controller'
        canonical=sketch/'build/esp32.esp32.esp32s3';canonical.mkdir(parents=True)
        for folder in ['application','rollback','automation']:(self.package/folder).mkdir()
        self.config=dict(build_id='125d981b02a3',firmware_commit='a'*40,tools_commit='b'*40,
            tools_repo=str(self.root/'host'),firmware_sketch=str(sketch),backup=str(self.root/'backup'),
            artifact_hashes={},tool_hashes={})
        (self.root/'backup').write_bytes(bytes(16777216))
        self.config['backup_sha256']=release.digest(self.root/'backup')
        app=self.package/'application/MATDOG_Controller.ino.bin';app.write_bytes(b'SYNTHETIC_TEST_ONLY 125d981b02a3')
        self.config['application_sha256']=release.digest(app)
        (canonical/'MATDOG_Controller.ino.partitions.bin').write_bytes(b'SYNTHETIC_TABLE')
        self.config['partition_sha256']=release.digest(canonical/'MATDOG_Controller.ino.partitions.bin')
        (self.package/'application/sdkconfig').write_text('synthetic SDK')
        fields=dict(SOURCE_COMMIT=self.config['firmware_commit'],BUILD_ID=self.config['build_id'],SOURCE_STATE='CLEAN',
            HARDWARE_PROFILE='ROBOT_POWERED',OTA_INGEST_ENABLED='0',LAYOUT_ID='MATDOG_16M_2x5M_NVS_V1',
            APPLICATION_SHA256=self.config['application_sha256'],PARTITION_TABLE_SHA256=self.config['partition_sha256'],
            APPLICATION_SIZE=str(app.stat().st_size),APP_PARTITION_SIZE='5242880')
        manifest=self.package/'application/matdog_build_manifest.txt'
        manifest.write_text(''.join(k+'='+v+'\n' for k,v in fields.items()))
        for name in ['MATDOG_Controller.ino.bin','matdog_build_manifest.txt','sdkconfig']:
            shutil.copy2(self.package/'application'/name,canonical/name)
        old=self.package/'rollback/MATDOG_Controller.ino.bin';old.write_bytes(b'SYNTHETIC_OLD')
        self.config['rollback_sha256']=release.digest(old)
        (self.package/'rollback/matdog_build_manifest.txt').write_text('APPLICATION_SHA256='+self.config['rollback_sha256']+'\nHARDWARE_PROFILE=ROBOT_POWERED\n')
        tool=self.package/'automation/tool.py';tool.write_text('# synthetic tool\n')
        self.config['tool_hashes']={'automation/tool.py':release.digest(tool)}
        release.write_new(self.package/'session-config.json',self.config)
        release.write_new(self.package/'initial-pose-plan.json',dict(qualified_path=False))
        self.ops=OfflineOps(self.config)
    def tearDown(self): self.tmp.cleanup()
    def test_pinned_files_pass_without_device_access(self):
        release.preflight(self.package,self.config,self.ops)
        self.assertEqual(release.main(['flash','--package',str(self.package),'--offline-check'],self.ops),0)
    def test_changed_or_dirty_git_provenance_blocks(self):
        for fault in ['moved_host','moved_firmware','dirty_host','dirty_firmware']:
            with self.subTest(fault=fault):
                self.ops.bad=fault
                with self.assertRaises(release.ReleaseFailure):release.preflight(self.package,self.config,self.ops)
    def test_changed_artifacts_refused(self):
        targets=['application/MATDOG_Controller.ino.bin','application/matdog_build_manifest.txt',
                 'automation/tool.py','rollback/MATDOG_Controller.ino.bin']
        for name in targets:
            path=self.package/name;original=path.read_bytes();path.write_bytes(original+b'corrupt')
            with self.subTest(path=name),self.assertRaises(release.ReleaseFailure):release.preflight(self.package,self.config,self.ops)
            path.write_bytes(original)
        self.config['backup_sha256']='0'*64
        with self.assertRaisesRegex(release.ReleaseFailure,'backup'):release.preflight(self.package,self.config,self.ops)


class GitTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.repo=self.root/'repo';self.repo.mkdir();self.directory=self.root/'session';self.directory.mkdir()
        self.git('init','-b','main');self.git('config','user.email','offline@test');self.git('config','user.name','Offline test')
        source=self.repo/'05_Firmware/MATDOG_Controller/src/sample.cpp';source.parent.mkdir(parents=True);source.write_text('baseline firmware\n')
        (self.repo/'doc.md').write_text('base\n');self.git('add','.');self.git('commit','-m','base')
        self.base=self.git('rev-parse','HEAD')
        tree=self.git('rev-parse','HEAD^{tree}')
        gait=self.git('commit-tree',tree,'-p',self.base,'-m','separate gait')
        self.git('branch','feat/g5a-stabilization-feasibility',gait)
        remote=self.root/'origin.git';subprocess.run(['git','init','--bare',str(remote)],check=True,capture_output=True)
        self.git('remote','add','origin',str(remote));self.git('push','origin','main')
        self.git('checkout','-b','fix/calibration-post-abort-thermal')
        (self.repo/'doc.md').write_text('feature documentation\n');self.git('add','.');self.git('commit','-m','feature')
        self.config=dict(tools_repo=str(self.repo),firmware_commit=self.base)
        self.ops=LocalGitOps();self.ops.checked=False
    def tearDown(self):self.tmp.cleanup()
    def git(self,*args):
        p=subprocess.run(['git',*args],cwd=self.repo,text=True,capture_output=True,check=True);return p.stdout.strip()
    def test_controlled_merge_local_remote_no_history_rewrite(self):
        new=release.finalize_git(self.config,self.directory,self.ops)
        self.assertEqual(self.git('rev-parse','main'),new);self.assertTrue(self.ops.checked)
        self.assertEqual(len(self.git('rev-list','--parents','-n','1',new).split()),3)
    def test_dirty_git_blocks_before_integration(self):
        (self.repo/'dirty.txt').write_text('preserve')
        with self.assertRaisesRegex(release.ReleaseFailure,'pre-existing'):release.finalize_git(self.config,self.directory,self.ops)
        self.assertEqual(self.git('rev-parse','main'),self.base)
    def test_remote_unavailable_preserves_main(self):
        self.git('remote','set-url','origin',str(self.root/'missing'))
        with self.assertRaises(release.ReleaseFailure):release.finalize_git(self.config,self.directory,self.ops)
        self.assertEqual(self.git('rev-parse','main'),self.base)
    def test_push_rejected_retains_local_merge_without_force(self):
        remote=self.root/'origin.git'
        hook=remote/'hooks/pre-receive'
        hook.write_text('#!/bin/sh\nexit 1\n');hook.chmod(0o755)
        with self.assertRaises(release.ReleaseFailure):release.finalize_git(self.config,self.directory,self.ops)
        new=self.git('rev-parse','main')
        self.assertNotEqual(new,self.base)
        self.assertEqual(len(self.git('rev-list','--parents','-n','1',new).split()),3)
        remote_main=subprocess.run(['git','--git-dir',str(remote),'rev-parse','main'],capture_output=True,text=True,check=True).stdout.strip()
        self.assertEqual(remote_main,self.base)
    def test_different_firmware_blocks_merge(self):
        (self.repo/'05_Firmware/MATDOG_Controller/src/sample.cpp').write_text('different unvalidated firmware\n')
        self.git('add','.');self.git('commit','-m','unvalidated')
        with self.assertRaisesRegex(release.ReleaseFailure,'differs'):release.finalize_git(self.config,self.directory,self.ops)
        self.assertEqual(self.git('rev-parse','main'),self.base);self.assertFalse(self.ops.checked)
    def test_conflicts_preserve_main_and_isolated_result(self):
        self.git('checkout','main');(self.repo/'doc.md').write_text('conflicting main\n');self.git('add','.');self.git('commit','-m','main change')
        before=self.git('rev-parse','main');self.git('push','origin','main');self.git('checkout','fix/calibration-post-abort-thermal')
        with self.assertRaises(release.ReleaseFailure):release.finalize_git(self.config,self.directory,self.ops)
        self.assertEqual(self.git('rev-parse','main'),before)
        self.assertTrue((self.directory/'integration-worktree').exists())
    def test_gait_ancestry_blocks_integration(self):
        self.git('branch','-f','feat/g5a-stabilization-feasibility','HEAD')
        with self.assertRaisesRegex(release.ReleaseFailure,'gait'):release.finalize_git(self.config,self.directory,self.ops)
        self.assertEqual(self.git('rev-parse','main'),self.base)

if __name__=='__main__':unittest.main()
