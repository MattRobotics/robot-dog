#!/usr/bin/env python3
"""V3 gate regression: temporary Git repository/files only, no hardware."""
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import release_candidate_gate as g
import build_manifest as bm

class Gate(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name); self.root = self.base / 'repo'; self.root.mkdir()
        self.art = self.base / 'art'; self.art.mkdir()
        self.git('init', '-q'); self.git('config', 'user.email', 'offline@example.invalid')
        self.git('config', 'user.name', 'offline test')
        self.write(g.FW + '/src/test.cpp', 'int source;\n')
        self.write(g.FW + '/scripts/test.py', '# qualified test\n')
        self.write(g.CSP_FIXTURE, "platform=r'''offline fixture'''\n"); self.commit()
        self.source = self.git('rev-parse', 'HEAD').strip()
        table = bm.matdog_layout.build_table_bytes() if hasattr(bm.matdog_layout, 'build_table_bytes') else None
        if table is None:
            # The canonical encoder is Arduino's gen_esp32part, without a device.
            script = Path.home()/'.arduino15/packages/esp32/hardware/esp32/3.3.11/tools/gen_esp32part.py'
            subprocess.run([sys.executable, str(script), str(Path(__file__).resolve().parents[2]/'partitions.csv'), str(self.art/'table.bin')], check=True, capture_output=True)
            table = (self.art/'table.bin').read_bytes()
        elf = b'offline synthetic ELF'; image = bytearray(1024)
        image[32:72] = self.source.encode(); image[0xb0:0xd0] = hashlib.sha256(elf).digest()
        marker = g.layout.LAYOUT_MARKER.encode(); image[256:256+len(marker)] = marker
        (self.art/'MATDOG_Controller.ino.bin').write_bytes(image)
        (self.art/'MATDOG_Controller.ino.elf').write_bytes(elf)
        (self.art/'MATDOG_Controller.ino.partitions.bin').write_bytes(table)
        (self.art/'sdkconfig').write_text('offline test\n')
        m = bm.render_manifest(source_commit=self.source, build_id=self.source[:12], source_state='CLEAN',
            profile='ROBOT_POWERED', ota_ingest_enabled='0', fqbn=g.FQBN,
            application_binary='MATDOG_Controller.ino.bin', application_size=len(image), application_sha256=g.sha(image),
            layout_id=g.layout.LAYOUT_ID, partition_table_sha256=g.sha(table), app_partition_size=g.layout.APP_SLOT_SIZE)
        m += 'BOOT_MOTION_AUTHORIZED=0\nMOTION_AUTHORITY=RUNTIME_GOVERNED\nSUPERVISED_VALIDATION_CAPABLE=0\n'
        (self.art/'matdog_build_manifest.txt').write_text(m)
        self.d = dict(schema=3, release_ready='PASS', source_commit=self.source, qualification_commit=self.source,
            artifact_directory=str(self.art), application_sha256=g.sha(image),
            artifacts={n:g.sha((self.art/n).read_bytes()) for n in g.ARTIFACTS},
            compiled_files=g.inventory(self.root,self.source,g.compiled), qualified_tools=g.inventory(self.root,self.source,g.tooling), evidence={})
        for role in g.ROLES:
            p = self.base/(role+'.log'); p.write_text('QUALIFIED_SOURCE='+self.source+'\nSTATIC_AUDIT = PASS\n')
            self.d['evidence'][role] = dict(path=str(p),sha256=g.sha(p.read_bytes()),source_commit=self.source,
                qualification_commit=self.source,result='PASS',exit_code=0)
        self.path = self.root/'09_Logs/designation.json'; self.save()
    def git(self,*args):
        return subprocess.check_output(['git','-C',str(self.root),*args],stderr=subprocess.DEVNULL,text=True)
    def write(self,n,s):
        p=self.root/n; p.parent.mkdir(parents=True,exist_ok=True); p.write_text(s)
    def commit(self):
        self.git('add','.'); self.git('commit','-qm','offline test')
    def save(self):
        self.write('09_Logs/designation.json',json.dumps(self.d,sort_keys=True)); self.commit()
    def refuses(self):
        with self.assertRaises((ValueError,subprocess.CalledProcessError)):g.verify(self.path,self.root)
    def test_positive_and_later_documentation_commit(self):
        self.assertEqual(g.verify(self.path,self.root),(self.source,self.art))
        self.write('09_Logs/notes.md','later docs'); self.commit(); g.verify(self.path,self.root)
    def test_dirty(self):
        self.write(g.FW+'/src/test.cpp','dirty'); self.refuses()
    def test_committed_source_changed(self):
        self.write(g.FW+'/src/test.cpp','changed'); self.commit(); self.refuses()
    def test_ignored_source_injected(self):
        self.write('.gitignore','injected.cpp\n'); self.commit()
        self.write(g.FW+'/src/injected.cpp','injected'); self.refuses()
    def test_tool_changed(self):
        self.write(g.FW+'/scripts/test.py','changed'); self.commit(); self.refuses()
    def test_binary_changed(self):
        (self.art/'MATDOG_Controller.ino.bin').write_bytes(b'tampered'); self.refuses()
    def test_elf_changed(self):
        (self.art/'MATDOG_Controller.ino.elf').write_bytes(b'tampered'); self.refuses()
    def test_manifest_changed(self):
        with (self.art/'matdog_build_manifest.txt').open('a') as f:f.write('tampered\n')
        self.refuses()
    def test_evidence_changed(self):
        Path(self.d['evidence']['workflow']['path']).write_text('forged PASS'); self.refuses()
    def test_missing_evidence_role(self):
        del self.d['evidence']['workflow']; self.save(); self.refuses()
    def test_old_campaign_provenance(self):
        self.d['evidence']['targeted']['source_commit']='2c2c13b'; self.save(); self.refuses()
    def test_failed_audit_even_with_updated_hash(self):
        e=self.d['evidence']['full_static_audit']; p=Path(e['path'])
        p.write_text('QUALIFIED_SOURCE='+self.source+'\nSTATIC_AUDIT = FAIL\n'); e['sha256']=g.sha(p.read_bytes()); self.save(); self.refuses()
    def test_wrong_profile_even_with_updated_manifest_hash(self):
        p=self.art/'matdog_build_manifest.txt'; p.write_text(p.read_text().replace('HARDWARE_PROFILE=ROBOT_POWERED','HARDWARE_PROFILE=USB_ONLY'))
        self.d['artifacts'][p.name]=g.sha(p.read_bytes()); self.save(); self.refuses()
    def test_supervised_flag_is_not_falsified(self):
        p=self.art/'matdog_build_manifest.txt'; p.write_text(p.read_text().replace('SUPERVISED_VALIDATION_CAPABLE=0','SUPERVISED_VALIDATION_CAPABLE=1'))
        self.d['artifacts'][p.name]=g.sha(p.read_bytes()); self.save(); self.refuses()
    def composite(self):
        p=self.root/g.CSP_FIXTURE;before=p.read_text();p.write_text(before.replace("platform=r'''",g.CSP_FIXTURE_INCLUDE+"platform=r'''",1));self.commit()
        qualified=self.git('rev-parse','HEAD').strip();self.d['qualification_commit']=qualified
        self.d['qualified_tools']=g.inventory(self.root,qualified,g.tooling)
        for e in self.d['evidence'].values():e['qualification_commit']=qualified
        e=self.d['evidence']['full_static_audit'];p=Path(e['path'])
        p.write_text('QUALIFIED_SOURCE='+self.source+"\nSTATIC_AUDIT = FAIL (1 finding(s))\n - run_host_tests.sh: test_ap_only_transport.py error: ‘kDashboardContentSecurityPolicy’ was not declared in this scope\n")
        e.update(sha256=g.sha(p.read_bytes()),result='FAIL',exit_code=1,mode='COMPOSED_CSP_FIXTURE')
        p=self.base/'resolution.log';p.write_text('QUALIFIED_SOURCE='+self.source+'\n'+'\n'.join('COMMAND='+n for n in g.REMAINDER)+'\nHOST_REMAINDER = PASS\n')
        e['resolution']=dict(path=str(p),sha256=g.sha(p.read_bytes()),source_commit=self.source,qualification_commit=qualified,result='PASS',exit_code=0)
        self.save()
    def test_exact_composed_resolution(self):
        self.composite();g.verify(self.path,self.root)
    def test_composite_cannot_hide_other_audit_failure(self):
        self.composite();e=self.d['evidence']['full_static_audit'];p=Path(e['path']);p.write_text(p.read_text().replace('1 finding(s)','2 finding(s)'));e['sha256']=g.sha(p.read_bytes());self.save();self.refuses()
    def test_composite_requires_all_remaining_tests(self):
        self.composite();e=self.d['evidence']['full_static_audit']['resolution'];p=Path(e['path']);p.write_text(p.read_text().replace('COMMAND=test_dev_identity_build.py','SKIPPED'));e['sha256']=g.sha(p.read_bytes());self.save();self.refuses()
    def test_composite_resolution_log_tamper(self):
        self.composite();Path(self.d['evidence']['full_static_audit']['resolution']['path']).write_text('forged');self.refuses()
    def test_composite_fixture_must_be_only_the_include_fix(self):
        self.composite();p=self.root/g.CSP_FIXTURE;p.write_text(p.read_text()+'# unreviewed weakening\n');self.commit()
        q=self.git('rev-parse','HEAD').strip();self.d['qualification_commit']=q;self.d['qualified_tools']=g.inventory(self.root,q,g.tooling)
        for e in self.d['evidence'].values():e['qualification_commit']=q
        self.d['evidence']['full_static_audit']['resolution']['qualification_commit']=q
        self.save();self.refuses()
    def test_missing_artifact(self):
        (self.art/'sdkconfig').unlink(); self.refuses()

if __name__=='__main__':unittest.main()
