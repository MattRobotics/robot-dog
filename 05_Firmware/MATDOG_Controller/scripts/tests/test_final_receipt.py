#!/usr/bin/env python3
"""Offline regression for existing final-receipt provenance; never flashes."""
import json
import hashlib
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import verify_designated_candidate as gate

class FinalReceiptTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / 'FINAL_RECEIPT.json'
        self.preenable = 'manifest' in json.loads(RECEIPT.read_text())
        names = ('FINAL_RECEIPT.json','SHA256SUMS.txt','head.txt','source-hashes-before-build.json','source-hashes-after-build.check') if self.preenable else ('FINAL_RECEIPT.json','build-source-hashes.json','source-hashes-after-build.json')
        for name in names:
            shutil.copyfile(RECEIPT.parent / name, self.path.parent / name)

    def rehash(self):
        # Mutations are isolated fixtures, never the authentic evidence.
        if self.preenable:
            names = ('FINAL_RECEIPT.json','head.txt','source-hashes-before-build.json','source-hashes-after-build.check')
            (self.path.parent/'SHA256SUMS.txt').write_text(''.join(hashlib.sha256((self.path.parent/n).read_bytes()).hexdigest()+'  ./'+n+'\n' for n in names))

    def inventories(self):
        return ('source-hashes-before-build.json',) if self.preenable else ('build-source-hashes.json','source-hashes-after-build.json')

    def alter(self, file, fn):
        p = self.path.parent / file
        d = json.loads(p.read_text()); fn(d); p.write_text(json.dumps(d)); self.rehash()

    def test_real_reviewed_candidate(self):
        source, directory = gate.verify(self.path)
        self.assertEqual(source, 'f838f677a1826c4eaabc5889473ef016b1ebad98')
        self.assertEqual(directory / 'MATDOG_Controller.ino.bin', Path(json.loads(self.path.read_text())['app']))

    def test_wrong_image_hash(self):
        self.alter('FINAL_RECEIPT.json', lambda d: d.update(sha256='0'*64))
        with self.assertRaisesRegex(ValueError, 'image mismatch'): gate.verify(self.path)

    def test_changed_manifest(self):
        self.alter('FINAL_RECEIPT.json', lambda d: d.update({('manifest' if self.preenable else 'build_manifest'):d['manifest' if self.preenable else 'build_manifest']+'EXTRA=1\n'}))
        with self.assertRaisesRegex(ValueError, 'manifest differs'): gate.verify(self.path)

    def test_pre_post_mismatch(self):
        if self.preenable:
            (self.path.parent/'source-hashes-after-build.check').write_text('SOURCES_CHANGED_DURING_BUILD ["x"]\n'); self.rehash()
        else:
            self.alter('source-hashes-after-build.json', lambda d: d.pop(next(iter(d))))
        with self.assertRaisesRegex(ValueError, 'during build'): gate.verify(self.path)

    def test_source_hash_mismatch(self):
        name = '05_Firmware/MATDOG_Controller/src/simple/SimpleMotion.cpp'
        for file in self.inventories():
            self.alter(file, lambda d: d.update({name: '0'*64}))
        with self.assertRaisesRegex(ValueError, 'source changed'): gate.verify(self.path)

    def test_missing_compiled_input(self):
        name = '05_Firmware/MATDOG_Controller/src/simple/SimpleMotion.cpp'
        for file in self.inventories():
            self.alter(file, lambda d: d.pop(name))
        with self.assertRaisesRegex(ValueError, 'file set differs'): gate.verify(self.path)

    def test_incompatible_partition(self):
        import matdog_layout
        artifact = self.path.parent/'artifact'; artifact.mkdir()
        image = Path(json.loads(self.path.read_text())['app'])
        for name in (image.name,'matdog_build_manifest.txt','MATDOG_Controller.ino.partitions.bin'):
            shutil.copyfile(image.parent/name,artifact/name)
        self.alter('FINAL_RECEIPT.json', lambda d:d.update(app=str(artifact/image.name)))
        gate.verify(self.path)  # ensure the cloned fixture is otherwise valid
        table=artifact/'MATDOG_Controller.ino.partitions.bin'
        raw=bytearray(table.read_bytes());raw[0]^=1;table.write_bytes(raw)
        with self.assertRaises(matdog_layout.LayoutRefusal): gate.verify(self.path)

    def test_dirty_build_is_identified(self):
        d=json.loads(self.path.read_text())
        self.assertIn('SOURCE_STATE=DIRTY',d['manifest' if self.preenable else 'build_manifest'])
        gate.verify(self.path)

    def test_tampered_evidence(self):
        if not self.preenable: self.skipTest('sidecar assertion schema only')
        (self.path.parent/'source-hashes-after-build.check').write_text('SOURCES_CHANGED_DURING_BUILD ["x"]')
        with self.assertRaisesRegex(ValueError,'evidence hash mismatch'): gate.verify(self.path)

if __name__ == '__main__':
    RECEIPT = Path(sys.argv.pop(1))
    unittest.main()
