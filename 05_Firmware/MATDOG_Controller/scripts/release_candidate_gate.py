#!/usr/bin/env python3
"""Offline V3 release provenance. No serial, subprocess flasher or hardware API.

The designation is itself committed in the clean checkout. Its hashes bind
external artifacts/evidence; Git independently supplies both inventories.
Compiled source and qualification/tooling commits are separate identities.
"""
import hashlib
import json
import subprocess
from pathlib import Path

import build_manifest as bm
import matdog_layout as layout

FW = '05_Firmware/MATDOG_Controller'
FQBN = 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=custom,DebugLevel=none,PSRAM=opi'
ARTIFACTS = {'MATDOG_Controller.ino.bin', 'MATDOG_Controller.ino.elf',
             'MATDOG_Controller.ino.partitions.bin', 'matdog_build_manifest.txt', 'sdkconfig'}
ROLES = {'full_static_audit', 'targeted', 'build_usb', 'build_powered', 'workflow'}

def sha(data):
    return hashlib.sha256(data).hexdigest()

def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args])

def names(root, ref):
    return git(root, 'ls-tree', '-r', '--name-only', ref).decode().splitlines()

def compiled(name):
    return (name.startswith(FW + '/src/') or name in
            (FW + '/MATDOG_Controller.ino', FW + '/partitions.csv') or
            name.startswith('06_Software/Matdog_Core/motion/paired_prone/'))

def tooling(name):
    return (name.startswith(FW + '/scripts/') or
            name.startswith('06_Software/Matdog_Core/host_bridge/') or
            name.startswith('06_Software/Matdog_Core/viewer/'))

def inventory(root, ref, predicate):
    return {n: sha(git(root, 'show', ref + ':' + n))
            for n in names(root, ref) if predicate(n)}

def require(ok, why):
    if not ok:
        raise ValueError(why)

def verify(path, root):
    root = Path(root).resolve(); path = Path(path).resolve()
    require(not git(root, 'status', '--porcelain').strip(), 'dirty worktree')
    require(path.is_relative_to(root), 'designation must be committed inside checkout')
    relative = path.relative_to(root).as_posix()
    require(path.read_bytes() == git(root, 'show', 'HEAD:' + relative), 'uncommitted designation')
    d = json.loads(path.read_text())
    require(d.get('schema') == 3 and d.get('release_ready') == 'PASS', 'release not qualified')
    source, qualified = d['source_commit'], d['qualification_commit']
    for older, newer in ((source, qualified), (qualified, 'HEAD')):
        subprocess.run(['git', '-C', str(root), 'merge-base', '--is-ancestor', older, newer],
                       check=True, stdout=subprocess.DEVNULL)
    for key, ref, predicate in (('compiled_files', source, compiled), ('qualified_tools', qualified, tooling)):
        expected = inventory(root, ref, predicate)
        require(bool(expected) and d[key] == expected, key + ' inventory differs from Git')
        require(set(expected) == {n for n in names(root, 'HEAD') if predicate(n)}, key + ' file set drift')
        for name, digest in expected.items():
            require(sha((root / name).read_bytes()) == digest, 'source/tool bytes changed: ' + name)
    # Reject ignored/untracked C++ injected into the Arduino recursive src tree.
    actual = {p.relative_to(root).as_posix() for p in (root / FW / 'src').rglob('*') if p.is_file()}
    require(actual == {n for n in d['compiled_files'] if n.startswith(FW + '/src/')}, 'unrecorded src input')
    directory = Path(d['artifact_directory']).resolve()
    require(set(d['artifacts']) == ARTIFACTS, 'artifact inventory incomplete')
    for name, digest in d['artifacts'].items():
        p = directory / name
        require(p.is_file() and not p.is_symlink() and sha(p.read_bytes()) == digest, 'artifact changed: ' + name)
    app = directory / 'MATDOG_Controller.ino.bin'; data = app.read_bytes()
    manifest = bm.parse_manifest((directory / 'matdog_build_manifest.txt').read_text())
    table_hash, _ = layout.check_table_bytes((directory / 'MATDOG_Controller.ino.partitions.bin').read_bytes())
    verdict = bm.verify_manifest(manifest, head_commit=source, expected_fqbn=FQBN, tree_state='CLEAN',
        binary_exists=True, binary_size=len(data), binary_sha256=sha(data), requested_profile='ROBOT_POWERED',
        requested_ota_ingest='0', build_partition_table_sha256=table_hash,
        binary_embeds_layout_id=layout.binary_embeds_layout_id(data))
    require(verdict.ok, 'manifest refused: ' + str(verdict))
    require(d['application_sha256'] == sha(data), 'designated app mismatch')
    require(source.encode() in data and source[:12].encode() in data, 'embedded source mismatch')
    require(data[0xb0:0xd0].hex() == d['artifacts']['MATDOG_Controller.ino.elf'], 'ELF/app descriptor mismatch')
    require(manifest.get('BOOT_MOTION_AUTHORIZED') == '0' and
            manifest.get('MOTION_AUTHORITY') == 'RUNTIME_GOVERNED' and
            manifest.get('SUPERVISED_VALIDATION_CAPABLE') == '0', 'ordinary runtime candidate policy mismatch')
    require(set(d['evidence']) == ROLES, 'qualification roles missing')
    for role, item in d['evidence'].items():
        log = Path(item['path'])
        require(item['source_commit'] == source and item['qualification_commit'] == qualified and
                item['exit_code'] == 0 and item['result'] == 'PASS', 'qualification provenance/status mismatch')
        require(log.is_file() and not log.is_symlink() and sha(log.read_bytes()) == item['sha256'], 'evidence changed: ' + role)
        text = log.read_text()
        require(('QUALIFIED_SOURCE=' + source) in text, 'evidence source header absent: ' + role)
        if role == 'full_static_audit':
            require(text.rstrip().endswith('STATIC_AUDIT = PASS'), 'full final static audit not PASS')
    return source, directory
