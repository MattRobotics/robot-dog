#!/usr/bin/env python3
"""Verify the frozen offline entry bundle, receipts, sources and17 meshes.
Reads files only; never opens a device or grants movement authority.
"""
import argparse,hashlib,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--bundle',type=Path,required=True);a=p.parse_args()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read_certificate(path):
 d=json.loads(path.read_text());body=d['certificate']
 assert hashlib.sha256(json.dumps(body,sort_keys=True,separators=(',',':')).encode()).hexdigest()==d['certificate_sha256'],f'certificate digest mismatch:{path}'
 return d
bundle=read_certificate(a.bundle);b=bundle['certificate'];assert b['status']=='PASS_COMPUTATIONAL_ENTRY' and not b['hardware_authorized']
receipts={}
for k,r in b['references'].items():
 path=a.repo/r['path'];assert sha(path)==r['file_sha256'];d=read_certificate(path);assert d['certificate_sha256']==r['certificate_sha256'];assert d['certificate']['status']==r['status'];receipts[k]=d['certificate']
main=receipts['entry_full_tube_4mrad'];assert main['tube_rad']==b['tube_rad']==.004
for k in ['entry_full_tube_4mrad','independent_lower_box','directed_belly_hip']:
 assert receipts[k]['start_rad']==b['start_rad'];assert receipts[k]['head_rad']==b['head_rad']
for path,digest in b['evidence_provenance']['proof_tool_sha256'].items():assert sha(a.repo/path)==digest,f'proof source mismatch:{path}'
for name,pin in b['evidence_provenance']['collision_meshes'].items():assert sha(a.repo/pin['path'])==pin['sha256'],f'mesh mismatch:{name}'
assert len(b['evidence_provenance']['collision_meshes'])==17
paired=a.repo/'06_Software/Matdog_Core/motion/paired_prone'
for path,digest in main['inputs']['proof_sources_sha256'].items():assert sha(paired/path)==digest
calibration=a.repo/'06_Software/Matdog_Core/calibration'
for field,name in [('mesh_kernel_sha256','matdog_geometry_mesh_kernel.py'),('geometry_scene_sha256','matdog_geometry_scene_v5.py'),('mesh_separation_bound_sha256','matdog_startup_geometry_v5.py')]:assert sha(calibration/name)==main['inputs'][field]
assert sha(a.repo/'03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf')==main['inputs']['urdf_sha256']
assert sha(a.repo/'09_Logs/Validation_Reports/Motion_Night_Run_2026-10-12/continuous_pair_path.json')==main['inputs']['route_json_sha256']
assert sha(a.repo/'05_Firmware/MATDOG_Controller/src/motion/PairedProneData.h')==main['inputs']['firmware_route_data_sha256']
assert len(receipts['independent_lower_box']['rows'])==120 and all(r['pass_'] for r in receipts['independent_lower_box']['rows'])
assert len(main['results']['adjacent'])==12 and all(r['status']=='PASS_CONTINUOUS' and r['excluded_triangles']==0 for r in main['results']['adjacent'])
assert all(r['status']=='PASS_NO_NESTED_SURFACE' for r in main['results']['initial_adjacent_nesting'])
assert all(r['pass_'] for r in main['results']['base_link_ventral'])
assert receipts['blocked_full_tube_10mrad']['status']=='BLOCKED'
print('PASS_COMPUTATIONAL_ENTRY',bundle['certificate_sha256'])
print('17_meshes=VERIFIED receipts=VERIFIED sources=VERIFIED hardware_authorized=0')
