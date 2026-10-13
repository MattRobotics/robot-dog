#!/usr/bin/env python3
"""Verify actual table bytes, metadata and immutable geometry pins offline."""
import hashlib,json,re
from pathlib import Path
import numpy as np
FW=Path(__file__).resolve().parents[2];ROOT=FW.parents[1];REPORT=ROOT/'09_Logs/Validation_Reports/Final_Release_Candidate_2026-10-13'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
header=FW/'src/motion/FirstStrideData.h';text=header.read_text();r=json.loads((REPORT/'compiled_first_stride_receipt.json').read_text());assert sha(header)==r['header_sha256']
rows=re.findall(r'^ \{\{([^}]+)\},\{([^}]+)\},\{([^}]+)\}\},',text,re.M);assert len(rows)==r['frames']==19081
q=np.array([[float(v.removesuffix('f')) for v in row[0].split(',')] for row in rows],dtype='<f4');v=np.array([[int(x) for x in row[1].split(',')] for row in rows],dtype='<i2');a=np.array([[int(x) for x in row[2].split(',')] for row in rows],dtype='<i2');digest=hashlib.sha256(q.tobytes()+v.tobytes()+a.tobytes()).hexdigest();assert digest==r['data_sha256'];assert digest in text
pins=(FW/'src/motion/CertifiedMotionEvidence.h').read_text();assert digest in pins
for name,status in [('fixture_final_margin_route_6mrad.json','PASS_FIXTURE_ROUTE_TUBE'),('stand_tracking_tube_6mrad.json','PASS_GEOMETRY_TUBE'),('first_stride_30mm_qualified_clock_swept_6mrad.json','PASS_CONTINUOUS_TARGET_BOXES')]:
 p=REPORT/name;d=json.loads(p.read_text());assert d['status']==status and sha(p) in pins
 if name.startswith('first_stride'):assert d['parameters']['sway_x_m']==r['parameters']['sway_x_m']==.03 and len(d['nonadjacent'])==19080 and len(d['adjacent'])==12
 assert d['hardware_qualified'] is False
for name,expected in [('kMinimum',q.min(axis=0)),('kMaximum',q.max(axis=0))]:
 values=np.array(list(map(float,re.search(name+r'\[12\]=\{([^}]+)',text)[1].split(','))));assert np.array_equal(values,expected.astype(float))
assert abs(float(np.max(np.abs(np.diff(q,axis=0))))-r['maximum_joint_target_step_rad'])<1e-12
assert r['runtime_error_tube_rad']==.006-r['position_error_rad']
canonical=REPORT.parent/'Motion_Night_Run_2026-10-12/continuous_pair_path.json';assert sha(canonical)=='f86ad336ffdaa1bf287f54d8a1a233494bf935156439610fd35ebc25bed411f2'
print('PINNED_MOTION_ARTIFACTS PASS: actual 19081-row payload, float error, bounds, 3 continuous proofs, canonical route immutable; hardware=false')
