#!/usr/bin/env python3
"""Strengthen saved segment bounds without regenerating contacts or ground proof."""
import argparse,hashlib,json
from pathlib import Path
from continuous_pair_path import Continuous
from paired_acquisition import OUT

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--minimum-separation-m',type=float,default=.0001);a=p.parse_args()
    if a.output.exists():raise SystemExit('Output exists; keep frozen evidence or choose a new output')
    if not 0<a.minimum_separation_m<=.003:raise SystemExit('Expected positive proof margin <= reference policy 3mm')
    path=OUT/'continuous_pair_path.json';source=json.loads(path.read_text());c=Continuous();results=[]
    for i,s in enumerate(source['segments']):
        ok,r=c.self_proof(s['start_parameters'],s['end_parameters'],a.minimum_separation_m);results.append({'segment':i,'pass':ok,'certificate':r})
        if not ok:break
        if i%20==0:print('strengthened',i,flush=True)
    passed=len(results)==len(source['segments']) and all(r['pass'] for r in results)
    a.output.write_text(json.dumps({'all_251_segments_pass':passed,'source_certificate_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'proof_margin_m':a.minimum_separation_m,'trajectory_changed':False,'policy_3mm_qualified':passed and a.minimum_separation_m>=.003,'segments':results},indent=2)+'\n')
    return 0 if passed else 1
if __name__=='__main__':raise SystemExit(main())
