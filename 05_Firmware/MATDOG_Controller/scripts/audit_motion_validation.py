#!/usr/bin/env python3
"""Read-only 16 Oct log audit. Never opens serial/network or replays commands."""
import argparse,csv,hashlib,json
from pathlib import Path

def analyze(path):
    samples=[];events=[];changes=[];last=None
    for n,line in enumerate(path.open(errors='replace'),1):
        if '@TEL {' in line:
            try: data=json.loads(line.split('@TEL ',1)[1])
            except json.JSONDecodeError: continue
            entry={'line':n,'utc':line.split()[0],'data':data};samples.append(entry)
            sequence=data.get('last_sequence',0)
            if sequence!=last:
                if sequence in (4,5): changes.append({'sequence':sequence,'before':samples[-2] if len(samples)>1 else None,'after':entry})
                last=sequence
        elif any(k in line for k in ('heap_free','MOTION_AUTHORITY','RESTORE_DECISION',' TX ','OPERATIONAL_SESSION','MOTION_ADMISSION')):
            events.append({'line':n,'text':line.rstrip()})
    return {'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'samples':len(samples),
            'command_brackets':changes,'events':events,'max_reported_raw_age_ms':max((s['data'].get('channels',{}).get('raw',{}).get('max_age_ms') or 0 for s in samples),default=0),
            'executor_states':sorted({s['data'].get('executor_state','') for s in samples}),
            'note':'USB observations bracket command processing; admission instant/joint stamps cannot be recovered exactly without MOTION_ADMISSION trace.'}

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('log',type=Path);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    result=analyze(args.log);args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'audit_evidence.json').write_text(json.dumps(result,indent=2)+'\n')
    with (args.output/'commands_4_5_feedback.csv').open('w') as f:
        writer=csv.writer(f);writer.writerow(['command','side','log_line','utc','now_ms','slot','servo_id','joint_at_ms','age_ms','mapped','torque'])
        for c in result['command_brackets']:
            for side in ('before','after'):
                s=c[side]
                if not s: continue
                d=s['data']
                for j,v in enumerate(d.get('servos',[])):
                    writer.writerow([c['sequence'],side,s['line'],s['utc'],d['now_ms'],j,v['id'],d['telemetry']['joint_at_ms'][j],v['age_ms'],v['mapped'],v['energized']])
    print(json.dumps({k:v for k,v in result.items() if k not in ('events','command_brackets')},indent=2))
if __name__=='__main__':main()
