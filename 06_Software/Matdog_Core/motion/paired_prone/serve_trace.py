#!/usr/bin/env python3
"""Host-only localhost viewer, preserving source sample timestamps."""
import argparse,os
from pathlib import Path
from http.server import SimpleHTTPRequestHandler,ThreadingHTTPServer
ROOT=Path(__file__).resolve().parents[4]
def main():
    p=argparse.ArgumentParser();p.add_argument('--trace',required=True,type=Path);p.add_argument('--port',type=int,default=5193);a=p.parse_args();trace=a.trace.resolve()
    if not trace.is_file() or trace.stat().st_size>10*1024*1024:raise SystemExit('Expected trace <=10MiB')
    dist=ROOT/'06_Software/Matdog_Core/viewer/matdog_bno085_fullbody_viewer/dist'
    if not (dist/'index.html').is_file():raise SystemExit('Build the host viewer first: npm run build')
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self,*args,**kw):super().__init__(*args,directory=str(dist),**kw)
        def do_GET(self):
            if self.path.split('?')[0]=='/motion/adaptive_motion_synthetic.jsonl':
                data=trace.read_bytes();self.send_response(200);self.send_header('Content-Type','application/jsonl');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
            else:super().do_GET()
        def log_message(self,*args):pass
    print(f'OFFLINE trace only, no device connection: http://127.0.0.1:{a.port}/?motion=trace',flush=True)
    ThreadingHTTPServer(('127.0.0.1',a.port),Handler).serve_forever()
if __name__=='__main__':main()
