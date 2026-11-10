#!/usr/bin/env python3
"""Worst static stack path from a root through GCC -fcallgraph-info=su files (.ci)."""
import re,sys,glob,subprocess,functools
nodes={};edges={}
for f in glob.glob(sys.argv[1]+'/*.ci'):
    t=open(f).read()
    for m in re.finditer(r'node: \{ title: "([^"]+)" label: "([^"]*)"',t):
        title,label=m.groups();sz=re.search(r'(\d+) bytes \((static|dynamic|bounded)',label)
        nodes.setdefault(title,(int(sz.group(1)) if sz else None,label.split('\\n')[0],sz.group(2) if sz else 'external'))
        if sz and nodes[title][0] is None: nodes[title]=(int(sz.group(1)),label.split('\\n')[0],sz.group(2))
    for m in re.finditer(r'edge: \{ sourcename: "([^"]+)" targetname: "([^"]+)"',t):
        edges.setdefault(m.group(1),set()).add(m.group(2))
sys.setrecursionlimit(10000)
@functools.lru_cache(None)
def worst(n,depth=0):
    own=(nodes.get(n,(0,n,'?'))[0] or 0)
    best=(0,())
    for c in edges.get(n,()):
        if c==n: continue
        w=worst(c,depth+1)
        if w[0]>best[0]: best=w
    return (own+best[0],(n,)+best[1])
root=[k for k in nodes if sys.argv[2] in k]
for r in root:
    tot,path=worst(r)
    dem=subprocess.run(['c++filt'],input='\n'.join(path),capture_output=True,text=True).stdout.split('\n')
    print(f'ROOT {dem[0][:90]} worst static path {tot} B')
    for p,d in zip(path,dem): print(f'  {nodes.get(p,(0,"",""))[0] or 0:5} B  {nodes.get(p,(0,"",""))[2]:8} {d[:130]}')
