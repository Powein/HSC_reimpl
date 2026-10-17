"""Read public upstream metadata/trees; download no executable content here."""
from pathlib import Path
import concurrent.futures
import json
import re
import sys
import urllib.request

sys.stdout.reconfigure(encoding='utf-8')
OUT=Path(__file__).resolve().parent/'sources'
OUT.mkdir(parents=True,exist_ok=True)
REPOS=['KemengHuang/Stiff-GIPC','spiriMirror/libuipc-samples',
       'spiriMirror/libuipc','projectchrono/chrono','ipc-sim/ipc-toolkit',
       'ipc-sim/IPC']

def get(url):
    request=urllib.request.Request(url,headers={'User-Agent':'HC-reproduction-research','Accept':'application/vnd.github+json'})
    with urllib.request.urlopen(request,timeout=35) as r:return json.load(r)

def inspect(repo):
    stem=repo.replace('/','__')
    meta=get('https://api.github.com/repos/'+repo)
    tree=get('https://api.github.com/repos/'+repo+'/git/trees/'+meta['default_branch']+'?recursive=1')
    (OUT/(stem+'.meta.json')).write_text(json.dumps(meta,indent=2),encoding='utf-8')
    (OUT/(stem+'.tree.json')).write_text(json.dumps(tree,indent=2),encoding='utf-8')
    pattern=r'hmmwv|hummer|humvee|vehicle|tire|tyre|suspension|car\b|/car[_/\.]|ccd|broad_phase|friction'
    hits=[{k:x[k] for k in ['path','type','size'] if k in x} for x in tree['tree'] if re.search(pattern,x['path'],re.I)]
    return {'repo':repo,'branch':meta['default_branch'],'sha':tree['sha'],
            'license':meta.get('license'),'truncated':tree['truncated'],
            'match_count':len(hits),'first_40_matches':hits[:40],
            'full_inventory':str(OUT/(stem+'.tree.json'))}

with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
    tasks={pool.submit(inspect,r):r for r in REPOS}
    for task in concurrent.futures.as_completed(tasks):
        try:print(json.dumps(task.result(),ensure_ascii=False),flush=True)
        except Exception as e:print(json.dumps({'repo':tasks[task],'error':str(e)}),flush=True)
