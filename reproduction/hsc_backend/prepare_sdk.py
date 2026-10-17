"""Acquire Microsoft's Windows SDK NuGet packages locally, without an installer."""
from prepare_build import BUILD, ROOT
from pathlib import Path
import concurrent.futures, hashlib, json, shutil, urllib.request, zipfile

VERSION='10.0.26100.3916'

def acquire(name):
    dest=BUILD/'downloads'/f'{name}.{VERSION}.nupkg'
    url=f'https://api.nuget.org/v3-flatcontainer/{name}/{VERSION}/{name}.{VERSION}.nupkg'
    if not dest.exists():
        with urllib.request.urlopen(urllib.request.Request(url,method='HEAD'),timeout=60) as r:
            size=int(r.headers['Content-Length'])
        print(f'{name}: {size} bytes',flush=True)
        chunk=4*1024*1024
        def part(start):
            end=min(size,start+chunk)-1
            path=BUILD/'downloads'/f'{name}-{start}.chunk'
            if path.exists() and path.stat().st_size==end-start+1:return path
            for attempt in range(4):
                try:
                    req=urllib.request.Request(url,headers={'Range':f'bytes={start}-{end}'})
                    with urllib.request.urlopen(req,timeout=90) as r:
                        if r.status!=206:raise RuntimeError('Range unsupported')
                        data=r.read()
                    if len(data)!=end-start+1:raise RuntimeError('Partial range')
                    path.write_bytes(data)
                    print(f'{name}: chunk {start}',flush=True)
                    return path
                except Exception:
                    if attempt==3:raise
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
            parts=list(pool.map(part,range(0,size,chunk)))
        with dest.open('wb') as out:
            for path in parts:
                with path.open('rb') as src:shutil.copyfileobj(src,out)
    sdk=BUILD/'sdk_packages'/name
    sdk.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(dest) as archive:
        for item in archive.infolist():
            target=(sdk/item.filename).resolve()
            if not target.is_relative_to(sdk.resolve()):raise RuntimeError('Unsafe SDK path')
            if item.is_dir():target.mkdir(parents=True,exist_ok=True)
            else:
                target.parent.mkdir(parents=True,exist_ok=True)
                with archive.open(item) as src,target.open('wb') as out:shutil.copyfileobj(src,out)
    result={'name':name,'version':VERSION,'url':url,'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'path':str(sdk)}
    print(json.dumps(result),flush=True)
    return result

if __name__=='__main__':
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        records=list(pool.map(acquire,['microsoft.windows.sdk.cpp','microsoft.windows.sdk.cpp.x64']))
    (ROOT/'windows_sdk.json').write_text(json.dumps(records,indent=2))
