"""Fetch pinned upstream integration sources without changing prior assets."""
from pathlib import Path
import concurrent.futures,hashlib,json,urllib.request
ROOT=Path(__file__).resolve().parent
PRIOR=ROOT.parent/'vehicle_research'
COMMIT=json.loads((PRIOR/'download_manifest.json').read_text())['commits']['spiriMirror/libuipc']
tree=json.loads((PRIOR/'sources/spiriMirror__libuipc.tree.json').read_text())['tree']
prefixes=('src/backends/cuda/linear_system/','src/backends/common/',
          'include/uipc/backend/','src/core/backend/',
          'src/backends/cuda/coupling_system/','src/backends/cuda/affine_body/affine_body_linear_',
          'src/backends/cuda/finite_element/finite_element_linear_')
explicit={'CMakeLists.txt','xmake.lua','src/backends/cuda/CMakeLists.txt','src/backends/cuda/xmake.lua',
          'src/backends/CMakeLists.txt','src/backends/cuda/sim_system.h','src/backends/cuda/sim_system.cu',
          'src/backends/cuda/sim_engine.cu','src/backends/cuda/sim_engine.h',
          'include/uipc/backend.h','.gitmodules','cmake/setup_targets.cmake'}
files=[e['path'] for e in tree if e['type']=='blob' and (e['path'].startswith(prefixes) or e['path'] in explicit)]

def fetch(path):
    dest=ROOT/'upstream'/path
    url=f'https://raw.githubusercontent.com/spiriMirror/libuipc/{COMMIT}/{path}'
    if not dest.exists():
        request=urllib.request.Request(url,headers={'User-Agent':'HSC-integration'})
        with urllib.request.urlopen(request,timeout=45) as response:data=response.read()
        dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data)
    return {'path':path,'url':url,'sha256':hashlib.sha256(dest.read_bytes()).hexdigest()}

if __name__=='__main__':
    records=[];errors=[]
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
        pending={pool.submit(fetch,path):path for path in files}
        for task in concurrent.futures.as_completed(pending):
            try:records.append(task.result())
            except Exception as e:errors.append({'path':pending[task],'error':str(e)})
    (ROOT/'source_manifest.json').write_text(json.dumps({'commit':COMMIT,'files':records,'errors':errors},indent=2))
    print(json.dumps({'files':len(records),'errors':errors}))
