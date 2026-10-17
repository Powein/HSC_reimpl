"""Additional primary-source snapshots for proxy/joint/bead implementation."""
from pathlib import Path
import urllib.request
import hashlib
import json
import concurrent.futures

ROOT=Path(__file__).resolve().parent
PRIOR=ROOT.parent/'vehicle_research'
commits=json.loads((PRIOR/'download_manifest.json').read_text())['commits']
files={
 'spiriMirror/libuipc':[
  'src/constitution/soft_vertex_stitch.cpp',
  'src/backends/cuda/inter_primitive_effect_system/constitutions/soft_vertex_stitch.cu',
  'src/backends/cuda/inter_primitive_effect_system/constitutions/details/soft_vertex_stitch.inl',
  'docs/specification/constitutions/affine_body_prismatic_joint_limit.md',
  'src/constitution/affine_body_driving_prismatic_joint.cpp',
  'src/constitution/affine_body_prismatic_joint_external_force.cpp',
  'docs/specification/constitutions/affine_body_prismatic_joint_external_force.md',
  'docs/specification/constitutions/affine_body_prismatic_joint.md',
  'src/constitution/affine_body_prismatic_joint.cpp',
  'src/backends/cuda/affine_body/affine_body_dynamics.cu'],
 'spiriMirror/libuipc-samples':['examples/22_soft_vertex_stitch/main.py','examples/36_vertex_stitch_family/main.py'],
 'projectchrono/chrono':['src/chrono_models/vehicle/hmmwv/HMMWV_Chassis.cpp']}

def fetch(item):
 repo,path=item
 url=f'https://raw.githubusercontent.com/{repo}/{commits[repo]}/{path}'
 request=urllib.request.Request(url,headers={'User-Agent':'HC-assembly-research'})
 with urllib.request.urlopen(request,timeout=45) as response:data=response.read()
 dest=ROOT/'references'/repo.replace('/','__')/path
 dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data)
 return {'repo':repo,'commit':commits[repo],'url':url,'path':dest.relative_to(ROOT).as_posix(),'sha256':hashlib.sha256(data).hexdigest()}

if __name__=='__main__':
 records=[];errors=[]
 with concurrent.futures.ThreadPoolExecutor(max_workers=5) as pool:
  pending={pool.submit(fetch,(r,p)):(r,p) for r,paths in files.items() for p in paths}
  for task in concurrent.futures.as_completed(pending):
   try:records.append(task.result())
   except Exception as e:errors.append({'item':pending[task],'error':str(e)})
 (ROOT/'references_manifest.json').write_text(json.dumps({'files':records,'errors':errors},indent=2))
 print(json.dumps({'files':len(records),'errors':errors}))
