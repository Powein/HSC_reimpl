"""Fetch a curated set of public source/asset files at recorded Git commits."""
from pathlib import Path
import concurrent.futures
import datetime
import hashlib
import json
import sys
import urllib.request

sys.stdout.reconfigure(encoding='utf-8')
ROOT = Path(__file__).resolve().parent
CHRONO = ['LICENSE', 'data/vehicle/hmmwv/README.md']
CHRONO += ['data/vehicle/hmmwv/' + p for p in [
    'hmmwv_chassis_simple.obj', 'hmmwv_chassis_col.obj', 'hmmwv_chassis.mtl',
    'hmmwv_rim.obj', 'hmmwv_rim.mtl', 'hmmwv_tire_coarse_closed.obj',
    'hmmwv_tire_left.obj', 'hmmwv_tire_left.mtl',
    'chassis/HMMWV_Chassis.json', 'suspension/HMMWV_DoubleWishboneFront.json',
    'suspension/HMMWV_DoubleWishboneRear.json', 'vehicle/HMMWV_Vehicle.json',
    'wheel/HMMWV_Wheel.json', 'tire/HMMWV_FEATire.json',
    'tire/HMMWV_lugged_coarse.inp', 'tire/HMMWV_RigidMeshTire_CoarseClosed.json']]
CHRONO += ['src/demos/python/vehicle/demo_VEH_HMMWV.py']
CHRONO += ['src/chrono_vehicle/wheeled_vehicle/tire/ChFEATire.cpp',
           'src/chrono_vehicle/wheeled_vehicle/tire/FEATire.cpp']
FILES = {
    'projectchrono/chrono': CHRONO,
    'spiriMirror/libuipc': ['LICENSE','python/LICENSE','README.md','python/pyproject.toml',
        'src/backends/cuda/contact_system/contact_models/ipc_simplex_normal_contact.cu',
        'src/backends/cuda/contact_system/contact_models/ipc_simplex_frictional_contact.cu',
        'src/backends/cuda/collision_detection/filters/stackless_bvh_simplex_trajectory_filter.cu',
        'src/backends/cuda/utils/distance/details/ccd.inl',
        'src/backends/cuda/linear_system/linear_pcg.cu',
        'src/backends/cuda/linear_system/linear_pcg.h',
        'src/backends/cuda/linear_system/iterative_solver.h',
        'src/backends/cuda/linear_system/global_linear_system.h',
        'src/backends/cuda/coupling_system/abd_fem_linear_subsystem.cu',
        'src/backends/cuda/line_search/line_searcher.cu',
        'docs/specification/constitutions/soft_vertex_stitch.md',
        'docs/specification/constitutions/soft_vertex_triangle_stitch.md',
        'docs/specification/constitutions/affine_body_prismatic_joint.md',
        'docs/specification/constitutions/affine_body_revolute_joint.md',
        'docs/specification/constitutions/hookean_spring.md'],
    'spiriMirror/libuipc-samples': ['LICENSE','README.md',
        'examples/37_uipc_assets/main.py',
        'examples/3_periodically_pressed_tetrahedron/main.py',
        'examples/38_abd_prismatic_joint_driving_and_external_force/main.py',
        'examples/39_abd_revolute_joint_driving_and_external_torque/main.py',
        'examples/20_contact_system_feature/main.py'],
    'KemengHuang/Stiff-GIPC': ['LICENSE','README.md',
        'StiffGIPC/collision/ACCD.cu','StiffGIPC/collision/FrictionUtils.cuh'],
    'ipc-sim/ipc-toolkit': ['LICENSE','README.md','src/ipc/ccd/additive_ccd.cpp']
}

def get(url):
    req = urllib.request.Request(url,headers={'User-Agent':'HC-reproduction-asset-audit'})
    with urllib.request.urlopen(req, timeout=90) as r:
        return r.read()

def download(item):
    repo, commit, rel = item
    url=f'https://raw.githubusercontent.com/{repo}/{commit}/{rel}'
    dest=ROOT/'upstream'/repo.replace('/','__')/rel
    dest.parent.mkdir(parents=True, exist_ok=True)
    data=get(url)
    dest.write_bytes(data)
    return {'repository':repo,'commit':commit,'upstream_path':rel,'url':url,
            'local_path':dest.relative_to(ROOT).as_posix(),'bytes':len(data),
            'sha256':hashlib.sha256(data).hexdigest()}

def main():
    manifest=ROOT/'download_manifest.json'
    old=json.loads(manifest.read_text(encoding='utf-8')) if manifest.exists() else {}
    commits=old.get('commits',{})
    existing={(r['repository'],r['commit'],r['upstream_path']):r for r in old.get('files',[])}
    records=[]
    tasks=[]
    for repo,files in FILES.items():
        if repo not in commits:
            meta=json.loads((ROOT/'sources'/(repo.replace('/','__')+'.meta.json')).read_text())
            data=json.loads(get(f'https://api.github.com/repos/{repo}/commits/{meta["default_branch"]}'))
            commits[repo]=data['sha']
        for p in files:
            item=(repo,commits[repo],p)
            record=existing.get(item)
            dest=ROOT/record['local_path'] if record else None
            if dest and dest.is_file() and hashlib.sha256(dest.read_bytes()).hexdigest()==record['sha256']:
                records.append(record)
            else:
                tasks.append(item)
    errors=[]
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
        futures={pool.submit(download,t):t for t in tasks}
        for f in concurrent.futures.as_completed(futures):
            try:records.append(f.result())
            except Exception as e:errors.append({'item':futures[f],'error':str(e)})
    payload={'retrieved_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
             'commits':commits,'files':sorted(records,key=lambda x:(x['repository'],x['upstream_path'])),
             'errors':errors}
    manifest.write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'downloaded':len(records),'total_bytes':sum(r['bytes'] for r in records),
                      'commits':commits,'errors':errors},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
