"""Package sources, states, licenses and derived assets; omit binary runtimes."""
from common import ROOT,RESEARCH
from pathlib import Path
import hashlib,importlib.metadata,json,platform,shutil,subprocess,sys,zipfile


def main():
    repro=ROOT.parent;workspace=repro.parent
    licenses=ROOT/'licenses';licenses.mkdir(exist_ok=True)
    paths={
        'projectchrono-BSD-3-Clause.txt':RESEARCH/'upstream/projectchrono__chrono/LICENSE',
        'libuipc-root-Apache-2.0.txt':RESEARCH/'upstream/spiriMirror__libuipc/LICENSE',
        'libuipc-python-GPL-3.0.txt':RESEARCH/'upstream/spiriMirror__libuipc/python/LICENSE',
        'libuipc-samples-Apache-2.0.txt':RESEARCH/'upstream/spiriMirror__libuipc-samples/LICENSE',
        'coacd-MIT.txt':RESEARCH/'_geometry/coacd-1.0.14.dist-info/licenses/LICENSE',
        'manifold3d-Apache-2.0.txt':RESEARCH/'_geometry/manifold3d-3.5.4.dist-info/licenses/LICENSE',
        'trimesh-MIT.txt':RESEARCH/'_geometry/trimesh-5.1.1.dist-info/licenses/LICENSE.md',
        'scipy-BSD-and-bundled.txt':RESEARCH/'_geometry/scipy-1.18.1.dist-info/LICENSE.txt',
    }
    notices=[]
    for name,path in paths.items():
        shutil.copyfile(path,licenses/name)
        notices.append({'file':name,'copied_from':path.relative_to(workspace).as_posix(),
                        'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    (licenses/'manifest.json').write_text(json.dumps(notices,indent=2))
    versions={}
    for directory in [repro/'_deps',RESEARCH/'_geometry',RESEARCH/'_runtime']:
        for path in directory.glob('*.dist-info'):
            distribution=importlib.metadata.Distribution.at(path)
            versions[distribution.metadata['Name']]=distribution.version
    try:
        gpu=subprocess.run(['nvidia-smi','--query-gpu=name,driver_version,memory.total','--format=csv,noheader'],
                           capture_output=True,text=True,timeout=10,check=True).stdout.strip()
    except Exception as e:gpu=str(e)
    env={'python':sys.version,'platform':platform.platform(),'gpu':gpu,'packages':versions,
         'runtime_note':'PyPI pyuipc 0.0.28; reference source commit is recorded separately, not asserted to be its build commit.'}
    (ROOT/'environment.json').write_text(json.dumps(env,indent=2))
    selected=[]
    for base in [ROOT,RESEARCH]:
        for path in base.rglob('*'):
            relative=path.relative_to(base)
            if any(part in {'__pycache__','_runtime','_geometry'} for part in relative.parts):continue
            if not path.is_file() or path.suffix in {'.zip','.pyc'}:continue
            if path.stem.endswith('_partial') and (path.parent/'result.json').exists():continue
            if path.name=='runtime_location.json':continue
            selected.append(path)
    selected.extend(repro/name for name in ['bootstrap.py','setup.ps1','requirements-lock.txt'])
    files=[{'path':p.relative_to(workspace).as_posix(),'bytes':p.stat().st_size,
            'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(selected)]
    target=repro/'vehicle_assembly_bundle.zip'
    with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
        for path in selected:archive.write(path,path.relative_to(workspace).as_posix())
        archive.writestr('BUNDLE_README.txt',
            'Open reproduction/vehicle_assembly/README.md.\n'
            'Sources, derived proxies, real FEM/ABD snapshots and licenses are included.\n'
            'Binary Python/CUDA dependencies are omitted. Run vehicle_assembly/setup.ps1 with Python 3.12 on Windows.\n'
            'This is a replacement assembly, not the original paper scene or the HSC solver.\n')
        archive.writestr('BUNDLE_MANIFEST.json',json.dumps(files,indent=2))
    with zipfile.ZipFile(target) as archive:
        assert archive.testzip() is None
        for entry in files:assert hashlib.sha256(archive.read(entry['path'])).hexdigest()==entry['sha256']
    print(json.dumps({'bundle':str(target),'bytes':target.stat().st_size,'files':len(files),
                     'zip_integrity':'PASS','all_entry_hashes':'PASS'},ensure_ascii=True))


if __name__=='__main__':main()
