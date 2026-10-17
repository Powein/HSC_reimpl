"""Prepare an isolated, pinned Windows/CUDA build; never install system-wide."""
from pathlib import Path
import concurrent.futures, hashlib, json, os, shutil, subprocess, sys, tempfile, urllib.request, zipfile
for stream in (sys.stdout,sys.stderr):
    if hasattr(stream,'reconfigure'):stream.reconfigure(encoding='utf-8',errors='replace')

ROOT = Path(__file__).resolve().parent
COMMIT = '9c748a759bb9952a7aeedd59076a70591dc61d6c'
LOCATION = ROOT / 'build_location.json'
if LOCATION.exists():
    BUILD = Path(json.loads(LOCATION.read_text())['root'])
else:
    BUILD = Path(tempfile.mkdtemp(prefix='hc-hsc-build-'))
    LOCATION.write_text(json.dumps({'root': str(BUILD), 'commit': COMMIT}, indent=2))
DOWNLOADS = BUILD / 'downloads'
DOWNLOADS.mkdir(exist_ok=True)

def download(url, name, digest=None):
    dest = DOWNLOADS / name
    if not dest.exists():
        print('Downloading ' + url, flush=True)
        if name == 'cuda_nvcc.zip':
            # NVIDIA's official redistributable supports byte ranges. Independent
            # chunks avoid the low throughput of one long Windows connection.
            total=120912792; chunk=4*1024*1024
            def part(start):
                end=min(total,start+chunk)-1
                target=DOWNLOADS/f'nvcc-{start}.chunk'
                if target.exists() and target.stat().st_size==end-start+1:return target
                request=urllib.request.Request(url,headers={'Range':f'bytes={start}-{end}'})
                for attempt in range(4):
                    try:
                        with urllib.request.urlopen(request,timeout=120) as response:
                            if response.status!=206:raise RuntimeError('Server did not honor byte range')
                            data=response.read()
                        break
                    except Exception:
                        if attempt==3:raise
                if len(data)!=end-start+1:raise RuntimeError('Incomplete range')
                target.write_bytes(data)
                print(f'CUDA chunk {start}/{total}',flush=True)
                return target
            with concurrent.futures.ThreadPoolExecutor(max_workers=16) as pool:
                parts=list(pool.map(part,range(0,total,chunk)))
            with dest.with_suffix('.part').open('wb') as output:
                for item in parts:
                    with item.open('rb') as src:shutil.copyfileobj(src,output)
        else:
            request = urllib.request.Request(url, headers={'User-Agent': 'HC-reproduction'})
            with urllib.request.urlopen(request, timeout=120) as response, dest.with_suffix('.part').open('wb') as output:
                shutil.copyfileobj(response, output)
        dest.with_suffix('.part').replace(dest)
    sha = hashlib.sha256(dest.read_bytes()).hexdigest()
    if digest and digest != sha:
        raise RuntimeError(f'SHA256 mismatch: {dest}')
    print(f'Ready {name}: {dest.stat().st_size} bytes, sha256={sha}', flush=True)
    return dest

def extract_strip(archive, target):
    target.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive) as z:
        for item in z.infolist():
            parts = Path(item.filename).parts[1:]
            if not parts:
                continue
            dest = target.joinpath(*parts).resolve()
            if not dest.is_relative_to(target.resolve()):
                raise ValueError('Unsafe archive path')
            if item.is_dir():
                dest.mkdir(parents=True, exist_ok=True)
            else:
                dest.parent.mkdir(parents=True, exist_ok=True)
                with z.open(item) as src, dest.open('wb') as out:
                    shutil.copyfileobj(src, out)

def run(command, **kwargs):
    print('RUN ' + subprocess.list2cmdline(list(map(str, command))), flush=True)
    process=subprocess.Popen(list(map(str, command)),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                             text=True,encoding='utf-8',errors='replace',creationflags=subprocess.CREATE_NO_WINDOW,**kwargs)
    for line in process.stdout:print(line,end='',flush=True)
    if process.wait():raise subprocess.CalledProcessError(process.returncode,command)

def main():
    manifest = json.loads((ROOT/'cuda_redistrib.json').read_text())
    jobs = [(f'https://codeload.github.com/spiriMirror/libuipc/zip/{COMMIT}', 'libuipc.zip', None, BUILD/'source')]
    for name in ('cuda_nvcc','cuda_cudart','cuda_cccl'):
        item = manifest[name]['windows-x86_64']
        jobs.append(('https://developer.download.nvidia.com/compute/cuda/redist/'+item['relative_path'], name+'.zip', item['sha256'], BUILD/'cuda'))
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        pending = {pool.submit(download, *job[:3]): job[3] for job in jobs}
        for future in concurrent.futures.as_completed(pending):
            archive = future.result()
            target = pending[future]
            marker = target/('.extracted-'+archive.stem)
            if not marker.exists():
                extract_strip(archive, target)
                marker.touch()
    venv = BUILD/'venv'
    if not (venv/'Scripts/python.exe').exists():
        run([sys.executable, '-m', 'venv', venv])
    python = venv/'Scripts/python.exe'
    if not (venv/'Scripts/cmake.exe').exists():
        run([python, '-m', 'pip', 'install', 'cmake==3.31.10', 'ninja', 'pybind11', 'pybind11-stubgen', 'numpy', 'sympy', 'requests', 'setuptools'])
    vcpkg = BUILD/'vcpkg'
    if not (vcpkg/'.git').exists():
        run(['git','clone','--depth','1','--branch','2025.07.25','https://github.com/microsoft/vcpkg.git',vcpkg])
    if not (vcpkg/'vcpkg.exe').exists():
        run(['cmd','/d','/c',str(vcpkg/'bootstrap-vcpkg.bat'),'-disableMetrics'])
    (ROOT/'build_prepared.json').write_text(json.dumps({'root':str(BUILD),'python':str(python),'source':str(BUILD/'source'),'cuda':str(BUILD/'cuda'),'vcpkg':str(vcpkg)},indent=2))
    print('Prepared '+str(BUILD),flush=True)

if __name__ == '__main__':
    main()
