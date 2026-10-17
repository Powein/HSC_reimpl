"""Build libuipc's pinned C++ dependencies while the CUDA toolkit downloads."""
from prepare_build import BUILD, ROOT, run
import json, os, subprocess, sys

venv=BUILD/'venv'
if not (venv/'Scripts/python.exe').exists():
    run([sys.executable,'-m','venv',venv])
python=venv/'Scripts/python.exe'
run([python,'-m','pip','install','cmake==3.31.10','ninja','pybind11','pybind11-stubgen','numpy','sympy','requests','setuptools'])
vcpkg=BUILD/'vcpkg'
if not (vcpkg/'.git').exists():
    run(['git','clone','--depth','1','--branch','2025.07.25','https://github.com/microsoft/vcpkg.git',vcpkg])
if not (vcpkg/'vcpkg.exe').exists():
    run(['cmd','/d','/c',vcpkg/'bootstrap-vcpkg.bat','-disableMetrics'])
binary=BUILD/'build'
binary.mkdir(exist_ok=True)
run([python,BUILD/'source/scripts/gen_vcpkg_json.py',binary,'--dev_mode=OFF','--with_usd_support=OFF','--with_vdb_support=OFF','--with_cuda_backend=ON'])
from local_registry import prepare
prepare()
from json_release_port import apply as json_release
json_release()
from prefetch_dependencies import prefetch
prefetch()
from build_backend import environment
env=environment()
(BUILD/'vcpkg_cache').mkdir(exist_ok=True)
run([vcpkg/'vcpkg.exe','install','--triplet=x64-windows','--overlay-triplets='+str(BUILD/'triplets'),'--x-manifest-root='+str(binary),'--x-install-root='+str(binary/'vcpkg_installed')],env=env)
(ROOT/'dependencies_ready.json').write_text(json.dumps({'root':str(BUILD)},indent=2))
