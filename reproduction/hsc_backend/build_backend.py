"""Compile the patched backend and Python package in an isolated local venv."""
from pathlib import Path
import argparse, json, os, subprocess, sys, time
from prepare_build import BUILD, ROOT, run
from apply_backend import apply

def environment():
    vs=Path(r'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat')
    script=BUILD/'vcenv.cmd'
    script.write_text(f'@call "{vs}" -no_logo -arch=x64 >nul\n@set\n',encoding='ascii')
    result=subprocess.run(['cmd','/d','/c',str(script)],capture_output=True,text=True,
                          creationflags=subprocess.CREATE_NO_WINDOW,check=True)
    env=os.environ.copy()
    for line in result.stdout.splitlines():
        if '=' in line:
            k,v=line.split('=',1);env[k.upper()]=v
    env['PATH']=str(BUILD/'venv/Lib/site-packages/cmake/data/bin')+';'+str(BUILD/'venv/Scripts')+';'+str(BUILD/'cuda/bin')+';'+env.get('PATH','')
    env['VCPKG_FORCE_SYSTEM_BINARIES']='1'
    env['CUDA_PATH']=str(BUILD/'cuda')
    env['PYTHONUTF8']='1'
    env['VSLANG']='1033'
    env['VCPKG_DISABLE_METRICS']='1'
    env['VCPKG_BINARY_SOURCES']='clear;files,'+str(BUILD/'vcpkg_cache')+',readwrite'
    env['VCPKG_DOWNLOADS']=str(BUILD/'vcpkg_downloads')
    env['VCPKG_MAX_CONCURRENCY']='6'
    (BUILD/'temporary').mkdir(exist_ok=True)
    env['TEMP']=env['TMP']=str(BUILD/'temporary')
    sdk=BUILD/'sdk_packages/microsoft.windows.sdk.cpp/c'
    sdk_lib=BUILD/'sdk_packages/microsoft.windows.sdk.cpp.x64/c'
    version='10.0.26100.0'
    if not (sdk/'Include'/version/'ucrt/stdio.h').exists():
        raise RuntimeError('Local Windows SDK missing; run prepare_sdk.py')
    env['INCLUDE']=';'.join([env.get('INCLUDE','')]+[str(sdk/'Include'/version/x) for x in ('ucrt','shared','um','winrt','cppwinrt')])
    env['LIB']=';'.join([env.get('LIB',''),str(sdk_lib/'um/x64'),str(sdk_lib/'ucrt/x64')])
    env['PATH']=str(sdk/'bin'/version/'x64')+';'+env['PATH']
    env['WINDOWSSDKDIR']=str(sdk)+'\\'
    env['WINDOWSSDKVERSION']=version+'\\'
    env['WINDOWSSDKLIBVERSION']=version+'\\'
    env['UNIVERSALCRTSDKDIR']=str(sdk)+'\\'
    env['UCRTVERSION']=version
    triplets=BUILD/'triplets';triplets.mkdir(exist_ok=True)
    (triplets/'x64-windows.cmake').write_text('''set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE dynamic)
set(VCPKG_BUILD_TYPE release)
set(VCPKG_LOAD_VCVARS_ENV OFF)
set(VCPKG_ENV_PASSTHROUGH PATH INCLUDE LIB LIBPATH WINDOWSSDKDIR WINDOWSSDKVERSION WINDOWSSDKLIBVERSION UNIVERSALCRTSDKDIR UCRTVERSION TEMP TMP VCPKG_FORCE_SYSTEM_BINARIES)
''')
    return env

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--jobs',type=int,default=4)
    parser.add_argument('--configure-only',action='store_true')
    parser.add_argument('--wait-for-deps',action='store_true')
    args=parser.parse_args()
    if args.wait_for_deps:
        deadline=time.monotonic()+1800
        while not (ROOT/'dependencies_ready.json').exists():
            if time.monotonic()>deadline:raise TimeoutError('Dependency build did not complete')
            print('Waiting for the active dependency build...',flush=True)
            time.sleep(30)
    apply()
    env=environment()
    cmake=BUILD/'venv/Lib/site-packages/cmake/data/bin/cmake.exe'
    binary=BUILD/'build'
    command=[cmake,'-S',BUILD/'source','-B',binary,'-G','Ninja',
             '-DCMAKE_BUILD_TYPE=Release',
             '-DCMAKE_TOOLCHAIN_FILE='+str(BUILD/'vcpkg/scripts/buildsystems/vcpkg.cmake'),
             '-DVCPKG_OVERLAY_TRIPLETS='+str(BUILD/'triplets'),
             '-DCMAKE_MAKE_PROGRAM='+str(BUILD/'venv/Scripts/ninja.exe'),
             '-DUIPC_BUILD_PYBIND=ON','-DUIPC_BUILD_EXAMPLES=OFF','-DUIPC_BUILD_TESTS=OFF',
             '-DUIPC_BUILD_BENCHMARKS=OFF','-DUIPC_DEV_MODE=ON',
             '-DUIPC_WITH_CUDA_BACKEND=ON','-DUIPC_WITH_CUDA_LEGACY_COLLISION=ON',
             '-DUIPC_CUDA_ARCHITECTURES=86','-DCMAKE_CUDA_ARCHITECTURES=86',
             '-DCUDAToolkit_ROOT='+str(BUILD/'cuda'),
             '-DCMAKE_CUDA_COMPILER='+str(BUILD/'cuda/bin/nvcc.exe'),
             '-DCMAKE_CUDA_FLAGS=-allow-unsupported-compiler -Xcompiler=/Zc:__cplusplus -Xcompiler=/bigobj',
             '-DUIPC_PYTHON_EXECUTABLE_PATH='+str(BUILD/'venv/Scripts/python.exe')]
    run(command,env=env)
    if args.configure_only:return
    run([cmake,'--build',binary,'--config','Release','--parallel',str(args.jobs)],env=env)
    runtime=binary/'python/src'
    assert (runtime/'uipc/_native/uipc_backend_cuda.dll').is_file()
    info={'root':str(BUILD),'runtime':str(runtime),'python':str(BUILD/'venv/Scripts/python.exe'),
          'source_commit':'9c748a759bb9952a7aeedd59076a70591dc61d6c','solver':'linear_hsc'}
    (ROOT/'runtime.json').write_text(json.dumps(info,indent=2))
    print('HSC runtime ready: '+str(runtime),flush=True)

if __name__=='__main__':main()
