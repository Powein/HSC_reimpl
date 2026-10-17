"""Compile and run the constitutive derivative/feasible-segment checks once."""
import json
import re
import subprocess
from build_backend import environment
from prepare_build import BUILD, ROOT

entries = json.loads((BUILD / 'build/compile_commands.json').read_text(encoding='utf-8'))
entry = next(e for e in entries if e['file'].endswith('log_neo_hookean_3d.cu'))
includes = ['-I' + str(ROOT / 'overlays/src/backends/cuda')]
includes += re.findall(r'-I(?:"[^"]+"|\S+)', entry['command'])
includes += ['-I' + str(BUILD / 'build/vcpkg_installed/x64-windows/include'),
             '-I' + str(BUILD / 'build/vcpkg_installed/x64-windows/include/eigen3')]
target = BUILD / 'log_nh_check.exe'
command = [str(BUILD / 'cuda/bin/nvcc.exe'), '-x', 'cu', '-O3', '-DNDEBUG', '-std=c++20', '-allow-unsupported-compiler',
           '-arch=sm_86', '--expt-relaxed-constexpr', '-Xcompiler=/Zc:__cplusplus', '-Xcompiler=/bigobj',
           *[p.replace('"', '') for p in includes], str(ROOT / 'check_log_nh.cpp'), '-o', str(target)]
subprocess.run(command, env=environment(), check=True)
result = subprocess.run([str(target)], capture_output=True, text=True)
if result.returncode:
    print(result.stdout, result.stderr)
    result.check_returncode()
data = json.loads(result.stdout)
(ROOT / 'log_nh_check.json').write_text(json.dumps(data, indent=2), encoding='utf-8')
print(result.stdout)
