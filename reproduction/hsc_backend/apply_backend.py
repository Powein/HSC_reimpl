"""Apply the inspectable HSC patch to a pristine, pinned libuipc checkout."""
from pathlib import Path
import difflib, hashlib, json, shutil
from prepare_build import BUILD, ROOT

def apply():
    source=BUILD/'source'
    old={}
    paths=['src/backends/cuda/linear_system/global_linear_system.h',
           'src/backends/cuda/linear_system/global_linear_system.cu',
           'src/core/core/scene_default_config.cpp',
           'src/pybind/pyuipc/constitution/module.cpp',
           'src/backends/cuda/line_search/line_searcher.cu',
           'src/backends/cuda/engine/advance_ipc.cu']
    backups=ROOT/'pristine'
    for name in paths:
        cache=backups/name
        if not cache.exists():
            cache.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(source/name,cache)
        old[name]=cache.read_text(encoding='utf-8')
    new=dict(old)
    name=paths[0]
    token='        CDenseVectorView b() { return m_b; }'
    assert new[name].count(token)==1
    new[name]=new[name].replace(token,token+'\n        CBCOOMatrixView A() const; // HSC: assembled IPC/FEM/ABD Hessian')
    name=paths[1]
    token='REGISTER_SIM_SYSTEM(GlobalLinearSystem);'
    new[name]=new[name].replace(token,token+'''

GlobalLinearSystem::CBCOOMatrixView GlobalLinearSystem::SolvingInfo::A() const
{
    return m_impl->bcoo_A.cview();
}
''')
    name=paths[2]
    new[name]=new[name].replace('Json::array({"fused_pcg", "linear_pcg"})','Json::array({"fused_pcg", "linear_pcg", "linear_hsc"})')
    token='    add("linear_system/fem_preconditioner",'
    entries=''
    for key,value,kind,desc,limits in [
        ('max_rank','IndexT{128}','integer','HSC fixed ACA rank cap (local setting; original scene rank is unpublished).','Json{{"minimum", 1}}'),
        ('neumann_order','IndexT{5}','integer','HSC Neumann series terms, for ABD inverse and FEM preconditioner.','Json{{"minimum", 1}}'),
        ('max_iter','IndexT{2000}','integer','HSC Schur PCG iteration cap; failure is explicit.','Json{{"minimum", 1}}'),
        ('adaptive_abd_inverse','IndexT{1}','integer','Extend ABD Neumann polynomial by doubling to resolve stiff joint collective modes; 0 reproduces fixed-r approximation.','Json{{"minimum", 0}, {"maximum", 1}}'),
        ('abd_inverse_tol','Float{1e-5}','number','Normalized residual limit of the extended ABD inverse.','Json{{"exclusiveMinimum", 0.0}}'),
        ('full_residual_limit','Float{0.1}','number','Inexact Newton safety limit on original full-system relative residual.','Json{{"exclusiveMinimum", 0.0}}'),
        ('omega','Float{0.7}','number','HSC weighted block Jacobi relaxation.','Json{{"exclusiveMinimum", 0.0}, {"maximum", 1.0}}')]:
        entries+=f'    add("linear_system/hsc/{key}", {value}, "{kind}",\n        "{desc}",\n        {{"src/backends/cuda/linear_system/linear_hsc.cu"}}, {limits});\n'
    assert new[name].count(token)==1
    new[name]=new[name].replace(token,entries+'\n'+token)
    name=paths[3]
    new[name]=new[name].replace('#include <pyuipc/constitution/stable_neo_hookean.h>',
        '#include <pyuipc/constitution/stable_neo_hookean.h>\n#include <pyuipc/constitution/log_neo_hookean.h>')
    new[name]=new[name].replace('    PyStableNeoHookean{m};',
        '    PyStableNeoHookean{m};\n    PyLogNeoHookean{m};')
    name=paths[4]
    # A trial outside an elastic energy's domain has +infinity energy and must
    # backtrack. Initial iterates must be finite; NaN/-infinity remain errors.
    for value in ('energy', 'total_energy'):
        token=f'!std::isnan({value}) && std::isfinite({value})'
        assert new[name].count(token)==1
        new[name]=new[name].replace(token,
            f'!std::isnan({value}) && (std::isfinite({value}) || (!is_initial && {value} > 0))')
    name=paths[5]
    token='if(converged)\n                            break;'
    assert new[name].count(token)==1
    new[name]=new[name].replace(token, 'if(converged && std::isfinite(E))\n                            break;')
    solver='src/backends/cuda/linear_system/linear_hsc.cu'
    old[solver]=''
    new[solver]=(ROOT/'linear_hsc.cu').read_text(encoding='utf-8')
    for overlay in sorted((ROOT/'overlays').rglob('*')):
        if overlay.is_file():
            name=overlay.relative_to(ROOT/'overlays').as_posix()
            old[name]=''
            new[name]=overlay.read_text(encoding='utf-8')
    diff=[];manifest=[]
    for name,data in new.items():
        target=source/name
        target.parent.mkdir(parents=True,exist_ok=True)
        if not target.exists() or target.read_text(encoding='utf-8')!=data:
            target.write_text(data,encoding='utf-8')
        diff.extend(difflib.unified_diff(old[name].splitlines(True),data.splitlines(True),fromfile='/dev/null' if not old[name] else 'a/'+name,tofile='b/'+name))
        manifest.append({'path':name,'original_sha256':hashlib.sha256(old[name].encode()).hexdigest(),
                         'patched_sha256':hashlib.sha256(data.encode()).hexdigest()})
    (ROOT/'integration.patch').write_text(''.join(diff),encoding='utf-8')
    (ROOT/'patch_manifest.json').write_text(json.dumps(manifest,indent=2))
    print('Applied native linear_hsc and config schema to '+str(source),flush=True)

if __name__=='__main__':apply()
