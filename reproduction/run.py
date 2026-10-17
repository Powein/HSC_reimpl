"""Reproduce the inspectable core and write every measurement to results/."""
import bootstrap
from pathlib import Path
import argparse
import csv
import hashlib
import importlib.metadata
import json
import platform
import re
import shutil
import time
import traceback
import zipfile
import numpy as np
from core import ACA, HSC, Neumann, BlockInverse, aca, pcg, refinement, manufactured_system


ROOT = bootstrap.ROOT
PROJECT = ROOT.parent


def save_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")


def audit():
    original = PROJECT / "Guo-2026-HSC"
    missing = []
    for p in original.rglob("*.cuh"):
        for inc in re.findall(r'^#include\s+"([^"]+)"', p.read_text(), re.M):
            # A quoted standard C header still resolves on a standard include path.
            if inc in {"math.h"}:
                continue
            if not (p.parent / inc).exists() and not (original / inc).exists():
                missing.append({"file": str(p.relative_to(original)), "include": inc})
    unchanged = True
    with zipfile.ZipFile(PROJECT / "Guo-2026-HSC.zip") as z:
        files = [i for i in z.infolist() if not i.is_dir()]
        for item in files:
            if (PROJECT / item.filename).read_bytes() != z.read(item):
                unchanged = False
    hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
              for p in [PROJECT / "HC.pdf", PROJECT / "Guo-2026-HSC.zip"]}
    all_code = "\n".join(p.read_text() for p in original.rglob("*.cuh"))
    current_zip=ROOT/'downloads'/'author-current-code.zip'
    current_identical=(hashlib.sha256(current_zip.read_bytes()).hexdigest()==hashes['Guo-2026-HSC.zip']) if current_zip.exists() else None
    return {"source_hashes": hashes, "original_files_unchanged": unchanged,
            "author_current_zip_identical_to_user_zip":current_identical,
            "archive_file_count": len(files), "missing_local_includes": missing,
            "nvcc_on_path": shutil.which("nvcc"), "has_standalone_build": False,
            "has_scene_assets": False, "has_ACA_implementation": bool(re.search(r"\bACA\b|adaptive_cross", all_code)),
            "has_Schur_implementation": bool(re.search(r"\bSchur\b|schur", all_code)),
            "has_documented_PCG_fem_abd_coupling_cuda": "PCG_fem_abd_coupling_cuda" in all_code,
            "missing_runtime_components": ["FEM/ABD full energy assembly and time integrator",
                  "IPC barrier/friction and CCD", "full HSC/ACA driver",
                  "paper scenes, meshes, initial/boundary conditions and benchmark outputs"],
            "source_paper_parameter_differences": {
                "ACA_termination": {"technical_appendix_page_2":"fixed maximum rank only",
                                    "reference_main":"fixed kmax=min(m,n), roundoff-level zero-pivot guard",
                                    "reference_extra":"tolerance-based sweep separately labeled diagnostic"},
                "omega": {"paper_section_3_3": 0.7, "source_PCG_default": 0.6667},
                "neumann_order": {"paper_experiment_default": 5, "source_PCG_default": 1},
                "tolerance": {"paper_section_5": "1e-4 (norm convention unspecified)",
                              "paper_figure_2": "relative residual < 1e-5, max 1000",
                              "source": "squared residual; tol=1e-12, relative_residual=false"},
                "block_12_inverse": "jacobi_cuda.cuh uses scalar diagonal inversion for block_size > 3; reference uses full 12x12 block inverse"}}


def numerical_checks():
    checks = []
    def check(name, value, tolerance):
        checks.append({"name": name, "error": float(value), "tolerance": tolerance,
                       "passed": bool(value <= tolerance)})
    rng = np.random.default_rng(501)
    U, V = rng.normal(size=(24, 4)), rng.normal(size=(45, 4))
    E = U @ V.T
    factors = aca(E, rtol=1e-12)
    check("ACA exact rank-4 reconstruction", np.linalg.norm(E-factors.U@factors.V.T)/np.linalg.norm(E), 1e-12)
    check("ACA recovered rank", abs(factors.rank - 4), 0)
    fixed=aca(E,max_rank=3)
    check("ACA fixed-rank budget respected",abs(fixed.rank-3),0)
    empty = aca(np.zeros((12, 30)))
    check("zero coupling rank", empty.rank, 0)
    A = rng.normal(size=(24,24)); A = A.T @ A + 50*np.eye(24)
    op = Neumann(A, 12, 5, .7)
    T = np.eye(24) - .7*op.Binv(A)
    expected = sum(np.linalg.matrix_power(T, k) for k in range(5)) @ (.7*op.Binv(np.eye(24)))
    actual = op(np.eye(24))
    check("Neumann recurrence vs explicit polynomial", np.max(abs(expected-actual)), 1e-12)
    check("Neumann I-MA=T^r identity", np.max(abs(np.eye(24)-actual@A-np.linalg.matrix_power(T,5))), 1e-12)
    check("Neumann inverse symmetry", np.max(abs(actual-actual.T)), 1e-12)
    D, A, E, H, b, truth = manufactured_system(stiffness=1e4, nx=4, ny=2, nz=2)
    solver = HSC(D,A,E,exact_affine=True)
    x, info = solver.solve(b, rtol=1e-12)
    direct = np.linalg.solve(H,b)
    check("exact Schur vs independent dense solve", np.linalg.norm(x-direct)/np.linalg.norm(direct), 1e-8)
    check("exact Schur full-system residual", np.linalg.norm(H@x-b)/np.linalg.norm(b), 1e-10)
    direction = rng.normal(size=len(D))
    direct_mv = (D-E.T@np.linalg.solve(A,E))@direction
    check("low-rank vs explicit Schur matvec", np.linalg.norm(solver.matvec(direction)-direct_mv)/np.linalg.norm(direct_mv), 1e-12)
    for scale in [1., 1e-12]:
        x, info = pcg(lambda z: H@z, b*scale, BlockInverse(H,[3]*(len(D)//3)+[12]*3), rtol=1e-10)
        check(f"PCG RHS scale {scale:g} true relative residual", np.linalg.norm(H@x-b*scale)/np.linalg.norm(b*scale), 1e-10)
    x, info = pcg(lambda z: H@z, np.zeros(len(H)))
    check("PCG zero RHS", np.linalg.norm(x), 0)
    return {"status": "PASS" if all(c["passed"] for c in checks) else "FAIL", "checks": checks}


def experiments(out):
    results = []
    for stiffness in [1e2, 1e4, 1e6]:
        D, A, E, H, b, truth = manufactured_system(stiffness)
        n, m = len(D), len(A)
        direct = np.linalg.solve(H,b)
        baseline_x, baseline = pcg(lambda z:H@z, b, BlockInverse(H,[3]*(n//3)+[12]*(m//12)),rtol=1e-9)
        row = {"stiffness": stiffness, "seed": 2026, "deformable_dofs": n, "affine_dofs": m,
               "condition_H": float(np.linalg.cond(H)),
               "direct_relative_residual": float(np.linalg.norm(H@direct-b)),
               "direct_error_to_manufactured_solution": float(np.linalg.norm(direct-truth)/np.linalg.norm(truth)),
               "baseline_mixed_block_jacobi_PCG": baseline, "orders": []}
        for order in [1,3,5,8]:
            solver = HSC(D,A,E,order=order,omega=.7,aca_max_rank=min(m,n))
            start=time.perf_counter()
            x, inner = solver.solve(b)
            single_time=time.perf_counter()-start
            x_refined, outer = refinement(H,b,solver)
            row["orders"].append({"order": order, "omega": .7, "aca_rtol": None,
                "aca_mode":"fixed_max_rank_with_numerical_zero_guard", "aca_max_rank":min(m,n),
                "aca_rank": solver.factors.rank, "aca_relative_error": solver.factors.residuals[-1],
                "aca_history": solver.factors.residuals,
                "rho_affine": solver.affine_neumann.spectral_radius(),
                "rho_deformable": solver.deformable_preconditioner.spectral_radius(),
                "condition_approximate_Schur": float(np.linalg.cond(solver.schur)),
                "inner_Schur_PCG": inner,
                "single_step_original_relative_residual": float(np.linalg.norm(H@x-b)),
                "single_step_solution_relative_error": float(np.linalg.norm(x-direct)/np.linalg.norm(direct)),
                "single_step_cpu_seconds_diagnostic_only": single_time,
                "quadratic_refinement": outer,
                "refined_original_relative_residual": float(np.linalg.norm(H@x_refined-b)),
                "refined_solution_relative_error": float(np.linalg.norm(x_refined-direct)/np.linalg.norm(direct))})
            print(f"stiffness={stiffness:g}, r={order}: inner={inner['iterations']}, "
                  f"one-step full residual={row['orders'][-1]['single_step_original_relative_residual']:.3e}, "
                  f"corrected={outer['history'][-1]:.3e} ({outer['steps']} steps)",flush=True)
        exact=HSC(D,A,E,exact_affine=True)
        x, info=exact.solve(b)
        row["exact_affine_Schur_control"]={**info,"original_relative_residual":float(np.linalg.norm(H@x-b)),
                "solution_relative_error":float(np.linalg.norm(x-direct)/np.linalg.norm(direct))}
        np.savez_compressed(out/f"fixture_{stiffness:.0e}.npz",D=D,A=A,E=E,H=H,b=b,x_star=truth,x_direct=direct)
        results.append(row)
    # Separate compression sweep: changing the coupling is allowed to change
    # the original residual; never judge accuracy only on the compressed matrix.
    sweep=[]
    D,A,E,H,b,_=manufactured_system()
    for tol in [1e-1,1e-2,1e-4,1e-10]:
        solver=HSC(D,A,E,aca_rtol=tol)
        x,info=solver.solve(b)
        xr,outer=refinement(H,b,solver)
        sweep.append({"aca_rtol":tol,"rank":solver.factors.rank,
                      "coupling_relative_error":solver.factors.residuals[-1],
                      "one_step_original_relative_residual":float(np.linalg.norm(H@x-b)),
                      "corrected_original_relative_residual":float(np.linalg.norm(H@xr-b)),
                      "quadratic_correction_steps":outer['steps'],"converged":outer['converged']})
    rank_sweep=[]
    for rank in [4,8,12,18]:
        solver=HSC(D,A,E,aca_max_rank=rank)
        x,info=solver.solve(b)
        xr,outer=refinement(H,b,solver)
        rank_sweep.append({"max_rank":rank,"actual_rank":solver.factors.rank,
            "coupling_relative_error":solver.factors.residuals[-1],
            "one_step_original_relative_residual":float(np.linalg.norm(H@x-b)),
            "corrected_original_relative_residual":float(np.linalg.norm(H@xr-b)),
            "quadratic_correction_steps":outer['steps'],"converged":outer['converged']})
    return {"scope":"manufactured SPD systems, not paper benchmark scenes", "systems":results,
            "ACA_fixed_rank_sweep":rank_sweep,"ACA_tolerance_sweep_nonpaper_diagnostic":sweep}


def plot_results(results,out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family":"DejaVu Sans","font.size":10,"axes.spines.top":False,
                         "axes.spines.right":False,"figure.dpi":150})
    fig,axs=plt.subplots(2,2,figsize=(11.5,7.5),layout="constrained")
    row=results['systems'][-1]
    colors=["#64748b","#0f766e","#2563eb","#9333ea"]
    for entry,color in zip(row['orders'],colors):
        r=entry['order']
        axs[0,0].semilogy(entry['inner_Schur_PCG']['history'],label=f"r={r}",color=color)
        axs[0,1].semilogy(entry['quadratic_refinement']['history'],label=f"r={r}",color=color,marker='o',ms=3)
    axs[0,0].set(title="Reduced Schur system: inner PCG",xlabel="PCG iterations",ylabel="True relative Schur residual")
    axs[0,1].set(title="Original full system: quadratic corrections",xlabel="Outer correction steps",ylabel="True relative full residual")
    axs[0,1].axhline(1e-9,color="#c2410c",ls="--",lw=1,label="target 1e-9")
    for system in results['systems']:
        axs[1,0].semilogy([x['order'] for x in system['orders']],
                         [x['single_step_original_relative_residual'] for x in system['orders']],
                         marker='o',label=f"stiffness {system['stiffness']:.0e}")
    axs[1,0].set(title="One HSC solve is inexact",xlabel="Neumann order",ylabel="Original-system relative residual",xticks=[1,3,5,8])
    hist=row['orders'][2]['aca_history']
    axs[1,1].semilogy(range(len(hist)),np.maximum(hist,1e-16),marker='o',color="#0f766e")
    axs[1,1].set(title="ACA coupling reconstruction",xlabel="ACA rank",ylabel="Relative Frobenius error")
    for ax in axs.flat:
        ax.grid(alpha=.18)
        if ax is not axs[1,1]: ax.legend(fontsize=8)
    fig.suptitle("HSC core validation | manufactured 180 + 36 DOF systems\nCPU NumPy reference; no paper scene or GPU performance claim",fontsize=14)
    fig.savefig(out/'core_validation.png')
    fig.savefig(out/'core_validation.svg')
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--skip-gpu',action='store_true')
    parser.add_argument('--output',type=Path,default=ROOT/'results')
    args=parser.parse_args()
    out=args.output.resolve(); out.mkdir(exist_ok=True,parents=True)
    start=time.perf_counter()
    save_json(out/'source_audit.json',audit())
    environment={"python":platform.python_version(),"executable":str(Path(__import__('sys').executable)),
                 "platform":platform.platform(),"numpy":np.__version__,
                 "packages":{d.metadata['Name']:d.version for d in importlib.metadata.distributions(path=[str(bootstrap.DEPS)])}}
    save_json(out/'environment.json',environment)
    checks=numerical_checks(); save_json(out/'numerical_checks.json',checks)
    print('NUMERICAL CHECKS:',checks['status'],flush=True)
    results=experiments(out); save_json(out/'experiments.json',results)
    with (out/'summary.csv').open('w',newline='',encoding='utf-8-sig') as f:
        fields=['stiffness','order','aca_rank','inner_iterations','one_step_full_residual','correction_steps','corrected_full_residual','relative_solution_error','converged']
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
        for system in results['systems']:
            for row in system['orders']:
                writer.writerow(dict(zip(fields,[system['stiffness'],row['order'],row['aca_rank'],row['inner_Schur_PCG']['iterations'],row['single_step_original_relative_residual'],row['quadratic_refinement']['steps'],row['refined_original_relative_residual'],row['refined_solution_relative_error'],row['quadratic_refinement']['converged']])))
    plot_results(results,out)
    gpu={"status":"SKIPPED","reason":"--skip-gpu"}
    if not args.skip_gpu:
        try:
            import cuda_original
            gpu=cuda_original.validate(PROJECT/'Guo-2026-HSC',out/'gpu')
        except Exception as e:
            gpu={"status":"UNAVAILABLE","error":repr(e),"traceback":traceback.format_exc()}
    save_json(out/'gpu_checks.json',gpu)
    print('ORIGINAL GPU KERNEL CHECKS:',gpu['status'],flush=True)
    if gpu['status']=='UNAVAILABLE':print(gpu['traceback'],flush=True)
    all_converged=all(x['quadratic_refinement']['converged'] for s in results['systems'] for x in s['orders'])
    target_converged=all(x['quadratic_refinement']['converged'] for s in results['systems'] for x in s['orders'] if x['order']==5)
    summary={"scope":"PARTIAL_CORE_REPRODUCTION", "paper_scene_verdict":"CANNOT_VERIFY",
             "numerical_checks":checks['status'],"gpu_kernel_checks":gpu['status'],
             "all_manufactured_corrections_converged":all_converged,
             "order_5_main_configuration_converged":target_converged,
             "outer_correction_budget":50,
             "elapsed_seconds":time.perf_counter()-start,
             "no_paper_runtime_or_speedup_comparison":True}
    save_json(out/'status.json',summary)
    print(json.dumps(summary,indent=2),flush=True)
    gpu_ok=gpu['status']=='PASS' or (args.skip_gpu and gpu['status']=='SKIPPED')
    return 0 if checks['status']=='PASS' and target_converged and gpu_ok else 1


if __name__=='__main__':
    raise SystemExit(main())
