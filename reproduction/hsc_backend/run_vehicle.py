"""Advance the actual IPC vehicle scene with the native HSC solver.

This is a simulation entry point, not the previous assembly validation suite.
The solver selection is explicit; a missing native HSC build is a hard error.
"""
from pathlib import Path
import argparse, hashlib, json, os, re, shutil, sys, time

ROOT=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--steps',type=int,default=40)
    parser.add_argument('--obstacles',type=int,choices=range(13),default=3)
    parser.add_argument('--profile',choices=['assembly','paper_stiffness'],default='paper_stiffness')
    parser.add_argument('--motor-speed',type=float,default=2.)
    parser.add_argument('--start-x',type=float,default=-2.6)
    parser.add_argument('--rank',type=int,default=128)
    parser.add_argument('--fixed-neumann',action='store_true',help='Use paper fixed five ABD terms; full residual must still meet the inexact Newton limit')
    parser.add_argument('--label',default=time.strftime('hsc_%Y%m%d_%H%M%S'))
    args=parser.parse_args()
    if args.steps<1 or args.rank<1:parser.error('steps and rank must be positive')
    if not re.fullmatch(r'[A-Za-z0-9_-]+',args.label):parser.error('label must use ASCII letters, digits, _ or -')
    record=ROOT/'runtime.json'
    if not record.exists():raise RuntimeError('Build the native HSC backend first: build_backend.py')
    runtime=json.loads(record.read_text())
    os.environ['HC_UIPC_RUNTIME']=runtime['runtime']
    os.environ['HC_LINEAR_SOLVER']='linear_hsc'
    os.environ['HC_HSC_RANK']=str(args.rank)
    os.environ['HC_HSC_ADAPTIVE_ABD']='0' if args.fixed_neumann else '1'
    sys.path.insert(0,str(ROOT.parent/'vehicle_assembly'))
    from scene import build_vehicle, Logger, RUNTIME_ROOT
    import numpy as np
    Logger.set_level(Logger.Level.Warn)
    out=ROOT/'results'/args.label
    out.mkdir(parents=True,exist_ok=False)
    with (Path(runtime['runtime'])/'uipc/_native/uipc_backend_cuda.dll').open('rb') as binary:
        backend_sha=hashlib.file_digest(binary,'sha256').hexdigest()
    status={'status':'INITIALIZING','solver':'linear_hsc','parameters':vars(args),
            'runtime':runtime,'completed_steps':0,
            'backend_sha256':backend_sha}
    (out/'status.json').write_text(json.dumps(status,indent=2))
    sim=build_vehicle(args.profile,args.obstacles,args.motor_speed,args.label,args.start_x)
    sim.export_manifest(out/'scene_manifest.json')
    native_log=RUNTIME_ROOT/('hc_assembly_'+args.label)/'hsc_stats.jsonl'
    np.savez_compressed(out/'geometry.npz',
        **{f'body_v_{i}':b.vertices for i,b in enumerate(sim.bodies)},
        **{f'body_f_{i}':b.faces for i,b in enumerate(sim.bodies)},
        tire_faces=sim.wheels[0].faces,tire_tets=sim.wheels[0].tets)
    frames=[];metrics=[];body=[];tire=[];start=time.perf_counter()
    def save_frame():
        frames.append(sim.world.frame())
        body.append(np.array([b.transform() for b in sim.bodies]))
        tire.append(np.array([w.vertices() for w in sim.wheels]))
        np.savez_compressed(out/'snapshots.npz',frames=np.array(frames),
            body_transforms=np.array(body),tire_vertices=np.array(tire))
        (out/'trajectory.json').write_text(json.dumps(metrics,indent=2))
        if native_log.exists():shutil.copy2(native_log,out/'hsc_stats.jsonl')
    try:
        sim.init();metrics.append(sim.metrics());save_frame()
        print(f'HSC vehicle initialized: {len(sim.bodies)} ABD, {len(sim.wheels)} FEM tires',flush=True)
        for step in range(args.steps):
            sim.advance();metrics.append(sim.metrics())
            status.update(status='RUNNING',completed_steps=step+1,elapsed_s=time.perf_counter()-start)
            (out/'status.json').write_text(json.dumps(status,indent=2))
            if (step+1)%5==0 or step+1==args.steps:save_frame()
            print(f'Frame {step+1}/{args.steps}; elapsed {time.perf_counter()-start:.1f}s',flush=True)
        if not native_log.exists():raise RuntimeError('No native HSC solver records: integration did not run')
        solves=[json.loads(x) for x in native_log.read_text().splitlines() if x.strip()]
        if not solves or any(x['solver']!='linear_hsc' or x['fallback'] for x in solves):
            raise RuntimeError('Unexpected solver selection')
        status.update(status='COMPLETED',linear_solves=len(solves),
            max_schur_relative_residual=max(x['schur_relative_residual'] for x in solves),
            max_full_relative_residual=max(x['full_relative_residual'] for x in solves),
            max_active_mesh_contacts=max(x['active_mesh_contacts'] for x in solves),
            max_active_ground_contacts=max(x['active_ground_contacts'] for x in solves),
            rank_range=[min(x['rank'] for x in solves),max(x['rank'] for x in solves)],
            abd_inverse_mode='fixed_neumann' if args.fixed_neumann else 'adaptive_neumann',
            neumann_doublings_range=[min(x['neumann_doublings'] for x in solves),max(x['neumann_doublings'] for x in solves)],
            max_abd_inverse_residual=max(x['abd_inverse_residual'] for x in solves),
            chassis_displacement_x_m=float(body[-1][0,0,3]-body[0][0,0,3]),
            loose_rim_displacements_m={last['name']:(np.array(last['rim_center'])-np.array(first['rim_center'])).tolist()
                for first,last in zip(metrics[0]['wheels'],metrics[-1]['wheels']) if last['name'].startswith('loose_')},
            scope='Native HSC in real Newton/IPC loop. Hybrid CPU setup/GPU solve; not paper performance equivalence.')
    except BaseException as exc:
        status.update(status='FAILED',error=repr(exc),elapsed_s=time.perf_counter()-start)
        if native_log.exists():shutil.copy2(native_log,out/'hsc_stats.jsonl')
        raise
    finally:
        (out/'status.json').write_text(json.dumps(status,indent=2))
    print(json.dumps(status,indent=2),flush=True)

if __name__=='__main__':main()
