"""Run one complete vehicle passage with the explicitly selected native HSC backend."""
import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import time

from completion import PassageMonitor
from storage import SolverRecords, append_json, write_json

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[2]


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def rewind_file(path, length):
    """Retain an interrupted tail as evidence before restoring a checkpoint offset."""
    if not path.exists() or path.stat().st_size <= length:
        return
    with path.open('r+b') as stream:
        stream.seek(length)
        tail = stream.read()
        archive = path.with_name(path.name + f'.interrupted-{time.time_ns()}')
        archive.write_bytes(tail)
        stream.truncate(length)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=HERE / 'config.json')
    parser.add_argument('--case', required=True)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding='utf-8'))
    case = next((c for c in config['cases'] if c['id'] == args.case), None)
    if case is None:
        parser.error(f'Unknown case: {args.case}')
    if config['obstacles'] != 3:
        parser.error('The passage monitor currently requires the three-tire row')
    effective = {**config, 'case': case}
    config_sha = hashlib.sha256(json.dumps(effective, sort_keys=True).encode()).hexdigest()
    runtime = json.loads((PROJECT / 'reproduction/hsc_backend/runtime.json').read_text(encoding='utf-8'))
    os.environ['HC_UIPC_RUNTIME'] = runtime['runtime']
    sys.path.insert(0, str(PROJECT / 'reproduction/vehicle_assembly'))
    from scene import build_vehicle, Logger
    import numpy as np

    Logger.set_level(Logger.Level.Info if os.environ.get('HC_VERBOSE') else Logger.Level.Warn)
    out = PROJECT / config['result_directory'] / case['id']
    out.mkdir(parents=True, exist_ok=True)
    # Windows releases this byte lock even after a native process abort. A second
    # launcher waits, then reads the finished status instead of running twice.
    import msvcrt
    execution_lock = (out / '.execution.lock').open('a+b')
    while True:
        execution_lock.seek(0)
        try:
            msvcrt.locking(execution_lock.fileno(), msvcrt.LK_NBLCK, 1)
            break
        except OSError:
            time.sleep(5)
    native = Path(runtime['root']) / 'runs' / config['experiment'] / case['id']
    backend = Path(runtime['runtime']) / 'uipc/_native/uipc_backend_cuda.dll'
    backend_sha = digest(backend)
    checkpoint = None
    if (out / 'status.json').exists():
        previous = json.loads((out / 'status.json').read_text(encoding='utf-8'))
        if previous['config_sha256'] != config_sha:
            raise RuntimeError('Existing result has different parameters; choose a new result directory')
        if previous['status'] == 'COMPLETED_FULL_PASS':
            print(f'{case["id"]}: already completed; results preserved', flush=True)
            return
        if not args.resume:
            raise RuntimeError('Result already exists; use --resume to recover its last checkpoint')
        checkpoint = json.loads((out / 'checkpoint.json').read_text(encoding='utf-8'))
        if checkpoint['backend_sha256'] != backend_sha:
            raise RuntimeError('Backend changed since checkpoint; use a separate experiment directory')
        provenance = json.loads((out / 'provenance.json').read_text(encoding='utf-8'))
        scene_source = PROJECT / 'reproduction/vehicle_assembly/scene.py'
        scene_key = str(scene_source.relative_to(PROJECT))
        if provenance['source_sha256'][scene_key] != digest(scene_source):
            raise RuntimeError('Scene source changed since checkpoint; use a separate experiment directory')
    elif native.exists():
        raise RuntimeError(f'Unmatched native workspace exists: {native}')

    native.mkdir(parents=True, exist_ok=True)
    (out / 'frames').mkdir(exist_ok=True)
    if checkpoint is None:
        write_json(out / 'parameters.json', effective)
        sources = [HERE / name for name in ('config.json', 'run_case.py', 'completion.py', 'storage.py')]
        sources += [PROJECT / 'reproduction/vehicle_assembly/scene.py',
                    PROJECT / 'reproduction/hsc_backend/linear_hsc.cu',
                    PROJECT / 'reproduction/hsc_backend/integration.patch',
                    PROJECT / 'reproduction/hsc_backend/patch_manifest.json',
                    PROJECT / 'reproduction/hsc_backend/log_nh_check.json']
        sources += sorted((PROJECT / 'reproduction/hsc_backend/overlays').rglob('*.*'))
        write_json(out / 'provenance.json', {
            'backend_sha256': backend_sha, 'runtime': runtime,
            'source_sha256': {str(p.relative_to(PROJECT)): digest(p) for p in sources},
            'native_workspace': str(native), 'config_sha256': config_sha,
            'snapshot_dtype': 'float32 (visualization); solver and measurements use float64',
        })
        for source in sources:
            copy = out / 'source_snapshot' / source.relative_to(PROJECT)
            copy.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, copy)
    status = {
        'status': 'INITIALIZING', 'case': case['id'], 'solver': 'linear_hsc',
        'config_sha256': config_sha, 'backend_sha256': backend_sha,
        'native_workspace': str(native), 'completed_frames': 0,
        'simulation_time_s': 0., 'elapsed_s': 0.,
    }
    write_json(out / 'status.json', status)
    solver = SolverRecords(native / 'hsc_stats.jsonl', out / 'hsc_stats.jsonl')
    sim = build_vehicle(
        'paper_stiffness', config['obstacles'], config['motor_speed_rad_s'],
        case['id'], config['start_x'], dt=config['dt'],
        material_overrides={**config['materials'], **{k: v for k, v in case.items() if k != 'id'}},
        solver_options=config['hsc'], native_workspace=native, strict_mode=True,
        newton_max_iter=config['newton_max_iter'],
        motor_ramp_time=config['motor_ramp_time_s'],
        material_model=config['material_model'],
    )
    if checkpoint is None:
        sim.export_manifest(out / 'scene_manifest.json')
        np.savez_compressed(out / 'geometry.npz',
            **{f'body_v_{i}': b.vertices for i, b in enumerate(sim.bodies)},
            **{f'body_f_{i}': b.faces for i, b in enumerate(sim.bodies)},
            tire_faces=sim.wheels[0].faces, tire_tets=sim.wheels[0].tets)
    reference_local = []
    for wheel in sim.wheels:
        transform = wheel.rim.transform()
        reference_local.append((wheel.ref_vertices - transform[:3, 3]) @ np.linalg.inv(transform[:3, :3]).T)

    def geometry_measurements():
        mounted, obstacles, car_vertices = [], [], []
        wheel_vertices = [w.vertices() for w in sim.wheels]
        for body in sim.bodies:
            if not body.name.startswith('loose_'):
                car_vertices.append(body.points())
        for wheel, vertices in zip(sim.wheels, wheel_vertices):
            center = wheel.rim.transform()[:3, 3].tolist()
            item = {'name': wheel.name, 'center': center}
            if wheel.name.startswith('loose_'):
                points = np.concatenate([vertices, wheel.rim.points()])
                item.update(min_y=float(points[:, 1].min()), max_y=float(points[:, 1].max()),
                            max_z=float(points[:, 2].max()), max_x=float(points[:, 0].max()))
                obstacles.append(item)
            else:
                mounted.append(item)
                car_vertices.append(vertices)
        return float(min(p[:, 0].min() for p in car_vertices)), mounted, obstacles, wheel_vertices

    car_min, wheels, obstacles, _ = geometry_measurements()
    monitor = PassageMonitor(max(o['max_x'] for o in obstacles), config['clearance_m'], config['clear_frames'])
    started = time.perf_counter()
    previous_elapsed = checkpoint['elapsed_s'] if checkpoint else 0.

    def elapsed():
        return previous_elapsed + time.perf_counter() - started

    def snapshot(frame, vertices):
        np.savez_compressed(out / 'frames' / f'frame_{frame:06d}.npz',
                            frame=frame,
                            body_transforms=np.asarray([b.transform() for b in sim.bodies], dtype=np.float32),
                            tire_vertices=np.asarray(vertices, dtype=np.float32))

    def save_checkpoint():
        if not sim.world.dump():
            raise RuntimeError('Native checkpoint dump failed')
        paths = ['trajectory.jsonl', 'frame_stats.jsonl', 'hsc_stats.jsonl']
        write_json(out / 'checkpoint.json', {
            'frame': sim.world.frame(), 'backend_sha256': backend_sha,
            'config_sha256': config_sha, 'elapsed_s': elapsed(),
            'monitor': asdict(monitor), 'solver_offset': solver.offset,
            'solver_summary': solver.summary,
            'file_offsets': {p: (out / p).stat().st_size if (out / p).exists() else 0 for p in paths},
            'damping_last_distance': [s['last_d'] for s in sim.suspensions],
        })

    def record(frame, solves):
        car_min, mounted, loose, vertices = geometry_measurements()
        mesh_contacts = max((s['active_mesh_contacts'] for s in solves), default=0)
        passage = monitor.update(frame, car_min, max(o['max_x'] for o in loose), mounted, loose, mesh_contacts)
        metrics = sim.metrics()
        metrics.update(time_s=frame * config['dt'], passage=passage,
                       chassis_center=sim.bodies[0].transform()[:3, 3].tolist(),
                       obstacles=loose, elapsed_s=elapsed())
        for i, wheel in enumerate(sim.wheels):
            transform = wheel.rim.transform()
            local = (vertices[i] - transform[:3, 3]) @ np.linalg.inv(transform[:3, :3]).T
            metrics['wheels'][i]['max_elastic_displacement_m'] = float(np.linalg.norm(local - reference_local[i], axis=1).max())
        metrics['max_abd_orthogonality_error_squared'] = max(float(np.linalg.norm(
            b.transform()[:3, :3] @ b.transform()[:3, :3].T - np.eye(3)) ** 2) for b in sim.bodies)
        append_json(out / 'trajectory.jsonl', metrics)
        min_jacobian = np.asarray([w['min_tet_volume_ratio'] for w in metrics['wheels']]).min()
        if not np.isfinite(min_jacobian) or min_jacobian <= 0:
            snapshot(frame, vertices)
            raise RuntimeError(f'Invalid tetrahedron at frame {frame}: minimum determinant ratio {min_jacobian}')
        if frame % config['snapshot_every'] == 0 or passage['passed']:
            snapshot(frame, vertices)
        status.update(status='RUNNING', completed_frames=frame,
                      simulation_time_s=frame * config['dt'], elapsed_s=elapsed(),
                      passage=passage, wheel_crossings=monitor.crossings, hsc=solver.summary)
        write_json(out / 'status.json', status)
        return passage

    try:
        sim.init()
        if checkpoint:
            # The native backend appends solver records. Preserve abandoned tails before replay.
            rewind_file(native / 'hsc_stats.jsonl', checkpoint['solver_offset'])
            for name, length in checkpoint['file_offsets'].items():
                rewind_file(out / name, length)
            if not sim.world.recover(checkpoint['frame']):
                raise RuntimeError('Native recovery failed')
            sim.world.retrieve()
            monitor = PassageMonitor(**checkpoint['monitor'])
            solver.offset = checkpoint['solver_offset']
            solver.summary = checkpoint['solver_summary']
            for suspension, distance in zip(sim.suspensions, checkpoint['damping_last_distance']):
                suspension['last_d'] = distance
        else:
            record(0, [])
            save_checkpoint()
        print(f'{case["id"]}: HSC initialized at frame {sim.world.frame()}', flush=True)
        for frame in range(sim.world.frame() + 1, config['max_frames'] + 1):
            sim.advance()
            solves = solver.collect()
            stats = sim.engine.frame_stats()
            append_json(out / 'frame_stats.jsonl', stats)
            if not stats['completed'] or not stats['converged'] or stats['hit_newton_limit'] or stats['hit_line_search_limit']:
                raise RuntimeError(f'Frame {frame} did not converge: {stats}')
            passage = record(frame, solves)
            print(f'{case["id"]}: frame={frame} t={frame * config["dt"]:.3f}s '
                  f'clearance={passage["clearance_m"]:.3f}m crossings={len(monitor.crossings)}/4 '
                  f'Newton={stats["newton_iterations"]} elapsed={elapsed():.1f}s', flush=True)
            if frame % config['checkpoint_every'] == 0 or passage['passed']:
                save_checkpoint()
            if passage['passed']:
                status['status'] = 'COMPLETED_FULL_PASS'
                break
        else:
            raise RuntimeError('Frame budget exhausted before all wheels and the car tail passed')
    except BaseException as exc:
        status.update(status='FAILED', error=repr(exc), elapsed_s=elapsed())
        raise
    finally:
        solver.collect()
        status.update(elapsed_s=elapsed(), hsc=solver.summary)
        write_json(out / 'status.json', status)
    print(json.dumps(status, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
