"""Extract experiment tables from saved measurements; does not run physics."""
import argparse
import csv
import json
from pathlib import Path

from storage import write_json

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[2]


def read_rows(path):
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines() if line.strip()]


def summarize_case(path):
    status = json.loads((path / 'status.json').read_text(encoding='utf-8'))
    parameters = json.loads((path / 'parameters.json').read_text(encoding='utf-8'))
    manifest = json.loads((path / 'scene_manifest.json').read_text(encoding='utf-8'))
    trajectory = read_rows(path / 'trajectory.jsonl')
    frames = read_rows(path / 'frame_stats.jsonl')
    first, last = trajectory[0], trajectory[-1]
    mounted = [w for row in trajectory for w in row['wheels'] if w['name'].startswith('wheel_')]
    loose = [w for row in trajectory for w in row['wheels'] if w['name'].startswith('loose_')]
    all_wheels = mounted + loose
    return {
        'case': parameters['case']['id'], 'status': status['status'],
        'mounted_E_Pa': manifest['materials']['mounted_E'], 'loose_E_Pa': manifest['materials']['loose_E'],
        'dt_s': parameters['dt'], 'frames': last['frame'], 'simulation_time_s': last['time_s'],
        'wall_time_s': status['elapsed_s'],
        'chassis_travel_x_m': last['chassis_center'][0] - first['chassis_center'][0],
        'final_tail_clearance_m': last['passage']['clearance_m'],
        'wheel_crossings': status.get('wheel_crossings', {}),
        'mounted_peak_elastic_displacement_m': max(w['max_elastic_displacement_m'] for w in mounted),
        'loose_peak_elastic_displacement_m': max(w['max_elastic_displacement_m'] for w in loose),
        'max_bead_error_m': max(w['max_bead_error_m'] for w in all_wheels),
        'min_tet_volume_ratio': min(w['min_tet_volume_ratio'] for w in all_wheels),
        'min_tire_vertex_z_m': min(w['min_z_m'] for w in all_wheels),
        'max_abd_orthogonality_error_squared': max(r['max_abd_orthogonality_error_squared'] for r in trajectory),
        'max_newton_iterations': max((f['newton_iterations'] for f in frames), default=0),
        'all_frames_converged': all(f['converged'] and f['completed'] and not f['hit_newton_limit']
                                    and not f['hit_line_search_limit'] for f in frames),
        'hsc': status.get('hsc', {}),
        'obstacle_final_centers_m': {o['name']: o['center'] for o in last['obstacles']},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, default=PROJECT / 'result/vehicle_full_pass')
    args = parser.parse_args()
    cases = [summarize_case(p.parent) for p in sorted(args.result.glob('*/parameters.json'))
             if (p.parent / 'trajectory.jsonl').exists()]
    cases.sort(key=lambda case: case['loose_E_Pa'], reverse=True)
    parameters = [json.loads((args.result / c['case'] / 'parameters.json').read_text(encoding='utf-8')) for c in cases]
    provenance = [json.loads((args.result / c['case'] / 'provenance.json').read_text(encoding='utf-8')) for c in cases]
    common_parameters = [{k: v for k, v in p.items() if k != 'case'} for p in parameters]
    controlled_cases = [{k: v for k, v in p['case'].items() if k not in ('id', 'loose_E')} for p in parameters]
    source_hashes = [{k.replace('\\', '/'): v for k, v in p['source_sha256'].items()} for p in provenance]
    scene_key = 'reproduction/vehicle_assembly/scene.py'
    comparison_control = {
        'same_common_configuration': len(common_parameters) == 2 and common_parameters[0] == common_parameters[1],
        'only_loose_E_changes': len(controlled_cases) == 2 and controlled_cases[0] == controlled_cases[1]
                               and parameters[0]['case']['loose_E'] != parameters[1]['case']['loose_E'],
        'same_backend': len(provenance) == 2 and provenance[0]['backend_sha256'] == provenance[1]['backend_sha256'],
        'same_scene_source': len(source_hashes) == 2 and source_hashes[0][scene_key] == source_hashes[1][scene_key],
        'changed_parameter': 'loose_E',
        'mounted_E_Pa': [c['mounted_E_Pa'] for c in cases],
        'loose_E_Pa': [c['loose_E_Pa'] for c in cases],
    }
    write_json(args.result / 'comparison.json', {'cases': cases,
        'comparison_control': comparison_control,
        'both_completed': len(cases) == 2 and all(c['status'] == 'COMPLETED_FULL_PASS' for c in cases)})
    if cases:
        columns = [key for key, value in cases[0].items() if not isinstance(value, dict)]
        with (args.result / 'comparison.csv').open('w', encoding='utf-8-sig', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=columns, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(cases)
    print(json.dumps(cases, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
