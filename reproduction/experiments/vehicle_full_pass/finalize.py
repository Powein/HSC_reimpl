"""Produce tables, figures and the report once both physical trajectories finish."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time

from storage import write_json

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, default=PROJECT / 'result/vehicle_full_pass')
    args = parser.parse_args()
    result = args.result.resolve()
    status_files = sorted(result.glob('*/status.json'))
    if len(status_files) != 2 or any(json.loads(p.read_text(encoding='utf-8'))['status'] != 'COMPLETED_FULL_PASS' for p in status_files):
        raise RuntimeError('Both formal cases must finish before finalization')
    import msvcrt
    lock = (result / '.postprocess.lock').open('a+b')
    while True:
        lock.seek(0)
        try:
            msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
            break
        except OSError:
            time.sleep(2)
    sources = [HERE / name for name in ('summarize.py', 'render.py', 'report.py')]
    sources.append(PROJECT / 'reproduction/vehicle_assembly/mesh_render.py')
    fingerprint = hashlib.sha256(b''.join(p.read_bytes() for p in status_files + sources)).hexdigest()
    marker = result / 'postprocess.json'
    if marker.exists() and json.loads(marker.read_text(encoding='utf-8')).get('fingerprint') == fingerprint:
        print('Tables, figures and report already match the completed results.', flush=True)
        return
    for name, extra in [('summarize.py', []), ('render.py', ['--animate']), ('report.py', [])]:
        subprocess.run([sys.executable, str(HERE / name), '--result', str(result), *extra], check=True)
    write_json(marker, {'status': 'COMPLETED', 'fingerprint': fingerprint,
                        'outputs': ['comparison.json', 'comparison.csv', 'preview/passage.png',
                                    'preview/passage.gif', 'preview/comparison.png', 'report.md']})


if __name__ == '__main__':
    main()
