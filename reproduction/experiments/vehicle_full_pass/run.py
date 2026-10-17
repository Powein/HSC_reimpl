"""Launch the configured cases, capture native failures, and preserve completed results."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

from storage import write_json

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[2]


def main():
    os.environ['PYTHONIOENCODING'] = 'utf-8'
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=HERE / 'config.json')
    parser.add_argument('--case', help='Run one case; omission runs both sequentially')
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding='utf-8'))
    runtime = json.loads((PROJECT / 'reproduction/hsc_backend/runtime.json').read_text(encoding='utf-8'))
    failures = []
    selected = [c for c in config['cases'] if not args.case or c['id'] == args.case]
    if not selected:
        parser.error('Case is not present in config.json')
    for case in selected:
        out = PROJECT / config['result_directory'] / case['id']
        out.mkdir(parents=True, exist_ok=True)
        command = [runtime['python'], '-u', str(HERE / 'run_case.py'),
                   '--config', str(args.config.resolve()), '--case', case['id']]
        if args.resume:
            command.append('--resume')
        print(f'Running {case["id"]}; progress: {out / "status.json"}', flush=True)
        with (out / 'run.log').open('ab') as log:
            process = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, cwd=PROJECT)
        if process.returncode:
            failures.append(case['id'])
            path = out / 'status.json'
            status = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
            # Native aborts bypass the Python exception handler.
            if status.get('status') != 'COMPLETED_FULL_PASS':
                status.update(status='FAILED', process_returncode=process.returncode)
                write_json(path, status)
            print(f'{case["id"]} failed ({process.returncode}); see run.log', flush=True)
        else:
            print(f'{case["id"]} completed', flush=True)
    all_results = [PROJECT / config['result_directory'] / c['id'] / 'status.json' for c in config['cases']]
    if not failures and all(p.exists() and json.loads(p.read_text(encoding='utf-8')).get('status') == 'COMPLETED_FULL_PASS' for p in all_results):
        subprocess.run([runtime['python'], str(HERE / 'finalize.py'),
                        '--result', str(PROJECT / config['result_directory'])], check=True)
    return int(bool(failures))


if __name__ == '__main__':
    sys.exit(main())
