"""Follow an already running experiment and finalize after both cases succeed."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[2]


def last_solve(path):
    if not path.exists() or not path.stat().st_size:
        return {}
    with path.open('rb') as stream:
        stream.seek(max(0, path.stat().st_size - 16384))
        lines = stream.read().splitlines()
    for line in reversed(lines):
        try:
            return json.loads(line)
        except (ValueError, UnicodeError):
            continue
    return {}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, default=PROJECT / 'result/vehicle_full_pass')
    args = parser.parse_args()
    while True:
        paths = sorted(args.result.glob('*/status.json'))
        states = [json.loads(p.read_text(encoding='utf-8')) for p in paths]
        if any(s['status'] == 'FAILED' for s in states):
            raise RuntimeError('An experiment case failed; inspect its status.json and run.log')
        if len(states) == 2 and all(s['status'] == 'COMPLETED_FULL_PASS' for s in states):
            subprocess.run([sys.executable, str(HERE / 'finalize.py'), '--result', str(args.result)], check=True)
            return
        for state in states:
            solve = last_solve(Path(state['native_workspace']) / 'hsc_stats.jsonl')
            print(f'{state["case"]}: completed={state["completed_frames"]} '
                  f't={state["simulation_time_s"]:.3f}s '
                  f'crossings={len(state.get("wheel_crossings", {}))}/4 '
                  f'current_frame={solve.get("frame", 0)} Newton={solve.get("newton", 0)} '
                  f'status={state["status"]}', flush=True)
        time.sleep(45)


if __name__ == '__main__':
    main()
