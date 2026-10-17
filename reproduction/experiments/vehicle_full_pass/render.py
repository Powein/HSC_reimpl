"""Render saved full-passage geometry and stiffness comparisons, without simulation."""
import argparse
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[2]
sys.path.insert(0, str(PROJECT / 'reproduction/vehicle_assembly'))
import common  # Project-local dependency search paths; does not import the simulator.
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
from mesh_render import objects, draw, projection
from summarize import read_rows

AZIMUTH, ELEVATION = -78, 22


def load_case(path):
    manifest = json.loads((path / 'scene_manifest.json').read_text(encoding='utf-8'))
    status = json.loads((path / 'status.json').read_text(encoding='utf-8'))
    files = [p for p in sorted((path / 'frames').glob('frame_*.npz'))
             if int(p.stem.split('_')[-1]) <= status['completed_frames']]
    states = {'frames': [], 'body_transforms': [], 'tire_vertices': []}
    for file in files:
        with np.load(file) as saved:
            states['frames'].append(int(saved['frame']))
            states['body_transforms'].append(saved['body_transforms'])
            states['tire_vertices'].append(saved['tire_vertices'])
    states = {key: np.asarray(value) for key, value in states.items()}
    return (manifest, np.load(path / 'geometry.npz'), states), status, read_rows(path / 'trajectory.jsonl')


def bounds(case):
    matrix, _ = projection(AZIMUTH, ELEVATION)
    lows, highs = [], []
    for i in range(len(case[2]['frames'])):
        points = np.concatenate([v for v, _, _ in objects(case, i)]) @ matrix
        lows.append(points.min(0))
        highs.append(points.max(0))
    lo, hi = np.min(lows, axis=0), np.max(highs, axis=0)
    return (lo[0] - .3, hi[0] + .3, min(lo[1] - .3, -.65), hi[1] + .3)


def scene_panel(ax, case, index, limits):
    mesh = objects(case, index)
    # Color only the loose tires to make the obstacle row legible.
    n_bodies = len(case[0]['bodies'])
    for i, wheel in enumerate(case[0]['wheels']):
        if wheel['name'].startswith('loose_'):
            vertices, faces, _ = mesh[n_bodies + i]
            mesh[n_bodies + i] = (vertices, faces, '#a86d38')
    matrix, _ = draw(ax, mesh, az=AZIMUTH, el=ELEVATION, limits=limits)
    ground = np.array([[-7, -2, 0.], [14, -2, 0.], [14, 2, 0.], [-7, 2, 0.]]) @ matrix
    ax.fill(ground[:, 0], ground[:, 1], color='#e1e7eb', zorder=-10)
    frame = int(case[2]['frames'][index])
    ax.set_title(f'Ground E = {case[0]["materials"]["loose_E"]:.0e} Pa | t = {frame * case[0]["config"]["dt"]:.2f} s', fontsize=12)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, default=PROJECT / 'result/vehicle_full_pass')
    parser.add_argument('--animate', action='store_true')
    args = parser.parse_args()
    entries = [(p.parent, *load_case(p.parent)) for p in sorted(args.result.glob('*/scene_manifest.json'))
               if (p.parent / 'frames/frame_000000.npz').exists()]
    if not entries:
        raise RuntimeError('No saved frames')
    entries.sort(key=lambda entry: entry[1][0]['materials']['loose_E'], reverse=True)
    preview = args.result / 'preview'
    preview.mkdir(exist_ok=True)
    limits_list = [bounds(case) for _, case, _, _ in entries]
    limits = (min(b[0] for b in limits_list), max(b[1] for b in limits_list),
              min(b[2] for b in limits_list), max(b[3] for b in limits_list))
    fig, axes = plt.subplots(len(entries), 3, figsize=(18, 3.8 * len(entries)), squeeze=False,
                             facecolor='#f5f7f9')
    for row, (_, case, status, trajectory) in enumerate(entries):
        snapshots = case[2]['frames']
        # Select the first rear-wheel crossing as the middle view when available.
        crossing = status.get('wheel_crossings', {}).get('wheel_2', {}).get('frame', int(snapshots[len(snapshots) // 2]))
        middle = int(np.argmin(abs(snapshots - crossing)))
        for ax, index in zip(axes[row], [0, middle, len(snapshots) - 1]):
            scene_panel(ax, case, index, limits)
        axes[row, 2].text(.02, .02, status['status'], transform=axes[row, 2].transAxes, fontsize=9)
    fig.suptitle('HSC vehicle passage | saved physical geometry', fontsize=18)
    fig.text(.025, .015, 'Chrono replacement assets. Mounted tires: E = 1e10 Pa. Deformation is not magnified.', fontsize=11)
    fig.subplots_adjust(top=.89, bottom=.08, wspace=.025, hspace=.08)
    fig.savefig(preview / 'passage.png', dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    for _, case, status, trajectory in entries:
        label = f'Ground E = {case[0]["materials"]["loose_E"]:.0e} Pa'
        t = [row['time_s'] for row in trajectory]
        axes[0, 0].plot(t, [r['chassis_center'][0] - trajectory[0]['chassis_center'][0] for r in trajectory], label=label)
        axes[0, 1].plot(t, [r['passage']['clearance_m'] for r in trajectory], label=label)
        axes[1, 0].plot(t, [max(w['max_elastic_displacement_m'] for w in r['wheels'][:4]) * 1000 for r in trajectory], label=label)
        axes[1, 1].plot(t, [max(w['max_elastic_displacement_m'] for w in r['wheels'][4:]) * 1000 for r in trajectory], label=label)
    titles = [('Chassis travel', 'X displacement [m]'), ('Entire car relative to obstacle row', 'Tail clearance [m]'),
              ('Mounted tires: largest elastic displacement', 'Displacement in rim frame [mm]'),
              ('Loose tires: largest elastic displacement', 'Displacement in rim frame [mm]')]
    for ax, (title, ylabel) in zip(axes.flat, titles):
        ax.set(title=title, xlabel='Simulated time [s]', ylabel=ylabel)
        ax.grid(alpha=.22)
        ax.legend()
    axes[0, 1].axhline(.15, color='#777777', lw=1, ls='--')
    fig.savefig(preview / 'comparison.png', dpi=150)
    plt.close(fig)
    if args.animate:
        frame_numbers = sorted(set(int(f) for _, case, _, _ in entries for f in case[2]['frames']))
        selected = np.unique(np.linspace(0, len(frame_numbers) - 1, min(140, len(frame_numbers)), dtype=int))
        images = []
        for position in selected:
            frame = frame_numbers[position]
            fig, axes = plt.subplots(len(entries), 1, figsize=(12, 3.8 * len(entries)), squeeze=False,
                                     dpi=85, facecolor='#f5f7f9')
            for ax, (_, case, status, _) in zip(axes.flat, entries):
                index = int(np.argmin(abs(case[2]['frames'] - frame)))
                scene_panel(ax, case, index, limits)
            fig.suptitle('Native HSC + IPC | full vehicle passage', fontsize=16)
            fig.text(.03, .015, 'Actual saved deformation; fixed camera; no scaling of displacement.', fontsize=10)
            fig.subplots_adjust(top=.91, bottom=.065, hspace=.05)
            fig.canvas.draw()
            images.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()))
            plt.close(fig)
        dt = entries[0][1][0]['config']['dt']
        duration = max(40, round((frame_numbers[-1] - frame_numbers[0]) * dt * 1000 / max(1, len(images) - 1)))
        images[0].save(preview / 'passage.gif', save_all=True, append_images=images[1:], duration=duration, loop=0)
    print(preview)


if __name__ == '__main__':
    main()
