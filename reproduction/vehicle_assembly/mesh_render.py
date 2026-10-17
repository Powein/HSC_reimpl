"""Shared, side-effect-free mesh preparation and orthographic rendering."""
from functools import lru_cache
import sys

from common import RESEARCH
import numpy as np
from matplotlib.collections import PolyCollection
from matplotlib.colors import to_rgb

sys.path.insert(0, str(RESEARCH))
from prepare_assets import read_obj

SOURCE = RESEARCH / 'upstream/projectchrono__chrono/data/vehicle/hmmwv'
COLORS = {'chassis': '#698467', 'rim': '#c0955c', 'tire': '#353e46',
          'proxy': '#39a0b0', 'spindle': '#b1c5d1'}


def transform(vertices, matrix):
    return vertices @ matrix[:3, :3].T + matrix[:3, 3]


@lru_cache(maxsize=1)
def chassis_visual():
    return read_obj(SOURCE / 'hmmwv_chassis_simple.obj')


def objects(case, index=-1, visual=True, mounted_only=False):
    manifest, geometry, states = case
    result = []
    for i, body in enumerate(manifest['bodies']):
        name = body['name']
        if mounted_only and name.startswith('loose'):
            continue
        if name == 'chassis' and visual:
            vertices, faces = chassis_visual()
            color = COLORS['chassis']
        else:
            vertices, faces = geometry[f'body_v_{i}'], geometry[f'body_f_{i}']
            color = COLORS['proxy'] if name == 'chassis' else COLORS['rim'] if name.endswith('_rim') else COLORS['spindle']
        result.append((transform(vertices, states['body_transforms'][index, i]), faces, color))
    for i, wheel in enumerate(manifest['wheels']):
        if mounted_only and wheel['name'].startswith('loose'):
            continue
        result.append((states['tire_vertices'][index, i], geometry['tire_faces'], COLORS['tire']))
    return result


def projection(az=-57, el=27):
    az, el = np.deg2rad([az, el])
    camera = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    right = np.array([-np.sin(az), np.cos(az), 0.])
    up = np.cross(camera, right)
    return np.stack([right, up], axis=1), camera


def draw(ax, objects, az=-57, el=27, limits=None, edge=False):
    matrix, camera = projection(az, el)
    triangles, colors = [], []
    light = np.array([-.25, -.5, 1.])
    light /= np.linalg.norm(light)
    for vertices, faces, color in objects:
        tri = vertices[faces]
        normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-20)
        brightness = .4 + .6 * np.abs(normal @ light)
        colors.append(np.clip(np.array(to_rgb(color))[None, :] * brightness[:, None], 0, 1))
        triangles.append(tri)
    tri = np.concatenate(triangles)
    polygons = tri @ matrix
    order = np.argsort((tri @ camera).mean(1))
    ax.add_collection(PolyCollection(polygons[order], facecolors=np.concatenate(colors)[order],
                      edgecolors='#164a54' if edge else 'none', linewidths=.15 if edge else 0))
    lo, hi = polygons.reshape(-1, 2).min(0), polygons.reshape(-1, 2).max(0)
    if limits is None:
        limits = (lo[0] - .2, hi[0] + .2, lo[1] - .25, hi[1] + .25)
    ax.set(xlim=limits[:2], ylim=limits[2:], aspect='equal')
    ax.set_axis_off()
    return matrix, limits
