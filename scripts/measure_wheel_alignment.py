#!/usr/bin/env python3
"""Measure how far each wheel glb's axle is from the link y axis.

The exported wheel meshes are tilted slightly off their hub axis. When the gym
bridge spins the wheel TF about link y, any residual angle between the mesh
axle and link y shows up as a visible wobble. This script finds the axle from
the mesh geometry (PCA over the vertices - a wheel is thin along its axle) and
prints the exact visual-origin rpy that makes the axle coincide with link y.

Paste the printed values into the WHEEL ALIGNMENT block of
urdf/racecar_mesh.xacro.

Only needs numpy + scipy. Run from the repo root:

    python3 scripts/measure_wheel_alignment.py
"""
import json
import pathlib
import struct
import sys

import numpy as np
from scipy.spatial.transform import Rotation

MESH_DIR = pathlib.Path(__file__).resolve().parent.parent / 'meshes'
WHEELS = [
    'roboracer_wheel_front_left',
    'roboracer_wheel_front_right',
    'roboracer_wheel_rear_left',
    'roboracer_wheel_rear_right',
]

COMPONENT_DTYPES = {5120: np.int8, 5121: np.uint8, 5122: np.int16,
                    5123: np.uint16, 5125: np.uint32, 5126: np.float32}
TYPE_COUNTS = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}


def load_glb(path):
    data = path.read_bytes()
    assert data[:4] == b'glTF', f'{path} is not a glb'
    offset = 12
    gltf, bin_chunk = None, None
    while offset < len(data):
        clen, ctype = struct.unpack_from('<II', data, offset)
        payload = data[offset + 8:offset + 8 + clen]
        if ctype == 0x4E4F534A:  # JSON
            gltf = json.loads(payload)
        elif ctype == 0x004E4942:  # BIN
            bin_chunk = payload
        offset += 8 + clen
    return gltf, bin_chunk


def read_accessor(gltf, bin_chunk, index):
    acc = gltf['accessors'][index]
    view = gltf['bufferViews'][acc['bufferView']]
    dtype = COMPONENT_DTYPES[acc['componentType']]
    ncomp = TYPE_COUNTS[acc['type']]
    start = view.get('byteOffset', 0) + acc.get('byteOffset', 0)
    stride = view.get('byteStride') or ncomp * np.dtype(dtype).itemsize
    out = np.empty((acc['count'], ncomp), dtype=dtype)
    itemsize = ncomp * np.dtype(dtype).itemsize
    for i in range(acc['count']):
        out[i] = np.frombuffer(bin_chunk, dtype=dtype, count=ncomp,
                               offset=start + i * stride)
    return out.astype(np.float64)


def node_matrix(node):
    if 'matrix' in node:
        return np.array(node['matrix'], dtype=np.float64).reshape(4, 4).T
    m = np.eye(4)
    if 'rotation' in node:
        m[:3, :3] = Rotation.from_quat(node['rotation']).as_matrix()
    if 'scale' in node:
        m[:3, :3] = m[:3, :3] @ np.diag(node['scale'])
    if 'translation' in node:
        m[:3, 3] = node['translation']
    return m


def collect_vertices(gltf, bin_chunk):
    """All POSITION vertices with node transforms applied (world = as loaded)."""
    verts = []

    def visit(node_index, parent):
        node = gltf['nodes'][node_index]
        world = parent @ node_matrix(node)
        if 'mesh' in node:
            mesh = gltf['meshes'][node['mesh']]
            for prim in mesh.get('primitives', []):
                if 'POSITION' in prim.get('attributes', {}):
                    v = read_accessor(gltf, bin_chunk, prim['attributes']['POSITION'])
                    vh = np.hstack([v, np.ones((len(v), 1))])
                    verts.append((world @ vh.T).T[:, :3])
        for child in node.get('children', []):
            visit(child, world)

    scene = gltf['scenes'][gltf.get('scene', 0)]
    for root in scene['nodes']:
        visit(root, np.eye(4))
    return np.vstack(verts)


def main():
    print(f'{"wheel":26s} {"tilt":>7s}  correction rpy [deg]  (visual origin, link frame)')
    for name in WHEELS:
        gltf, bin_chunk = load_glb(MESH_DIR / f'{name}.glb')
        verts = collect_vertices(gltf, bin_chunk)
        center = verts.mean(axis=0)
        cov = np.cov((verts - center).T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        axle = eigvecs[:, 0]  # smallest spread = the axle (wheels are thin)
        if axle[1] < 0:
            axle = -axle
        target = np.array([0.0, 1.0, 0.0])
        tilt = np.degrees(np.arccos(np.clip(axle @ target, -1, 1)))
        axis = np.cross(axle, target)
        norm = np.linalg.norm(axis)
        if norm < 1e-12:
            rpy = np.zeros(3)
        else:
            rot = Rotation.from_rotvec(axis / norm * np.radians(tilt))
            rpy = rot.as_euler('xyz', degrees=True)  # URDF rpy convention
        print(f'{name:26s} {tilt:6.2f}°  '
              f'rpy = [{rpy[0]:+7.3f}, {rpy[1]:+7.3f}, {rpy[2]:+7.3f}]  '
              f'center offset = [{center[0]:+.4f}, {center[1]:+.4f}, {center[2]:+.4f}]')


if __name__ == '__main__':
    sys.exit(main())
