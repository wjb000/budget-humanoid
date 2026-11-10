#!/usr/bin/env python3
"""Tessellate + decimate vendor STEP files into light visual meshes (cad/.cache/*.npz) used for the viewer/GLB.
Uses cad/vendor/ (committed) and cad/vendor_fetched/ (python cad/fetch_vendor.py). Missing Pi 5 -> envelope box is used instead."""
import os, sys, time, numpy as np, trimesh, cadquery as cq
HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, '.cache'); os.makedirs(CACHE, exist_ok=True)
V, F = os.path.join(HERE, 'vendor'), os.path.join(HERE, 'vendor_fetched')

def cluster(m, cell):
    """vertex clustering decimation (robust on non-manifold vendor meshes)"""
    q = np.round(m.vertices / cell).astype(np.int64)
    keys, first, inv = np.unique(q, axis=0, return_index=True, return_inverse=True)
    inv = inv.reshape(-1)
    verts = np.zeros((len(keys), 3)); cnt = np.zeros(len(keys))
    np.add.at(verts, inv, m.vertices); np.add.at(cnt, inv, 1); verts /= cnt[:, None]
    f = inv[m.faces]
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    return trimesh.Trimesh(verts, f, process=True)

def run(name, path, tol, faces=None, cell=None):
    if not os.path.exists(path):
        print('skip', name, '(missing', path + ')'); return
    t = time.time()
    s = cq.importers.importStep(path).val()
    v, f = s.tessellate(tol, 0.5)
    m = trimesh.Trimesh(np.array([[p.x, p.y, p.z] for p in v]), np.array(f), process=True); m.merge_vertices()
    n0 = len(m.faces)
    if cell:
        m = cluster(m, cell)
    elif faces and n0 > faces:
        m = m.simplify_quadric_decimation(face_count=faces)
    print(f'{name}: {n0} -> {len(m.faces)} faces ({time.time()-t:.0f}s)')
    np.savez(os.path.join(CACHE, f'{name}_dec.npz'), v=m.vertices, f=m.faces)

if __name__ == '__main__':
    run('sts3215', f'{V}/STS3215_03a.step', 0.15, 2500)
    run('bno085', f'{V}/adafruit_4754_BNO085.step', 0.15, 800)
    run('max98357', f'{V}/adafruit_3006_MAX98357.step', 0.15, 500)
    run('speaker40', f'{V}/adafruit_3968_speaker_40mm.step', 0.2, 600)
    run('pi5', f'{F}/rpi5_no_graphics.step', 0.3, cell=2.5)
