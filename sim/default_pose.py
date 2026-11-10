#!/usr/bin/env python3
"""Solve the bent-knee default ("ready") pose used as the policy's action offset.

For a chosen knee flexion the hip pitch and ankle pitch are solved so that (a) each sole is parallel to the floor and
(b) the whole-robot COM sits over the middle of the sole box. Output: sim/default_pose.json (joint name -> rad).
Uses only budget19.xml, so it is valid for whatever CAD revision exported that file.
Usage: python sim/default_pose.py [--knee 0.45]"""
import argparse, json, os, numpy as np, mujoco
from scipy.optimize import least_squares

here = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser(); ap.add_argument('--knee', type=float, default=0.45); a = ap.parse_args()
m = mujoco.MjModel.from_xml_path(os.path.join(here, 'budget19.xml')); d = mujoco.MjData(m)
jid = lambda n: m.joint(n).qposadr[0]
base = m.body('base').id

def fk(q):
    d.qpos[:] = 0; d.qpos[3] = 1
    for k, v in q.items(): d.qpos[jid(k)] = v
    mujoco.mj_forward(m, d)

pose = {}
sign = {'r': -1.0, 'l': 1.0}   # knee flexion sign (left joint axis is the y-flipped copy of the right one)
soles = {s: m.geom(f'{s}_foot_sole').id for s in 'rl'}
feet = {s: m.geom(f'{s}_foot_sole').bodyid[0] for s in 'rl'}
def res(x):
    q = {}
    for k, s in enumerate('rl'):
        q[f'{s}_knee'] = sign[s] * a.knee; q[f'{s}_hip_pitch'] = x[2 * k]; q[f'{s}_ankle_pitch'] = x[2 * k + 1]
    fk(q)
    com = d.subtree_com[base]
    r = [com[0] - 0.5 * (d.geom_xpos[soles['r']][0] + d.geom_xpos[soles['l']][0])]
    for s in 'rl':
        r.append(d.xmat[feet[s]].reshape(3, 3)[2, 0] * 0.2)  # sole pitch
    return np.array(r) * 20.0
best = None
for h0 in (-0.5, 0.3, 0.6):
    sol = least_squares(res, [h0, 0.0, -h0, 0.0])
    if best is None or sol.cost < best.cost: best = sol
for k, s in enumerate('rl'):
    pose[f'{s}_hip_pitch'], pose[f'{s}_ankle_pitch'] = float(best.x[2 * k]), float(best.x[2 * k + 1])
    pose[f'{s}_knee'] = sign[s] * a.knee
print('residual cost', best.cost, {k: round(v, 3) for k, v in pose.items()})
# arms slightly forward of the hanging pose are not needed; keep everything else at zero
for i in range(m.nu):
    pose.setdefault(m.actuator(i).name, 0.0)
q = dict(pose); fk(q)
json.dump({k: round(v, 5) for k, v in pose.items()}, open(os.path.join(here, 'default_pose.json'), 'w'), indent=1)
