#!/usr/bin/env python3
"""Load budget19.xml in MuJoCo and check the robot can stand (position-hold at the zero pose) for a few seconds.
Usage: pip install mujoco numpy; python sim/check_mujoco.py [--seconds 5] [--kp 8 --kv 0.35] [--render out.png]"""
import argparse, os, sys, numpy as np, mujoco

ap = argparse.ArgumentParser()
ap.add_argument('--seconds', type=float, default=5.0)
ap.add_argument('--kp', type=float, default=None)
ap.add_argument('--kv', type=float, default=None)
ap.add_argument('--derate', type=float, default=1.0, help='fraction of datasheet stall torque you actually measured on a real servo (BRIDGE paper uses calibrated, not datasheet, limits); utilization is reported against it')
ap.add_argument('--render', default=None, help='write a PNG of the final pose (needs EGL/OSMesa)')
a = ap.parse_args()

here = os.path.dirname(os.path.abspath(__file__))
m = mujoco.MjModel.from_xml_path(os.path.join(here, 'budget19.xml'))
d = mujoco.MjData(m)
if a.kp is not None:
    m.actuator_gainprm[:, 0] = a.kp; m.actuator_biasprm[:, 1] = -a.kp
if a.kv is not None:
    m.actuator_biasprm[:, 2] = -a.kv
print(f'model: {m.nbody} bodies, {m.njnt} joints (1 free), {m.nu} actuators, total mass {m.body_mass.sum():.3f} kg')
mujoco.mj_resetDataKeyframe(m, d, 0)
mujoco.mj_forward(m, d)
base = m.body('base').id
com0 = d.subtree_com[base].copy()
print('initial COM (m):', np.round(com0, 3))
steps = int(a.seconds / m.opt.timestep)
zs, tilts, tq = [], [], []
fid = [m.geom(n).id for n in ('r_foot_sole', 'l_foot_sole')]
for i in range(steps):
    mujoco.mj_step(m, d)
    if i % 50 == 0:
        com = d.subtree_com[base]
        R = d.xmat[base].reshape(3, 3)
        tilts.append(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
        zs.append(com[2])
        tq.append(np.abs(d.actuator_force) / m.actuator_forcerange[:, 1])
zs, tilts, tq = np.array(zs), np.array(tilts), np.array(tq)
com1 = d.subtree_com[base]
contacts = [(m.geom(c.geom1).name, m.geom(c.geom2).name) for c in d.contact[:d.ncon]]
feet_ok = all(any(f in c for c in contacts) for f in ('r_foot_sole', 'l_foot_sole'))
sag = (com0[2] - zs.min()) * 1000
print(f'after {a.seconds:.1f} s: COM z {com1[2]:.3f} m (start {com0[2]:.3f}, max sag {sag:.1f} mm), COM xy drift {np.linalg.norm(com1[:2]-com0[:2])*1000:.1f} mm')
print(f'torso tilt: final {tilts[-1]:.2f} deg, max {tilts.max():.2f} deg; both feet in contact: {feet_ok}')
worst = tq.max(0)
names = [m.actuator(i).name for i in range(m.nu)]
top = sorted(zip(worst, names), reverse=True)[:5]
worst = worst / a.derate
top = sorted(zip(worst, names), reverse=True)[:5]
print(f'peak actuator load (fraction of per-servo stall torque x {a.derate:g} usable):', ', '.join(f'{n} {w:.2f}' for w, n in top))
sat = [n for w, n in zip(worst, names) if w > 0.8]
print('high-load joints (peak load > 0.8, joints near stall in this test; confirm across seeds and motions):', ', '.join(sat) if sat else 'none in this stand test')
print('settled actuator load (last 1 s mean):', ', '.join(f'{n} {w:.2f}' for w, n in sorted(zip(tq[-20:].mean(0), names), reverse=True)[:5]))
stood = (sag < 30) and tilts.max() < 5 and feet_ok
print('RESULT:', 'PASS - stands (position hold at zero pose)' if stood else 'FAIL - did not stand')
if a.render:
    r = mujoco.Renderer(m, 480, 360); r.update_scene(d, camera=-1); import PIL.Image as I; I.fromarray(r.render()).save(a.render)
sys.exit(0 if stood else 1)
