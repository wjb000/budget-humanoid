#!/usr/bin/env python3
"""Evaluate an exported numpy policy (policy.npz) against the do-nothing baseline (hold the default pose) in sim/env.py.
python sim/eval_policy.py runs/walk0 --task walk --episodes 20 [--no-dr]
Reports survival (fraction of episodes reaching the time limit), mean episode length, mean forward speed and command tracking."""
import argparse, json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from env import Budget19Env
import obs as O

def rollout(env, policy, seed):
    o, _ = env.reset(seed=seed); n = 0; vx = []; trk = []
    while True:
        a = policy(o) if policy else np.zeros(O.N_JOINTS)
        o, r, term, trunc, info = env.step(a); n += 1
        R = env.data.xmat[env.base].reshape(3, 3); yaw = np.arctan2(R[1, 0], R[0, 0])
        v = env.data.qvel[:2]; vf = np.cos(yaw) * v[0] + np.sin(yaw) * v[1]
        vx.append(vf); trk.append(abs(vf - env.cmd[0]))
        if term or trunc: return n, term, np.mean(vx), np.mean(trk), info['tau_rms_max_frac_rated']

def main():
    ap = argparse.ArgumentParser(); ap.add_argument('run'); ap.add_argument('--task', default='walk'); ap.add_argument('--episodes', type=int, default=20)
    ap.add_argument('--no-dr', action='store_true'); ap.add_argument('--json', default=None); a = ap.parse_args()
    pn = os.path.join(a.run, 'policy.npz')
    z = np.load(pn) if os.path.exists(pn) else {k: np.array(v, np.float32) for k, v in json.load(open(os.path.join(a.run, 'policy.json'))).items()}
    W = [z[f'W{i}'] for i in range(3)]; b = [z[f'b{i}'] for i in range(3)]
    pol = lambda o: np.clip(W[2] @ np.tanh(W[1] @ np.tanh(W[0] @ o + b[0]) + b[1]) + b[2], -1, 1)
    res = {}
    for name, p in (('policy', pol), ('hold_default_pose', None)):
        env = Budget19Env(a.task, randomize=not a.no_dr, seed=123)
        rows = [rollout(env, p, 1000 + i) for i in range(a.episodes)]
        n, term, vx, trk, tau = zip(*rows)
        res[name] = dict(episodes=a.episodes, survived_frac=float(1 - np.mean(term)), mean_len_s=float(np.mean(n) / O.CONTROL_HZ), mean_forward_speed_m_s=float(np.mean(vx)),
                         mean_abs_speed_error_m_s=float(np.mean(trk)), peak_rms_torque_frac_of_rated=float(np.max(tau)))
        print(name, json.dumps(res[name]))
    if a.json: json.dump(res, open(a.json, 'w'), indent=1)

if __name__ == '__main__':
    main()
