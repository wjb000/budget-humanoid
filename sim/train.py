#!/usr/bin/env python3
"""Train a stand/walk policy for Budget-19 with stable-baselines3 PPO on sim/env.py (plain MuJoCo, CPU).
pip install mujoco numpy scipy gymnasium stable-baselines3 torch
python sim/train.py --task walk --steps 2000000 --envs 8 --out runs/walk0
Writes: <out>/model.zip, <out>/progress.csv (episode return/length vs steps), <out>/policy.npz + policy_meta.json (numpy, for the Pi).
This is a plain baseline to prove the loop (env -> PPO -> export -> Pi runtime) works, not a tuned walking controller."""
import argparse, csv, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import BaseCallback
import torch
from env import Budget19Env

class Log(BaseCallback):
    def __init__(self, path):
        super().__init__(); self.path = path; self.rows = []; self.t0 = time.time(); self.ret = None
    def _on_step(self):
        for info, done in zip(self.locals['infos'], self.locals['dones']):
            if 'episode' in info:
                self.rows.append((self.num_timesteps, info['episode']['r'], info['episode']['l'], time.time() - self.t0))
        return True
    def _on_training_end(self):
        with open(self.path, 'w', newline='') as f:
            w = csv.writer(f); w.writerow(['timesteps', 'episode_return', 'episode_len', 'wall_s']); w.writerows(self.rows)

def make(task, seed):
    def f():
        from stable_baselines3.common.monitor import Monitor
        return Monitor(Budget19Env(task, seed=seed))
    return f

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--task', default='walk', choices=['stand', 'walk']); ap.add_argument('--steps', type=int, default=200_000)
    ap.add_argument('--envs', type=int, default=8); ap.add_argument('--out', default='runs/test'); ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True); torch.set_num_threads(2)
    venv = SubprocVecEnv([make(a.task, a.seed * 100 + i) for i in range(a.envs)])
    model = PPO('MlpPolicy', venv, n_steps=256, batch_size=2048, n_epochs=5, gamma=0.99, gae_lambda=0.95, learning_rate=3e-4, clip_range=0.2,
                ent_coef=0.0, policy_kwargs=dict(net_arch=dict(pi=[128, 128], vf=[128, 128]), activation_fn=torch.nn.Tanh, log_std_init=-1.5), seed=a.seed, verbose=1, device='cpu')
    t = time.time()
    model.learn(total_timesteps=a.steps, callback=Log(os.path.join(a.out, 'progress.csv')))
    print(f'trained {a.steps} steps in {time.time() - t:.0f} s ({a.steps / (time.time() - t):.0f} steps/s)')
    model.save(os.path.join(a.out, 'model'))
    from export_policy import export
    export(os.path.join(a.out, 'model.zip'), a.out)
