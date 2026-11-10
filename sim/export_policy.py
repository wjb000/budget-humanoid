#!/usr/bin/env python3
"""Export the PPO actor (deterministic mean action) to plain numpy so the Pi needs only numpy, not torch/SB3.
python sim/export_policy.py runs/walk0/model.zip runs/walk0   ->  policy.npz + policy_meta.json"""
import json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import obs as O

def export(model_zip, out_dir):
    from stable_baselines3 import PPO
    m = PPO.load(model_zip, device='cpu')
    sd = {k: v.cpu().numpy() for k, v in m.policy.state_dict().items()}
    keys = [('mlp_extractor.policy_net.0', ), ('mlp_extractor.policy_net.2', ), ('action_net', )]
    arrs = {}
    for i, (k,) in enumerate(keys):
        arrs[f'W{i}'] = sd[k + '.weight'].astype(np.float32); arrs[f'b{i}'] = sd[k + '.bias'].astype(np.float32)
    np.savez(os.path.join(out_dir, 'policy.npz'), **arrs)
    pose = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'default_pose.json')))
    jm = [l.strip().split(',') for l in open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'joint_map.csv'))][1:]
    ids = {r[1]: int(r[0]) for r in jm}
    import mujoco
    mm = mujoco.MjModel.from_xml_path(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'budget19.xml'))
    names = [mm.actuator(i).name for i in range(mm.nu)]
    meta = dict(obs_dim=O.OBS_DIM, action_dim=O.N_JOINTS, action_scale=O.ACTION_SCALE, control_hz=O.CONTROL_HZ, gait_period_s=O.GAIT_PERIOD_S,
                activation='tanh', layers=[arrs['W0'].shape[0], arrs['W1'].shape[0], arrs['W2'].shape[0]],
                joint_order=names, servo_ids=[ids[n] for n in names], default_pose=[pose[n] for n in names],
                joint_lo=[float(x) for x in mm.actuator_ctrlrange[:, 0]], joint_hi=[float(x) for x in mm.actuator_ctrlrange[:, 1]])
    json.dump(meta, open(os.path.join(out_dir, 'policy_meta.json'), 'w'), indent=1)
    return meta

if __name__ == '__main__':
    export(sys.argv[1], sys.argv[2]); print('exported to', sys.argv[2])
