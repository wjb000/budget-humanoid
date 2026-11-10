"""Gymnasium + MuJoCo environment for Budget-19: `stand` (hold balance under pushes) and `walk` (track a forward/yaw command).

Plain MuJoCo (no MJX/Isaac), 50 Hz policy, 500 Hz physics, the STS3215 model from sim/actuator.py between policy and joints,
domain randomisation from sim/domain_randomization.json. The MJCF position actuators are re-purposed as torque motors so the
Python actuator model (latency, deadband, torque-speed curve, supply sag) is what drives the joints.

Honest scope: only the foot soles collide (torso/arms/head have no collision geoms yet), so episodes end on tilt/height rather
than on impact. Nothing here has been validated against hardware. See sim/SIM2REAL.md.
"""
import json, os
import numpy as np
import mujoco
import gymnasium as gym
from gymnasium import spaces
from actuator import STS3215Bank, STS3215Params
import obs as O

HERE = os.path.dirname(os.path.abspath(__file__))


def load_json(name):
    with open(os.path.join(HERE, name)) as f:
        return json.load(f)


class Budget19Env(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, task='walk', episode_s=10.0, randomize=True, seed=0, dr_path='domain_randomization.json'):
        assert task in ('stand', 'walk')
        self.task, self.randomize = task, randomize
        self.model = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'budget19.xml'))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.dr = load_json(dr_path)
        self.rng = np.random.default_rng(seed)
        self.nu = m.nu
        assert self.nu == O.N_JOINTS
        self.act_names = [m.actuator(i).name for i in range(self.nu)]
        self.jid = [m.actuator_trnid[i, 0] for i in range(self.nu)]
        self.qadr = np.array([m.jnt_qposadr[j] for j in self.jid]); self.dadr = np.array([m.jnt_dofadr[j] for j in self.jid])
        self.lo = m.actuator_ctrlrange[:, 0].copy(); self.hi = m.actuator_ctrlrange[:, 1].copy()   # joint limits from the MJCF
        pose = load_json('default_pose.json')
        self.q_default = np.array([pose[n] for n in self.act_names])
        # re-purpose position actuators as torque motors (gain 1, no bias); the torque clamp is the datasheet stall
        m.actuator_gainprm[:, 0] = 1.0; m.actuator_gainprm[:, 1:] = 0.0
        m.actuator_biasprm[:] = 0.0; m.actuator_biastype[:] = 0
        m.actuator_ctrllimited[:] = 1; m.actuator_ctrlrange[:] = np.array([-2.94, 2.94])
        m.actuator_forcelimited[:] = 1; m.actuator_forcerange[:] = np.array([-2.94, 2.94])
        # nominal values for DR
        self._nom = dict(mass=m.body_mass.copy(), inertia=m.body_inertia.copy(), fl=m.dof_frictionloss.copy(),
                         damp=m.dof_damping.copy(), arm=m.dof_armature.copy(), fric=m.geom_friction.copy(),
                         ipos=m.body_ipos.copy())
        self.base = m.body('base').id
        self.sole_ids = [m.geom('r_foot_sole').id, m.geom('l_foot_sole').id]
        self.floor = m.geom('floor').id
        self.imu_quat = m.sensor('imu_quat').adr[0]; self.imu_gyro = m.sensor('imu_gyro').adr[0]
        self.substeps = int(round(1.0 / O.CONTROL_HZ / m.opt.timestep))
        self.max_steps = int(episode_s * O.CONTROL_HZ)
        self.bank = STS3215Bank(self.nu, m.opt.timestep, STS3215Params(), self.rng)
        self.action_space = spaces.Box(-1.0, 1.0, (self.nu,), np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf, (O.OBS_DIM,), np.float32)
        self.base_z0 = None
        self._gtilt = np.zeros(3)

    # ------------------------------------------------------------------------------------------------------------
    def _randomize(self):
        m, nom, dr, rng = self.model, self._nom, self.dr, self.rng
        u = lambda r, n=None: rng.uniform(r[0], r[1], n)
        m.body_mass[:] = nom['mass']; m.body_inertia[:] = nom['inertia']; m.dof_frictionloss[:] = nom['fl']
        m.dof_damping[:] = nom['damp']; m.dof_armature[:] = nom['arm']; m.geom_friction[:] = nom['fric']; m.body_ipos[:] = nom['ipos']
        p = STS3215Params()
        self._gtilt = np.zeros(3)
        if self.randomize and dr.get('enabled', True):
            s = u(dr['body_mass']['range'], m.nbody); s[0] = 1.0
            m.body_mass[:] = nom['mass'] * s; m.body_inertia[:] = nom['inertia'] * s[:, None]
            m.body_ipos[self.base] += u(dr['com_offset_m']['range'], 3)
            idx = self.dadr
            m.dof_frictionloss[idx] = u(dr['joint_frictionloss']['range'], self.nu)
            m.dof_damping[idx] = u(dr['joint_damping']['range'], self.nu)
            m.dof_armature[idx] = nom['arm'][idx] * u(dr['joint_armature']['range'], self.nu)
            m.geom_friction[self.sole_ids, 0] = u(dr['floor_friction']['range'], 2)
            p.latency_s = u(dr['latency_s']['range']); p.backlash_rad = u(dr['backlash_rad']['range'])
            p.v_batt = u(dr['v_batt']['range']); p.r_batt = u(dr['r_batt']['range'])
            self.bank = STS3215Bank(self.nu, m.opt.timestep, p, rng)
            self.bank.kp = p.kp * u(dr['kp']['range'], self.nu); self.bank.kd = p.kd * u(dr['kd']['range'], self.nu)
            self.bank.tau_stall = p.tau_stall * u(dr['tau_stall']['range'], self.nu)
            self.bank.zero_offset = u(dr['zero_offset_rad']['range'], self.nu)
            ang = np.radians(u(dr['gravity_tilt_deg']['range'])); az = rng.uniform(0, 2 * np.pi)
            self._gtilt = np.array([np.sin(az), np.cos(az), 0.0]) * np.sin(ang)
        else:
            self.bank = STS3215Bank(self.nu, m.opt.timestep, p, rng)
        self.push_t = rng.uniform(*dr['push']['interval_s']) if (self.randomize and dr.get('enabled', True)) else 1e9

    # ------------------------------------------------------------------------------------------------------------
    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        q0 = self.q_default + (self.rng.uniform(-0.05, 0.05, self.nu) if self.randomize else 0.0)
        q0 = np.clip(q0, self.lo, self.hi)
        d.qpos[self.qadr] = q0; d.qpos[3:7] = [1, 0, 0, 0]; d.qpos[0:2] = 0
        mujoco.mj_forward(m, d)
        sole_bottom = min(d.geom_xpos[g][2] - m.geom_size[g][2] for g in self.sole_ids)
        d.qpos[2] += -sole_bottom + 0.0005
        mujoco.mj_forward(m, d)
        self._randomize()
        mujoco.mj_forward(m, d)
        self.base_z0 = d.subtree_com[self.base][2]       # whole-robot COM height (the base frame sits at floor level at the zero pose)
        self.bank.reset(d.qpos[self.qadr])
        self.last_action = np.zeros(self.nu); self.steps = 0; self.phase = 0.0; self.t = 0.0
        self.cmd = self._sample_cmd()
        return self._obs(), {}

    def _sample_cmd(self):
        if self.task == 'stand':
            return np.zeros(3)
        r = self.rng
        return np.array([r.uniform(0.0, 0.25), r.uniform(-0.1, 0.1), r.uniform(-0.3, 0.3)]) if r.random() > 0.15 else np.zeros(3)

    def _obs(self):
        d = self.data
        n = self.dr['obs_noise'] if self.randomize else {k: 0.0 for k in ('gyro_rad_s', 'gravity', 'joint_pos_rad', 'joint_vel_rad_s')}
        g = O.gravity_in_body(d.sensordata[self.imu_quat:self.imu_quat + 4]) + self._gtilt
        g = g / np.linalg.norm(g)
        gyro = d.sensordata[self.imu_gyro:self.imu_gyro + 3] + self.rng.normal(0, n['gyro_rad_s'], 3)
        g = g + self.rng.normal(0, n['gravity'], 3)
        q = self.bank.read_position(d.qpos[self.qadr]) + self.rng.normal(0, n['joint_pos_rad'], self.nu)
        qd = d.qvel[self.dadr] + self.rng.normal(0, n['joint_vel_rad_s'], self.nu)
        # readings are in the servo frame (include zero offset), which is what the real policy would see
        return O.build_obs(gyro, g, q, qd, self.q_default, self.last_action, self.cmd, self.phase)

    def _contacts(self):
        d, f = self.data, [False, False]
        for i in range(d.ncon):
            c = d.contact[i]
            for k, g in enumerate(self.sole_ids):
                if (c.geom1 == g and c.geom2 == self.floor) or (c.geom2 == g and c.geom1 == self.floor):
                    f[k] = True
        return f

    def step(self, action):
        m, d = self.model, self.data
        action = np.clip(np.asarray(action, np.float64), -1, 1)
        target = O.targets_from_action(action, self.q_default, self.lo, self.hi)
        tau_sum = np.zeros(self.nu)
        for _ in range(self.substeps):
            tau = self.bank.step(target, d.qpos[self.qadr], d.qvel[self.dadr])
            d.ctrl[:] = tau
            d.xfrc_applied[self.base, :3] = self._push_force()
            mujoco.mj_step(m, d)
            tau_sum += tau ** 2
            self.t += m.opt.timestep
        d.xfrc_applied[:] = 0
        self.steps += 1
        tau_rms = np.sqrt(tau_sum / self.substeps)
        self.phase = O.advance_phase(self.phase, self.cmd)
        R = d.xmat[self.base].reshape(3, 3)
        up_z = R[2, 2]
        z = d.subtree_com[self.base][2]
        fallen = (up_z < 0.6) or (z < 0.6 * self.base_z0) or (not np.all(np.isfinite(d.qpos)))
        reward = self._reward(action, tau_rms, R, fallen)
        self.last_action = action
        if self.task == 'walk' and self.steps % 150 == 0:
            self.cmd = self._sample_cmd()
        obs = self._obs()
        info = {'tau_rms_max_frac_rated': float(tau_rms.max() / self.bank.p.tau_rated), 'z': float(z)}
        return obs, float(reward), bool(fallen), self.steps >= self.max_steps, info

    def _push_force(self):
        if self.t < self.push_t:
            return np.zeros(3)
        pc = self.dr['push']
        if self.t > self.push_t + pc['duration_s']:
            self.push_t = self.t + self.rng.uniform(*pc['interval_s'])
            return np.zeros(3)
        if not hasattr(self, '_pf') or self._pf_t != self.push_t:
            a = self.rng.uniform(0, 2 * np.pi); self._pf = self.rng.uniform(*pc['force_n']) * np.array([np.cos(a), np.sin(a), 0.0]); self._pf_t = self.push_t
        return self._pf

    def _reward(self, action, tau_rms, R, fallen):
        d = self.data
        yaw = np.arctan2(R[1, 0], R[0, 0]); c, s = np.cos(yaw), np.sin(yaw)
        v = d.qvel[0:3]; vx, vy = c * v[0] + s * v[1], -s * v[0] + c * v[1]
        wz = d.qvel[5]
        gz = R[2, :]       # body z axis in world; upright => (0,0,1)
        r = 0.0
        r += 1.5 * np.exp(-((vx - self.cmd[0]) ** 2 + (vy - self.cmd[1]) ** 2) / 0.04)
        r += 0.5 * np.exp(-((wz - self.cmd[2]) ** 2) / 0.1)
        r += 0.3                                                             # alive
        r -= 2.0 * (gz[0] ** 2 + gz[1] ** 2)                                 # upright
        r -= 20.0 * (d.subtree_com[self.base][2] - self.base_z0) ** 2                          # keep pelvis height
        r -= 0.05 * np.sum((action - self.last_action) ** 2)                 # smoothness
        r -= 0.01 * np.sum(tau_rms ** 2)                                     # effort
        keep = [i for i, n in enumerate(self.act_names) if any(k in n for k in ('hip_yaw', 'hip_roll', 'ankle_roll', 'shoulder', 'elbow', 'gripper', 'neck'))]
        r -= 0.3 * np.sum((d.qpos[self.qadr][keep] - self.q_default[keep]) ** 2)
        if self.task == 'walk' and np.linalg.norm(self.cmd) > 0.05:
            want = [np.sin(2 * np.pi * self.phase) < 0, np.sin(2 * np.pi * self.phase) >= 0]   # right foot in stance for the first half
            got = self._contacts()
            r += 0.3 * sum(float(w == g) for w, g in zip(want, got))
        else:
            r += 0.3 * float(all(self._contacts()))
        if fallen:
            r -= 5.0
        return r
