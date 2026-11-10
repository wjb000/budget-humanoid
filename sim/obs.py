"""Observation / action definition shared by the training env (sim/env.py) and the Pi runtime (deploy/pi_runtime.py).
Keeping ONE implementation is what makes the deployed policy see the same numbers it was trained on."""
import numpy as np

N_JOINTS = 19
ACTION_SCALE = 0.4          # rad: target = default_pose + ACTION_SCALE * action
CONTROL_HZ = 50
GAIT_PERIOD_S = 0.7
OBS_DIM = 3 + 3 + N_JOINTS * 3 + 3 + 2   # gyro, gravity, q-q0, qd, last action, command, phase sin/cos = 68


def gravity_in_body(quat_wxyz):
    """Unit gravity direction expressed in the body frame, from the IMU quaternion (w, x, y, z)."""
    w, x, y, z = quat_wxyz
    # third row of R^T applied to (0,0,-1) == -(R[2,:])
    return -np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])


def build_obs(gyro, gravity_b, q, qd, q_default, last_action, cmd, phase):
    return np.concatenate([
        np.asarray(gyro) * 0.25,
        np.asarray(gravity_b),
        np.asarray(q) - np.asarray(q_default),
        np.asarray(qd) * 0.05,
        np.asarray(last_action),
        np.asarray(cmd) * np.array([2.0, 2.0, 0.25]),
        [np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)],
    ]).astype(np.float32)


def advance_phase(phase, cmd, dt=1.0 / CONTROL_HZ):
    """Gait clock only runs while a motion command is active (standing keeps phase 0)."""
    if np.linalg.norm(np.asarray(cmd)) < 0.05:
        return 0.0
    return (phase + dt / GAIT_PERIOD_S) % 1.0


def targets_from_action(action, q_default, lo, hi):
    return np.clip(np.asarray(q_default) + ACTION_SCALE * np.clip(action, -1.0, 1.0), lo, hi)
