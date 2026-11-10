"""Software stand-in for the ESP32 bridge: speaks deploy/protocol.py to the Pi runtime and drives the MuJoCo model through the
STS3215 actuator model (sim/actuator.py). It lets the Pi runtime, the framing, the watchdog and an exported policy be tested end to end
with no hardware. This is a test double, not a hardware-in-the-loop result."""
import os, sys, math
import numpy as np
import mujoco
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'sim')); sys.path.insert(0, HERE)
import protocol as P
from env import Budget19Env
from actuator import RAD_PER_COUNT


class FakeESP32:
    """Duck-typed serial port: write() takes Pi bytes, read(n) returns ESP bytes."""
    def __init__(self, seed=0, randomize=False, push_n=0.0, wdog_ms=100.0):
        self.env = Budget19Env('stand', randomize=randomize, seed=seed)
        self.env.reset(seed=seed)
        e = self.env
        self.target = e.q_default.copy(); self.torque_on = False; self.seq = 0
        self.parser = P.Parser(); self.out = bytearray(); self.last_cmd_t = None; self.wdog_ms = wdog_ms; self.wdog = False
        self.t = 0.0; self.push_n = push_n
        self.counts_default = None

    def write(self, data):
        for t, payload in self.parser.feed(bytes(data)):
            if t == P.T_CMD:
                c = P.unpack_cmd(payload); self.seq = c['seq']; self.torque_on = c['torque_on']
                self.target = (np.array(c['counts']) - 2048) * RAD_PER_COUNT
                self.last_cmd_t = self.t; self.wdog = False
                self._advance(1.0 / 50)         # one control period of physics per command (the Pi loop is 50 Hz)
                self.out += self._state()

    def _advance(self, dt):
        e, d, m = self.env, self.env.data, self.env.model
        for _ in range(int(round(dt / m.opt.timestep))):
            if self.last_cmd_t is not None and (self.t - self.last_cmd_t) * 1000 > self.wdog_ms:
                self.wdog = True
            on = self.torque_on and not self.wdog
            tau = e.bank.step(self.target, d.qpos[e.qadr], d.qvel[e.dadr]) if on else np.zeros(e.nu)
            d.ctrl[:] = tau
            d.xfrc_applied[e.base, :3] = (self.push_n if 0.5 < self.t < 0.6 else 0.0), 0, 0
            mujoco.mj_step(m, d); self.t += m.opt.timestep
        d.xfrc_applied[:] = 0

    def advance_without_commands(self, seconds):
        self._advance(seconds)

    def _state(self):
        e, d = self.env, self.env.data
        pos = np.clip(np.round(e.bank.read_position(d.qpos[e.qadr]) / RAD_PER_COUNT + 2048), 0, 4095)
        spd = np.clip(np.round(d.qvel[e.dadr] / RAD_PER_COUNT), -32767, 32767)
        quat = d.sensordata[e.imu_quat:e.imu_quat + 4]; gyro = d.sensordata[e.imu_gyro:e.imu_gyro + 3]
        return P.pack_state(self.seq, int(self.t * 1e6), quat, gyro, P.ST_IMU_OK | (P.ST_WDOG if self.wdog else 0), pos, spd, 9000, 111)

    def read(self, n):
        out, self.out = bytes(self.out[:n]), self.out[n:]
        return out

    def reset_input_buffer(self):
        pass
