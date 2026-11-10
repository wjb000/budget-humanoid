#!/usr/bin/env python3
"""Pi 5 policy runtime: numpy MLP @ 50 Hz -> ESP32 bridge (USB CDC) -> two STS3215 buses.

  python deploy/pi_runtime.py --policy runs/stand0 --port /dev/ttyACM0            # hardware (FIRST: read deploy/README.md bring-up)
  python deploy/pi_runtime.py --policy runs/stand0 --sim --seconds 5               # same code path against deploy/fake_esp32.py

Safety, enforced here (the ESP32 repeats the watchdog independently, and the physical e-stop cuts servo power):
  * targets clamped to the CAD joint limits and rate-limited (--max-step rad per tick)
  * torque only turns on after --arm (a held 'ready' pose ramp over 2 s from the measured pose, never a jump)
  * tilt > 50 deg, IMU lost, e-stop open, stale state (> 60 ms) or ESP watchdog -> torque off
Observation construction is sim/obs.py, the same module the training env uses.
"""
import argparse, json, os, sys, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'sim')); sys.path.insert(0, HERE)
import obs as O
import protocol as P
from actuator import RAD_PER_COUNT


class Policy:
    def __init__(self, run_dir):
        z = np.load(os.path.join(run_dir, 'policy.npz')); self.W = [z[f'W{i}'] for i in range(3)]; self.b = [z[f'b{i}'] for i in range(3)]
        self.meta = json.load(open(os.path.join(run_dir, 'policy_meta.json')))
        assert self.meta['obs_dim'] == O.OBS_DIM and self.meta['action_dim'] == O.N_JOINTS and abs(self.meta['action_scale'] - O.ACTION_SCALE) < 1e-9
        self.q_default = np.array(self.meta['default_pose']); self.lo = np.array(self.meta['joint_lo']); self.hi = np.array(self.meta['joint_hi'])

    def __call__(self, obs):
        h = np.tanh(self.W[0] @ obs + self.b[0]); h = np.tanh(self.W[1] @ h + self.b[1])
        return np.clip(self.W[2] @ h + self.b[2], -1, 1)


class Calib:
    """Per-servo direction (+1/-1) and zero offset in counts, found at bring-up (deploy/README.md step 4). Default: identity.
    counts = 2048 + offset + direction * rad * 4096/(2 pi)"""
    def __init__(self, path=None):
        d = json.load(open(path)) if path else {}
        self.dir = np.array(d.get('direction', [1] * O.N_JOINTS), float); self.off = np.array(d.get('offset_counts', [0] * O.N_JOINTS), float)

    def to_counts(self, rad):
        return np.clip(np.round(2048 + self.off + self.dir * np.asarray(rad) / RAD_PER_COUNT), 0, 4095)

    def to_rad(self, counts):
        return self.dir * (np.asarray(counts, float) - 2048 - self.off) * RAD_PER_COUNT


class Link:
    """Frames over a serial-like object (pyserial Serial or FakeESP32)."""
    def __init__(self, ser):
        self.ser, self.parser, self.seq, self.state, self.t_state = ser, P.Parser(), 0, None, 0.0

    def send(self, torque_on, counts):
        self.seq = (self.seq + 1) & 0xFFFF
        self.ser.write(P.pack_cmd(self.seq, torque_on, counts))

    def poll(self):
        data = self.ser.read(4096) if hasattr(self.ser, 'read') else b''
        for t, payload in self.parser.feed(data):
            if t == P.T_STATE:
                self.state = P.unpack_state(payload); self.t_state = time.monotonic()
        return self.state


def run(args, ser=None, clock=time.monotonic, sleep=time.sleep, log=print):
    pol = Policy(args.policy); cal = Calib(getattr(args, 'calib', None))
    link = Link(ser)
    dt = 1.0 / O.CONTROL_HZ
    q_cmd = None; last_action = np.zeros(O.N_JOINTS); phase = 0.0; cmd = np.array(args.cmd, float)
    torque_on = False; ramp_t = 0.0; q_start = None; reason = 'finished'; tilt = None
    n = int(args.seconds * O.CONTROL_HZ); t_next = clock(); stats = []
    link.send(False, np.full(O.N_JOINTS, 2048))
    for k in range(n):
        st = link.poll()
        if st is None:
            sleep(dt); link.send(False, np.full(O.N_JOINTS, 2048)); continue
        q = cal.to_rad(st['pos'])
        qd = cal.dir * np.array(st['speed']) * RAD_PER_COUNT
        g = O.gravity_in_body(np.array(st['quat']))
        ok = bool(st['status'] & P.ST_IMU_OK) and not (st['status'] & (P.ST_ESTOP | P.ST_WDOG))
        tilt = np.degrees(np.arccos(np.clip(-g[2], -1, 1)))
        if torque_on and (not ok or tilt > args.max_tilt_deg or (clock() - link.t_state) > 0.06):
            torque_on = False; reason = f'torque off: ok={ok} tilt={tilt:.0f} deg'; log(reason); break
        if q_cmd is None:
            q_cmd = q.copy(); q_start = q.copy()
        if args.arm and not torque_on and k > 5:
            torque_on = True; ramp_t = 0.0
        t0 = time.perf_counter()
        if torque_on:
            ramp_t += dt; a = min(1.0, ramp_t / 2.0)        # 2 s blend from the measured pose into the policy
            obs = O.build_obs(st['gyro'], g, q, qd, pol.q_default, last_action, cmd, phase)
            action = pol(obs)
            tgt = O.targets_from_action(action, pol.q_default, pol.lo, pol.hi)
            tgt = (1 - a) * q_start + a * tgt
            q_cmd = q_cmd + np.clip(tgt - q_cmd, -args.max_step, args.max_step)
            last_action = action; phase = O.advance_phase(phase, cmd, dt)
        stats.append(time.perf_counter() - t0)
        counts = cal.to_counts(q_cmd)
        link.send(torque_on, counts)
        t_next += dt
        s = t_next - clock()
        if s > 0: sleep(s)
    link.send(False, np.full(O.N_JOINTS, 2048))
    log(f'runtime stopped ({reason}); policy compute mean {1000*np.mean(stats):.3f} ms, max {1000*np.max(stats):.3f} ms over {len(stats)} ticks' if stats else f'runtime stopped ({reason}); no ticks')
    return dict(reason=reason, ticks=len(stats), tilt_deg=None if tilt is None else float(tilt))


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--policy', required=True); ap.add_argument('--port'); ap.add_argument('--sim', action='store_true')
    ap.add_argument('--seconds', type=float, default=10.0); ap.add_argument('--arm', action='store_true', help='actually turn torque on')
    ap.add_argument('--cmd', type=float, nargs=3, default=[0, 0, 0]); ap.add_argument('--max-step', type=float, default=0.08)
    ap.add_argument('--max-tilt-deg', type=float, default=50.0); ap.add_argument('--calib', default=None, help='json with direction[19], offset_counts[19]')
    a = ap.parse_args()
    if a.sim:
        from fake_esp32 import FakeESP32
        run(a, FakeESP32(randomize=True))
    else:
        import serial
        run(a, serial.Serial(a.port, 921600, timeout=0.001))
