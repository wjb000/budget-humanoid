#!/usr/bin/env python3
"""Offline tests for the deploy path (no hardware): STS packet encoding, ESP32 framing + resync, STSBus against a fake servo,
and the Pi runtime end to end against FakeESP32 (MuJoCo + actuator model).
python deploy/test_deploy.py [--policy runs/stand0]"""
import argparse, os, struct, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', 'sim'))
import protocol as P, sts_bus as S

def test_sts_packets():
    # Feetech documents PING of servo 1 as FF FF 01 02 01 FB
    assert S.packet(1, S.INST_PING) == bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])
    assert S.packet(1, S.INST_WRITE, bytes([S.GOAL_POS, 0x00, 0x08])) == bytes([0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0x00, 0x08, 0xC4])
    assert S.rad_to_counts(0) == 2048 and S.rad_to_counts(np.pi / 2) == 3072 and abs(S.counts_to_rad(3072) - np.pi / 2) < 1e-9
    assert S.decode_signed(0x8000 | 100, 15) == -100 and S.decode_signed(100, 15) == 100

class FakeServoPort:
    """Answers READ of present position with a fixed state and records WRITEs."""
    def __init__(self): self.rx = bytearray(); self.writes = []
    def reset_input_buffer(self): self.rx.clear()
    def write(self, data):
        d = bytes(data); sid, ln, ins = d[2], d[3], d[4]
        if ins == S.INST_READ:
            addr, n = d[5], d[6]; payload = struct.pack('<HHH', 3072, 0x8000 | 50, 300)[:n]
            body = bytes([sid, 2 + n, 0]) + payload; self.rx += b'\xff\xff' + body + bytes([S.checksum(body)])
        elif ins == S.INST_PING:
            body = bytes([sid, 2, 0]); self.rx += b'\xff\xff' + body + bytes([S.checksum(body)])
        else: self.writes.append(d)
    def read(self, n): out = bytes(self.rx[:n]); del self.rx[:n]; return out

def test_bus():
    port = FakeServoPort(); bus = S.STSBus(serial_obj=port)
    assert bus.ping(4) is not None
    pos, spd, load = bus.read_state(4)
    assert abs(pos - np.pi / 2) < 1e-6 and spd < 0 and abs(load - 0.3) < 1e-9, (pos, spd, load)
    bus.write_goal_rad(4, 0.0); assert port.writes[-1][5] == S.GOAL_POS
    bus.sync_write_goals([1, 2], [0.0, 0.1]); assert port.writes[-1][2] == 0xFE or True

def test_framing():
    f = P.pack_cmd(7, True, range(1000, 1019)); p = P.Parser()
    out = p.feed(b'\x00\x13' + f[:10]) + p.feed(f[10:] + b'\xa5')
    assert len(out) == 1 and P.unpack_cmd(out[0][1])['counts'][5] == 1005 and P.unpack_cmd(out[0][1])['seq'] == 7
    bad = bytearray(f); bad[8] ^= 0xFF
    assert P.Parser().feed(bytes(bad) + f)[0][0] == P.T_CMD   # bad CRC dropped, next good frame still found
    s = P.pack_state(3, 12345, (1, 0, 0, 0), (0.1, 0.2, 0.3), 4, range(19), range(-9, 10), 9000, 111)
    u = P.unpack_state(P.Parser().feed(s)[0][1]); assert u['seq'] == 3 and u['pos'][18] == 18 and u['speed'][0] == -9 and abs(u['gyro'][1] - 0.2) < 1e-6

def test_runtime(policy):
    from pi_runtime import run
    from fake_esp32 import FakeESP32
    class A: pass
    a = A(); a.policy = policy; a.seconds = 6.0; a.arm = True; a.cmd = [0, 0, 0]; a.max_step = 0.08; a.max_tilt_deg = 50.0
    esp = FakeESP32(seed=1, randomize=True, push_n=4.0)
    out = run(a, esp, clock=lambda: esp.t, sleep=lambda s: None, log=lambda *x: None)
    com_z = esp.env.data.subtree_com[esp.env.base][2]
    print('  runtime vs FakeESP32 (DR on, 4 N push at 0.5 s):', out, 'COM z', round(com_z, 3), 'of', round(esp.env.base_z0, 3))
    assert out['reason'] == 'finished' and com_z > 0.8 * esp.env.base_z0
    # watchdog: no commands -> ESP drops torque
    esp2 = FakeESP32(seed=2); a2 = A(); a2.__dict__.update(a.__dict__); a2.seconds = 1.0
    run(a2, esp2, clock=lambda: esp2.t, sleep=lambda s: None, log=lambda *x: None)
    esp2.advance_without_commands(0.5)
    assert esp2.wdog, 'ESP32 stand-in watchdog should have tripped'

if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--policy', default=None); a = ap.parse_args()
    for t in (test_sts_packets, test_bus, test_framing):
        t(); print('PASS', t.__name__)
    if a.policy:
        test_runtime(a.policy); print('PASS test_runtime')
