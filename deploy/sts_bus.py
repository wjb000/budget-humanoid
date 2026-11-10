"""Feetech STS3215 serial-bus driver (Pi or laptop, via a Waveshare Bus Servo Adapter or any 1 Mbaud half-duplex UART adapter).

Protocol (Feetech STS/SCS, packet = FF FF ID LEN INSTR PARAMS... CHECKSUM, checksum = ~(ID+LEN+INSTR+PARAMS) & 0xFF, little-endian words):
  instructions PING 0x01, READ 0x02, WRITE 0x03, SYNC_WRITE 0x83.
  registers (checked against the Feetech STS memory table, ftservo.github.io, and the rustypot/beam-bots STS3215 tables):
  torque enable 40, acceleration 41, goal position 42 (2 B), goal time 44, goal speed 46, torque limit 48 (2 B, 0.1 %),
  present position 56 (2 B, 0.0879 deg), present speed 58 (2 B, sign bit 15), present load 60 (2 B, sign bit 10), voltage 62, temperature 63, current 69.
  Position is 0..4095 counts for 360 deg; counts = 2048 + rad * 4096 / (2 pi) for a servo whose zero is the mid position.
NOT TESTED AGAINST A REAL SERVO: the packet builder/parser is unit-tested against a loopback fake (deploy/test_deploy.py). First contact with
hardware must be: one servo, torque limit low, `python deploy/sts_bus.py --port ... --scan`.
"""
import math, struct, time

COUNTS = 4096
TORQUE_ENABLE, ACCEL, GOAL_POS, GOAL_SPEED, TORQUE_LIMIT, PRESENT_POS = 40, 41, 42, 46, 48, 56
INST_PING, INST_READ, INST_WRITE, INST_SYNC_WRITE = 0x01, 0x02, 0x03, 0x83


def checksum(body):
    return (~sum(body)) & 0xFF


def packet(sid, instr, params=b''):
    body = bytes([sid, len(params) + 2, instr]) + params
    return b'\xff\xff' + body + bytes([checksum(body)])


def rad_to_counts(r):
    return int(max(0, min(COUNTS - 1, round(2048 + r * COUNTS / (2 * math.pi)))))


def counts_to_rad(c):
    return (c - 2048) * 2 * math.pi / COUNTS


def decode_signed(v, bit):
    return -(v & ~(1 << bit)) if v & (1 << bit) else v


class STSBus:
    def __init__(self, port=None, baud=1_000_000, serial_obj=None, timeout=0.01):
        if serial_obj is None:
            import serial  # pyserial
            serial_obj = serial.Serial(port, baud, timeout=timeout)
        self.s = serial_obj

    def _txrx(self, pkt, expect_reply=True):
        self.s.reset_input_buffer()
        self.s.write(pkt)
        # half-duplex adapters echo our own bytes on RX: skip them
        echo = self.s.read(len(pkt)) if getattr(self, 'echo', False) else b''
        if not expect_reply:
            return None
        hdr = self.s.read(4)
        if len(hdr) < 4 or hdr[:2] != b'\xff\xff':
            return None
        rest = self.s.read(hdr[3])
        if len(rest) < hdr[3]:
            return None
        body = hdr[2:] + rest[:-1]
        if checksum(body) != rest[-1]:
            return None
        return rest[1:-1]  # skip error byte, drop checksum

    def ping(self, sid):
        return self._txrx(packet(sid, INST_PING))

    def read(self, sid, addr, n):
        r = self._txrx(packet(sid, INST_READ, bytes([addr, n])))
        return r if r is not None and len(r) == n else None

    def write(self, sid, addr, data):
        self._txrx(packet(sid, INST_WRITE, bytes([addr]) + bytes(data)), expect_reply=False)

    def enable(self, sid, on=True):
        self.write(sid, TORQUE_ENABLE, [1 if on else 0])

    def set_torque_limit(self, sid, per_mille):
        self.write(sid, TORQUE_LIMIT, struct.pack('<H', int(max(0, min(1000, per_mille)))))

    def write_goal_rad(self, sid, rad):
        self.write(sid, GOAL_POS, struct.pack('<H', rad_to_counts(rad)))

    def read_position_rad(self, sid):
        r = self.read(sid, PRESENT_POS, 2)
        return counts_to_rad(struct.unpack('<H', r)[0]) if r else None

    def read_state(self, sid):
        """(position rad, speed rad/s, load fraction) in one 6-byte read."""
        r = self.read(sid, PRESENT_POS, 6)
        if r is None:
            return None, None, None
        pos, spd, load = struct.unpack('<HHH', r)
        return counts_to_rad(pos & 0x7FFF), decode_signed(spd, 15) * 0.732 * 2 * math.pi / 60.0, decode_signed(load, 10) / 1000.0

    def sync_write_goals(self, ids, rads):
        """One broadcast packet moves every listed servo (the whole point of a shared bus at 50 Hz)."""
        data = b''.join(bytes([i]) + struct.pack('<H', rad_to_counts(r)) for i, r in zip(ids, rads))
        body = bytes([0xFE, len(data) + 4, INST_SYNC_WRITE, GOAL_POS, 2]) + data
        self.s.write(b'\xff\xff' + body + bytes([checksum(body)]))


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(); ap.add_argument('--port', required=True); ap.add_argument('--scan', action='store_true'); a = ap.parse_args()
    bus = STSBus(a.port)
    if a.scan:
        for i in range(1, 21):
            r = bus.ping(i)
            print(i, 'ok' if r is not None else '-', end='  ')
        print()
