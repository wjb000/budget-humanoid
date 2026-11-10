"""Pi <-> XIAO ESP32-S3 framing over USB CDC. Mirrors deploy/esp32/firmware/firmware.ino (keep both in sync).

Frame: A5 5A TYPE LEN PAYLOAD CRC16-CCITT(TYPE LEN PAYLOAD) little-endian.
 CMD   0x01 (Pi -> ESP): seq u16, flags u8 (bit0 torque on), goal counts 19 x u16 (policy joint order)
 STATE 0x81 (ESP -> Pi): seq u16 (echo of last CMD), t_us u32, quat wxyz 4 x f32, gyro rad/s 3 x f32, status u8
        (bit0 e-stop open, bit1 watchdog tripped, bit2 imu ok, bit3 servo read error), pos counts 19 x u16, speed 19 x i16 (counts/s),
        bus_cycle_us u16, vbat_decivolt u16
"""
import struct

N = 19
SYNC = b'\xa5\x5a'
T_CMD, T_STATE = 0x01, 0x81
CMD_FMT = '<HB' + 'H' * N
STATE_FMT = '<HI4f3fB' + 'H' * N + 'h' * N + 'HH'
ST_ESTOP, ST_WDOG, ST_IMU_OK, ST_READ_ERR = 1, 2, 4, 8


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(ftype, payload):
    body = bytes([ftype, len(payload)]) + payload
    return SYNC + body + struct.pack('<H', crc16(body))


def pack_cmd(seq, torque_on, counts):
    return frame(T_CMD, struct.pack(CMD_FMT, seq & 0xFFFF, 1 if torque_on else 0, *[int(c) for c in counts]))


def pack_state(seq, t_us, quat, gyro, status, pos, speed, bus_us, vbat_dv):
    return frame(T_STATE, struct.pack(STATE_FMT, seq & 0xFFFF, t_us & 0xFFFFFFFF, *quat, *gyro, status, *[int(p) for p in pos], *[int(s) for s in speed], int(bus_us), int(vbat_dv)))


class Parser:
    """Feed bytes, get (type, payload) frames. Resynchronises after garbage or a bad CRC."""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                self.buf = self.buf[-1:] if self.buf[-1:] == SYNC[:1] else bytearray(); break
            del self.buf[:i]
            if len(self.buf) < 4: break
            n = self.buf[3]
            if len(self.buf) < 4 + n + 2: break
            body = bytes(self.buf[2:4 + n]); crc, = struct.unpack('<H', self.buf[4 + n:6 + n])
            if crc16(body) == crc:
                out.append((body[0], body[2:])); del self.buf[:6 + n]
            else:
                del self.buf[:2]
        return out


def unpack_state(payload):
    v = struct.unpack(STATE_FMT, payload)
    return dict(seq=v[0], t_us=v[1], quat=v[2:6], gyro=v[6:9], status=v[9], pos=v[10:10 + N], speed=v[10 + N:10 + 2 * N], bus_us=v[10 + 2 * N], vbat_dv=v[11 + 2 * N])


def unpack_cmd(payload):
    v = struct.unpack(CMD_FMT, payload)
    return dict(seq=v[0], torque_on=bool(v[1] & 1), counts=v[2:])
