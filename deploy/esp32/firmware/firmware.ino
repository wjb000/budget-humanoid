// Budget-19 bridge firmware for the Seeed XIAO ESP32-S3.
// Role: real-time layer between the Pi 5 policy (USB CDC, deploy/protocol.py framing) and the two STS3215 buses, plus IMU, e-stop sense, watchdog.
// STATUS: written against the Arduino-ESP32 core and Adafruit_BNO08x APIs but NOT COMPILED OR FLASHED (no toolchain was available when this was
// written). Expect to fix compile errors. Pin choices are proposals; confirm against your XIAO pinout and the Waveshare adapter's UART header.
//
// Libraries: Adafruit BNO08x (+ Adafruit BusIO). Board: "XIAO_ESP32S3", USB CDC On Boot = Enabled.
// Wiring: legs bus adapter  -> Serial1  TX=D6 (GPIO43)  RX=D7 (GPIO44)   (adapter jumper in UART position, verify in the Waveshare wiki)
//         arm/neck adapter  -> Serial2  TX=D0 (GPIO1)   RX=D1 (GPIO2)
//         BNO085 I2C        -> SDA=D4 (GPIO5) SCL=D5 (GPIO6), address 0x4A (Adafruit board default)
//         e-stop sense      -> D2 (GPIO3), input with pull-up; the NC e-stop contact in series with servo power also reads this line (HIGH = e-stop open/pressed)
#include <Wire.h>
#include <Adafruit_BNO08x.h>

static const int N = 19;
// Policy joint order (sim/budget19.xml actuator order) -> servo ID and bus (0 = legs, 1 = arms/neck). Keep equal to sim/joint_map.csv.
static const uint8_t SERVO_ID[N]  = {1, 2, 3, 4, 5, 6, 13, 14, 15, 7, 8, 9, 10, 11, 12, 16, 17, 18, 19};
static const uint8_t SERVO_BUS[N] = {0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1};
static const int PIN_ESTOP = 3;
static const uint32_t WDOG_MS = 100;
HardwareSerial &BUSSER(int b) { return b == 0 ? Serial1 : Serial2; }

// ---- Feetech STS protocol -------------------------------------------------------------------------------------------------------------------
static uint8_t chk(const uint8_t *b, int n) { uint8_t s = 0; for (int i = 0; i < n; i++) s += b[i]; return ~s; }

static void sendPacket(int bus, uint8_t id, uint8_t instr, const uint8_t *p, int np) {
  uint8_t pkt[64]; int n = 0;
  pkt[n++] = 0xFF; pkt[n++] = 0xFF; pkt[n++] = id; pkt[n++] = np + 2; pkt[n++] = instr;
  for (int i = 0; i < np; i++) pkt[n++] = p[i];
  pkt[n] = chk(pkt + 2, n - 2); n++;
  BUSSER(bus).write(pkt, n);
}

// SYNC WRITE goal position (reg 42, 2 bytes) to all servos on a bus whose cmd[] is set
static void syncWriteGoals(int bus, const uint16_t *goal) {
  uint8_t p[2 + 3 * N]; int n = 0; int cnt = 0;
  p[n++] = 42; p[n++] = 2;
  for (int i = 0; i < N; i++) if (SERVO_BUS[i] == bus) { p[n++] = SERVO_ID[i]; p[n++] = goal[i] & 0xFF; p[n++] = goal[i] >> 8; cnt++; }
  if (cnt) sendPacket(bus, 0xFE, 0x83, p, n);
}
static void writeTorque(int bus, uint8_t on) { uint8_t p[2] = {40, on}; sendPacket(bus, 0xFE, 0x03, p, 2); }   // broadcast torque enable

// READ present position (56) + speed (58) = 4 bytes from one servo. Returns false on timeout/bad checksum.
static bool readState(int i, uint16_t &pos, int16_t &spd) {
  int bus = SERVO_BUS[i]; HardwareSerial &s = BUSSER(bus);
  while (s.available()) s.read();
  uint8_t p[2] = {56, 4}; sendPacket(bus, SERVO_ID[i], 0x02, p, 2);
  uint8_t r[16]; int n = 0; uint32_t t0 = micros();
  // the Waveshare adapter does not echo our TX on RX (verify; if it does, skip the first 9 bytes here)
  while (n < 10 && micros() - t0 < 1500) { if (s.available()) r[n++] = s.read(); }
  if (n < 10 || r[0] != 0xFF || r[1] != 0xFF || r[2] != SERVO_ID[i]) return false;
  if (chk(r + 2, 7) != r[9]) return false;
  pos = r[5] | (r[6] << 8); pos &= 0x7FFF;
  uint16_t sp = r[7] | (r[8] << 8); int32_t v = (int32_t)(sp & 0x7FFF) * 50; if (v > 32767) v = 32767;   // unit is 0.732 rpm = ~50 counts/s; report counts/s
  spd = (sp & 0x8000) ? -(int16_t)v : (int16_t)v;
  return true;
}

// ---- Framing (deploy/protocol.py) ----------------------------------------------------------------------------------------------------------
static uint16_t crc16(const uint8_t *d, int n) { uint16_t c = 0xFFFF; for (int i = 0; i < n; i++) { c ^= (uint16_t)d[i] << 8; for (int k = 0; k < 8; k++) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : (c << 1); } return c; }
static uint8_t rx[256]; static int rxn = 0;
static uint16_t goal[N]; static uint8_t torqueOn = 0; static uint16_t lastSeq = 0; static uint32_t lastCmdMs = 0; static bool wdogTripped = false;
static uint16_t pos[N]; static int16_t spd[N]; static bool readErr = false;
Adafruit_BNO08x bno(-1); static float quat[4] = {1, 0, 0, 0}, gyro[3] = {0, 0, 0}; static bool imuOk = false;
static uint16_t busCycleUs = 0;

static void handleCmd(const uint8_t *pl, int len) {
  if (len != 3 + 2 * N) return;
  lastSeq = pl[0] | (pl[1] << 8); uint8_t flags = pl[2];
  for (int i = 0; i < N; i++) goal[i] = pl[3 + 2 * i] | (pl[4 + 2 * i] << 8);
  bool wasOn = torqueOn; torqueOn = flags & 1;
  lastCmdMs = millis(); wdogTripped = false;
  if (torqueOn != wasOn) { writeTorque(0, torqueOn); writeTorque(1, torqueOn); }
  if (torqueOn) { syncWriteGoals(0, goal); syncWriteGoals(1, goal); }
}

static void pumpUsb() {
  while (Serial.available() && rxn < (int)sizeof(rx)) rx[rxn++] = Serial.read();
  int i = 0;
  while (rxn - i >= 4) {
    if (rx[i] != 0xA5 || rx[i + 1] != 0x5A) { i++; continue; }
    int len = rx[i + 3]; if (rxn - i < 4 + len + 2) break;
    uint16_t c = rx[i + 4 + len] | (rx[i + 5 + len] << 8);
    if (crc16(rx + i + 2, 2 + len) == c) { if (rx[i + 2] == 0x01) handleCmd(rx + i + 4, len); i += 6 + len; } else i += 2;
  }
  if (i) { memmove(rx, rx + i, rxn - i); rxn -= i; }
}

static void sendState() {
  uint8_t pl[2 + 4 + 28 + 1 + 4 * N + 4]; int n = 0;
  auto put16 = [&](uint16_t v) { pl[n++] = v & 0xFF; pl[n++] = v >> 8; };
  auto put32 = [&](uint32_t v) { for (int k = 0; k < 4; k++) pl[n++] = (v >> (8 * k)) & 0xFF; };
  auto putf = [&](float f) { uint32_t u; memcpy(&u, &f, 4); put32(u); };
  put16(lastSeq); put32(micros());
  for (int k = 0; k < 4; k++) putf(quat[k]);
  for (int k = 0; k < 3; k++) putf(gyro[k]);
  uint8_t st = 0; if (digitalRead(PIN_ESTOP)) st |= 1; if (wdogTripped) st |= 2; if (imuOk) st |= 4; if (readErr) st |= 8;
  pl[n++] = st;
  for (int i = 0; i < N; i++) put16(pos[i]);
  for (int i = 0; i < N; i++) put16((uint16_t)spd[i]);
  put16(busCycleUs); put16(0);   // battery voltage: add a READ of register 62 on one servo if wanted
  uint8_t f[4 + sizeof(pl) + 2]; int m = 0; f[m++] = 0xA5; f[m++] = 0x5A; f[m++] = 0x81; f[m++] = n;
  memcpy(f + m, pl, n); m += n;
  uint16_t c = crc16(f + 2, 2 + n); f[m++] = c & 0xFF; f[m++] = c >> 8;
  Serial.write(f, m);
}

void setup() {
  Serial.begin(921600);
  Serial1.begin(1000000, SERIAL_8N1, 44, 43);   // RX=GPIO44 (D7), TX=GPIO43 (D6)
  Serial2.begin(1000000, SERIAL_8N1, 2, 1);     // RX=GPIO2  (D1), TX=GPIO1  (D0)
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  Wire.begin(5, 6); Wire.setClock(400000);
  if (bno.begin_I2C(0x4A, &Wire)) {
    bno.enableReport(SH2_ROTATION_VECTOR, 5000);     // 200 Hz
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000);
    imuOk = true;
  }
  writeTorque(0, 0); writeTorque(1, 0);
  lastCmdMs = millis();
}

void loop() {
  pumpUsb();
  // watchdog: no valid command for WDOG_MS -> torque off (the robot goes limp rather than holding a stale target)
  if (torqueOn && millis() - lastCmdMs > WDOG_MS) { torqueOn = 0; wdogTripped = true; writeTorque(0, 0); writeTorque(1, 0); }
  sh2_SensorValue_t v;
  while (imuOk && bno.getSensorEvent(&v)) {
    if (v.sensorId == SH2_ROTATION_VECTOR) { quat[0] = v.un.rotationVector.real; quat[1] = v.un.rotationVector.i; quat[2] = v.un.rotationVector.j; quat[3] = v.un.rotationVector.k; }
    if (v.sensorId == SH2_GYROSCOPE_CALIBRATED) { gyro[0] = v.un.gyroscope.x; gyro[1] = v.un.gyroscope.y; gyro[2] = v.un.gyroscope.z; }
  }
  // one state refresh per received command (host loop is 50 Hz)
  static uint16_t lastSent = 0xFFFF;
  if (lastSeq != lastSent) {
    uint32_t t0 = micros(); readErr = false;
    for (int i = 0; i < N; i++) if (!readState(i, pos[i], spd[i])) readErr = true;
    busCycleUs = min<uint32_t>(65535, micros() - t0);
    sendState(); lastSent = lastSeq;
  }
}
