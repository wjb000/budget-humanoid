"""STS3215 actuator model for sim2real.

What this models (everything numeric is tagged with where it comes from):
  * Position target is quantised to servo counts (4096/rev, datasheet 12-bit encoder).        [datasheet]
  * Command latency (target arrives `latency_s` late: USB/bus/servo update).                   [PLACEHOLDER, measure with sim/sysid.py]
  * Servo dead zone: no drive while |error| < deadband (default 2 counts).                     [PLACEHOLDER, STS register 26/27 default is small]
  * Internal position loop approximated as PD: tau = kp*err - kd*qdot.                         [PLACEHOLDER kp/kd, fitted by sysid.py]
  * Torque-speed curve: linear from stall torque at 0 speed to 0 at the no-load speed, scaled with supply voltage.
        STS3215: stall 2.94 N m (30 kg cm), no-load 4.72 rad/s (0.222 s/60 deg); STS3250: stall 4.90 N m (50 kg cm), 7.87 rad/s (0.133 s/60 deg),
        both at 12 V. Per-joint arrays are filled from sim/joint_map.csv by the env.                [datasheets]
  * Supply sag: V = V_batt - R_batt * sum(|tau|/kt). kt (N m/A at the output) from stall current ~2.7 A. [PLACEHOLDER, estimated]
  * Encoder read-back: quantised, with hysteresis `backlash_rad` when the direction reverses.   [PLACEHOLDER. The real lash is inside
        the position loop (encoder is on the output; datasheet backlash <= 0.5 deg for both models) and this does NOT model that; it only models what the policy sees.]
Not modelled: motor temperature derating, rated (0.98 N m) continuous limit (reported through `thermal_state`), stiction beyond the
MuJoCo joint frictionloss, gear compliance, bus read-time jitter.
All parameters are dataclass fields so the domain-randomisation config and the system-ID fit can overwrite them.
"""
from dataclasses import dataclass, field, asdict
import numpy as np

COUNTS = 4096.0
RAD_PER_COUNT = 2 * np.pi / COUNTS


@dataclass
class STS3215Params:
    kp: float = 20.0               # N m/rad  (matches MJCF starting value; identify with sysid.py)
    kd: float = 0.5                # N m s/rad
    tau_stall: float = 2.94        # N m at 12 V (datasheet 30 kg cm)
    tau_rated: float = 0.98        # N m rated continuous (datasheet 10 kg cm), only used for the thermal indicator
    omega_noload: float = 4.72     # rad/s at 12 V
    v_nominal: float = 12.0        # V the datasheet numbers refer to
    v_batt: float = 11.1           # V, 3S LiPo nominal (12.6 full, ~9.9 empty)
    r_batt: float = 0.08           # ohm battery + wiring + fuse + e-stop (placeholder)
    kt: float = 1.09               # N m/A at the output (2.94 N m / ~2.7 A stall current, estimated)
    latency_s: float = 0.02        # s (placeholder)
    deadband_rad: float = 2 * RAD_PER_COUNT
    backlash_rad: float = 0.0087   # 0.5 deg encoder hysteresis seen by the policy (placeholder)
    zero_offset_rad: float = 0.0   # horn mounting error; per joint when arrays are passed in
    quantize: bool = True

    def to_dict(self):
        return {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in asdict(self).items()}


class STS3215Bank:
    """Vectorised bank of n servos sharing a supply. Call `step` at the physics rate (it is stateful about the latency line)."""

    def __init__(self, n, dt, params=None, rng=None):
        self.n, self.dt = n, dt
        self.p = params or STS3215Params()
        self.rng = rng or np.random.default_rng(0)
        self.kp = np.full(n, self.p.kp); self.kd = np.full(n, self.p.kd)
        self.tau_stall = np.full(n, self.p.tau_stall)
        self.omega_nl = np.full(n, self.p.omega_noload)     # per-joint arrays: the design mixes STS3215 and STS3250 (see joint_map.csv)
        self.kt = np.full(n, self.p.kt); self.tau_rated = np.full(n, self.p.tau_rated)
        self.zero_offset = np.full(n, self.p.zero_offset_rad)
        self.set_latency(self.p.latency_s)
        self.reset(np.zeros(n))

    def set_latency(self, s):
        self.delay_steps = max(0, int(round(s / self.dt)))
        self._buf = None

    def reset(self, q0):
        self._buf = np.tile(np.asarray(q0, float), (self.delay_steps + 1, 1))
        self._enc = np.asarray(q0, float).copy()     # hysteresis state (read-back)
        self._last_tau = np.zeros(self.n)
        self.vscale = np.full(self.n, self.p.v_batt / self.p.v_nominal)
        self.energy = 0.0

    # --- what the servo reports to the Pi -----------------------------------------------------------------------
    def read_position(self, q):
        """Encoder reading (rad) with quantisation and reversal hysteresis. Joint zero offset is applied (reading = true + offset)."""
        x = np.asarray(q, float) + self.zero_offset
        h = 0.5 * self.p.backlash_rad
        d = x - self._enc
        self._enc = np.where(d > h, x - h, np.where(d < -h, x + h, self._enc))
        out = self._enc
        if self.p.quantize:
            out = np.round(out / RAD_PER_COUNT) * RAD_PER_COUNT
        return out

    # --- torque computation -------------------------------------------------------------------------------------
    def step(self, target, q, qd):
        """target, q, qd in joint rad (true values). Returns torque (N m) to apply to the joints."""
        self._buf = np.roll(self._buf, -1, axis=0); self._buf[-1] = target
        tgt = self._buf[0]
        if self.p.quantize:
            tgt = np.round(tgt / RAD_PER_COUNT) * RAD_PER_COUNT
        # controller sees quantised position (without the policy-side hysteresis: that is a read-back effect)
        qm = np.round((q + self.zero_offset) / RAD_PER_COUNT) * RAD_PER_COUNT if self.p.quantize else q + self.zero_offset
        err = tgt - qm   # servo frame: s = q + zero_offset, so a horn mounting error shifts where the joint ends up
        err = np.where(np.abs(err) < self.p.deadband_rad, 0.0, err)
        tau = self.kp * err - self.kd * qd
        # supply sag from last step's total current
        i_tot = float(np.sum(np.abs(self._last_tau) / self.kt))
        v = max(0.4 * self.p.v_nominal, self.p.v_batt - self.p.r_batt * i_tot)
        self.vscale[:] = v / self.p.v_nominal
        # torque-speed limit: driving torque falls linearly with speed in the direction of motion; braking is not limited
        w = qd * np.sign(tau)
        drive_lim = self.tau_stall * self.vscale * np.clip(1.0 - w / (self.omega_nl * self.vscale), 0.0, 1.0)
        brake_lim = self.tau_stall * self.vscale
        lim = np.where(w > 0, drive_lim, brake_lim)
        tau = np.clip(tau, -lim, lim)
        self._last_tau = tau
        self.energy += float(np.sum(np.abs(tau * qd))) * self.dt
        return tau

    def thermal_state(self, tau_rms):
        """Fraction of the rated continuous torque the given RMS torque uses (>1 means the servo will heat)."""
        return np.asarray(tau_rms) / self.tau_rated


def bank_from_dict(n, dt, d, rng=None):
    p = STS3215Params(**{k: v for k, v in d.items() if k in STS3215Params.__dataclass_fields__})
    return STS3215Bank(n, dt, p, rng)
