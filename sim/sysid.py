#!/usr/bin/env python3
"""System identification for the STS3215 model in sim/actuator.py.

Three modes:
  record   run the excitation protocol on ONE real servo (Pi/laptop -> Waveshare adapter -> servo) and log it. HARDWARE REQUIRED.
  fit      fit kp, kd, latency, viscous + Coulomb friction (and optionally deadband) to a log from `record`.
  selftest generate a synthetic log from a known "true" parameter set (different from the defaults, plus encoder noise) and check that
           `fit` recovers it. This validates the fitting CODE only. It is NOT real servo data and says nothing about the real servo.
           Result of the selftest as committed: kp, latency and friction are recovered; kd is only weakly identified by this excitation (stays
           near its starting value), so treat a fitted kd with suspicion and prefer step-response overshoot to pin it down.

Protocol for `record` (see sim/SIM2REAL.md for the bench set-up):
  1. servo unloaded or carrying a known inertia (a printed disc or bar on the horn; give --inertia in kg m^2), torque limit capped (--torque-limit, 0.3 default).
  2. small steps (+/-0.05, 0.1, 0.2 rad about the start angle), 1.5 s each          -> kp, kd, latency
  3. chirp 0.2 -> 4 Hz, amplitude 0.1 rad, 20 s                                     -> bandwidth, latency, friction
  4. slow ramp up/down 0.3 rad over 6 s                                             -> dead band and reversal hysteresis (backlash seen at the encoder)
  Also logs present speed/load/voltage/current so torque-speed (kt, stall) can be derived from separate stall/lever tests (documented, not automated).

Usage:
  python sim/sysid.py selftest
  python sim/sysid.py record --port /dev/ttyUSB0 --id 4 --inertia 2.0e-4 --out data/knee4.npz
  python sim/sysid.py fit --data data/knee4.npz --inertia 2.0e-4 --out sim/identified_params.json
"""
import argparse, json, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from actuator import STS3215Bank, STS3215Params, RAD_PER_COUNT

DT = 0.002


def protocol(rate_hz=200.0, q0=0.0):
    """Return (t, target_rad) for the full excitation sequence at rate_hz."""
    segs = []
    for amp in (0.05, 0.1, 0.2):
        for sgn in (1, -1, 1, -1):
            segs.append(np.full(int(1.5 * rate_hz), q0 + sgn * amp))
    tc = np.arange(int(20 * rate_hz)) / rate_hz
    f = 0.2 + (4.0 - 0.2) * tc / tc[-1]
    segs.append(q0 + 0.1 * np.sin(2 * np.pi * np.cumsum(f) / rate_hz))
    n = int(6 * rate_hz)
    ramp = np.concatenate([np.linspace(0, 0.3, n), np.linspace(0.3, 0, n)])
    segs.append(q0 + ramp)
    tgt = np.concatenate(segs)
    return np.arange(len(tgt)) / rate_hz, tgt


def simulate_plant(params, inertia, fv, fc, t, target, q_init=0.0, noise=0.0, rng=None, dt=DT):
    """One joint driven by the actuator model. `t` may be coarser than dt; target is zero-order held. Returns encoder readings at t."""
    rng = rng or np.random.default_rng(0)
    bank = STS3215Bank(1, dt, params)
    q, qd = q_init, 0.0
    bank.reset(np.array([q]))
    out = np.zeros(len(t)); k = 0
    steps = int(round(t[-1] / dt)) + 1
    for i in range(steps):
        ti = i * dt
        while k < len(t) and t[k] <= ti + 1e-9:
            out[k] = bank.read_position(np.array([q]))[0]; k += 1
        idx = min(np.searchsorted(t, ti, side='right') - 1, len(t) - 1)
        tau = bank.step(np.array([target[idx]]), np.array([q]), np.array([qd]))[0]
        qdd = (tau - fv * qd - fc * np.tanh(qd / 0.02)) / inertia
        qd += qdd * dt; q += qd * dt
    out[k:] = bank.read_position(np.array([q]))[0]
    return out + rng.normal(0, noise, len(t))


def fit(t, target, meas, inertia, p0=None, verbose=True):
    """Fit [kp, kd, fv, fc] by least squares for each candidate latency and keep the best. Deadband/backlash kept at defaults (reported)."""
    from scipy.optimize import least_squares
    p0 = p0 or STS3215Params()
    best = None
    for lat in np.arange(0.0, 0.0601, 0.01):
        def res(x):
            p = STS3215Params(**{**p0.__dict__, 'kp': x[0], 'kd': x[1], 'latency_s': lat})
            sim = simulate_plant(p, inertia, x[2], x[3], t, target)
            return (sim - meas) / 0.01
        s = least_squares(res, [p0.kp, p0.kd, 0.02, 0.03], bounds=([1, 0.01, 0.0, 0.0], [200, 10, 1.0, 0.5]), diff_step=0.05, max_nfev=25, loss='soft_l1')
        if verbose: print(f'  latency {lat*1000:4.0f} ms  cost {s.cost:10.2f}  kp {s.x[0]:.2f} kd {s.x[1]:.3f} fv {s.x[2]:.3f} fc {s.x[3]:.3f}', flush=True)
        if best is None or s.cost < best[0]: best = (s.cost, lat, s.x)
    cost, lat, x = best
    rms = float(np.sqrt(np.mean((simulate_plant(STS3215Params(**{**p0.__dict__, 'kp': x[0], 'kd': x[1], 'latency_s': lat}), inertia, x[2], x[3], t, target) - meas) ** 2)))
    return dict(kp=float(x[0]), kd=float(x[1]), latency_s=float(lat), viscous_friction=float(x[2]), coulomb_friction=float(x[3]), rms_error_rad=rms,
                held_at_defaults=dict(deadband_rad=p0.deadband_rad, backlash_rad=p0.backlash_rad, tau_stall=p0.tau_stall))


def cmd_selftest(a):
    true = STS3215Params(kp=14.0, kd=0.8, latency_s=0.03)
    inertia, fv, fc = 2.0e-4, 0.05, 0.08
    t, tgt = protocol(rate_hz=100.0)
    t, tgt = t[:int(8 * 100)], tgt[:int(8 * 100)]     # steps only keeps the test fast
    print('synthetic truth (NOT real servo data):', dict(kp=true.kp, kd=true.kd, latency_s=true.latency_s, viscous=fv, coulomb=fc))
    meas = simulate_plant(true, inertia, fv, fc, t, tgt, noise=0.0015, rng=np.random.default_rng(3))
    res = fit(t, tgt, meas, inertia)
    print('recovered:', json.dumps({k: v for k, v in res.items() if k != 'held_at_defaults'}))
    ok = abs(res['kp'] - true.kp) / true.kp < 0.25 and abs(res['latency_s'] - true.latency_s) <= 0.011 and res['rms_error_rad'] < 0.01
    print('SELFTEST', 'PASS' if ok else 'FAIL', '(pipeline validation only)')
    return 0 if ok else 1


def cmd_record(a):
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'deploy'))
    from sts_bus import STSBus
    bus = STSBus(a.port, a.baud)
    sid = a.id
    if bus.ping(sid) is None:
        sys.exit(f'servo {sid} did not answer on {a.port}')
    bus.set_torque_limit(sid, int(a.torque_limit * 1000)); bus.enable(sid, True)
    q0 = bus.read_position_rad(sid)
    t_nom, tgt = protocol(a.rate, q0=q0)
    log = []
    t_start = time.perf_counter()
    try:
        for i, (tn, g) in enumerate(zip(t_nom, tgt)):
            while time.perf_counter() - t_start < tn: pass
            ts = time.perf_counter() - t_start
            bus.write_goal_rad(sid, g)
            pos, spd, load = bus.read_state(sid)
            log.append((ts, g, pos, spd, load))
    finally:
        bus.enable(sid, False)
    arr = np.array(log)
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    np.savez(a.out, t=arr[:, 0], target=arr[:, 1], pos=arr[:, 2], speed=arr[:, 3], load=arr[:, 4], servo_id=sid, inertia=a.inertia, q0=q0)
    dts = np.diff(arr[:, 0])
    print(f'saved {a.out}: {len(arr)} samples, mean loop {1/dts.mean():.0f} Hz, p99 dt {np.percentile(dts, 99)*1000:.1f} ms')


def cmd_fit(a):
    d = np.load(a.data)
    t, tgt, meas = d['t'], d['target'], d['pos']
    # resample onto a uniform 100 Hz grid for the fit
    tu = np.arange(0, t[-1], 0.01)
    tgt_u = np.interp(tu, t, tgt); meas_u = np.interp(tu, t, meas)
    q0 = float(meas_u[0]); tgt_u -= q0; meas_u -= q0
    res = fit(tu, tgt_u, meas_u, a.inertia)
    res['servo_id'] = int(d['servo_id']) if 'servo_id' in d else None
    print(json.dumps(res, indent=1))
    if a.out:
        json.dump(res, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('selftest')
    r = sub.add_parser('record'); r.add_argument('--port', required=True); r.add_argument('--baud', type=int, default=1_000_000)
    r.add_argument('--id', type=int, required=True); r.add_argument('--inertia', type=float, required=True); r.add_argument('--rate', type=float, default=200.0)
    r.add_argument('--torque-limit', type=float, default=0.3); r.add_argument('--out', required=True)
    f = sub.add_parser('fit'); f.add_argument('--data', required=True); f.add_argument('--inertia', type=float, required=True); f.add_argument('--out', default=None)
    a = ap.parse_args()
    sys.exit({'selftest': cmd_selftest, 'record': cmd_record, 'fit': cmd_fit}[a.cmd](a) or 0)
