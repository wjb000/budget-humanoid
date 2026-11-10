# Sim-to-real package: what exists, what was run, what was not

Budget-19 (19 DOF, 2.262 kg, STS3250 on hip pitch and knee, STS3215 elsewhere). Everything below was run in simulation or against software stand-ins on a Linux box. **Nothing has touched a servo, an ESP32, a Pi 5, or a printed part.**

## Pieces

| Piece | File | State |
|---|---|---|
| URDF + MJCF, masses/inertias per link from the CAD meshes, per-joint torque limits | `budget19.urdf`, `budget19.xml`, `mass_properties.json`, `joint_map.csv` | generated from CAD; masses are estimates (PLA 1.24 g/cc x 0.6 fill, servo masses, electronics estimates) |
| Servo model | `actuator.py` | torque-speed curve, supply sag, latency, dead band, quantisation, encoder hysteresis; **kp, kd, latency, backlash, battery resistance are placeholders until `sysid.py` has real data** |
| Domain randomisation | `domain_randomization.json` | engineering guesses, applied per episode by `env.py` (mass, COM, kp/kd, stall, friction, damping, armature, latency, zero offset, backlash, battery, floor friction, IMU tilt, noise, pushes) |
| Training env | `env.py`, `obs.py`, `default_pose.py` | Gymnasium + plain MuJoCo, 50 Hz policy / 500 Hz physics, `stand` and `walk`. Only the foot soles collide; torso/arms/head are not collision bodies, so episodes end on tilt/height |
| Training | `train.py` (PPO, stable-baselines3, CPU) | runs, about 2800 env steps/s on 8 cores |
| System ID | `sysid.py` | `record` (hardware, never run), `fit`, `selftest` (synthetic) |
| Policy export | `export_policy.py` | numpy/JSON MLP, the Pi needs no torch |
| Pi runtime | `../deploy/pi_runtime.py` | runs against `fake_esp32.py` (MuJoCo + actuator model) |
| ESP32 bridge | `../deploy/esp32/firmware/firmware.ino` | **not compiled, not flashed** |
| Servo bus driver | `../deploy/sts_bus.py` | unit-tested against documented packet bytes and a fake servo |

## What was actually run (results as committed)

- `check_mujoco.py`: zero-pose stand under a stiff position hold passes (sag 2.8 mm, tilt below 1.9 deg, both feet in contact).
- `deploy/test_deploy.py --policy runs/stand0`: STS packet bytes (PING matches the documented example), framing + resync, fake servo, and the Pi runtime driving the ESP32 stand-in for 6 s with domain randomisation and a 4 N push: stays up (tilt about 6 deg); the stand-in watchdog trips when commands stop.
- `sysid.py selftest`: with synthetic data from a different parameter set plus encoder noise, the fit recovers kp, latency and friction; kd is weakly identified by that excitation. This validates the code, not the servo.
- Stand task, `runs/stand0` (PPO 600k steps, deterministic policy, 30 episodes, DR on): see `runs/stand0/eval.json`. The learned policy survives 93% of episodes against 100% for simply holding the default pose, so **PPO did not beat the baseline on stand**; the default pose with the servo PD already stands under the randomised pushes. The stand policy is only used to exercise the deploy path.
- Walk task, `runs/walk0` (PPO 3M steps, 18 min on 8 CPU cores, deterministic policy, 30 episodes, DR on, commands 0 to 0.25 m/s): survives 83% of episodes (hold-pose 100%), mean forward speed **0.025 m/s** against a mean command of about 0.1 m/s, so **it does not walk**. At 2.2M steps the training return was still rising, so a longer run and reward tuning (foot clearance, step length) may get further; I did not run one. `runs/walk0/eval.json` has the numbers, `progress.csv` the learning curve. The peak RMS torque over a control step reached 1.27x the rated continuous torque of some servo during the walk rollouts, which is within stall but would heat that servo.

## What is NOT validated (read before trusting any number)

- Servo parameters: stall/no-load/rated torque and speed are datasheet values (STS3215 30 kg.cm, 0.222 s/60 deg; STS3250 50 kg.cm, 0.133 s/60 deg; backlash <= 0.5 deg per datasheet). kp/kd, latency, friction, supply sag are placeholders. Real stall under load is lower than the datasheet number (a third-party bench test measured about 33 kg.cm for an STS3215).
- Torque-speed curve is linear; real servos have current limiting and thermal derating (rated 0.98 N m STS3215, 1.57 N m STS3250 continuous).
- Contact: only foot boxes. No torso/arm/head collisions, no self-collision, flat floor only.
- Inertias come from printed-mesh estimates; the real robot will differ (wires, screws, battery position). DR covers +/-15% per body.
- Joint limits are per-joint collision sweeps, not combined-pose checks.
- Bus timing: 19 servos read at 50 Hz is assumed feasible. The firmware reports `bus_cycle_us`; if the read cycle is longer than about 12 ms, lower the policy rate and retrain.
- The hardware path (direction signs, zero offsets, IMU frame, e-stop wiring, watchdog behaviour) is untested.

## Next steps, in order

1. Print the two fit coupons (with an STS3215 and an STS3250); measure the real horn and case-hole positions.
2. Run `sysid.py record` on one servo of each type on the bench, then `fit`; put the numbers into `STS3215Params` and `domain_randomization.json`.
3. Stall/lever test on each servo type for the real torque; update `joint_map.csv` (via `cad/export.py`).
4. Add torso/head/arm collision geometry before training anything that falls or uses the arms.
5. Train walk on the identified model, add observation/action latency matching the measured bus cycle, then harness tests per `deploy/README.md`.
