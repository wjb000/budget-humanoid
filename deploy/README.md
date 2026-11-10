# /deploy: Pi 5 + XIAO ESP32-S3 runtime for Budget-19

```
 policy (numpy, 50 Hz)         USB CDC, 921600 baud          2 x UART 1 Mbaud, half-duplex adapters
 Pi 5: pi_runtime.py  <------------------------------>  XIAO ESP32-S3 (firmware.ino)  <---> Waveshare bus adapters -> 19 x STS3215
        obs.py (shared with training)    protocol.py                  + BNO085 IMU (I2C), e-stop sense, 100 ms watchdog
```

| File | What | Tested how |
|---|---|---|
| `pi_runtime.py` | Loads `policy.npz` + `policy_meta.json` (from `sim/train.py`), builds the observation with `sim/obs.py`, runs the 50 Hz loop with safety checks | end to end against `fake_esp32.py` (MuJoCo + actuator model) in `test_deploy.py` |
| `protocol.py` | Pi <-> ESP32 framing (sync, type, length, CRC16-CCITT) | unit tests, resync after garbage and a bad CRC |
| `sts_bus.py` | Feetech STS packet driver (ping, read, write, sync write); register map checked against the Feetech memory table | packet bytes against the documented PING example, a fake servo; **never against a real servo** |
| `fake_esp32.py` | ESP32 stand-in used for the offline tests | it is a test double |
| `esp32/firmware/firmware.ino` | Bridge firmware: sync write of 19 goals, 19 position/speed reads, IMU, e-stop sense, watchdog | **not compiled, not flashed** |
| `test_deploy.py` | `python deploy/test_deploy.py --policy runs/stand0` | PASS (see SIM2REAL.md) |

## Bring-up order (nothing below has been done; this is the plan)

1. **One servo on the bench**, no robot: `python deploy/sts_bus.py --port /dev/ttyUSB0 --scan`, then set IDs 1..19 one at a time (each servo is delivered as ID 1) and write the mid position (torque register value 128 calibrates the current position to 2048) with the horn on the index spline.
2. **Servo identification**: `python sim/sysid.py record ...` then `fit` on a few joints, update `STS3215Params` and the ranges in `sim/domain_randomization.json`. Measure the real read/write cycle for 19 servos (the firmware reports `bus_cycle_us`); if it exceeds ~12 ms, lower the policy rate in `sim/obs.py` and retrain.
3. **Flash the ESP32**, check the e-stop sense line, IMU orientation (the quaternion must match the MJCF `imu` site: x forward, z up) and the watchdog (unplug USB: servos must go limp within 100 ms).
4. **Direction and zero per joint**: with the robot hung from a harness, torque low (`set_torque_limit`), command +0.1 rad on each joint and compare the direction with `sim/joint_map.csv` (left and right are mirrored); write `deploy/calibration.json` as `{"direction": [19 x +-1], "offset_counts": [19 x int]}` in policy joint order and pass `--calib`.
5. `pi_runtime.py` without `--arm` (torque stays off, you only watch observations), then with `--arm` on the harness, then on the ground with a stand policy only.

Safety layers: (1) physical e-stop cuts servo power, (2) ESP32 watchdog (100 ms) drops torque, (3) runtime drops torque on tilt > 50 deg, IMU loss, stale state or e-stop, (4) targets are clamped to the CAD joint limits and rate-limited, (5) 2 s blend from the measured pose into the policy on arming. Going limp on a fault will drop a standing robot; use a harness for every early test.
