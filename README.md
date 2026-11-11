# Budget-19: dirt-cheap 19-DOF humanoid

Open design for a ~0.61 m, ~2.2 kg, 19-DOF 3D-printed biped on Feetech STS3215 servos (STS3250 on hip pitch and knee) with a Raspberry Pi 5 + XIAO ESP32-S3. Includes a parametric printable frame, vendor CAD for the parts that have redistributable models, URDF/MuJoCo files, a full sim-to-real stack, and deploy firmware/runtime.

**Live site:** https://wjb000.github.io/budget-humanoid/

> **Status (November 2026):** Software-verified design. CAD is parametric and software-checked (watertight, bed fit, collisions, MuJoCo stand). Full sim2real package exists (actuator model, Gymnasium env, PPO training, system-ID tools, Pi runtime, ESP32 bridge). **No hardware has been printed, assembled, or run yet.** Masses, torque margins, gains, and walking performance are estimates. See `cad/README.md` and `HARDWARE_VALIDATION.md`.

## Goal
A low-cost (~$900 parts), open, capable small humanoid that can stand, walk, and pick up light objects, with a path from simulation to real hardware. Designed for makers who want to experiment with biped control and sim-to-real without commercial pricing.

## Where things are
- `site/`: website with interactive 3D viewer, DOF table, schematic, BOM, CAD status.
- `cad/`: parametric CadQuery frame (23 printed parts), print guide, vendor CAD, export scripts.
- `sim/`: URDF, MuJoCo MJCF, joint map, actuator model, training env, system-ID, domain randomization.
- `deploy/`: Pi 5 runtime, ESP32-S3 firmware, protocol, bus driver, tests against a fake ESP32.
- `HARDWARE_VALIDATION.md`: prioritized checklist to close the gaps and get a working robot.
- `tools_gen_data.py`: regenerates site data.

## Quick start (software)
```bash
# Stand check (no CAD tools needed)
pip install mujoco numpy && python sim/check_mujoco.py

# Full rebuild (CadQuery + MuJoCo)
pip install cadquery trimesh numpy python-fcl rtree fast-simplification mujoco
python cad/fetch_vendor.py && python cad/vendor_meshes.py && python cad/sim_export.py
python cad/fit_coupon.py   # generate the two small fit-test parts first

# Train a policy (example)
pip install gymnasium stable-baselines3 torch
python sim/train.py --task stand --steps 600000 --out runs/stand0
python deploy/test_deploy.py --policy runs/stand0
```

## Path to a working robot
1. Print the two fit coupons and verify a real STS3215 and STS3250 fit the sleeve and horn plate.
2. Follow `cad/PRINT_GUIDE.md` and `HARDWARE_VALIDATION.md`.
3. Bench-test servos, flash ESP32, calibrate directions/zeros on a harness.
4. Run system-ID, update actuator model, retrain if needed.
5. Start with a stand policy on the harness, then ground.

Use a harness and physical e-stop for every early powered test. Stalled legs can draw high current.

## Licenses
This repository (generated frame, simulation, deploy code, and documentation) is under the Apache License 2.0. See `LICENSE`.

Vendor CAD remains under its own licenses (Apache-2.0 for the STS3215 STEP, MIT for Adafruit parts and Raspberry Pi). See `cad/vendor/` and `cad/README.md`.

Design study. No warranty. Verify fit, torque, power, and safety before building or operating.
