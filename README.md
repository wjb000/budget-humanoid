# Budget-19: dirt-cheap 19-DOF humanoid

Open design for a ~0.61 m, ~2.2 kg, 19-DOF 3D-printed biped. Current baseline uses Feetech STS3215/STS3250 commercial servos + Raspberry Pi 5 + XIAO ESP32-S3. Includes parametric CadQuery frame, vendor CAD where redistributable, URDF/MuJoCo, full sim-to-real stack, and deploy firmware.

**Live site:** https://wjb000.github.io/budget-humanoid/

> **Status:** Software-verified design study. Nothing printed or run on hardware yet. See `cad/README.md` for known gaps.

## Long-term dirt-cheap direction
Inspired by full AI-generated humanoid CAD+sim designs (e.g. Galatea-style), but optimized for extreme cost: replace commercial servos with **our own 3D-printed, machine-wound actuators** and a matching 3D-printed coil winder. See `CUSTOM_ACTUATORS.md` for the plan, references (Robossembler winder, Berkeley Humanoid Lite cycloidal modules, open QDDs), and next steps. Target is a fully self-made actuator path that drops the BOM well below the current ~$900 commercial-servo version while keeping the open, parametric, sim-verified workflow.

## Where things are
- `site/`: website with interactive 3D viewer, DOF table, schematic, BOM.
- `cad/`: parametric CadQuery frame, print guide, export scripts.
- `sim/`: URDF, MuJoCo, joint map, actuator model, training env, system-ID.
- `deploy/`: Pi runtime + ESP32 bridge.
- `CUSTOM_ACTUATORS.md`: plan for custom wound actuators + winder.
- `HARDWARE_VALIDATION.md`: (on other branch) prioritized path to working hardware.

## Quick start
```bash
pip install mujoco numpy && python sim/check_mujoco.py
# Full rebuild (needs CadQuery etc.):
# python cad/fetch_vendor.py && python cad/vendor_meshes.py && python cad/sim_export.py
# python cad/fit_coupon.py
```

Design study, no warranty. Verify fit, torque, and power before building. Custom actuator path is experimental.
