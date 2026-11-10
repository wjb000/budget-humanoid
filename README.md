# Budget-19: dirt-cheap 19-DOF humanoid

Open design for a ~0.61 m, ~2.2 kg, 19-DOF 3D-printed biped on Feetech STS3215 servos with a Raspberry Pi 5 + XIAO ESP32-S3. Includes a parametric printable frame, vendor CAD for the parts that have redistributable models, and URDF/MuJoCo files.

**Live site:** https://wjb000.github.io/budget-humanoid/

> Status: software-verified only. The frame has **not been printed or assembled**; sim gains are starting values. See `cad/README.md` ("What is real" and "Known gaps").

## Where things are
- `site/`: the website (component map, interactive 3D viewer of the CAD assembly, DOF table, schematic, BOM, CAD status). No build step.
- `cad/`: parametric CadQuery frame (23 printed parts in 14 unique designs: each servo sits in a rounded pod that is also the bracket, plus front cover, battery door, head lid), `cad/DFM.md` (injection-molding notes per part), one STL + STEP each in `cad/stl`, `cad/step` (already in print orientation), `cad/PRINT_GUIDE.md`, vendor CAD in `cad/vendor` with licenses, `cad/README.md`.
- `sim/`: `budget19.urdf`, `budget19.xml` (MuJoCo), `joint_map.csv` (servo IDs), `check_mujoco.py` (load + stand check), Isaac Lab notes in `sim/README.md`.
- `tools_gen_data.py`: regenerates `site/assets/data.js` (BOM, pins, DOF table).
- `.github/workflows/pages.yml`: deploys `site/` to GitHub Pages on push to main (Pages source must be *GitHub Actions*).

## Quick start
```bash
pip install mujoco numpy && python sim/check_mujoco.py        # stand check, no CAD tools needed
pip install cadquery trimesh numpy python-fcl rtree fast-simplification
python cad/fetch_vendor.py && python cad/vendor_meshes.py && python cad/sim_export.py   # rebuild everything
python cad/fit_coupon.py                                      # print these two small parts first
```

## Licenses
Vendor CAD stays under its own license (Apache-2.0 STS3215, MIT Adafruit, MIT Raspberry Pi); see `cad/vendor/` and `cad/README.md`. No license has been chosen yet for the generated frame, sim files and site: add one before sharing widely.

Design study, no warranty. Verify fit, torque and power before building.
