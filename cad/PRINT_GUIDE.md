# Print guide: Budget-22 printed frame

24 printed parts, one STL each in `cad/stl/` (ASCII STL, mm). **Every STL is already rotated into its print orientation and sits on z = 0**: import, do not rotate. Left/right parts are separate mirrored files, so print one of each file. Matching STEP files (assembly frame, for CAD work) are in `cad/step/`.

Status: **designed and checked in software only** (watertight meshes, 220 x 220 mm bed fit, collision-free zero pose, wall check). It has **not been printed or assembled** yet. Print the fit coupons first.

## 0. Fit test first (about 15 to 25 g, well under an hour)

```
python cad/fit_coupon.py          # writes cad/stl/fit_coupon_sleeve.stl and fit_coupon_plate.stl
```

1. `fit_coupon_sleeve`: a real STS3215 must drop into the pocket with the 4 case-screw holes lining up with the servo's case holes and the cable notch clear. Print **opening up**.
2. `fit_coupon_plate`: must sit on the servo horn with its 4 counterbored holes over the horn's 4 holes (9.8 mm square pattern).

The horn hole pattern, case-screw positions and cable-slot position were **measured from the SO-ARM100 STS3215 STEP**, not from a physical servo. If a coupon is off, change `HORN_PATTERN`, `CASE_HOLES`, `CLR` or `GAP` in `cad/lib.py`, run `python cad/export.py`, and every part updates.

## 1. Settings

| | |
|---|---|
| Printer | any FDM, 220 x 220 mm bed or larger (largest part: torso, see table) |
| Material | PLA+ or PETG for the frame. PETG for feet, ankle and shin (impact, creep). Not plain PLA near the battery or in a hot car. |
| Nozzle / layer | 0.4 mm / 0.2 mm |
| Perimeters | 4 (1.6 mm) minimum; designed wall is 2.0 to 2.4 mm |
| Infill | 20% gyroid; 40% for the load-bearing leg parts (thigh, shin, ankle link, foot) |
| Top/bottom | 5 layers |
| Supports | Mostly none: overhang area is under 9% on every part. The pockets open toward +z or sideways; turn on "supports touching build plate" or tree supports and read the preview for torso and forearm. |
| Tolerance | pocket clearance 0.3 mm per side, horn-plate gap 0.1 mm (set in `cad/lib.py`) |
| Printed mass | about 583 g estimated (60% effective fill), roughly 24 parts |

## 2. Parts

| Part | Qty | Servos it carries | Print size (mm) | Overhang | Est. mass* | Supports |
|---|---|---|---|---|---|---|
| `r_foot` | 1 | - | 122 x 56 x 42 | 0.4% | 26 g | none needed |
| `r_ankle_link` | 1 | r_ankle_roll | 81 x 30 x 38 | 1.5% | 16 g | none needed |
| `r_shin` | 1 | r_ankle_pitch | 30 x 125 x 38 | 0.6% | 20 g | none needed |
| `r_thigh` | 1 | r_knee | 30 x 128 x 38 | 0.2% | 20 g | none needed |
| `r_hip_roll_link` | 1 | r_hip_pitch | 43 x 53 x 40 | 3.2% | 17 g | tree/touching-bed supports, check preview |
| `r_hip_yaw_link` | 1 | r_hip_roll | 55 x 30 x 45 | 1.6% | 16 g | none needed |
| `r_shoulder_link` | 1 | r_shoulder_roll | 81 x 30 x 38 | 1.5% | 16 g | none needed |
| `r_upper_arm` | 1 | r_elbow | 32 x 77 x 38 | 0.8% | 18 g | none needed |
| `r_forearm` | 1 | r_gripper | 37 x 39 x 141 | 8.8% | 27 g | supports on cable/screw pockets (see preview) |
| `r_finger` | 1 | - | 24 x 87 x 29 | 1.2% | 8 g | none needed |
| `l_foot` | 1 | - | 122 x 56 x 42 | 0.4% | 26 g | none needed |
| `l_ankle_link` | 1 | l_ankle_roll | 81 x 30 x 38 | 1.5% | 16 g | none needed |
| `l_shin` | 1 | l_ankle_pitch | 30 x 125 x 38 | 0.6% | 20 g | none needed |
| `l_thigh` | 1 | l_knee | 30 x 128 x 38 | 0.2% | 20 g | none needed |
| `l_hip_roll_link` | 1 | l_hip_pitch | 43 x 53 x 40 | 3.2% | 17 g | tree/touching-bed supports, check preview |
| `l_hip_yaw_link` | 1 | l_hip_roll | 55 x 30 x 45 | 1.6% | 16 g | none needed |
| `l_shoulder_link` | 1 | l_shoulder_roll | 81 x 30 x 38 | 1.5% | 16 g | none needed |
| `l_upper_arm` | 1 | l_elbow | 32 x 77 x 38 | 0.8% | 18 g | none needed |
| `l_forearm` | 1 | l_gripper | 37 x 39 x 141 | 8.8% | 27 g | supports on cable/screw pockets (see preview) |
| `l_finger` | 1 | - | 24 x 87 x 29 | 1.2% | 8 g | none needed |
| `pelvis` | 1 | r_hip_yaw, l_hip_yaw | 30 x 136 x 38 | 1.8% | 38 g | none needed |
| `torso` | 1 | r_shoulder_pitch, l_shoulder_pitch, neck_yaw | 80 x 181 x 180 | 8.1% | 149 g | supports on cable/screw pockets (see preview) |
| `neck_bracket` | 1 | - | 30 x 45 x 40 | 1.3% | 6 g | none needed |
| `head` | 1 | neck_pitch | 51 x 38 x 51 | 6.8% | 22 g | supports on cable/screw pockets (see preview) |

\* Mass estimate = solid volume x 1.24 g/cc (PLA) x 0.6 effective fill. Weigh your parts and update `PLA_DENSITY`/`FILL` in `cad/export.py` so the sim masses match.

## 3. Hardware (not printed)

- 22 x Feetech STS3215 servos (the CAD and sim use the STS3215 outline for every joint, see "Known gaps" in `cad/README.md`)
- 4 x M2 screws per servo for the sleeve (case holes), self-tapping into the servo case, length about 8 to 10 mm (confirm with the sleeve coupon)
- 4 x M3 screws per servo for the horn plate (the plate holes are counterbored from the outside), length about 8 mm. Confirm against your servo's horn.
- Pi 5 on the 4 printed standoffs inside the torso back wall (M2.5), e-stop hole in the torso top plate (22 mm class panel button, check yours)
- Electronics are modeled as real vendor CAD (Pi 5, BNO085 IMU, MAX98357 amp, speaker) or as clearly marked size envelopes (camera, XIAO, buck converter, battery, bus adapters, mic, e-stop); see `cad/README.md`.

## 4. Assembly order

1. Fit-test, then print all parts.
2. Fit servos in sleeves (screws into the case holes), horn plate on each horn.
3. Build each leg from the foot up (foot, ankle link, shin, thigh, hip roll link, hip yaw link), arms from the hand down, then pelvis, torso, neck bracket, head.
4. Run servo ID setup (`sim/joint_map.csv` has the ID, name, direction and limit for each joint) **before** closing the sleeves, since servos must be daisy-chained with IDs 1 to 22.
5. Zero pose: all servos at their mid position (2048 counts) is the URDF/MJCF zero pose. Mount horns at that position.

## 5. Safe joint ranges

The ranges in `sim/joint_map.csv` come from a collision sweep of the final geometry (one joint at a time, others at zero). Combined motions are not checked. Hip yaw and roll are intentionally narrow (the legs are close together), and the arms cannot cross the torso.
