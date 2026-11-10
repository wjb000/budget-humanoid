# Print guide: Budget-22 printed frame

{{N}} printed parts, one STL each in `cad/stl/` (ASCII STL, mm). **Every STL is already rotated into its print orientation and sits on z = 0**: import, do not rotate. Left/right parts are separate mirrored files, so print one of each file. Matching STEP files (assembly frame, for CAD work) are in `cad/step/`.

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
| Printed mass | about {{TOTAL}} g estimated (60% effective fill), roughly {{N}} parts |

## 2. Parts

{{TABLE}}

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
