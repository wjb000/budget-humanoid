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
| Supports | Mostly none: overhang area is under 12% on every part (forearm is the worst). The pockets open toward +z or sideways; turn on "supports touching build plate" or tree supports and read the preview for torso and forearm. |
| Tolerance | pocket clearance 0.3 mm per side, horn-plate gap 0.1 mm (set in `cad/lib.py`) |
| Printed mass | about {{TOTAL}} g estimated (60% effective fill), roughly {{N}} parts |

## 2. Parts

{{TABLE}}

\* Mass estimate = solid volume x 1.24 g/cc (PLA) x 0.6 effective fill. Weigh your parts and update `PLA_DENSITY`/`FILL` in `cad/export.py` so the sim masses match.

## 3. Hardware (not printed)

- 22 x Feetech STS3215 servos (the CAD and sim use the STS3215 outline for every joint, see "Known gaps" in `cad/README.md`)
- 4 x M2 screws per servo for the sleeve (case holes), self-tapping into the servo case, length about 8 to 10 mm (confirm with the sleeve coupon)
- 4 x M3 screws per servo for the horn plate (the plate holes are counterbored from the outside), length about 8 mm. Confirm against your servo's horn.
- **Fasteners are M2 and M3 only.**
  - Servo sleeves: 4 x M2 x 8 per servo (88 for 22 servos), self-tapping into the servo case. Horn plates: 4 x M3 x 8 per servo (88), plate holes counterbored from outside. Confirm both against the coupons and your servo.
  - Torso front cover: 4 x M2 x 6 into 4 x M2 heat-set inserts (pockets 3.2 mm, 6.5 mm deep) in the front bosses. Battery door: 4 x M2 x 6 into 4 more inserts. That is 8 inserts and 8 screws.
  - Head lid: 2 x M2 x 8 self-tapping into 1.8 mm pilot holes in the head's back wall.
  - Pi 5: 4 x M2 x 8 self-tapping into 1.7 mm pilot holes in the 4 torso standoffs. The Pi's own holes are M2.5 (2.75 mm), so M2 screws fit loosely; they hold, they do not locate. Check and shim if the board rocks.
  - No bearings are used. Opposite-horn bearing supports are not designed (see `cad/README.md`, known gap 8).
- Battery: 3S 2200 mAh (assumed 105 x 35 x 25 mm) sits between two printed ribs in the rear bay and is held by 2 hook-and-loop straps through the floor slots. 2.5 x 100 mm cable ties (about 30) go through the tie slots in the thigh, shin and torso side walls.
- E-stop: 22 mm panel-mount mushroom button in the top plate hole (the key notch stops it turning). XT60 panel-mount connector (about $1.20) in the battery door cutout (15.8 x 8.2 mm). Both are envelopes in the model.
- XIAO ESP32-S3 snaps between the rails on the inside of the front cover (it has no mounting holes, so this is a friction fit; add a dab of hot glue if it rattles).
- Electronics are modeled as real vendor CAD (Pi 5, BNO085 IMU, MAX98357 amp, speaker) or as clearly marked size envelopes (camera, XIAO, buck converter, battery, bus adapters, mic, e-stop); see `cad/README.md`.

## 4. Assembly order

1. Fit-test, then print all parts.
2. Fit servos in sleeves (screws into the case holes), horn plate on each horn.
3. Press the 8 M2 heat-set inserts into the torso bosses (soldering iron, 220 C for PLA). Mount the Pi on its standoffs, the battery between the ribs, then close the battery door and the front cover.
4. Build each leg from the foot up (foot, ankle link, shin, thigh, hip roll link, hip yaw link), arms from the hand down, then pelvis, torso, neck bracket, head.
5. Run servo ID setup (`sim/joint_map.csv` has the ID, name, direction and limit for each joint) **before** closing the sleeves, since servos must be daisy-chained with IDs 1 to 22.
5. Zero pose: all servos at their mid position (2048 counts) is the URDF/MJCF zero pose. Mount horns at that position.

## 5. Safe joint ranges

The ranges in `sim/joint_map.csv` come from a collision sweep of the final geometry (one joint at a time, others at zero). Combined motions are not checked. Hip yaw and roll are intentionally narrow (the legs are close together), and the arms cannot cross the torso.
