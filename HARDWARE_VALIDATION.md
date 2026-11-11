# Hardware Validation Roadmap for Budget-19

The design is software-complete (CAD, sim, deploy stack) but has **never been printed or run on hardware**. This document lists the prioritized steps to close the known gaps and produce a working robot. Work top to bottom. Update this file as items are completed.

## Phase 0: Fit and print verification (do first)
- [ ] Print `cad/fit_coupon_sleeve.stl` and `cad/fit_coupon_plate.stl`.
- [ ] Confirm a real STS3215 drops into the sleeve and seats on the horn plate with M2/M3 screws.
- [ ] Repeat with an STS3250 (same outline, but verify mounting hole positions and any differences).
- [ ] If fit fails, adjust `cad/lib.py` / `cad/robot.py` and regenerate.
- [ ] Print one full limb (e.g. thigh + shin + ankle + foot) and assemble with servos to check clearances, cable notches, and hub engagement.

## Phase 1: Mechanical improvements (highest leverage)
Known gaps from `cad/README.md`:

1. **No opposing bearing on the servo horn.** The rear stub is too short/small for a standard bearing. Options:
   - Design a short fork bracket that captures the rear of the servo and adds a small bearing or bushing.
   - Or accept the current design for early builds and add it later if backlash or wear appears.
2. **Cable routing.** Currently notches and one torso slot. Add:
   - Dedicated cable channels or clips on thigh/shin/upper arm.
   - Strain relief at the torso exit and at each joint.
3. **STS3250 mounting holes.** Measure a real unit and update the model if they differ from the STS3215 STEP.
4. **Heat-set insert pockets** and cover fits: test-print the torso front cover and battery door.

## Phase 2: Electronics and power
- [ ] Wire one bus (legs or arms) on the bench with the Waveshare adapter and confirm current draw under load.
- [ ] Verify the Pololu buck supplies stable 5 V to the Pi 5 under load.
- [ ] Confirm e-stop cuts power and the ESP32 watchdog drops torque within ~100 ms.
- [ ] Measure real stall/current for STS3215 and STS3250; update fuse and wiring if needed (legs can pull tens of amps when stalled).

## Phase 3: Servo bring-up and system identification
Follow `deploy/README.md`:
- [ ] Set IDs 1–19, zero each joint at mid position.
- [ ] Run `sim/sysid.py` on several joints (especially hip pitch, knee, hip roll).
- [ ] Update `sim/actuator.py` and `domain_randomization.json` with measured torque-speed, latency, deadband.
- [ ] Re-run `sim/check_mujoco.py` and retrain policies if parameters change significantly.

## Phase 4: Calibration and first motion
- [ ] Hang the assembled robot from a harness.
- [ ] Flash ESP32 firmware.
- [ ] With torque limited, command small motions and record direction signs + offsets into `deploy/calibration.json`.
- [ ] Run `pi_runtime.py` without `--arm`, then with a stand policy on the harness.
- [ ] Only then try ground contact with the stand policy.

## Phase 5: Walking and product polish
- [ ] Train or refine a walking policy in sim with the updated actuator model.
- [ ] Transfer to hardware on the harness, then floor.
- [ ] Add collision geometry for arms/torso/head in the MJCF if self-collision becomes an issue.
- [ ] Consider optional upgrades: shoulder roll (if desired), better grippers, battery management, remote logging.

## Safety notes (non-negotiable)
- Physical e-stop and harness for all early tests.
- Start with low torque limits.
- Monitor temperature on hip pitch / knee / hip roll servos.
- Never operate without the ability to cut power immediately.

When a phase is done, note the date, any parameter changes, and photos or videos in an issue or PR. The goal is a reproducible path from this design study to a robot that stands and walks.
