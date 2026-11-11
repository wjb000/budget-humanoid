# Custom Dirt-Cheap Actuators for Budget-19

Inspired by agentic full-robot designs (e.g. Galatea-style CAD + sim + visual checks) but optimized for **extreme cost**: replace commercial Feetech STS3215/STS3250 servos with our own 3D-printed, machine-wound actuators.

**Scale note:** For a robot big enough to do useful work, see `SCALE_AND_PAYLOAD.md` (1.0–1.3 m, 12–22 kg, 1–3 kg hand payload). Actuator sizing, copper volume, cooling, and structure all scale with that target. Cost per actuator will be higher than the tiny version but still far below commercial high-torque units.

## Goal
- Per-actuator cost as low as practical (target $25–80 depending on joint torque class).
- Fully printable housings + gears where possible; selective metal only where strength or heat demands it.
- Coils wound on a simple 3D-printed machine we also design/print.
- Compatible with the existing parametric CadQuery + URDF + sim-to-real stack (resized for useful-work scale).
- Overall robot still dirt-cheap relative to commercial humanoids.

## Approach options (ranked for dirt-cheap)

1. **Wound stator + 3D-printed housing/gears (recommended starting point)**
   - Reference: Robossembler open-source stator winder (~$150 machine, mostly printed) and their servo designs (GitLab: robossembler/cnc/motor-wire-winder and servo repos).
   - Wind enameled copper wire on a 3D-printed or simple laminated stator.
   - Pair with cheap magnets, bearings, and a low-ratio 3D-printed gearbox (planetary or cycloidal).
   - Control with SimpleFOC or similar on cheap MCUs.
   - Size differently per joint: more copper and better cooling on hip/knee/shoulder.

2. **Commercial small BLDC + 3D-printed cycloidal gearbox**
   - Reference: Berkeley Humanoid Lite actuators (5010/6512 style, printed cycloidal).
   - Higher performance and more proven, but less "make our own coils".
   - Useful as a faster path to a working larger robot while the wound versions mature.

3. **Ironless / Halbach printed QDD**
   - References: Caden Kraft Ironless-QDD, OpenQDD, etc.
   - High torque density possible, but magnets dominate cost.

## Coil Winder
A simple 3D-printed winder is essential for the wound-stator path.
- Open references: Robossembler motor-wire-winder, various Instructables/GitHub NEMA17 + lead-screw designs (Arduino/ESP32 control, ~$50–150 parts).
- Requirements: handle stator coils or bobbins for the sizes needed by the larger joints, consistent tension and turn counting.
- Plan: start by adapting an existing open design, then customize for our stator geometries.

## Integration with Budget-19
- Parametric CadQuery frame is the source of truth — resize it per `SCALE_AND_PAYLOAD.md` and regenerate URDF/MJCF.
- Actuator modules sized per joint class (high-torque vs low-torque).
- Update `sim/actuator.py` and domain randomization once we have measured torque/speed curves from real hardware.
- System-ID tools already exist — use them on the custom actuators.
- Safety and validation path still applies (fit tests, harness, e-stop, thermal checks under load).

## Next concrete steps
1. Lock specific height + payload from `SCALE_AND_PAYLOAD.md`.
2. Prototype one high-torque wound-stator (or hybrid) actuator and measure real performance under load.
3. Design/print a minimal coil winder and wind consistent test coils.
4. Resize CAD pods and structure, re-export URDF/MJCF.
5. Retrain policies with the new actuator model and payload randomization.
6. Document BOM and print settings for actuators, winder, and structure.

This keeps the project open, software-verified, and hardware-first while targeting a robot that can do useful work at a dirt-cheap price point.

**Status:** Plan only. No hardware built yet. See existing known gaps in `cad/README.md` and the scale targets in `SCALE_AND_PAYLOAD.md`.
