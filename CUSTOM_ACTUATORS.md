# Custom Dirt-Cheap Actuators for Budget-19

Inspired by agentic full-robot designs (e.g. Galatea-style CAD + sim + visual checks) but optimized for **extreme cost**: replace commercial Feetech STS3215/STS3250 servos with our own 3D-printed, machine-wound actuators.

## Goal
- Per-actuator cost well under $20–30 (vs ~$20–65 commercial).
- Fully printable housings + gears where possible.
- Coils wound on a simple 3D-printed machine we also design/print.
- Compatible with the existing Budget-19 kinematics, URDF, and sim-to-real stack.
- Still dirt-cheap overall robot (~$300–500 target if successful).

## Approach options (ranked for dirt-cheap)

1. **Wound stator + 3D-printed housing/gears (recommended starting point)**
   - Reference: Robossembler open-source stator winder (~$150 machine, mostly printed) and their servo designs (GitLab: robossembler/cnc/motor-wire-winder and servo repos).
   - Wind enameled copper wire on a 3D-printed or simple laminated stator.
   - Pair with cheap magnets, bearings, and a low-ratio 3D-printed gearbox (planetary or cycloidal).
   - Control with SimpleFOC or similar on cheap MCUs.

2. **Commercial small BLDC + 3D-printed cycloidal gearbox**
   - Reference: Berkeley Humanoid Lite actuators (5010/6512 style, printed cycloidal, ~$100–150 each including motor/ESC/encoder).
   - Higher performance and more proven, but less "make our own coils".
   - Still far cheaper than industrial QDDs.

3. **Ironless / Halbach printed QDD**
   - References: Caden Kraft Ironless-QDD, OpenQDD, etc. (~$40–70).
   - High torque density possible, but magnets dominate cost and winding/assembly is more involved.

## Coil Winder
A simple 3D-printed winder is essential for the wound-stator path.
- Open references: Robossembler motor-wire-winder, various Instructables/GitHub NEMA17 + lead-screw designs (Arduino/ESP32 control, ~$50–150 parts).
- Requirements for our use: handle stator coils or bobbins for ~20–30 mm diameter, tens to hundreds of turns of ~0.3–0.5 mm wire, basic tension and counting.
- Plan: start by adapting an existing open design, then customize for our stator geometry.

## Integration with Budget-19
- Keep the current parametric CadQuery frame as the baseline.
- New actuator modules must match the mounting envelope of the STS3215 pods (or update pods if size changes).
- Update `sim/actuator.py` and domain randomization once we have measured torque/speed curves.
- System-ID tools already exist — use them on the custom actuators.
- Safety and validation path from `HARDWARE_VALIDATION.md` still applies (fit tests, harness, e-stop, etc.).

## Next concrete steps
1. Prototype one simple wound-stator actuator (or adapt Robossembler/Berkeley style) and measure torque, speed, efficiency, heat.
2. Design/print a minimal coil winder and wind consistent test coils.
3. Update CAD pods if the custom actuator dimensions differ.
4. Re-export URDF/MJCF and retrain policies with the new actuator model.
5. Document BOM and print settings for both actuator and winder.

This keeps the project open, software-verified, and hardware-first while pushing the cost floor lower than commercial hobby servos. The AI-generated full-robot aesthetic is a visual target; the dirt-cheap custom actuators are the engineering differentiator.

**Status:** Plan only. No hardware built yet. See existing known gaps in `cad/README.md`.
