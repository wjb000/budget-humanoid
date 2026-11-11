# Scale and Payload Plan: Useful-Work Budget Humanoid

**Target:** A larger humanoid that can do real useful work (lift, carry, manipulate everyday objects, simple tool use, household tasks) while staying dirt-cheap via custom 3D-printed + machine-wound actuators.

Current Budget-19 (0.61 m, ~2.2 kg, commercial hobby servos) is a design study / walking research platform. This document defines the next scale.

## Target Specs (starting point)

| Parameter | Target | Notes |
|-----------|--------|-------|
| Height | 1.0–1.3 m | Adult-child / small adult proportions. Enough reach for tables, shelves, countertops. |
| Mass | 12–22 kg | Includes structure, actuators, battery, compute. |
| Payload (hands) | 1–3 kg sustained | Lift a water bottle, tool, small box, grocery item. Peak higher for short moves. |
| Reach | ~0.6–0.8 m forward | Useful for tabletop and shelf work. |
| DOF | 19–25+ | Keep current 19-DOF baseline; add simple 1–2 DOF grippers or 3–5 DOF hands later. |
| Runtime | 20–40 min useful work | Larger battery + efficient actuators. |
| Cost goal | <$800–1200 fully built | Dominated by copper, magnets, batteries, electronics — not commercial servos. |

## Joint Torque Priorities (for useful work)

High continuous / peak torque needed on:
1. Hip pitch + knee (support body + payload while standing/walking)
2. Shoulder pitch + elbow (lift and hold objects at reach)
3. Ankle (balance under load)

Lower torque / higher speed acceptable on:
- Wrist, head, finger joints (if added)

Custom wound actuators will be sized differently per joint class (more copper + better cooling on high-torque joints).

## Structure Changes from Tiny Budget-19

- Thicker printed walls and more infill on load-bearing parts.
- Heat-set inserts or metal standoffs at high-load interfaces.
- Selective reinforcement: aluminum extrusion, carbon tube, or printed-in metal inserts for leg and arm long links.
- Larger foot contact area and better ankle compliance for balance under payload.
- Cable routing channels sized for higher current.
- Modular actuator pods that can be swapped (different torque ratings).

## Power and Thermal

- Battery: 4S–6S LiPo or Li-ion pack, 2000–5000 mAh class, with proper BMS and e-stop.
- High-torque actuators need better thermal paths (printed heat sinks, optional small fans, or metal housings on hottest joints).
- Current limits and duty-cycle awareness in firmware so the robot does not overheat during continuous work.

## Hands / Grippers

Start simple:
- Parallel jaw or underactuated 1–2 DOF grippers (printable, low DOF).
- Later: 3–5 DOF hands if the arm torque budget allows.

These must stay within the dirt-cheap custom actuator philosophy.

## Integration with Existing Stack

- Parametric CadQuery remains the source of truth — scale the frame and regenerate URDF/MJCF.
- Actuator model + system-ID tools absorb the new torque/speed curves once measured.
- Training environment and domain randomization already support larger masses and payloads.
- Hardware validation path (fit coupons → mechanical → electrical → sysid → walking/manipulation) still applies; just at larger scale.

## Next Steps

1. Lock a specific height (e.g. 1.2 m) and payload (e.g. 2 kg).
2. Resize the parametric frame and actuator pods in CadQuery.
3. Prototype 1–2 high-torque custom wound actuators and measure real performance.
4. Update sim mass properties and retrain policies with payload randomization.
5. Print and fit-test structural parts before full assembly.

This keeps the project open, software-verified, and hardware-realistic while moving from "cute walking demo" to "robot that can do useful work."
