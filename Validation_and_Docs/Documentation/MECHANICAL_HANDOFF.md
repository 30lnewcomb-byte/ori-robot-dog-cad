# Ori Robot Dog — Mechanical Handoff

**Status:** CAD/analysis phase. Parametric source builds and automated CAD validation runs in GitHub Actions. No physical printing or hardware validation has been performed.

**CAD system:** Python/CadQuery + `Source/Parameters/master_parameters.py`.
**Primary material:** PETG structure with 626ZZ bearings, 6 mm steel shaft reinforcement, and M3 hardware.
**Printer target:** Bambu Lab A1 Mini, 180 × 180 × 180 mm.

## 1. Current architecture

- 4-legged quadruped generated from one master leg and transformed into FL/FR/RL/RR.
- **Current leg CAD:** 2 actuated joints per leg — hip pitch + knee.
- **Current leg DOF:** 8 total.
- Front 6-DOF arm with the grip actuator retained in the forearm/wrist and a passive interchangeable gripper.
- 2-DOF head: pan about Z and pitch about Y.
- **Current implemented actuated DOF:** 16 (8 leg + 6 arm + 2 head).
- **Target architecture:** 20 DOF (12 leg + 6 arm + 2 head). The four target hip-yaw mechanisms are not implemented in the current CAD.

See `KNOWN_BLOCKERS.md` and `software_handoff_jointmap.json` for the implementation/target distinction.

## 2. Current dimensional model

The master parameter file is authoritative for component dimensions; the assembled robot validator computes the actual current envelope. Do not copy old envelope numbers into new source code.

Current baseline parameters include:

| Quantity | Value |
|---|---:|
| Hip height nominal | 300 mm |
| Upper link | 175 mm |
| Lower link | 175 mm |
| Stance length target | 437 mm |
| Stance width target | 168 mm |
| Torso core | 300 × 150 × 110 mm |
| Head | 90 × 70 × 70 mm |
| A1 Mini build volume | 180³ mm |
| 626ZZ bearing | 6 × 19 × 6 mm |
| HTD-45H body | 51.1 × 20.14 × 40 mm |

Older documents contain alternative envelope numbers. Those are historical unless reproduced by the current validator.

## 3. Actuation and hardware

- HTD-45H baseline: 64 g nominal mass, 6 mm output shaft, 115200 bus, 9–12.6 V parameterized in the master system.
- Current CAD uses 8 leg servos + 6 arm servos + 2 head servos = **16 implemented actuators**.
- BOM includes 12 leg servo purchases because the current leg design needs 8 and retains 4 as spare units.
- 28 × 626ZZ are planned for the current 8 leg + 6 arm bearing-supported joints.
- 10 × 6 mm steel shafts reinforce the hip, knee, shoulder, elbow, and wrist axes.

## 4. Passive gripper

Current `arm.py` implementation uses:

**grip servo in forearm → printed spur gear → coupler gear → eccentric cam → two passive fingers.**

- Standard coupler diameter: 34 mm.
- Retention: 2 × M3 + Ø4 alignment dowel.
- Gear center distance: 18 mm.
- Passive tool contains no servo, motor, battery, or controller.
- Nominal passive gripper mass target: 22 g PETG.

The mechanical architecture is CAD-based and still requires physical gear backlash/wear and grip-force validation.

## 5. Load and structural assessment

The repository contains static load calculations, shaft sizing, and linear-static PETG FEA. These are engineering analyses, not physical certification.

Known result:
- Hip bulkhead FEA is documented as low-stress in the modeled load case.
- Upper-link yoke has a documented high-stress crash-only case. A gusset was deliberately deferred rather than introducing unverified geometry.
- The 500 g arm payload remains an **analysis target**, with printed-link stiffness and physical testing identified as the limiting uncertainties.

## 6. Manufacturing

All current component generators are intended to produce printable parts within the A1 Mini envelope. The repository tracks high-fidelity STEP and print-oriented STL deliverables separately from editable parametric source.

Canonical export helpers now resolve the real repository root and place generated STEP/STL output under `CAD_STEP/` and `CAD_STL/`.

## 7. Serviceability / assembly intent

- Bearings are intended to be replaceable press-fit components.
- 6 mm reinforcement shafts are removable through the joint stacks.
- Torso is split into printable halves with seam alignment and fastener features.
- Battery and electronics have planned service access.
- The gripper uses a removable mechanical coupler so future passive tools can share the same interface.

Actual insertion forces, seam strength, tool access, and final wiring remain physical validation tasks.

## 8. Software handoff

`software_handoff_jointmap.json` is now explicitly versioned as a **current-CAD + target-architecture** handoff:

- `IN_CAD` joints may be used as mechanical facts.
- `NOT_IN_CAD` hip-yaw joints are targets only and must not be driven as though they exist.
- Head axes match the current CAD: pan Z, pitch Y.

Firmware should consume the implementation state rather than assuming the old 20-DOF claim is already physically realized.

## 9. Validation

Use the repository root command:

```text
python validate.py
```

GitHub Actions additionally runs the CadQuery smoke build and the validation suite on pushes to `main` and pull requests.

## 10. Remaining decisions / blockers

- Implement a real four-joint hip-yaw stage, or deliberately freeze the robot as an 8-DOF leg architecture.
- Select Raspberry Pi 3 vs Pi 4 for final electronics packaging.
- Perform physical print/fit validation before calling the design build-ready.
- Validate gear backlash/wear, insert pull-out, seam strength, servo loading, and dynamic behavior.
