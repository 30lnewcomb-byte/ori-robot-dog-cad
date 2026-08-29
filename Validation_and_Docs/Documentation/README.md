# Ori Robot Dog - CAD Project

Root: `C:\Users\Liam Newcomb\Desktop\Ori Robot Dog`

**Status: MECHANICAL HANDOFF — CAD-final, parameterized, validated. No physical printing performed.**

Autonomous CAD engineering of a dimensionally-accurate, manufacturable quadruped
around the Hiwonder HTD-45H bus servo (20 actuated DOF: 12 leg + 6 arm + 2 head).
CadQuery 2.8 parametric generation driven by a single master parameter file.

## Project layout
```
Ori Robot Dog/
  Parameters/master_parameters.py   <- SINGLE SOURCE OF TRUTH (all dims, incl shafts + gripper)
  CAD/Master/build_common.py        <- export() STEP+STL, geometry helpers
  CAD/Master/hardware_lib.py        <- bearings, screws, inserts, D2F switch
  CAD/Master/shafts.py              <- 6mm steel shaft + 626ZZ bearing stacks (§4)
  CAD/Hardware/servos/htd45h.py     <- accurate HTD-45H envelope (51.1x20.14x40)
  CAD/Legs/master_leg.py            <- ONE leg architecture (hip-yaw/hip-pitch/knee-ankle-foot)
  CAD/Torso/torso.py                <- splittable chassis, hips, batt, arm port
  CAD/Head/head.py                  <- 2-DOF dome head + camera/mic/speaker mounts
  CAD/Arm/arm.py                    <- 6-DOF arm + servo-in-wrist passive gripper
  CAD/Assemblies/assembly.py        <- full robot, 4 legs via transform, standing IK
  CAD/Exports/                      <- STEP (CAD) + STL (print) per part
  Validation/validate_leg.py        <- 11 leg checks
  Validation/validate_robot.py      <- 29 global checks
  Validation/validate_wrist.py      <- 10 wrist §3 checks (cam/gear/coupler/backdrive/print)
  Validation/analyze_shafts.py      <- 5 shaft sizing checks (§4)
  Validation/analyze_loads.py       <- 4 torque-margin checks
  Validation/fea_structural.py      <- PETG linear-static FEA (scikit-fem+gmsh)
  BOM/BOM.csv                       <- real components, prices, REQUIRED/OPTIONAL/FUTURE
  Documentation/MECHANICAL_HANDOFF.md   <- handoff summary (read this first)
  Documentation/VERIFICATION_STATUS.md
  Documentation/software_handoff_jointmap.json  <- joint names/axes/limits/sensors for software
```

## How to regenerate
Every module is runnable directly after activating the venv:
```bash
source .venv/Scripts/activate
python CAD/Legs/master_leg.py          # builds + exports one leg
python CAD/Torso/torso.py              # builds + exports torso halves
python CAD/Head/head.py                # builds + exports head
python CAD/Arm/arm.py                  # builds + exports arm (parked pose)
python CAD/Assemblies/assembly.py      # builds + exports full robot
python Validation/validate_leg.py      # 11/11 expected
python Validation/validate_robot.py    # 29/29 expected
python Validation/validate_wrist.py    # 10/10 expected
python Validation/analyze_shafts.py    # 5/5 expected
```
Changing a value in `Parameters/master_parameters.py` propagates to every part.

## Key facts (verified)
- 16 × HTD-45H servos; 28 × 626ZZ; 10 × 6mm steel shafts (§4).
- Full robot **666 × 245 × 442 mm** (validate_robot 29/29).
- Passive gripper **22 g** (≤80 g); gear/cam, 18 mm center, servo-in-wrist.
- 500 g payload FEASIBLE (analysis). FEA: hip PASS; yoke 141 MPa crash-only (documented).
- Mechanical handoff complete; software integration points in software_handoff_jointmap.json.
