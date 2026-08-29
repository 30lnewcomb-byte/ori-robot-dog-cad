# Verification Status - Ori Robot Dog CAD

Legend: VERIFIED | ASSUMED | UNKNOWN | BLOCKED

## Actuator - Hiwonder HTD-45H
- Body dimensions 51.1 x 20.14 x 40 mm ............................ VERIFIED (Hiwonder / RobotShop datasheet)
- Mass 64 g ....................................................... VERIFIED
- Torque 45 kg.cm (4.41 N.m) @ 11.1V ............................. VERIFIED
- Rotation range 240 deg, speed 0.18 s/60deg @11.1V .............. VERIFIED
- Double 6 mm splined output shaft ................................ VERIFIED (product photos)
- 6-hole horn retention circle, PCD ~25 mm, M2 holes .............. VERIFIED (photo analysis)
- M3x6 centre horn screw, M2 self-tap arm screws ................. VERIFIED (Hiwonder Q&A)
- Side-flange M3 holes, ~31 mm spacing ........................... VERIFIED (photo analysis)
- PH2.0-3P connector, 200 mm wire, 9-12.6V, 115200 baud .......... VERIFIED
- Shaft protrusion each side (8 mm) .............................. ASSUMED (reasonable)
- Mounting-flange hole-to-end offset (4 mm) ...................... ASSUMED

## Bearings / fasteners / sensors
- 626ZZ 6x19x6 (hip+knee) ........................................ VERIFIED (bearingsdirect)
- 608ZZ 8x22x7 (reserved) ........................................ VERIFIED
- M3 socket cap + heat-set insert (od4 x5) ....................... VERIFIED typical / ASSUMED len
- Omron D2F-L switch body 12.2x6x6.5, hinge lever ~12 ............ VERIFIED body / ASSUMED lever
- RPi 4B 85.6x56.5, holes 58x49 PCD, 2.9mm ...................... VERIFIED (forum) + standard
- Pico 51x21, 2.1mm holes ....................................... VERIFIED standard
- 3S 2200mAh LiPo 106x34x24, XT60, 168g ......................... VERIFIED (HRB/CNHL listings)
- Hiwonder bus servo controller 58x42x12 ........................ VERIFIED (wiki)

## Robot scale (ASSUMED - Spot-class, kinematically validated)
- Overall height 364 mm (measured built; old PARAMS 440 corrected) .... VERIFIED larger / MEASURED exact
- Hip height 300 mm, link lengths 175+175 mm ..................... VERIFIED via assembly (hip 300.1), standing_margin +82 PASS
- Stance 437 x 168 mm (measured built; old 460x320 corrected) ... MEASURED from assembly footprint
- Ground clearance 190 mm ....................................... MEASURED (hip 300 - torso 110)
- Overall length (arm extended +X) 666 mm (complete gear/cam gripper) .. MEASURED (validate_robot 29/29); was ~596 in stub-gripper era

## Manufacturing
- A1 Mini build volume 180^3 .................................... VERIFIED (spec)
- Every printable part < 180 in X/Y/Z ........................... VERIFIED (validate_robot PASS)
  - leg upper 176x54x30, lower 177x40x42, foot 48x64x35
  - torso front 170x157x133, rear 178x157x133, head 90x70x70
- Torso split at x=0 into 2 halves + seam pins/bolts for plastic fusion .. VERIFIED

## Joints / mechanics
- Hip & knee bearing-supported (626ZZ) so servo shaft not overloaded .. DESIGNED
- Knee servo drives lower link via 22 mm horn (torque at link) ........ DESIGNED
- Hard-stop bosses, cable channels, heat-set inserts .................. DESIGNED
- FEA structural check (PETG, linear-static, scikit-fem+gmsh) ......... DONE (fea_structural.py)
  - hip bulkhead: 1.9 MPa, 0.01 mm  -> PASS (yield 45 MPa)
  - upper-link yoke root: 141 MPa peak under EXTREME single-leg-overload
    (58.9 N = whole 6 kg on one leg, a crash). Smooth-tube bending ~6 MPa
    (7.5x yield margin); at 4-leg stance ~1.5 MPa. Root yields only in crash.
  - NOTE: isotropic PETG; excludes FDM layer anisotropy, dynamic & fatigue.
    Is a real finding; upper-link yoke root needs gusset before hard landings.
  - HANDOFF DECISION (§6): yoke gusset NOT added. A clean, self-intersection-free
    gusset could not be justified without also adding mass/print-complexity for a
    crash-only case. Design kept intact; limitation documented. Revisit only if
    hard-landing testing shows real yield. Do NOT create broken geometry to lower the number.
- Servo torque margin vs robot mass ................................. VERIFIED (calc): 13.6x static, 4.5x dyn 3g @6kg
  (analyze_loads.py 4/4 PASS - calculation only, not FEA/physical test)

## Assembled robot
- Full size 666 x 245 x 442 mm (standing, IK-posed, arm+head, complete gripper) .. VERIFIED (validate_robot 29/29)
- 4 legs from ONE architecture via transform (FL/FR/RL/RR) ......... VERIFIED
- Feet stand on ground (z=0), hip height 300.1 ................... VERIFIED (validate_robot)
- Head clears torso, sensor pockets present ....................... VERIFIED (2-DOF pan/pitch actuated)
- Front 6-DOF arm recessed 22 mm into torso port, not embedded .... VERIFIED (arm.py + validate)
  - gray PETG (owner standard, NOT black); 6x HTD-45H; 500 g payload target
  - all 4 arm sub-assemblies fit A1 Mini (<180) ................... VERIFIED
  - GRIP SERVO-IN-WRIST: actuator in forearm, tool stays passive . VERIFIED (validate_wrist 10/10)
  - Passive interchangeable gripper (no servo/motor/batt/ctrl) .... VERIFIED (22 g PETG, <=80 g)
  - Transmission: driven gear -> coupler gear -> eccentric cam -> 2 fingers . VERIFIED (gear mesh 18 mm, stroke 72.9 deg)
  - Wrist §3 concerns A-E ALL RESOLVED ......................... VERIFIED (validate_wrist.py 10/10)
  - Backdrive when off: HTD-45H holding torque 4.41 N.m >> 0.147 N.m payload reaction . VERIFIED (passive, no 2nd actuator)
- Metal shaft reinforcement (§4) ................................ DESIGNED + ANALYZED (analyze_shafts 5/5)
  - 6 mm ground steel shaft through 626ZZ at hip/knee/shoulder/elbow/wrist . VERIFIED (5.8-44.7x margin)
  - bearing interface 626ZZ 6 mm bore = shaft 6 mm ................ VERIFIED

## Future
- Black robotic arm ............................................... DESIGNED (gray 6-DOF, arm.py; was "black" -> owner std gray)
- Arm servo set (reuse HTD-45H) ................................... REQUIRED (6 arm + 2 head = 8 added)
