ORI ROBOT DOG — CAD REPOSITORY
==============================

This repository is the mechanical/CAD workspace for Ori Robot Dog.

SOURCE OF TRUTH
  Source/Parameters/master_parameters.py   Master parameters for generated CAD.
  Source/CAD/                              Parametric Python/CadQuery generators.
  CAD_STEP/                               Authoritative high-fidelity STEP deliverables.
  CAD_STL/                                Print-ready STL deliverables.
  OpenSCAD/                               Simplified text-readable previews only.
  Validation_and_Docs/                    Validation, engineering analysis, BOM, and handoff.

IMPORTANT: STEP/STL exports are deliverables, not the design source. Regenerate them from
parametric source after a design change.

CURRENT CAD STATE
  Robot envelope: validated by the current assembly build (exact dimensions are reported by
                  the validation suite rather than copied here).
  Leg architecture currently generated: 2 actuated joints per leg (hip pitch + knee).
  Arm: 6-DOF with servo-in-wrist passive gripper.
  Head: 2-DOF pan + pitch.
  Current generated actuated DOF: 16 (8 legs + 6 arm + 2 head).
  Target architecture: 20 DOF (12 leg + 6 arm + 2 head).

The four hip-yaw joints exist in the software target map but are NOT implemented in the current
leg CAD. See Validation_and_Docs/Documentation/KNOWN_BLOCKERS.md before treating those joints as real.

ENGINEERING RULES
  Read ENGINEERING_RULES.md. AI-generated geometry is not accepted by appearance alone.
  Meaningful changes must be checked for geometry, interfaces, assembly, motion, manufacturability,
  structural assumptions, mass effects, and downstream breakage.

VALIDATION
  GitHub Actions runs the static repository audit, CadQuery smoke build, leg validation, robot
  validation, wrist validation, load analysis, shaft analysis, and arm/wrist load analysis.
  These are CAD/calculation checks only; no physical printing or hardware validation is implied.

UNRESOLVED ENGINEERING ITEMS
  - Implement or deliberately remove/freeze the four hip-yaw joints.
  - Pi 3 vs Pi 4 controller selection still affects electronics packaging.
  - Upper-link crash-case reinforcement remains intentionally deferred pending physical evidence.
  - Physical fit, backlash/wear, insert pull-out, seam strength, dynamics, and FDM anisotropy remain
    unverified.
