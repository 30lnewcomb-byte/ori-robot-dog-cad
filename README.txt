ORI ROBOT DOG — MECHANICAL HANDOFF (drop-in repository root)
=============================================================
Status: CAD-final, parameterized, validated. No physical printing performed.

This repository has FIVE top-level folders. They are the whole project.

1) Source/            Parametric Python/CadQuery CAD (master_parameters.py = single source of truth).
                        `pip install -r Source/requirements.txt` to set up the environment.
2) Validation_and_Docs/  Validators (all PASS) + MECHANICAL_HANDOFF + BOM + jointmap.json.
3) OpenSCAD/          Plain-text, AI-readable VIEW models (.scad). GitHub & ChatGPT can read these.
                        NOTE: these are simplified summaries, NOT the manufactured parts.
4) CAD_STEP/          THE AUTHORITATIVE GEOMETRY (18 .step files). Use these for printing/machining.
                        See CAD_STEP/READ_ME_FIRST.txt — the better files are NOT in OpenSCAD, they are HERE.
5) CAD_STL/           Print-ready meshes (12 .stl) matched to the STEP parts.

WHERE IS THE GOOD CAD?
  The high-fidelity STEP geometry is in CAD_STEP/ (and STL in CAD_STL/). The OpenSCAD/
  folder is only a readable preview so ChatGPT/GitHub can 'see' the robot in text. Both are
  in this repo; nothing is missing.

KEY NUMBERS
  Robot 666x245x442 mm. 16x HTD-45H. 20 DOF (12 leg+6 arm+2 head).
  28x 626ZZ bearings. 10x 6mm steel shafts. Passive gripper 22g. 500g payload feasible (analysis).
  Validators: leg 11/11, robot 29/29, wrist 10/10, shafts 5/5.

DECISIONS NEEDED (see Validation_and_Docs/Documentation/MECHANICAL_HANDOFF.md)
  - Pi 3 (control arch) vs Pi 4B (BOM): confirm board.
  - Accept documented 141 MPa upper-link crash case, or add gusset later.
