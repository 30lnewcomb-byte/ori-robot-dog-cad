# Ori Mechanical Handoff — Final Delta Report

Generated: 2026-08-25. All checks are CAD/analysis-only; no physical printing performed.

## 1. What changed
- **Gripper architecture reverted to the handoff baseline** (driven spur gear → coupler gear →
  eccentric cam → 2 passive fingers) with gear center distance fixed at **18 mm** (was a
  worm/rack design whose "self-locking" was only an approximation). Grip servo stays in the
  wrist — NOT moved into the tool.
- **Added metal shaft reinforcement (§4):** parametric 6 mm ground-steel shaft + 626ZZ bearing
  stacks at hip/knee/shoulder/elbow/wrist (`CAD/Master/shafts.py`), with a sizing analysis
  (`Validation/analyze_shafts.py`).
- **Rewrote `Validation/validate_wrist.py`** to prove the §3 A–E items (cam travel, gear pair,
  coupler, backdrive, print orientation).
- **Updated master parameters** with gear/cam/shaft fields; grip_span tuned to 44 mm.
- **Docs/BOM/exports** updated for handoff.

## 2. What was validated
| Suite | Result |
|---|---|
| Leg validator | 11/11 PASS |
| Full robot validator | 29/29 PASS |
| Wrist §3 (A–E) | 10/10 PASS |
| Shaft sizing (§4) | 5/5 PASS |
| Load analysis (torque margins) | 4/4 PASS |
| FEA | hip bulkhead PASS (1.9 MPa); yoke 141 MPa CRASH-ONLY (documented) |

## 3. What passed
- Gear/cam gripper genuinely closes (86 mm open → 0 mm closed, 72.9° stroke, 18 mm mesh).
- Backdrive when servo off: HTD-45H holding torque 4.41 N·m ≫ 0.147 N·m payload reaction (passive, no 2nd actuator).
- All 12 printable parts fit A1 Mini 180³.
- 6 mm standardized shaft adequate at every joint (5.8–44.7× margin), bearing interface 626ZZ=6 mm.
- Full robot assembles, stands, 16 HTD-45H consistent, arm recesses 22 mm (not embedded), head 2-DOF.

## 4. What remains intentionally unresolved
- **Upper-link yoke root 141 MPa** under full-body crash — gusset deliberately NOT added (clean
  gusset not justified for crash-only case; design kept intact, limitation documented).
- **Pi 3 vs Pi 4** discrepancy in handoff text vs BOM (needs Liam's call).
- **Print-stage items:** real gear wear/backlash, plastic-fusion seam strength, heat-set insert
  pull-out, dynamic FEA — none executed (no printing).
- **Servo IDs** not assigned (firmware-stage).

## 5. Exact final robot dimensions
- **666 × 245 × 442 mm** (L×W×H, arm extended +X, standing, head).
- Hip height 300 mm · links 175+175 mm · stance 437×168 mm · ground clearance 190 mm.

## 6. Exact actuator count
- **16 × HTD-45H** (8 leg + 6 arm + 2 head). Actuated DOF = 20 (12 leg + 6 arm + 2 head).

## 7. Exact bearing / shaft requirements
- **28 × 626ZZ** (6×19×6): 2 per joint × (8 leg + 6 arm).
- **10 × 6 mm ground-steel shafts** (hip, knee, shoulder, elbow, wrist × sides as applicable),
  circlip-ended, through 626ZZ.

## 8. Final passive gripper mass
- **22 g PETG** (target ≤80 g). Transmission servo-in-wrist; tool has no servo/motor/battery/controller.

## 9. Final 500 g payload assessment
- FEASIBLE (analysis): wrist 17.7× torque margin, shoulder/elbow 2.3×, gripper 6.6×. Limiting
  factor is printed-link stiffness, not torque.

## 10. Files created / updated
- NEW: `CAD/Master/shafts.py`, `Validation/analyze_shafts.py`, `Validation/validate_wrist.py` (rewritten),
  `CAD/Exports/export_handoff.py`, `Documentation/MECHANICAL_HANDOFF.md`,
  `Documentation/software_handoff_jointmap.json`.
- UPDATED: `Parameters/master_parameters.py`, `CAD/Arm/arm.py`, `Validation/validate_robot.py`,
  `BOM/BOM.csv`, `Documentation/VERIFICATION_STATUS.md`, `Documentation/README.md`.
- PRESERVED (untouched): `CAD/Arm/reference/arm_baseline.py`.

## 11. Export locations
- `C:\Users\Liam Newcomb\Desktop\Ori Robot Dog\CAD\Exports\handoff\` — 30 files
  (STEP+STL for leg/torso/head/arm parts; STEP for full robot + shafts).
- Separate from development/reference files; baseline NOT overwritten.

## 12. Issue requiring Liam's decision
- **Pi 3 (handoff control arch) vs Pi 4B (BOM/specs).** This changes the torso electronics bay
  and power budget. Everything else is mechanically complete and validated as CAD.
- Secondary: confirm whether to (a) leave the 141 MPa crash case as documented, or (b) invest in
  a yoke gusset later.
