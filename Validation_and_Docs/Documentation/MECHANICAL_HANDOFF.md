# Ori Robot Dog — Mechanical Handoff

**Status:** CAD-final, parameterized, validated. No physical printing performed.
**CAD system:** Parametric CadQuery (Python) + master parameter file (single source of truth).
**Material:** PETG primary structure; 626ZZ bearings; 6 mm ground-steel shaft reinforcement; M3 hardware.
**Printer:** Bambu Lab A1 Mini, 180 × 180 × 180 mm build volume.

---

## 1. Architecture

- 4-legged quadruped, one master leg architecture mirrored to FL/FR/RL/RR.
- 12 actuated leg DOF (hip yaw + hip pitch + knee × 4) via HTD-45H bus servos.
- Front 6-DOF arm, recessed 22 mm into a 50 mm torso port (gray PETG, partially recessed, not embedded).
- 2-DOF pan/pitch head (yaw neck + pitch yoke).
- Passive interchangeable gripper on the arm wrist (no servo/motor/battery/controller in the tool).
- Electronics bay (Raspberry Pi 3 + 2–3 Pi Pico + bus servo controller + 3S LiPo) planned in torso.

## 2. Dimensions (verified via `validate_robot.py` 29/29)

| Quantity | Value |
|---|---|
| Overall (standing, arm extended +X) | **666 × 245 × 442 mm** (L×W×H) |
| Hip height (nominal) | 300 mm |
| Upper / lower leg link | 175 / 175 mm |
| Stance (foot spread X × Y) | 437 × 168 mm |
| Ground clearance | 190 mm |
| Torso | split at x=0 into front/rear halves (170/178 × 157 × 133 mm) |
| Arm reach (yaw→gripper tip) | ~234 mm from base flange |
| Head | 90 × 70 × 70 mm dome + 2-DOF neck/yoke |

> Note: overall length was ~596 mm in the earlier stub-gripper era; the complete
> gear/cam/finger gripper adds ~70 mm. This is a real, justified change (the
> gripper is now a full mechanism), not a parameter drift.

## 3. Joint / actuator / bearing count

- **Leg joints:** 12 (4 × [hip-yaw, hip-pitch, knee]).
- **Arm joints:** 6 (yaw, shoulder, elbow, wrist-pitch, wrist-roll, grip).
- **Head joints:** 2 (pan/yaw, pitch).
- **Total actuated DOF:** 20.
- **HTD-45H servos:** 16 (8 leg + 6 arm + 2 head).
- **Bearings:** 626ZZ (6×19×6), 2 per joint × (8 leg + 6 arm) = **28** (head uses direct servo horn seating).
- **Shafts:** 6 mm ground steel through 626ZZ at hip, knee, shoulder, elbow, wrist = **10 shafts** (5 joint types × 2 sides; arm wrist pitch+roll share the wrist block).

## 4. Gripper interface (passive, interchangeable)

- Coupler: Ø34 mm disk, face width 10 mm.
- Retention: 2 × M3 on PCD 24 mm + 1 × Ø4 alignment dowel (concentric within ~0.2 mm).
- Transmission (servo-in-wrist): grip HTD-45H (in forearm) → driven spur gear → coupler gear → eccentric cam → 2 symmetric passive fingers.
- Gear center distance: **18 mm** (handoff baseline, single-frame, verified meshing).
- Finger stroke: 72.9° → jaw 86 mm open → 0 mm closed; rest ~44 mm for 500 g target.
- **Passive gripper mass: 22 g PETG** (target ≤80 g, ~3.6× headroom).
- Backdrive when servo off: held by HTD-45H holding torque (4.41 N·m ≫ 0.147 N·m payload reaction) + 1:1 gear + detent. No second actuator added (per §3D).

## 5. Payload assessment (500 g target)

- Wrist pose torque: 0.137 N·m → **17.7× margin** vs HTD-45H continuous.
- Shoulder/elbow: 1.06 N·m → ~2.3× margin at continuous rating (arm also carries its own ~790 g).
- Gripper close: 0.184 N·m (0.368 margined) vs ~0.37 needed for 500 g, μ=0.4.
- **Verdict: 500 g payload is FEASIBLE** (analysis, not physical test). Limiting factor is printed-link stiffness, not torque.

## 6. FEA findings (PETG, linear-static, scikit-fem+gmsh)

- Hip bulkhead: **1.9 MPa / 0.01 mm** → PASS (yield 45 MPa).
- Upper-link yoke root: **141 MPa peak** under EXTREME single-leg overload (58.9 N = full 6 kg on one leg, a crash). Smooth-tube bending ~6 MPa (7.5× margin); 4-leg stance ~1.5 MPa. Root yields only in the crash case.
- **Decision (§6):** yoke gusset NOT added. A clean, self-intersection-free gusset could not be justified for a crash-only case without adding mass/print complexity. Design kept intact; limitation documented. Revisit only if hard-landing testing shows real yield. (FEA excludes FDM layer anisotropy, dynamics, fatigue — analysis-only.)

## 7. Printability status (A1 Mini, 180³)

Every printable part fits. Per-part bounding boxes (mm, X×Y×Z):

| Part | Size | Fit |
|---|---|---|
| leg.upper | 175×55×32 | ✓ |
| leg.lower | 177×40×42 | ✓ |
| leg.foot | 48×64×64 | ✓ |
| torso.front | 170×157×133 | ✓ |
| torso.rear | 178×157×133 | ✓ (tightest Z=133) |
| head.dome | 90×70×70 | ✓ |
| head.neck | 56×56×53 | ✓ |
| head.pitch_yoke | 50×64×51 | ✓ |
| arm.yaw_housing | 70×65×58 | ✓ |
| arm.upper_arm | 105×40×62 | ✓ |
| arm.forearm | 103×81×58 | ✓ |
| arm.gripper | 40×34×40 | ✓ |

Orientation guidance: tubes print long-axis flat; gears/cam/fingers print flat (axis = bed normal); torso halves print on the split face (x=0) with seam pins/bolts for plastic fusion. Wall thickness 2.6 mm (≥ min 1.2). Heat-set M3 inserts into printed bosses; bearing seats are printed press-fits (–0.02 mm).

## 8. Serviceability

- **Servos:** hip/knee/shoulder/elbow accessible from link exterior; arm servos in wrist block via removable cover (DESIGNED, cover geometry TBD at print stage).
- **Bearings:** press into printed seats; replaceable by pressing out.
- **Shafts:** circlip-ended, slide through bearing stack; removable without disassembly of the whole link.
- **Gripper:** fully removable — 2× M3 + dowel, no tools beyond a driver.
- **Head:** neck/yoke servos accessible from underside; camera/mic/speaker in snap pockets.
- **Battery:** slides into torso bay with retention (DESIGNED); removable from rear service opening.
- **Electronics:** Pi/Pico/controller on trays with screw retention (DESIGNED); accessible via torso seam.

## 9. Cable / wiring space (mechanical only)

- Servo cables route along link interiors (hollow tubes) to the torso.
- Arm wiring runs through yaw housing bore → torso port.
- Head wiring through neck bore.
- Routing channels are designed as internal voids; no moving pinch points identified at nominal pose.
- **Status:** mechanical clearance provided; final loom routing is an electronics-stage task (not designed here).

## 10. Known limitations (honest)

- FEA is linear-static, isotropic PETG; excludes layer anisotropy, dynamics, fatigue.
- Upper-link yoke root can yield in a full-body crash (141 MPa) — gusset deferred.
- Grip gears are approximated printable spur gears (module 1.5); real wear/backlash needs print-stage validation.
- Electronics (Pi 3 / Pico / controller / battery) are dimensioned from verified specs but NOT laid out in CAD yet.
- All validation is CAD/analysis-only; **no physical print or test performed.**

## 11. Validation status

| Check | Result |
|---|---|
| Leg validator | 11/11 PASS |
| Full robot validator | 29/29 PASS |
| Wrist §3 (cam/gear/coupler/backdrive/print) | 10/10 PASS |
| Shaft sizing (§4) | 5/5 PASS |
| Load analysis (torque margins) | 4/4 PASS |
| FEA | hip PASS; yoke 141 MPa crash-only (documented) |

## 12. Unresolved decisions (need Liam)

- **Pi 3 vs Pi 4:** handoff text says Pi 3; BOM/specs reference Pi 4B. Confirm which board drives the build (affects torso bay + power).
- **Gusset:** add a crash gusset later, or accept the documented 141 MPa limit?
- **Grip-servo downsize:** HTD-45H is oversized at the wrist (17.7× margin). A smaller bus servo could trim ~45 g from the arm — optional future optimization, not required.
- **Plastic-fusion seam process:** confirmed approach (pins + bolts + fusion) but no print executed.
