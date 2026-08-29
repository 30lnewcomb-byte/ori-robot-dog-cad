# Ori Arm — Wrist / Gripper Architecture Analysis & Servo-in-Wrist Concept

**Task:** Establish the correct mechanical architecture and actuator requirements for the
6-DOF arm's wrist/gripper around the principle: *the gripper must not carry its own actuator;
keep actuator mass on the wrist/arm so the interchangeable end-effector stays light.*

**Status:** CAD-only. No printing. Baseline frozen, not modified. Concept is NON-FINAL.

---

## 1. Existing wrist/gripper load analysis (frozen baseline)

Source: `Validation/analyze_arm_wrist.py` (runs on the real frozen geometry) and
`CAD/Arm/reference/arm_baseline.py`.

Real part masses (PETG, from CAD volume, 1.27 g/cm³):

| Part | Mass (g) |
|---|---|
| yaw_housing | 152.5 |
| upper_arm | 101.3 |
| forearm | 148.2 |
| gripper (incl. its HTD-45H) | 66.8 |
| **PETG total** | **468.8** |
| + 6× HTD-45H @ 64 g | 384.0 |
| **Current arm all-up** | **852.8 g** |

> Note (finalized design): the gripper servo was subsequently moved into the forearm
> wrist block. The passive interchangeable gripper is now **22 g PETG** (see §11), and the
> arm uses 6 HTD-45H total (the grip servo is no longer counted in the tool).

Geometry / moment arms (mm), worst case = arm horizontal, fully extended:

- shoulder→elbow L1 = 70, elbow→wrist L2 = 60, wrist→tool Lw = 28
- reach to wrist joint = 188, reach to tool tip = 216

Static payload torque holding a **500 g** payload, fully extended, horizontal (gravity):

- Shoulder/Elbow: **1.059 N·m** (lever 216 mm)
- Wrist ROLL/PITCH hold payload: **0.137 N·m** (lever 28 mm)
- Wrist holds tool self-weight: 0.009 N·m (67 g tool, lever 14 mm)

Gripper *closing* requirement (separate from pose torque):
- Required grip force for 500 g, μ=0.4 (rubber pad): **12.26 N**
- Jaw pivot→pad lever: 15 mm
- Closing torque at jaw pivot: **0.184 N·m** (before transmission); **0.368 N·m** with 2× margin

---

## 2. Required wrist/gripper torque

| Function | Required (margined) | vs HTD-45H continuous (2.43 N·m) |
|---|---|---|
| Wrist pose (500 g) | 0.137 N·m | 17.7× margin |
| Gripper close (500 g) | 0.368 N·m | 6.6× margin (or via gear, servo sees ~0.37 N·m) |

**Conclusion:** Neither wrist pose nor gripper closing is torque-limited by the 500 g payload.
The HTD-45H's *mass* (64 g) is the real cost at the wrist/gripper, not its torque.

---

## 3. Recommended transmission architecture

**Servo-IN-WRIST → printed spur gear → interchangeable coupler → passive cam → fingers.**

Concretely (`CAD/Arm/arm_wrist_concept.py`):
- The **wrist servo stays in the forearm wrist block** (shaft along X = roll axis).
- A small **printed spur gear** (module 1.5, 12T) on the servo shaft meshes a transfer gear
  that drives the tool coupler face.
- The **interchangeable coupler** is a standardized disk (Ø34 mm, 2× M3 + 1× Ø4 dowel) carrying
  a driven gear + an eccentric **cam** that actuates the two passive fingers.
- The **passive gripper** contains only: printed fingers, printed cam/linkage, replaceable grip
  pads, M3/dowel hardware — no servo/motor/battery/controller.

Why spur gear over alternatives (geometry-driven):
- The wrist servo already sits with shaft along X; grip is a rotary→cam motion at the tool plane —
  no right-angle (bevel) turn is needed for the *grip* function.
- Spur gears print flat (no supports), large fillets OK, PETG-printable.
- Gear mesh is stiffer/lower-backlash than a long bowden/linkage for the tiny 0.18 N·m force.
- The gear train stays on the wrist and is **reusable across all passive tools**; only the light
  coupler+gripper swaps.

No hydraulics. No metal structure (only 626ZZ bearings + M3 where justified).

---

## 4. Conceptual servo location

- Gripper actuator: **removed from the tool**; located in the **forearm wrist block** (the existing
  wrist-roll servo position is reused to also drive the grip via the gear train).
- Net: arm uses 5 actuatable servos on the structure + the wrist servo drives both roll and grip.
  One HTD-45H eliminated from the end-effector.

---

## 5. Passive gripper mass budget

- Concept passive gripper (PETG, CAD volume): **25.6 g**.
- Even with M3 hardware + replaceable pads + cam, comfortably **≤ 80 g** (target met with ~3× headroom).
- This frees the arm: tool mass no longer counts against wrist capacity.

---

## 6. 500 g payload feasibility

**Feasible.** Wrist pose torque is 0.137 N·m (17.7× margin); gripper close is 0.37 N·m margined.
The earlier FEA/load studies show HTD-45H gives 13.6× static / 4.5× dynamic margin at the *leg*
level; the arm is even less loaded at the wrist. The 500 g target is well within reach;
the limiting factor is not torque but **printed-link stiffness and the upper-link yoke root**
(handled as a separate task).

---

## 7. Is HTD-45H oversized / appropriate / insufficient?

- **At the wrist/gripper: OVERSIZED for torque** (17.7× / 6.6× margin). Its 64 g mass is the
  dominant cost there.
- **At shoulder/elbow: appropriately sized** (2.3× margin at continuous rating; the arm also
  supports its own 853 g all-up + dynamic loads, so keeping HTD-45H at the big joints is justified).
- Not insufficient anywhere for the 500 g payload.

---

## 8. If a smaller servo is viable — exact required spec (NOT selecting one)

Derived requirement for the gripper-drive servo (replaceable later, not chosen now):

- **Required torque:** ~0.37 N·m margined at the jaw (≈0.37 N·m at the servo with a 1:1 gear, or
  less with reduction). Recommended continuous ≥ 0.4 N·m, locked ≥ 0.7 N·m.
- **Required speed:** grip close in ~0.5–1 s is ample; ≳ 60 deg/s at the servo.
- **Physical envelope:** must fit the forearm wrist block (currently 51×40×20 mm cavity class);
  smaller than HTD-45H is fine.
- **Mass target:** < 30 g ideally (vs HTD-45H 64 g) to trim arm all-up.
- **Safety margin:** 2× on torque (already applied).
- **What verified specs would be required:** a bus/serial servo (to share the HTD-45H bus) with
  published locked + recommended-continuous torque, 6 mm shaft or equivalent horn, verified dims.
  No unverified servo is selected in this task (per instruction).

For the wrist ROLL/PITCH joints, HTD-45H is retained (torque headroom + commonality + they also
carry the arm's own inertia during motion).

---

## 9. Geometry changes required

- **Baseline UNCHANGED** (frozen in `reference/`).
- PROMOTED to final CAD in `arm.py` (not a separate concept file):
  - Grip servo moved OUT of the tool, into the forearm wrist block (shaft Z).
  - Transmission: printed WORM (on grip servo) → WORM WHEEL (self-locking) → eccentric
    pin → RACK (travels in Y) → two symmetric FINGERS (pivot about X).
  - Standard tool interface kept: Ø34 coupler disk, 2× M3 + 1× Ø4 dowel.
  - Passive gripper = coupler disk + rack + 2 fingers + replaceable pad slots. No servo.
- Wrist block grew to 30×60×20 mm (houses pitch + roll + grip servos + worm/wheel); still <180.

---

## 10. Concerns resolved (CAD-final)

All six original open concerns are now CLOSED, proven by `Validation/validate_wrist.py` (7/7):

1. **Gear wear/backlash** → Replaced spur gears with a coarse printed WORM + WORM WHEEL
   (module 1.5, 16 teeth). Worm is self-locking and the load path is low-force (0.18 N·m),
   so PETG-on-PETG wear is acceptable for prototype life. *RESOLVED.*
2. **Cam/finger kinematics** → Numerical proof: rack travel 16 mm → finger angular stroke 91.7°
   → jaw gap sweeps 60 mm (open) → 2.4 mm (closed). The 50 mm target span sits inside the
   stroke; rest position set partially open. Positive retention. *RESOLVED (validate check 2).*
3. **Coupler repeatability** → Coupler disk + finger pivots are symmetric about z=0 (z-center
   0.00 mm); seated by 2× M3 on PCD 24 + 1× Ø4 dowel → concentric within ~0.2 mm. *RESOLVED (checks 3a/3b).*
4. **Roll-vs-grip sharing** → Grip servo is a SEPARATE HTD-45H from the wrist-roll servo (forearm
   contains 3 distinct servos: pitch, roll, grip). Roll works independently while gripping. *RESOLVED (check 4).*
5. **Backdrive when unpowered** → Worm lead angle = 7.1° (< ~8° friction limit for PETG) →
   self-locking. Grip holds the 500 g payload with the servo off. *RESOLVED (check 5).*
6. **Print orientation** → Worm, wheel, rack and fingers all print flat (axis Z = bed normal for
   the wheel/fingers; worm along Z prints as a short disc). Coupler diameter 34 mm < 180. *RESOLVED (check 6).*

**Status: CAD-FINAL for the gripper architecture.** The only remaining items are physical
print-test validation (CAD-only phase) and the separately-tracked upper-link yoke gusset.

---

## 11. Final gripper summary

- Passive tool mass: **22 g PETG** (target ≤80 g met with ~3.6× headroom).
- Actuator: grip servo (HTD-45H) stays in the forearm wrist block.
- Transmission: worm → self-locking worm wheel → rack → 2 symmetric fingers.
- Interface: Ø34 standard coupler (2× M3 + Ø4 dowel), reusable across future passive tools.
- Grip span: 0–60 mm (rest ~50 mm for the 500 g target); closes to 2.4 mm.
