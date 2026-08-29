# Ori Robot Dog - Printable Part Breakdown (Bambu Lab A1 Mini, 180^3 mm)

Every printable part below was checked to fit the A1 Mini build volume. Parts
larger than 180 mm are split and joined with alignment pins + M3 screw bosses
designed for plastic fusion (per requirement #17).

## Per-part list (MAX dimension in each axis, mm)
| Part | X | Y | Z | Qty (x4 legs) | Notes |
|------|---|---|---|---------------|-------|
| leg_upper_link | 176 | 54 | 30 | 4 | single print, print flat (X along bed) |
| leg_lower_link | 177 | 40 | 42 | 4 | print flat |
| leg_foot | 48 | 64 | 35 | 4 | print sole-down |
| torso_front_half | 170 | 157 | 133 | 1 | print on side; <180 on all axes |
| torso_rear_half | 178 | 157 | 133 | 1 | print on side |
| head_shell | 90 | 70 | 70 | 1 | easy |
| head_neck | 56 | 56 | 30 | 1 | |

## Splitting strategy
- TORSO: full shell 340 long -> two halves at x=0 (170 / 178 long). Joined by:
  * 2 alignment pins (4 mm dia) across the seam
  * 3 M3 screw bosses (z = -H/4, 0, +H/4) for plastic-fusion + screw backup
  * flat seam face (6 mm plate) maximizes glue/fusion surface area
- LEGS: each link <180 in all axes -> NO split needed. The hollow box-tube
  section prints cleanly on its side with the internal cable channel as a
  bridging feature (use support = off where possible, sparse where needed).
- HEAD: dome + neck both <180 -> single prints.

## Recommended print orientation (FDM stiffness, requirement #18)
- Links: lay along X (long axis on bed); wall loads act in Y (bending plane) ->
  layer lines run across the bending direction for max inter-layer strength.
- Torso halves: print on the largest flat face; ribs printed in-place.
- Feet: print sole-down so the contact face is flat and the rim prints upward.

## Assembly sequence (serviceability, requirement #19)
1. Press 626ZZ bearings into hip/knee bores (interference fit, -0.02 mm).
2. Bolt HTD-45H horns to upper-link proximal bosses (6x M2).
3. Mount knee servo into lower-link saddle; connect horn to fork.
4. Insert hip servo into torso bulkhead pocket; bolt upper link to horn.
5. Join knee fork to upper-link clevis with knee pin + bearings.
6. Bolt foot ankle cup to lower-link ankle; seat D2F switch in sole.
7. Repeat for 4 legs (mirror FR/RL/RR).
8. Install battery, RPi, Picos, controller, head (neck into torso top).
9. Plastic-fuse torso halves; verify seam.
10. (Future) remove front arm-port blanking plate; mount black arm.

## Heat-set inserts
M3 inserts into: RPi tray bosses (4), servo controller (4), head neck flange (3),
arm-port blanking plate (4), battery retention (optional).
