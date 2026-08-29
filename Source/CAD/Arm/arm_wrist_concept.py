"""
Ori Robot Dog - ARM WRIST / SERVO-IN-WRIST + PASSIVE GRIPPER (concept, NON-FINAL)

DESIGN PRINCIPLE (user):
  The gripper must NOT carry its own actuator. The wrist servo stays on the arm
  and drives a passive, interchangeable gripper through a light mechanical
  transmission. The end-effector stays light (<=80 g) so more arm capacity is
  free for payload.

This module is a CONCEPT / architecture study. It does NOT replace arm.py
(the frozen baseline). It is a separate, non-destructive proposal used to
establish the correct mechanical architecture and actuator requirement.

Conceptual chain realized here:
  ARM (forearm wrist block)
    -> WRIST SERVO (HTD-45H, shaft along X = roll axis)   [actuator stays on arm]
    -> spur gear on servo shaft (printed, PETG)
    -> idler/transfer printed gear
    -> INTERCHANGEABLE COUPLER (standard mechanical interface, M3 + dowel)
       -> passive CAM/rack that drives two printed fingers
    -> PASSIVE GRIPPER (no servo/motor/battery/controller; <=80 g)

Why a printed spur-gear transmission (not bevel, not linkage, not shaft-only):
  * Geometry: the wrist servo already sits in the wrist block with shaft along X.
    A simple spur pair steps the motion to the coupler face plane. No right-angle
    turn is required for the GRIP function (roll is a separate joint); grip is a
    linear/rotary cam at the tool plane.
  * Printability: spur gears print flat on the bed, no supports, large fillets ok.
  * Stiffness: gear mesh is stiffer and lower-backlash than a long bowden/linkage
    for the same force; finger force is tiny (0.18 N.m) so even modest module works.
  * Serviceability: coupler unscrews (2x M3 + 1 dowel) to swap tools; the gear
    stays on the wrist, reusable across all passive tools.
  * No metal structure; only 626ZZ bearings + M3 hardware where justified.

Torque budget (from analyze_arm_wrist.py):
  gripper closing torque at jaw pivot ~0.18 N.m (0.37 margined). A 2:1 gear
  reduction from the servo means the servo sees ~0.37 N.m at the grip-force side,
  trivially within any HTD-45H-class or smaller servo.

All dimensions parameterized in PARAMS.arm (added: gear_module, coupler_d, ...).
"""
import sys, math
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Master"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Hardware" / "servos"))
from build_common import cq, PARAMS, cyl, box_centered
from htd45h import make_htd45h


def _bearing_ring(od, id_, w, axis="X"):
    o = cyl(od / 2, w, axis=axis)
    i = cyl(id_ / 2, w + 2, axis=axis)
    return o.cut(i)


def _spur_gear(module, teeth, width, axis="X"):
    """Simple printed spur gear: root cylinder + trapezoidal teeth (approx).
    module m [mm], teeth z, width w. Pitch dia = m*z. Axis along X."""
    pd = module * teeth
    r_pitch = pd / 2.0
    r_out = r_pitch + module          # addendum ~ module
    r_root = r_pitch - 1.25 * module
    # body
    g = cyl(r_root, width, axis=axis)
    # teeth as small boxes around the pitch circle (printable approximation)
    for i in range(teeth):
        ang = 2 * math.pi * i / teeth
        tw = module * 1.4   # tooth tangential width
        th = box_centered(tw, 2 * module, width) if axis == "X" else box_centered(width, tw, 2 * module)
        # place at radius r_pitch, oriented tangentially
        tx = r_pitch * math.cos(ang)
        ty = r_pitch * math.sin(ang)
        th = th.translate((0, tx, ty)) if axis == "X" else th.translate((tx, ty, 0))
        # rotate tooth so its long axis is tangential
        g = g.union(th)
    # center bore
    g = g.cut(cyl(PARAMS.arm.bore_d / 2 + 0.1, width + 2, axis=axis))
    return g


# ---------------------------------------------------------------------------
# WRIST BLOCK (servo-in-wrist) — replaces the forearm's distal wrist block
# ---------------------------------------------------------------------------
def make_wrist_block(p=PARAMS):
    a = p.arm
    L = a.elbow_len
    # forearm tube (carried from make_forearm concept) - here just the wrist end
    block = box_centered(26, a.link_w - 2, a.link_t + 2).translate((L - 13, 0, 0)).edges("|Y").fillet(2.0)
    # WRIST SERVO seated in the block, shaft along X (roll axis) -> stays ON THE ARM
    srv = make_htd45h(p).rotate((0, 0, 0), (0, 1, 0), 90).translate((L - 13, 0, 0))
    # drive gear on servo shaft (small module, few teeth) -> transmits to coupler
    drive = _spur_gear(a.gear_module, a.gear_drive_teeth, 8, axis="X").translate((L - 13, 0, 0))
    # idler gear transferring to the coupler plane (slightly outboard)
    idler = _spur_gear(a.gear_module, a.gear_idler_teeth, 8, axis="X").translate((L - 13 + a.gear_pitch_d/2 + a.gear_idler_pitch_d/2, 0, 0))
    return block.union(srv).union(drive).union(idler)


# ---------------------------------------------------------------------------
# INTERCHANGEABLE COUPLER + PASSIVE GRIPPER (no actuator)
# ---------------------------------------------------------------------------
def make_passive_gripper(p=PARAMS):
    a = p.arm
    # coupler: standard interface disk (M3 + dowel) that receives the wrist gear
    coupler = cyl(a.coupler_d / 2, 10, axis="X")
    coupler = coupler.cut(cyl(a.bore_d / 2 + 0.1, 12, axis="X"))           # roll bore
    # 2x M3 tool-retention holes + 1 dowel (standard interface)
    for ang in (math.radians(120), math.radians(240)):
        bx, by = (a.coupler_d / 2 - 4) * math.cos(ang), (a.coupler_d / 2 - 4) * math.sin(ang)
        coupler = coupler.cut(cyl(3.0 / 2 + 0.1, 14, axis="X").translate((0, bx, by)))
    coupler = coupler.cut(cyl(2.0, 14, axis="X").translate((0, 0, 0)))     # dowel
    # driven gear on the coupler (meshes wrist idler); rotates a cam
    cgear = _spur_gear(a.gear_module, a.gear_idler_teeth, 8, axis="X").translate((0, 0, 0))
    coupler = coupler.union(cgear)
    # passive CAM: eccentric on the gear shaft pushes the two fingers apart/close
    cam = cyl(6.0, 22, axis="Y").translate((0, 0, 0))      # cam disc, axis Y, drives fingers in Y
    # two printed fingers (passive), pivot at coupler face, pads at tips
    finger = box_centered(a.grip_depth, a.finger_w, 20).translate((a.grip_depth / 2 + 4, 0, 0))
    finger = finger.cut(cyl(2.0, 22, axis="Z").translate((2, a.finger_w / 2, 0)))   # pivot
    # grip pad slot (replaceable)
    finger = finger.cut(box_centered(4, a.finger_w + 1, 6).translate((a.grip_depth - 2, 0, 0)))
    f1 = finger.translate((0, +a.grip_span / 2 - a.finger_w / 2, 0))
    f2 = finger.translate((0, -a.grip_span / 2 + a.finger_w / 2, 0))
    return coupler.union(cam).union(f1).union(f2)


# ---------------------------------------------------------------------------
# CONCEPT ASSEMBLY (parked pose) - for visualization / mass check only
# ---------------------------------------------------------------------------
def make_wrist_concept(p=PARAMS):
    a = p.arm
    wb = make_wrist_block(p)
    grip = make_passive_gripper(p).translate((a.elbow_len - 13 + 16, 0, 0))
    compound = wb.union(grip)
    return {"wrist_block": wb, "passive_gripper": grip, "full": compound}


if __name__ == "__main__":
    import build_common as bc
    out = make_wrist_concept()
    print("passive gripper size:", [round(v, 1) for v in bc.size_of(out["passive_gripper"])])
    print("wrist block size   :", [round(v, 1) for v in bc.size_of(out["wrist_block"])])
    gp_mass = out["passive_gripper"].val().Volume() * 1.27e-3
    print("passive gripper PETG mass (g):", round(gp_mass, 1), "(target <=80 g)")
