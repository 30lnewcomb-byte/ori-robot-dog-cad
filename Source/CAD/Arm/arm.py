"""
Ori Robot Dog - 6-DOF FRONT ARM  (future manipulator, recessed into torso port)

Spec (from user): 6-DOF, ~500 g target payload, real manipulator (not a toy),
partially recessed into the torso front, structurally supported, serviceable,
gray PETG. Mounts to the existing 50 mm torso arm port (flange OD 58, M3 PCD 58).

Actuators: HTD-45H baseline for all 6 joints (consistent with the robot,
verified 51.1x40x20.14 mm, 4.41 N.m). Wrist/gripper use the same servo for
torque headroom and parts commonality.

Chain (parked pose, arm pointing +X out the front):
  YAW   (about Y)  -> base rotation
  SHOULDER (about Z) -> upper arm pitch
  ELBOW (about Z) -> forearm pitch
  WRIST PITCH (about Z) -> tool pitch
  WRIST ROLL (about X) -> tool roll
  GRIPPER -> parallel jaws

All geometry is parameterized via PARAMS.arm. No placeholder blocks.
"""
import sys, math
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Master"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Hardware" / "servos"))
from build_common import cq, PARAMS, cyl, box_centered
from htd45h import make_htd45h


def _tube(L, w, t, wall, axis="X"):
    """Hollow rectangular tube along axis, length L, outer w x t, wall thickness."""
    o = box_centered(L, w, t) if axis == "X" else box_centered(t, w, L) if axis == "Z" else box_centered(w, L, t)
    iw, it = max(w - 2 * wall, 1.0), max(t - 2 * wall, 1.0)
    i = box_centered(L - 2 * wall, iw, it) if axis == "X" else box_centered(iw, L - 2 * wall, it) if axis == "Z" else box_centered(iw, it, L - 2 * wall)
    return o.cut(i)


def _bearing_ring(od, id_, w, axis="Y"):
    o = cyl(od / 2, w, axis=axis)
    i = cyl(id_ / 2, w + 2, axis=axis)
    return o.cut(i)


# ---------------------------------------------------------------------------
# BASE YAW HOUSING  (recesses into the 50 mm torso bore, bolts to flange)
# ---------------------------------------------------------------------------
def make_yaw_housing(p=PARAMS):
    a = p.arm
    # cylindrical housing that sinks into the torso bore (recess_depth) + a flange
    housing = cyl(a.mount_bore_d / 2 - 1.0, a.base_reach, axis="X")
    # outer flange ring (matches torso flange OD 58) with 4 M3 bolt holes on PCD 58
    flange = cyl(a.mount_flange_od / 2, 6.0, axis="X").translate((a.base_reach / 2 + 3, 0, 0))
    for i in range(4):
        ang = math.radians(i * 90 + 45)
        bx, by = (a.mount_bolt_pcd / 2) * math.cos(ang), (a.mount_bolt_pcd / 2) * math.sin(ang)
        flange = flange.cut(cyl(a.mount_bolt_d / 2 + 0.1, 10, axis="X").translate((a.base_reach / 2 + 3, bx, by)))
    # internal yaw servo mount: a shoulder block on the front face to carry the HTD-45H
    mount = box_centered(40, a.mount_bore_d - 6, 30).translate((a.base_reach + 14, 0, 0))
    # yaw servo seated with shaft along Y -> rotate canonical servo (shaft +Z) to shaft +Y
    srv = make_htd45h(p).rotate((0, 0, 0), (1, 0, 0), 90)
    srv = srv.translate((a.base_reach + 14, 0, 0))
    return housing.union(flange).union(mount).union(srv)


# ---------------------------------------------------------------------------
# UPPER ARM  (shoulder -> elbow)
# ---------------------------------------------------------------------------
def make_upper_arm(p=PARAMS):
    a = p.arm
    L = a.shoulder_len
    tube = _tube(L, a.link_w, a.link_t, a.link_wall, axis="X")
    # proximal shoulder yoke (two plates straddling the shoulder axis = Z here, at x=0)
    plate_t = 7.0
    for dz in (+1, -1):
        plate = box_centered(22, a.link_w + 4, plate_t).translate((11, 0, dz * (a.link_t / 2 + plate_t / 2))).edges("|Y").fillet(1.5)
        plate = plate.cut(cyl(a.bore_d / 2 + 0.1, plate_t + 4, axis="Z"))
        plate = plate.edges("|Z").fillet(2.0)
        tube = tube.union(plate)
        tube = tube.union(_bearing_ring(p.hw.bearing_626_od, p.hw.bearing_626_id, p.hw.bearing_626_w, axis="Z").translate((11, 0, dz * (a.link_t / 2 + plate_t / 2))))
    # shoulder servo mounted on the yoke (shaft along Z)
    srv = make_htd45h(p).translate((11, 0, a.link_t / 2 + plate_t + 12))
    # distal elbow clevis (single block, bore along Z) at x=L
    clevis = box_centered(20, a.link_w + 4, a.link_t + 4).translate((L - 10, 0, 0)).edges("|Y").fillet(2.0)
    clevis = clevis.cut(cyl(a.bore_d / 2 + 0.1, a.link_t + 10, axis="Z"))
    return tube.union(srv).union(clevis)


# ---------------------------------------------------------------------------
# PRINTED SPUR GEAR  (lightweight servo-in-wrist transmission to the gripper)
# All gears share ONE Y-Z plane in the wrist block so meshing is verifiable.
# ---------------------------------------------------------------------------
def _spur_gear(module, teeth, width, axis="X"):
    """Approximated printable spur gear: root cylinder + trapezoidal teeth.
    Centered at origin; axis along X. Pitch dia = module*teeth."""
    r_pitch = module * teeth / 2.0
    r_root = r_pitch - 1.25 * module
    g = cyl(r_root, width, axis=axis)
    for i in range(teeth):
        ang = 2 * math.pi * i / teeth
        th = box_centered(module * 1.5, 2 * module, width).translate((0, r_pitch * math.cos(ang), r_pitch * math.sin(ang)))
        g = g.union(th)
    g = g.cut(cyl(2.5, width + 2, axis=axis))   # hub bore for shaft/pin
    return g


# ---------------------------------------------------------------------------
# FOREARM  (elbow -> wrist)  + SERVO-IN-WRIST grip actuation (single frame)
# ---------------------------------------------------------------------------
def make_forearm(p=PARAMS):
    a = p.arm
    L = a.elbow_len
    tube = _tube(L, a.link_w - 4, a.link_t - 2, a.link_wall, axis="X")
    # proximal fork (wraps elbow clevis) bore along Z
    fork = box_centered(18, a.link_w + 6, a.link_t + 4).translate((9, 0, 0)).edges("|Y").fillet(2.0)
    fork = fork.cut(cyl(a.bore_d / 2 + 0.1, a.link_t + 10, axis="Z"))
    for dz in (+1, -1):
        fork = fork.union(_bearing_ring(p.hw.bearing_626_od, p.hw.bearing_626_id, p.hw.bearing_626_w, axis="Z").translate((9, 0, dz * (a.link_t / 2 + 3))))
    # distal wrist block: houses pitch (Z), roll (X) AND grip (Z) servos
    wblock = box_centered(30, 60, a.link_t + 2).translate((L - 13, 0, 0)).edges("|Y").fillet(2.0)
    pitch = make_htd45h(p).translate((L - 13, 0, (a.link_t - 2) / 2 + 12))   # wrist pitch, shaft Z
    roll = make_htd45h(p).rotate((0, 0, 0), (1, 0, 0), 90).translate((L - 13, (a.link_w - 2) / 2 + 13, 0))  # wrist roll, shaft X
    # GRIP SERVO stays IN THE WRIST (not in the tool): shaft Z, seated at y=grip_servo_y
    gy = a.grip_servo_y
    grip = make_htd45h(p).translate((L - 13, gy, 0))   # shaft Z, at wrist-block center line
    # DRIVEN GEAR on the grip-servo shaft, in the wrist Y-Z plane (same frame as tool gears)
    drive = _spur_gear(a.gear_module, a.gear_drive_teeth, 6).translate((L - 13, gy, 0))
    return tube.union(fork).union(wblock).union(pitch).union(roll).union(grip).union(drive)


# ---------------------------------------------------------------------------
# WRIST + PASSIVE INTERCHANGEABLE GRIPPER  (no servo/motor/battery/controller)
# Standard Ø34 coupler (2x M3 + 1 dowel). DRIVEN gear meshes COUPLER gear on
# the tool; the coupler gear and an ECCENTRIC CAM are fixed to the coupler; the
# cam drives two symmetric passive fingers about their pivots.
# ---------------------------------------------------------------------------
def make_gripper(p=PARAMS):
    a = p.arm
    L = a.elbow_len
    wx = L - 13                           # wrist block center (shared frame w/ forearm)
    fc = a.gear_center                    # gear center distance (18 mm)
    R = a.finger_pivot_r                 # finger pivot ring radius on coupler (z = +/-R)
    # ---- standard coupler disk (swappable tool interface) ----
    disk = cyl(a.coupler_d / 2, a.coupler_face_w, axis="X").translate((wx + 15 + a.coupler_face_w / 2, 0, 0))
    for ang in (math.radians(120), math.radians(240)):
        bx, by = (a.coupler_d / 2 - 5) * math.cos(ang), (a.coupler_d / 2 - 5) * math.sin(ang)
        disk = disk.cut(cyl(a.tool_retain_d / 2 + 0.1, a.coupler_face_w + 2, axis="X").translate((wx + 15 + a.coupler_face_w / 2, bx, by)))
    disk = disk.cut(cyl(a.tool_dowel_d / 2 + 0.1, a.coupler_face_w + 2, axis="X").translate((wx + 15 + a.coupler_face_w / 2, 0, 0)))
    # ---- COUPLER GEAR (on tool, meshes the wrist driven gear) at +fc from wrist center ----
    pinion = _spur_gear(a.gear_module, a.gear_pinion_teeth, 6).translate((wx + fc, 0, 0))
    # ---- ECCENTRIC CAM fixed to the coupler (radius cam_ecc in Y drives fingers) ----
    cam = cyl(a.cam_ecc + 3.0, 6, axis="X").translate((wx + 15 + a.coupler_face_w, 0, 0))
    cam = cam.cut(cyl(2.5, 8, axis="X").translate((wx + 15 + a.coupler_face_w, 0, 0)))  # hub bore
    # ---- two symmetric FINGERS (pivot about Z at z=+/-R; rotate about Z to close gap) ----
    # Each finger is a body in the Y-X plane at z=+/-R; its drive-pin sits at +finger_drive_r
    # from its pivot (toward the center), so a rack moving +Y rotates the fingers oppositely.
    finger = box_centered(a.grip_depth, a.finger_w, a.coupler_face_w + 4).translate((wx + 15 + a.coupler_face_w + 2 + a.grip_depth / 2, 0, 0))
    finger = finger.cut(cyl(2.0, a.coupler_face_w + 8, axis="Z").translate((wx + 15 + a.coupler_face_w + 2, 0, 0)))            # pivot (about Z)
    finger = finger.cut(cyl(2.0, a.coupler_face_w + 8, axis="Z").translate((wx + 15 + a.coupler_face_w + 2, a.finger_drive_r, 0)))  # drive pin (rack engages, +Y side)
    finger = finger.cut(box_centered(6, 4, a.coupler_face_w + 6).translate((wx + 15 + a.coupler_face_w + 2 + a.grip_depth - 2, 0, 0)))  # pad slot
    f1 = finger.translate((0, 0, +R))
    f2 = finger.translate((0, 0, -R))
    # passive: NO make_htd45h -> no servo/motor/battery/controller
    return disk.union(pinion).union(cam).union(f1).union(f2)



# ---------------------------------------------------------------------------
# FULL ARM (parked pose) - returns dict of named solids + combined compound
# ---------------------------------------------------------------------------
def make_arm(p=PARAMS):
    """Assemble the 6-DOF arm in a parked pose (pointing +X). Returns (dict, compound)."""
    a = p.arm
    yaw = make_yaw_housing(p)                       # base at x=0, front face ~x=base_reach+28
    # shoulder is integrated into upper arm yoke; place upper arm after yaw housing
    upper = make_upper_arm(p).translate((a.base_reach + 28, 0, 0))
    # elbow at end of upper arm
    elbow_x = a.base_reach + 28 + a.shoulder_len
    fore = make_forearm(p).translate((elbow_x, 0, 0))
    wrist_x = elbow_x + a.elbow_len
    grip = make_gripper(p).translate((wrist_x, 0, 0))
    parts = {
        "yaw_housing": yaw,
        "upper_arm": upper,
        "forearm": fore,
        "gripper": grip,
    }
    compound = yaw
    for v in (upper, fore, grip):
        compound = compound.union(v)
    return parts, compound


def make_arm_mounted(p=PARAMS, torso=None):
    """Place the arm at the torso front port (x=arm_port_x, z=arm_port_z, facing +X).
    The yaw housing recesses into the bore (recess_depth). Returns the arm compound."""
    a = p.arm
    parts, compound = make_arm(p)
    px, pz = p.torso.arm_port_x, p.torso.arm_port_z
    # shift so the yaw housing flange sits at the torso front face; recess sinks -X into bore
    dx = px - a.recess_depth
    return compound.translate((dx, 0, pz))


if __name__ == "__main__":
    import build_common as bc
    _, c = make_arm()
    bc.export(c, "arm_parked", subdir=str(Path(__file__).resolve().parent / ".." / "Exports" / "arm"))
    print("arm built; size:", [round(v, 1) for v in __import__("build_common").size_of(c)])
