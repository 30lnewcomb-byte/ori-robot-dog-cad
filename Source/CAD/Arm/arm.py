"""Ori Robot Dog — 6-DOF front arm and interchangeable passive gripper.

The arm is generated from ``PARAMS.arm``. The current mechanical implementation
uses six HTD-45H actuators on the arm structure, with the grip actuator retained
in the forearm/wrist rather than inside the interchangeable tool.

Important current-state note:
  The passive gripper transmission is represented as a lightweight printed spur
  gear + coupler + cam/finger mechanism. Its final dynamic grip behavior still
  requires physical validation; this file does not claim hardware-tested motion.
"""
import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Master"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Hardware" / "servos"))
from build_common import cq, PARAMS, cyl, box_centered
from htd45h import make_htd45h


def _tube(L, w, t, wall, axis="X"):
    """Hollow rectangular tube along ``axis`` with a parameterized wall."""
    o = box_centered(L, w, t) if axis == "X" else box_centered(t, w, L) if axis == "Z" else box_centered(w, L, t)
    iw, it = max(w - 2 * wall, 1.0), max(t - 2 * wall, 1.0)
    i = box_centered(L - 2 * wall, iw, it) if axis == "X" else box_centered(iw, L - 2 * wall, it) if axis == "Z" else box_centered(iw, it, L - 2 * wall)
    return o.cut(i)


def _bearing_ring(od, id_, w, axis="Y"):
    o = cyl(od / 2, w, axis=axis)
    i = cyl(id_ / 2, w + 2, axis=axis)
    return o.cut(i)


def make_yaw_housing(p=PARAMS):
    """Base yaw housing recessed into the torso arm port."""
    a = p.arm
    housing = cyl(a.mount_bore_d / 2 - 1.0, a.base_reach, axis="X")
    flange = cyl(a.mount_flange_od / 2, 6.0, axis="X").translate((a.base_reach / 2 + 3, 0, 0))
    for i in range(4):
        ang = math.radians(i * 90 + 45)
        bx, by = (a.mount_bolt_pcd / 2) * math.cos(ang), (a.mount_bolt_pcd / 2) * math.sin(ang)
        flange = flange.cut(cyl(a.mount_bolt_d / 2 + 0.1, 10, axis="X").translate((a.base_reach / 2 + 3, bx, by)))
    mount = box_centered(40, a.mount_bore_d - 6, 30).translate((a.base_reach + 14, 0, 0))
    srv = make_htd45h(p).rotate((0, 0, 0), (1, 0, 0), 90).translate((a.base_reach + 14, 0, 0))
    return housing.union(flange).union(mount).union(srv)


def make_upper_arm(p=PARAMS):
    a = p.arm
    L = a.shoulder_len
    tube = _tube(L, a.link_w, a.link_t, a.link_wall, axis="X")
    plate_t = 7.0
    for dz in (+1, -1):
        plate = box_centered(22, a.link_w + 4, plate_t).translate((11, 0, dz * (a.link_t / 2 + plate_t / 2))).edges("|Y").fillet(1.5)
        plate = plate.cut(cyl(a.bore_d / 2 + 0.1, plate_t + 4, axis="Z"))
        plate = plate.edges("|Z").fillet(2.0)
        tube = tube.union(plate)
        tube = tube.union(_bearing_ring(p.hw.bearing_626_od, p.hw.bearing_626_id, p.hw.bearing_626_w, axis="Z").translate((11, 0, dz * (a.link_t / 2 + plate_t / 2))))
    srv = make_htd45h(p).translate((11, 0, a.link_t / 2 + plate_t + 12))
    clevis = box_centered(20, a.link_w + 4, a.link_t + 4).translate((L - 10, 0, 0)).edges("|Y").fillet(2.0)
    clevis = clevis.cut(cyl(a.bore_d / 2 + 0.1, a.link_t + 10, axis="Z"))
    return tube.union(srv).union(clevis)


def _spur_gear(module, teeth, width, axis="X"):
    """Lightweight printable spur-gear approximation used for architecture checks."""
    r_pitch = module * teeth / 2.0
    r_root = r_pitch - 1.25 * module
    g = cyl(r_root, width, axis=axis)
    for i in range(teeth):
        ang = 2 * math.pi * i / teeth
        tooth = box_centered(module * 1.5, 2 * module, width).translate((0, r_pitch * math.cos(ang), r_pitch * math.sin(ang)))
        g = g.union(tooth)
    return g.cut(cyl(2.5, width + 2, axis=axis))


def make_forearm(p=PARAMS):
    """Forearm plus wrist block.

    Grip servo and its spur gear now share the SAME +X shaft axis. The gear center
    is in the wrist plane at x=L-13 and y=grip_servo_y; the passive tool gear is
    translated to the same x plane at y=0, giving the parameterized 18 mm mesh.
    """
    a = p.arm
    L = a.elbow_len
    tube = _tube(L, a.link_w - 4, a.link_t - 2, a.link_wall, axis="X")
    fork = box_centered(18, a.link_w + 6, a.link_t + 4).translate((9, 0, 0)).edges("|Y").fillet(2.0)
    fork = fork.cut(cyl(a.bore_d / 2 + 0.1, a.link_t + 10, axis="Z"))
    for dz in (+1, -1):
        fork = fork.union(_bearing_ring(p.hw.bearing_626_od, p.hw.bearing_626_id, p.hw.bearing_626_w, axis="Z").translate((9, 0, dz * (a.link_t / 2 + 3))))

    wrist_x = L - 13.0
    wblock = box_centered(30, 60, a.link_t + 2).translate((wrist_x, 0, 0)).edges("|Y").fillet(2.0)
    pitch = make_htd45h(p).translate((wrist_x, 0, (a.link_t - 2) / 2 + 12))
    roll = make_htd45h(p).rotate((0, 0, 0), (1, 0, 0), 90).translate((wrist_x, (a.link_w - 2) / 2 + 13, 0))

    # Grip actuator: shaft MUST be along +X because the grip gear axis is X.
    gy = a.grip_servo_y
    grip = make_htd45h(p).rotate((0, 0, 0), (0, 1, 0), 90).translate((wrist_x, gy, 0))
    drive = _spur_gear(a.gear_module, a.gear_drive_teeth, 6, axis="X").translate((wrist_x, gy, 0))
    return tube.union(fork).union(wblock).union(pitch).union(roll).union(grip).union(drive)


def make_gripper(p=PARAMS):
    """Build a passive tool around a standard coupler.

    Local tool frame:
      - coupler/gear axis = X
      - tool extends +X
      - the two finger pivots are separated in Y and rotate about Z.

    The printed cam/finger arrangement is intentionally kept lightweight; final
    grip kinematics remain a physical-validation item rather than being inferred
    from a visually plausible static model.
    """
    a = p.arm
    coupler_x = a.coupler_face_w / 2.0
    R = a.finger_pivot_r

    # Standard interchangeable face; centered on the local tool origin plane.
    disk = cyl(a.coupler_d / 2, a.coupler_face_w, axis="X").translate((coupler_x, 0, 0))
    for ang in (math.radians(120), math.radians(240)):
        bx, by = (a.coupler_d / 2 - 5) * math.cos(ang), (a.coupler_d / 2 - 5) * math.sin(ang)
        disk = disk.cut(cyl(a.tool_retain_d / 2 + 0.1, a.coupler_face_w + 2, axis="X").translate((coupler_x, bx, by)))
    disk = disk.cut(cyl(a.tool_dowel_d / 2 + 0.1, a.coupler_face_w + 2, axis="X").translate((coupler_x, 0, 0)))

    # Passive tool gear is on the same X-facing plane as the wrist drive gear.
    tool_gear = _spur_gear(a.gear_module, a.gear_pinion_teeth, 6, axis="X")

    # Eccentric cam representation: rotating shaft remains concentric; the cam lobe
    # is offset in +Y to create the intended follower motion envelope.
    cam_lobe = cyl(a.cam_ecc + 3.0, 6, axis="X").translate((coupler_x, a.cam_ecc, 0))
    cam_hub = cyl(5.0, 6, axis="X").translate((coupler_x, 0, 0))

    pivot_x = coupler_x + 7.0
    finger_len = a.grip_depth
    finger = box_centered(finger_len, a.finger_w, a.coupler_face_w + 4).translate((pivot_x + finger_len / 2, 0, 0))
    # Pivot and follower holes are vertical (Z axis), matching the intended finger rotation plane.
    finger = finger.cut(cyl(2.0, a.coupler_face_w + 8, axis="Z").translate((pivot_x, 0, 0)))
    finger = finger.cut(cyl(2.0, a.coupler_face_w + 8, axis="Z").translate((pivot_x, a.finger_drive_r, 0)))
    finger = finger.cut(box_centered(6, 4, a.coupler_face_w + 6).translate((pivot_x + finger_len - 2, 0, 0)))
    f1 = finger.translate((0, +R, 0))
    f2 = finger.translate((0, -R, 0))

    return disk.union(tool_gear).union(cam_hub).union(cam_lobe).union(f1).union(f2)


def make_arm(p=PARAMS):
    """Assemble the 6-DOF arm in its parked pose."""
    a = p.arm
    yaw = make_yaw_housing(p)
    upper = make_upper_arm(p).translate((a.base_reach + 28, 0, 0))
    elbow_x = a.base_reach + 28 + a.shoulder_len
    fore = make_forearm(p).translate((elbow_x, 0, 0))
    wrist_x = elbow_x + a.elbow_len
    # Gripper gear center is x=0 locally; align it with the forearm wrist plane at x=wrist_x-13.
    grip = make_gripper(p).translate((wrist_x - 13, 0, 0))
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
    """Place the arm at the torso front arm port, partially recessed."""
    _parts, compound = make_arm(p)
    px, pz = p.torso.arm_port_x, p.torso.arm_port_z
    return compound.translate((px - p.arm.recess_depth, 0, pz))


if __name__ == "__main__":
    import build_common as bc
    _parts, compound = make_arm()
    bc.export(compound, "arm_parked", subdir="arm")
    print("arm built; size:", [round(v, 1) for v in bc.size_of(compound)])
