"""Ori Robot Dog — master leg (front-left canonical).

One mechanical leg architecture is generated here; the full assembly mirrors and
rotates this design for FL/FR/RL/RR.

Canonical joint axes:
  HIP pitch, KNEE, and passive ANKLE are all Y-axis joints.
  The HTD-45H source model uses +Z as its shaft axis, so installation transforms
  use a -90° X rotation to place the shaft on +Y.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Master"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from build_common import cq, PARAMS, cyl, export
import hardware_lib as hw
from CAD.Hardware.servos.htd45h import make_htd45h


def _tube(length: float, w: float, t: float, wall: float, chan=6.0):
    """Hollow box tube along +X, with an internal round cable channel."""
    outer = cq.Workplane("XY").box(length, w, t).translate((length / 2, 0, 0)).edges("|Z").fillet(1.5)
    inner = cq.Workplane("XY").box(length - 2 * wall, w - 2 * wall, t + 6).translate((length / 2, 0, -3))
    tube = outer.cut(inner)
    ch = cq.Workplane("YZ").circle(chan / 2).extrude(length + 4).translate((length / 2, 0, -t / 2 + 1.5 - 2))
    return tube.cut(ch)


def _bore_y(r: float, len_y: float, x: float, z: float = 0.0):
    """Cylindrical hole along Y, centered at (x, 0, z)."""
    return cyl(r, len_y, axis="Y").translate((x, 0, z))


def _ring(od: float, id: float, width: float, x: float, z: float = 0.0):
    """626ZZ bearing seat ring on a Y-axis joint."""
    o = cyl(od / 2, width, axis="Y").translate((x, 0, z))
    i = cyl(id / 2, width + 2, axis="Y").translate((x, 0, z - 1))
    return o.cut(i)


def make_upper_link(p=PARAMS):
    L = p.leg.upper_link_length
    w = p.leg.link_w
    t = p.leg.link_t
    wall = p.leg.link_wall
    tube = _tube(L, w, t, wall, p.leg.cable_channel_d)

    # Proximal HIP yoke: two bearing-supported plates around the Y axis.
    plate_t = 9.0
    plate_len = 26.0
    for dy in (+1, -1):
        plate = cq.Workplane("XY").box(plate_len, plate_t, t + 10.0).translate((plate_len / 2, dy * (w / 2 + plate_t / 2), 0)).edges("|Z").fillet(2.0)
        plate = plate.cut(_bore_y(p.servo.shaft_d / 2 + 0.1, plate_t + 4, plate_len / 2, 0))
        plate = plate.edges("|Y").fillet(3.0)
        tube = tube.union(plate)
        tube = tube.union(_ring(p.hw.bearing_626_od, p.hw.bearing_626_id, p.hw.bearing_626_w, plate_len / 2, 0).translate((0, dy * (w / 2 + plate_t / 2), 0)))

    # Servo-horn interface: six M2 holes on the HTD-45H 25 mm PCD.
    import math as _m
    hub_face_z = t / 2 + 4.0
    for i in range(p.servo.horn_holes):
        a = _m.radians(i * 360.0 / p.servo.horn_holes)
        bx, by = (p.servo.horn_pcd / 2) * _m.cos(a), (p.servo.horn_pcd / 2) * _m.sin(a)
        boss = cyl(p.servo.horn_hole_d / 2 + 2.5, 9.0, axis="Z").translate((10.0, bx, hub_face_z - 4.5))
        tube = tube.union(boss)
        tube = tube.cut(cyl(p.servo.horn_hole_d / 2, 12.0, axis="Z").translate((10.0, bx, hub_face_z - 6.0)))

    stop = cq.Workplane("XY").box(10.0, 10.0, 10.0).translate((L * 0.18, w / 2 + 5, 0)).edges().fillet(1.5)
    tube = tube.union(stop)

    # Distal KNEE clevis, Y-axis bore.
    clevis = cq.Workplane("XY").box(22.0, w + 4.0, t + 6.0).translate((L - 11.0, 0, 0)).edges("|Z").fillet(2.0)
    clevis = clevis.cut(_bore_y(p.servo.shaft_d / 2 + 0.1, w + 10, L - 11.0, 0))
    clevis = clevis.edges("|Y").fillet(2.0)
    return tube.union(clevis)


def make_lower_link(p=PARAMS):
    L = p.leg.lower_link_length
    w = p.leg.link_w
    t = p.leg.link_t
    wall = p.leg.link_wall
    tube = _tube(L, w, t, wall, p.leg.cable_channel_d)

    # Proximal KNEE fork around the upper-link clevis.
    arm_t = 6.0
    fork_len = 22.0
    for dy in (+1, -1):
        arm = cq.Workplane("XY").box(fork_len, arm_t, t + 4.0).translate((fork_len / 2, dy * (w / 2 - arm_t / 2), 0)).edges("|Z").fillet(1.5)
        arm = arm.cut(_bore_y(p.hw.bearing_626_od / 2 + 0.1, arm_t + 4, fork_len / 2, 0))
        tube = tube.union(arm)
        tube = tube.union(_ring(p.hw.bearing_626_od, p.hw.bearing_626_id, p.hw.bearing_626_w, fork_len / 2, 0).translate((0, dy * (w / 2 - arm_t / 2), 0)))

    saddle = cq.Workplane("XY").box(34.0, w + 6.0, 18.0).translate((fork_len + 6.0, 0, -(t / 2 + 9.0))).edges("|Z").fillet(2.0)
    saddle = saddle.cut(_bore_y(p.servo.shaft_d / 2 + 0.1, 20.0, fork_len + 6.0, -(t / 2 + 9.0)))
    tube = tube.union(saddle)

    ankle = cq.Workplane("XY").box(20.0, w + 4.0, t + 4.0).translate((L - 10.0, 0, 0)).edges("|Z").fillet(1.5)
    ankle = ankle.cut(_bore_y(p.hw.bearing_626_od / 2 + 0.1, w + 8, L - 10.0, 0))
    return tube.union(ankle)


def make_foot(p=PARAMS):
    fh = p.leg.foot_height
    fd = p.leg.foot_d
    wall = p.leg.foot_wall
    w = p.leg.link_w

    cup_h = 16.0
    cup = cq.Workplane("XY").box(22.0, w, cup_h).translate((0, 0, fh - cup_h / 2)).edges("|Z").fillet(2.0)
    cup = cup.cut(_bore_y(p.hw.bearing_626_od / 2 + 0.1, w + 4, 0, fh - cup_h / 2))

    col_z0 = -fh + cup_h / 2
    col = cq.Workplane("XZ").box(fd * 0.7, fh - cup_h, fd * 0.5).translate((0, 0, col_z0))
    sole = cq.Workplane("XY").circle(fd / 2).extrude(wall).translate((0, 0, -fh + wall / 2))
    rim = cq.Workplane("XY").circle(fd / 2).circle(fd / 2 - 5.0).extrude(fh - cup_h).translate((0, 0, -fh + (fh - cup_h) / 2 + wall / 2))
    foot = cup.union(col).union(rim).union(sole)

    foot = foot.union(cyl(p.servo.shaft_d / 2, w + 6, axis="Y"))
    sw = hw.make_d2f_switch().rotate((0, 0, 0), (0, 0, 1), -90).translate((0, 0, -fh + 3.0))
    return foot, sw


def _servo_axis_y(p=PARAMS):
    # HTD-45H canonical shaft is +Z. Rx(-90°) maps +Z -> +Y.
    return make_htd45h(p).rotate((0, 0, 0), (1, 0, 0), -90)


def make_leg_assembly(p=PARAMS, export_parts=True):
    upper = make_upper_link(p)
    lower = make_lower_link(p).translate((p.leg.upper_link_length, 0, 0))
    foot, sw = make_foot(p)
    foot = foot.translate((p.leg.upper_link_length + p.leg.lower_link_length, 0, 0))

    hip_servo = _servo_axis_y(p)
    knee_servo = _servo_axis_y(p).translate((p.leg.upper_link_length + 28.0, 0, -(p.leg.link_t / 2 + 9.0)))

    parts = {
        "upper_link": upper,
        "lower_link": lower,
        "foot": foot,
        "foot_switch": sw,
        "hip_servo": hip_servo,
        "knee_servo": knee_servo,
    }
    if export_parts:
        names = {
            "upper_link": "leg_upper_link", "lower_link": "leg_lower_link",
            "foot": "leg_foot", "foot_switch": "leg_foot_switch",
            "hip_servo": "leg_hip_servo", "knee_servo": "leg_knee_servo",
        }
        for k, v in names.items():
            export(parts[k], names[k], subdir="leg")
        merged = upper.union(lower).union(foot).union(hip_servo).union(knee_servo)
        export(merged, "leg_master_merged", subdir="leg")
        parts["_merged"] = merged
    return parts


if __name__ == "__main__":
    import build_common as bc
    parts = make_leg_assembly()
    for k, v in parts.items():
        if k.startswith("_"):
            continue
        print(f"{k:14s} size(X,Y,Z)=", [round(r, 1) for r in bc.size_of(v)])


def make_leg_standing(fwd_offset=70.0, p=PARAMS):
    """Build one leg in a standing pose with hip at the origin."""
    import math as _m
    a = p.leg.upper_link_length
    b = p.leg.lower_link_length
    fh = p.leg.foot_height
    D = p.scale.hip_height_nominal

    tx, tz = float(fwd_offset), float(-(D - fh))
    r = _m.hypot(tx, tz)
    r = min(r, a + b - 1e-3)
    r = max(r, abs(a - b) + 1e-3)
    gamma = _m.atan2(tz, tx)
    A = _m.acos(max(-1.0, min(1.0, (a * a + r * r - b * b) / (2 * a * r))))
    d1 = gamma + A
    d2 = _m.atan2(tz - a * _m.sin(d1), tx - a * _m.cos(d1))

    knee = (a * _m.cos(d1), 0.0, a * _m.sin(d1))
    ankle = (knee[0] + b * _m.cos(d2), 0.0, knee[2] + b * _m.sin(d2))

    up_raw = make_upper_link(p)
    lo_raw = make_lower_link(p)
    ft_raw, sw_raw = make_foot(p)

    Ry1 = -_m.degrees(d1)
    up = up_raw.rotate((0, 0, 0), (0, 1, 0), Ry1)
    Ry2 = -_m.degrees(d2)
    lo = lo_raw.rotate((0, 0, 0), (0, 1, 0), Ry2).translate(knee)
    ft = ft_raw.rotate((0, 0, 0), (0, 1, 0), Ry2).translate(ankle)
    sw = sw_raw.rotate((0, 0, 0), (0, 1, 0), Ry2).translate(ankle)

    hs = _servo_axis_y(p).rotate((0, 0, 0), (0, 1, 0), Ry1)
    ks = _servo_axis_y(p).rotate((0, 0, 0), (0, 1, 0), Ry2).translate(knee)

    return {
        "upper_link": up,
        "lower_link": lo,
        "foot": ft,
        "foot_switch": sw,
        "hip_servo": hs,
        "knee_servo": ks,
        "knee": knee,
        "ankle": ankle,
        "_merged": up.union(lo).union(ft).union(sw).union(hs).union(ks),
    }
