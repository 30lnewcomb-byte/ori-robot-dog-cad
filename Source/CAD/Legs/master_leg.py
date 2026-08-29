"""
Ori Robot Dog - MASTER LEG (front-left canonical).

Single leg architecture; the other three legs are produced by transforming this
part in the assembly (mirror/rotate), NOT by separate designs.

Kinematic chain (leg-local, canonical rest pose = straight down):
    HIP joint  (pitch axis = +Y) at origin
       |  upper_link_length  (link runs +X)
       v
    KNEE joint (pitch axis = +Y) at x = upper_link_length
       |  lower_link_length (link runs +X)
       v
    ANKLE      (passive compliant, axis = +Y) at x = upper + lower
       |  foot_height
       v
    FOOT sole (contact) at x = upper + lower, bottom z = -foot_height

Engineering features:
  * Hip servo output axis = +Y (rotated HTD-45H); upper-link proximal hub bolts
    to the servo horn (6-hole PCD 25) and rides on 626ZZ bearings in the torso
    bulkhead -> servo shaft carries torque, bearings carry radial/axial loads.
  * Knee servo at the knee, output axis = +Y, drives the lower-link fork through
    a 22 mm horn -> torque transmitted at the link, not bare shaft.
  * 626ZZ (6x19x6) bearings at hip and knee (match 6 mm servo shaft).
  * Hollow box-tube links with internal 6 mm cable channel.
  * Mechanical hard-stop bosses limit joint travel.
  * Foot: compliant ankle (626ZZ), flared grippy sole, replaceable D2F switch.

All dims flow from Parameters/master_parameters.py (PARAMS).
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Master"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # project root
from build_common import cq, PARAMS, cyl, export
import hardware_lib as hw
from CAD.Hardware.servos.htd45h import make_htd45h


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _tube(length: float, w: float, t: float, wall: float, chan=6.0):
    """Hollow box tube along +X, x in [0, length], cross w (Y) x t (Z), wall thick.
    Internal round cable channel (dia chan) along X near the upper face."""
    outer = cq.Workplane("XY").box(length, w, t).translate((length / 2, 0, 0)).edges("|Z").fillet(1.5)
    inner = cq.Workplane("XY").box(length - 2 * wall, w - 2 * wall, t + 6).translate((length / 2, 0, -3))
    tube = outer.cut(inner)
    # cable channel along X at z = -t/2 + 1.5 (near bottom), centered y
    ch = cq.Workplane("YZ").circle(chan / 2).extrude(length + 4).translate((length / 2, 0, -t / 2 + 1.5 - 2))
    tube = tube.cut(ch)
    return tube


def _bore_y(r: float, len_y: float, x: float, z: float = 0.0):
    """Cylindrical hole (or boss) along +Y, centered at (x,0,z)."""
    return cyl(r, len_y, axis="Y").translate((x, 0, z))


def _ring(od: float, id: float, width: float, x: float, z: float = 0.0):
    """Bearing seat ring (annulus) axis along Y at (x,0,z)."""
    o = cyl(od / 2, width, axis="Y").translate((x, 0, z))
    i = cyl(id / 2, width + 2, axis="Y").translate((x, 0, z - 1))
    return o.cut(i)


# ---------------------------------------------------------------------------
# UPPER LINK  (proximal = hip at x=0, distal = knee at x=L)
# ---------------------------------------------------------------------------
def make_upper_link(p=PARAMS):
    L = p.leg.upper_link_length
    w = p.leg.link_w
    t = p.leg.link_t
    wall = p.leg.link_wall
    tube = _tube(L, w, p.leg.link_t, wall, p.leg.cable_channel_d)

    # proximal HIP yoke: two side plates straddling the hip axis, each with a
    # 6 mm bore (for servo shaft) and a 626ZZ bearing seat. Bearing boss spreads load.
    plate_t = 9.0
    plate_len = 26.0
    for dy in (+1, -1):
        plate = cq.Workplane("XY").box(plate_len, plate_t, t + 10.0).translate((plate_len / 2, dy * (w / 2 + plate_t / 2), 0)).edges("|Z").fillet(2.0)
        plate = plate.cut(_bore_y(p.servo.shaft_d / 2 + 0.1, plate_t + 4, plate_len / 2, 0))  # 6mm shaft bore
        plate = plate.edges("|Y").fillet(3.0)  # relieve bore-root stress concentration
        tube = tube.union(plate)
        # larger bearing boss spreads hip reaction into the plate -> lower root stress
        tube = tube.union(_ring(p.hw.bearing_626_od, p.hw.bearing_626_id, p.hw.bearing_626_w, plate_len / 2, 0).translate((0, dy * (w / 2 + plate_t / 2), 0)))

    # proximal horn bolt bosses: 6 holes matching servo horn PCD 25, on a front
    # face so the upper link bolts to the hip servo horn (M2 self-tapping).
    import math as _m
    hub_face_z = (t / 2) + 4.0
    for i in range(p.servo.horn_holes):
        a = _m.radians(i * 360.0 / p.servo.horn_holes)
        bx, by = (p.servo.horn_pcd / 2) * _m.cos(a), (p.servo.horn_pcd / 2) * _m.sin(a)
        boss = cyl(p.servo.horn_hole_d / 2 + 2.5, 9.0, axis="Z").translate((10.0, bx, hub_face_z - 4.5))
        tube = tube.union(boss)
        # drill M2 through-hole through the boss
        tube = tube.cut(cyl(p.servo.horn_hole_d / 2, 12.0, axis="Z").translate((10.0, bx, hub_face_z - 6.0)))

    # hard-stop boss (limits hip flexion) protruding from the upper side
    stop = cq.Workplane("XY").box(10.0, 10.0, 10.0).translate((L * 0.18, w / 2 + 5, 0)).edges().fillet(1.5)
    tube = tube.union(stop)

    # distal KNEE clevis: single block the lower-link fork wraps around, with a
    # Y-axis bore for the knee pin.
    clevis = cq.Workplane("XY").box(22.0, w + 4.0, t + 6.0).translate((L - 11.0, 0, 0)).edges("|Z").fillet(2.0)
    clevis = clevis.cut(_bore_y(p.servo.shaft_d / 2 + 0.1, w + 10, L - 11.0, 0))
    clevis = clevis.edges("|Y").fillet(2.0)  # relieve knee-pin bore-root concentration
    tube = tube.union(clevis)
    return tube


# ---------------------------------------------------------------------------
# LOWER LINK  (proximal = knee fork at x=0, distal = ankle at x=L)
# ---------------------------------------------------------------------------
def make_lower_link(p=PARAMS):
    L = p.leg.lower_link_length
    w = p.leg.link_w
    t = p.leg.link_t
    wall = p.leg.link_wall
    tube = _tube(L, w, t, wall, p.leg.cable_channel_d)

    # proximal KNEE fork: two arms straddling the upper-link clevis, each with a
    # Y bore aligned to the clevis bore. Knee servo mounts on the fork.
    arm_t = 6.0
    fork_len = 22.0
    for dy in (+1, -1):
        arm = cq.Workplane("XY").box(fork_len, arm_t, t + 4.0).translate((fork_len / 2, dy * (w / 2 - arm_t / 2), 0)).edges("|Z").fillet(1.5)
        arm = arm.cut(_bore_y(p.hw.bearing_626_od / 2 + 0.1, arm_t + 4, fork_len / 2, 0))
        tube = tube.union(arm)
        tube = tube.union(_ring(p.hw.bearing_626_od, p.hw.bearing_626_id, p.hw.bearing_626_w, fork_len / 2, 0).translate((0, dy * (w / 2 - arm_t / 2), 0)))

    # knee servo mount saddle on the lower link (outside fork, below link)
    saddle = cq.Workplane("XY").box(34.0, w + 6.0, 18.0).translate((fork_len + 6.0, 0, -(t / 2 + 9.0))).edges("|Z").fillet(2.0)
    saddle = saddle.cut(_bore_y(p.servo.shaft_d / 2 + 0.1, 20.0, fork_len + 6.0, -(t / 2 + 9.0)))  # servo shaft bore Z
    tube = tube.union(saddle)

    # distal ANKLE receiver: block with a Y bore for the compliant ankle pin.
    ankle = cq.Workplane("XY").box(20.0, w + 4.0, t + 4.0).translate((L - 10.0, 0, 0)).edges("|Z").fillet(1.5)
    ankle = ankle.cut(_bore_y(p.hw.bearing_626_od / 2 + 0.1, w + 8, L - 10.0, 0))
    tube = tube.union(ankle)
    return tube


# ---------------------------------------------------------------------------
# FOOT  (ankle at top z = foot_height, sole at bottom z = 0)
# ---------------------------------------------------------------------------
def make_foot(p=PARAMS):
    fh = p.leg.foot_height
    fd = p.leg.foot_d
    wall = p.leg.foot_wall
    w = p.leg.link_w

    # ankle cup: yoke riding on the lower-link ankle bearing (two short arms)
    cup_h = 16.0
    cup = cq.Workplane("XY").box(22.0, w, cup_h).translate((0, 0, fh - cup_h / 2)).edges("|Z").fillet(2.0)
    cup = cup.cut(_bore_y(p.hw.bearing_626_od / 2 + 0.1, w + 4, 0, fh - cup_h / 2))

    # compliant column from cup down to sole (sole at z = -fh, i.e. foot_height below ankle)
    col_z0 = -fh + cup_h / 2
    col = cq.Workplane("XZ").box(fd * 0.7, fh - cup_h, fd * 0.5).translate((0, 0, col_z0))

    # sole: flared disc with grippy rim, contact surface at z = -fh
    sole = cq.Workplane("XY").circle(fd / 2).extrude(wall).translate((0, 0, -fh + wall / 2))
    rim = cq.Workplane("XY").circle(fd / 2).circle(fd / 2 - 5.0).extrude(fh - cup_h).translate((0, 0, -fh + (fh - cup_h) / 2 + wall / 2))
    foot = cup.union(col).union(rim).union(sole)

    # ankle pin (6 mm) through cup
    foot = foot.union(cyl(p.servo.shaft_d / 2, w + 6, axis="Y"))

    # replaceable contact switch (D2F) seated in sole, lever pointing down
    sw = hw.make_d2f_switch().rotate((0, 0, 0), (0, 0, 1), -90).translate((0, 0, -fh + 3.0))
    return foot, sw


# ---------------------------------------------------------------------------
# SERVO MOUNTS (output axis = +Y)
# ---------------------------------------------------------------------------
def _servo_axis_y(p=PARAMS):
    return make_htd45h(p).rotate((0, 0, 0), (1, 0, 0), 90)  # shaft +Z -> +Y


def make_leg_assembly(p=PARAMS, export_parts=True):
    upper = make_upper_link(p)
    lower = make_lower_link(p)
    foot, sw = make_foot(p)

    # chain links in +X
    lower = lower.translate((p.leg.upper_link_length, 0, 0))
    foot = foot.translate((p.leg.upper_link_length + p.leg.lower_link_length, 0, 0))

    hip_servo = _servo_axis_y(p)                       # at origin
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
            export(parts[k], v, subdir="CAD/Exports/leg")
        merged = upper.union(lower).union(foot).union(hip_servo).union(knee_servo)
        export(merged, "leg_master_merged", subdir="CAD/Exports/leg")
        parts["_merged"] = merged
    return parts


if __name__ == "__main__":
    import build_common as bc
    parts = make_leg_assembly()
    for k, v in parts.items():
        if k.startswith("_"):
            continue
        print(f"{k:14s} size(X,Y,Z)=", [round(r, 1) for r in bc.size_of(v)])


# ---------------------------------------------------------------------------
# STANDING-POSE LEG  (built directly in a valid standing IK pose)
# ---------------------------------------------------------------------------
def make_leg_standing(fwd_offset=70.0, p=PARAMS):
    """Build one leg in a standing pose with the HIP at the origin.
    The leg lives in the X-Z sagittal plane: forward = +X, up = +Z.
    Foot contact lands at world (fwd_offset, -hip_height) by construction.

    Returns dict of parts, each already in the standing pose (hip at origin).
    """
    import math as _m
    a = p.leg.upper_link_length
    b = p.leg.lower_link_length
    fh = p.leg.foot_height
    D = p.scale.hip_height_nominal

    # ankle target (foot hangs fh below ankle)
    tx, tz = float(fwd_offset), float(-(D - fh))
    r = _m.hypot(tx, tz)
    r = min(r, a + b - 1e-3); r = max(r, abs(a - b) + 1e-3)
    gamma = _m.atan2(tz, tx)
    A = _m.acos(max(-1.0, min(1.0, (a * a + r * r - b * b) / (2 * a * r))))
    d1 = gamma + A                      # upper-link angle from +X (knee forward)
    d2 = _m.atan2(tz - a * _m.sin(d1), tx - a * _m.cos(d1))  # lower-link angle

    # joint world positions (hip at origin)
    knee = (a * _m.cos(d1), 0.0, a * _m.sin(d1))
    ankle = (knee[0] + b * _m.cos(d2), 0.0, knee[2] + b * _m.sin(d2))

    # build raw links (proximal at local origin) then rotate+translate into pose
    up_raw = make_upper_link(p)                       # hip at 0, extends +X
    lo_raw = make_lower_link(p)                       # knee at 0, extends +X
    ft_raw, sw_raw = make_foot(p)                     # ankle at 0, sole at -fh

    # upper: rotate by d1 about Y (hip), Ry convention: +X-> -Z at +90
    Ry1 = -_m.degrees(d1)
    up = up_raw.rotate((0, 0, 0), (0, 1, 0), Ry1)

    # lower: rotate by d2 about Y, then translate to knee
    Ry2 = -_m.degrees(d2)
    lo = lo_raw.rotate((0, 0, 0), (0, 1, 0), Ry2).translate(knee)

    # foot: rotate by d2 about Y, translate to ankle
    ft = ft_raw.rotate((0, 0, 0), (0, 1, 0), Ry2).translate(ankle)
    sw = sw_raw.rotate((0, 0, 0), (0, 1, 0), Ry2).translate(ankle)

    # servos: hip servo at hip, knee servo at knee (rotated by d1/d2)
    hs = _servo_axis_y(p).rotate((0, 0, 0), (0, 1, 0), Ry1)
    ks = _servo_axis_y(p).rotate((0, 0, 0), (0, 1, 0), Ry2).translate(knee)

    return {"upper_link": up, "lower_link": lo, "foot": ft, "foot_switch": sw,
            "hip_servo": hs, "knee_servo": ks,
            "knee": knee, "ankle": ankle,
            "_merged": up.union(lo).union(ft).union(sw).union(hs).union(ks)}
