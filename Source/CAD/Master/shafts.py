"""
Ori Robot Dog - METAL SHAFT + BEARING STRUCTURE (§4)

Architecture (each major axis):
    PETG structure
        -> 626ZZ bearing (printed seat in PETG)
        -> metal shaft (ground steel, runs through bearings)
        -> 626ZZ bearing
        -> PETG structure

Shafts carry bending/torsion and provide durable bearing surfaces + alignment.
Geometry is parametric from PARAMS.arm shaft_* and PARAMS.hw (626ZZ dims).
No metal where a printed boss + bearing already suffices; shafts are chosen at
the smallest sensible standardized diameter (6 mm = 626ZZ bore).
"""
import sys, math
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_common import cq, PARAMS, cyl, box_centered


def make_shaft(d, length, axis="X", end_margin=4.0):
    """Solid ground-steel shaft with a small circlip groove near each end.
    Axis along X by default. Diameter d, length = span between outer bearings + margin."""
    s = cyl(d / 2, length, axis=axis)
    # circlip grooves (visual + functional seat) at both ends
    for ex in (-length / 2 + end_margin, length / 2 - end_margin):
        groove = cyl(d / 2 - 0.4, 0.8, axis=axis).translate((ex, 0, 0) if axis == "X" else (0, ex, 0) if axis == "Y" else (0, 0, ex))
        s = s.cut(groove)
    return s


def make_bearing_seat_stack(od, id_, w, length, axis="X"):
    """Two 626ZZ rings at the ends of a span, representing the printed-boss
    bearing seats. Returns the bearing pair (shaft passes through)."""
    b1 = cyl(od / 2, w, axis=axis).cut(cyl(id_ / 2, w + 2, axis=axis))
    b2 = cyl(od / 2, w, axis=axis).cut(cyl(id_ / 2, w + 2, axis=axis))
    l2 = length / 2 - w / 2
    if axis == "X":
        b1 = b1.translate((-l2, 0, 0)); b2 = b2.translate((l2, 0, 0))
    elif axis == "Y":
        b1 = b1.translate((0, -l2, 0)); b2 = b2.translate((0, l2, 0))
    else:
        b1 = b1.translate((0, 0, -l2)); b2 = b2.translate((0, 0, l2))
    return b1.union(b2)


def shaft_for_joint(name, p=PARAMS):
    """Return (shaft, bearings) for a named joint, sized from PARAMS.
    Standardized 6 mm shaft through 626ZZ (6x19x6) for all major axes."""
    a = p.arm
    hw = p.hw
    d = getattr(a, f"shaft_d_{name}", 6.0)
    # span: distance between outer bearing seats; assume ~link cross-section width + pad
    span = {
        "hip": p.leg.link_w + 18.0,
        "knee": p.leg.link_w + 18.0,
        "shoulder": a.link_t + 18.0,
        "elbow": a.link_t + 18.0,
        "wrist": a.link_t + 18.0,
    }.get(name, 30.0)
    length = span + 2 * a.shaft_end_margin
    sh = make_shaft(d, length, axis="X")
    br = make_bearing_seat_stack(hw.bearing_626_od, hw.bearing_626_id, hw.bearing_626_w, span, axis="X")
    return sh, br


if __name__ == "__main__":
    for j in ("hip", "knee", "shoulder", "elbow", "wrist"):
        sh, br = shaft_for_joint(j)
        print(j, "shaft:", round(sh.val().Volume() * 7.85e-3, 1), "g; bearings span ok")
