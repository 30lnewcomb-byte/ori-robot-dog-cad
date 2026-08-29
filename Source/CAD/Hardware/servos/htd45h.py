"""
Hiwonder HTD-45H bus servo - accurate envelope.

Canonical frame:
  * output shaft along +Z
  * body long axis = X (51.1 mm)
  * body tall axis = Y (40.0 mm)
  * body thin axis = Z (20.14 mm)

Real features modeled from the current parameter set and verified hardware references:
  - body envelope 51.1 x 40.0 x 20.14 mm
  - raised output mounting plate on +Z face
  - 6-hole horn pattern, PCD 25 mm, M2 retention holes
  - central M3 retention bore
  - double 6 mm output shaft
  - side M3 flange holes, 31 mm spacing
  - rear PH2.0-3P connector envelope
"""
import sys
from pathlib import Path

# Source/CAD/Hardware/servos/htd45h.py -> Source/CAD
CAD_ROOT = Path(__file__).resolve().parents[2]
MASTER = CAD_ROOT / "Master"
if str(MASTER) not in sys.path:
    sys.path.insert(0, str(MASTER))

from build_common import cq, PARAMS, cyl, box_centered


def make_htd45h(p=PARAMS, with_connector=True):
    s = p.servo
    L, H, D = s.body_long, s.body_tall, s.body_short

    body = box_centered(L, H, D, fillet=1.5)

    plate_r = s.horn_pcd / 2 + 4.0
    plate = (
        cq.Workplane("XY")
        .workplane(offset=D / 2)
        .circle(plate_r)
        .extrude(1.5)
    )
    body = body.union(plate)

    shaft_len = D + 2 * s.shaft_len_each_side
    shaft = cyl(s.shaft_d / 2, shaft_len, axis="Z")
    body = body.union(shaft)

    import math as _math
    plate_top = D / 2 + 1.5
    for i in range(s.horn_holes):
        ang = _math.radians(i * 360.0 / s.horn_holes)
        x, y = (s.horn_pcd / 2) * _math.cos(ang), (s.horn_pcd / 2) * _math.sin(ang)
        hole = (
            cq.Workplane("XY")
            .workplane(offset=plate_top)
            .moveTo(x, y)
            .circle(s.horn_hole_d / 2)
            .extrude(-3.5)
        )
        body = body.cut(hole)

    cbore = (
        cq.Workplane("XY")
        .workplane(offset=plate_top)
        .circle(s.horn_center_screw / 2)
        .extrude(-6.5)
    )
    body = body.cut(cbore)

    fx = s.flange_spacing / 2.0
    for sign in (+1, -1):
        for xa in (+fx, -fx):
            hole = (
                cq.Workplane("XY")
                .workplane(offset=sign * (H / 2))
                .moveTo(xa, 0)
                .circle(s.flange_holes_d / 2)
                .extrude(-2.5)
            )
            body = body.cut(hole)

    if with_connector:
        boss = (
            cq.Workplane("YZ")
            .workplane(offset=-L / 2)
            .rect(10.0, 6.0)
            .extrude(6.0)
        )
        body = body.union(boss)
        slot = (
            cq.Workplane("YZ")
            .workplane(offset=-L / 2)
            .rect(3.0, 6.0)
            .extrude(7.0)
        )
        body = body.cut(slot)

    return body


if __name__ == "__main__":
    srv = make_htd45h()
    print("HTD-45H size (X,Y,Z):", [round(v, 2) for v in __import__("build_common").size_of(srv)])
