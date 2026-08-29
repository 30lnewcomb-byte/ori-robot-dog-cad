"""
Hiwonder HTD-45H bus servo - accurate envelope.

Oriented in canonical frame:
  * output SHAFT along +Z (the joint axis when installed)
  * body long axis = X (51.1 mm)
  * body tall axis  = Y (40.0 mm, incl. mounting face)
  * body thin axis  = Z (20.14 mm, along shaft)

Real features modeled (VERIFIED from datasheet + product photos):
  - body envelope 51.1 x 40.0 x 20.14 mm, rounded edges
  - raised circular output mounting plate on +Z face (horn seat)
  - 6-hole horn retention circle, PCD 25.0 mm, holes 2.0 mm (M2)
  - central M3 retention bore
  - double 6 mm splined output shaft, protruding both faces
  - two M3 side-flange mounting holes per long side, 31 mm spacing along X
  - rear PH2.0-3P connector boss on -X face
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "Master"))
from build_common import cq, PARAMS, cyl, box_centered


def make_htd45h(p=PARAMS, with_connector=True):
    s = p.servo
    L, H, D = s.body_long, s.body_tall, s.body_short   # X, Y, Z

    # --- main body ---
    body = box_centered(L, H, D, fillet=1.5)

    # --- output mounting plate (raised boss) on +Z face ---
    plate_r = s.horn_pcd / 2 + 4.0
    plate = (cq.Workplane("XY")
             .workplane(offset=D / 2)
             .circle(plate_r)
             .extrude(1.5))
    body = body.union(plate)

    # --- output shaft (double, along Z) ---
    shaft_len = D + 2 * s.shaft_len_each_side
    shaft = cyl(s.shaft_d / 2, shaft_len, axis="Z")
    body = body.union(shaft)

    # --- horn retention holes (6 x M2) on +Z face, PCD 25 ---
    import math as _math
    plate_top = D / 2 + 1.5
    for i in range(s.horn_holes):
        ang = _math.radians(i * 360.0 / s.horn_holes)
        x, y = (s.horn_pcd / 2) * _math.cos(ang), (s.horn_pcd / 2) * _math.sin(ang)
        hole = (cq.Workplane("XY")
                .workplane(offset=plate_top)
                .moveTo(x, y)
                .circle(s.horn_hole_d / 2)
                .extrude(-3.5))
        body = body.cut(hole)

    # --- central M3 retention bore (blind, from +Z) ---
    cbore = (cq.Workplane("XY")
             .workplane(offset=plate_top)
             .circle(s.horn_center_screw / 2)
             .extrude(-6.5))
    body = body.cut(cbore)

    # --- side flange M3 holes (2 per long side, 31 mm apart along X) ---
    fx = s.flange_spacing / 2.0
    for sign in (+1, -1):
        for xa in (+fx, -fx):
            hole = (cq.Workplane("XY")
                    .workplane(offset=sign * (H / 2))
                    .moveTo(xa, 0)
                    .circle(s.flange_holes_d / 2)
                    .extrude(-2.5))
            body = body.cut(hole)

    # --- rear connector boss (-X face) ---
    if with_connector:
        boss = (cq.Workplane("YZ")
                .workplane(offset=-L / 2)
                .rect(10.0, 6.0)
                .extrude(6.0))
        body = body.union(boss)
        # connector slot
        slot = (cq.Workplane("YZ")
                .workplane(offset=-L / 2)
                .rect(3.0, 6.0)
                .extrude(7.0))
        body = body.cut(slot)

    return body


if __name__ == "__main__":
    srv = make_htd45h()
    print("HTD-45H size (X,Y,Z):", [round(v, 2) for v in __import__("build_common").size_of(srv)])
