"""
Ori Robot Dog - shared HARDWARE LIBRARY (real components).

All parts are canonical Z-axis (axis of rotation / fastener along +Z).
Imported by legs, torso, head. Every dimension is real (VERIFIED/assumed noted).

Contents:
  make_bearing(bore, od, width)      -> generic ring (canonical Z)
  make_608_zz() / make_626_zz()      -> real bearings (VERIFIED dims)
  make_screw_m3(length)              -> M3 socket-cap screw repr (canonical Z, head +Z)
  make_heat_insert_m3()              -> M3 heat-set insert bore repr
  make_d2f_switch()                  -> Omron D2F-L microswitch (VERIFIED body, assumed lever)
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_common import cq, PARAMS


# ---------------------------------------------------------------------------
# Bearings
# ---------------------------------------------------------------------------
def make_bearing(bore: float, od: float, width: float):
    """Generic deep-groove ball bearing envelope, canonical Z (axis along Z)."""
    outer = cq.Workplane("XY").circle(od / 2).extrude(width)
    bore_cut = cq.Workplane("XY").circle(bore / 2).extrude(width + 2).translate((0, 0, -1))
    outer = outer.cut(bore_cut)
    # inner race (suggested) - a thinner collar inside
    inner_race = cq.Workplane("XY").circle(bore / 2 + 1.5).extrude(width)
    inner_bore = cq.Workplane("XY").circle(bore / 2).extrude(width + 2).translate((0, 0, -1))
    inner_race = inner_race.cut(inner_bore)
    outer = outer.union(inner_race)
    return outer


def make_608_zz():
    """608ZZ: 8x22x7 (VERIFIED)."""
    return make_bearing(8.0, 22.0, 7.0)


def make_626_zz():
    """626ZZ: 6x19x6 (VERIFIED). Used at hip & knee (match 6mm servo shaft)."""
    return make_bearing(6.0, 19.0, 6.0)


# ---------------------------------------------------------------------------
# Fasteners (representative geometry, real diameters)
# ---------------------------------------------------------------------------
def make_screw_m3(length: float, head_type="socket"):
    """M3 screw, canonical Z, head at +Z end. Head is a short cylinder + hex socket."""
    shaft = cq.Workplane("XY").circle(3.0 / 2).extrude(length)
    head_h = 3.0
    head = cq.Workplane("XY").workplane(offset=length).circle(5.5 / 2).extrude(head_h)
    if head_type == "socket":
        socket = cq.Workplane("XY").workplane(offset=length + head_h - 2.0).circle(2.0).extrude(2.2)
        head = head.cut(socket)
    screw = shaft.union(head)
    return screw


def make_heat_insert_m3():
    """M3 heat-set insert repr: knurled brass sleeve, bore 3.0, outer 4.0, len 5.0."""
    body = cq.Workplane("XY").circle(4.0 / 2).extrude(5.0)
    bore = cq.Workplane("XY").circle(3.0 / 2).extrude(5.2).translate((0, 0, -0.1))
    body = body.cut(bore)
    return body


# ---------------------------------------------------------------------------
# Sensors
# ---------------------------------------------------------------------------
def make_d2f_switch():
    """Omron D2F-L ultra-subminiature switch.
    Body 12.2 x 6.0 x 6.5 (VERIFIED), hinge lever ~12 mm (ASSUMED).
    Canonical: body in XY (L=x=12.2, W=y=6.0), thickness Z=6.5, lever along +X."""
    l, w, h = 12.2, 6.0, 6.5
    body = cq.Workplane("XY").box(l, w, h)
    # hinge lever (thin blade extending +X)
    lever = cq.Workplane("XY").workplane(offset=0).moveTo(l / 2 + 6.0, 0).rect(12.0, 1.2).extrude(2.0)
    body = body.union(lever)
    return body


if __name__ == "__main__":
    import build_common as bc
    for n, f in [("608zz", make_608_zz), ("626zz", make_626_zz)]:
        p = f()
        print(n, [round(v, 2) for v in bc.size_of(p)])
    s = make_screw_m3(20)
    print("M3x20", [round(v, 2) for v in bc.size_of(s)])
    sw = make_d2f_switch()
    print("D2F", [round(v, 2) for v in bc.size_of(sw)])
