"""
Ori Robot Dog - shared CAD build helpers.

Provides:
  * project-root path resolution so every module can `import Parameters`
  * cadquery import
  * export() -> writes both STEP (CAD deliverable) and STL (print/inspect)
  * small geometry helpers (centered cylinders, rounded boxes)

Run scripts from the project root so `import Parameters` resolves.
"""
import sys
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]   # CAD/Master -> CAD -> ROOT
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cadquery as cq
from Parameters.master_parameters import PARAMS, OriParams

# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def cyl(radius: float, length: float, axis: str = "Z"):
    """Cylinder of given length centered on the origin, along axis."""
    if axis == "Z":
        return cq.Workplane("XY").circle(radius).extrude(length).translate((0, 0, -length / 2))
    if axis == "X":
        return cq.Workplane("YZ").circle(radius).extrude(length).translate((0, 0, -length / 2)).rotate((0, 0, 0), (1, 0, 0), 90)
    if axis == "Y":
        return cq.Workplane("XZ").circle(radius).extrude(length).translate((0, 0, -length / 2)).rotate((0, 0, 0), (0, 1, 0), 90)
    raise ValueError(axis)


def box_centered(x: float, y: float, z: float, fillet: float = 0.0):
    wp = cq.Workplane("XY").box(x, y, z)
    if fillet > 0:
        wp = wp.edges().fillet(fillet)
    return wp


# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------

def export(part, name: str, subdir: str = "CAD/Exports", fmt=("step", "stl")):
    """Export a cadquery part (Workplane or Solid) to STEP and/or STL under ROOT/subdir.

    Returns dict of {ext: path}.
    """
    out = ROOT / subdir
    out.mkdir(parents=True, exist_ok=True)
    paths = {}
    if "step" in fmt:
        p = out / (name + ".step")
        cq.exporters.export(part, str(p))
        paths["step"] = p
    if "stl" in fmt:
        p = out / (name + ".stl")
        cq.exporters.export(part, str(p))
        paths["stl"] = p
    return paths


def bounds(part):
    """Return (xmin,xmax,ymin,ymax,zmin,zmax) bounding box of a part."""
    bb = part.val().BoundingBox() if hasattr(part, "val") else part.BoundingBox()
    return (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax)


def size_of(part):
    b = bounds(part)
    return (b[1] - b[0], b[3] - b[2], b[5] - b[4])


if __name__ == "__main__":
    print("build_common OK; ROOT =", ROOT)
