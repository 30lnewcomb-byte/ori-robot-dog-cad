"""Shared CAD build helpers for the Ori Robot Dog repository.

The repository root is three parents above this file because the canonical
layout is ``Source/CAD/Master/build_common.py``. Keeping this path calculation
correct is important: generated deliverables must land in the repository's
tracked CAD_STEP/CAD_STL folders rather than an accidental nested source path.
"""
import sys
from pathlib import Path

# Source/CAD/Master/build_common.py -> repo root.
ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "Source"
CAD_SOURCE = SOURCE / "CAD"

for path in (SOURCE, CAD_SOURCE):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import cadquery as cq
from Parameters.master_parameters import PARAMS, OriParams


def cyl(radius: float, length: float, axis: str = "Z"):
    """Cylinder of given length centered on the origin, along axis."""
    if axis == "Z":
        return cq.Workplane("XY").circle(radius).extrude(length).translate((0, 0, -length / 2))
    if axis == "X":
        return cq.Workplane("YZ").circle(radius).extrude(length).translate((0, 0, -length / 2)).rotate((0, 0, 0), (1, 0, 0), 90)
    if axis == "Y":
        return cq.Workplane("XZ").circle(radius).extrude(length).translate((0, 0, -length / 2)).rotate((0, 0, 0), (0, 1, 0), 90)
    raise ValueError(f"Unsupported cylinder axis: {axis!r}")


def box_centered(x: float, y: float, z: float, fillet: float = 0.0):
    """Create a centered rectangular solid, optionally filleted on all edges."""
    wp = cq.Workplane("XY").box(x, y, z)
    return wp.edges().fillet(fillet) if fillet > 0 else wp


def export(part, name: str, subdir: str = "handoff", fmt=("step", "stl")):
    """Export a CAD part to canonical tracked deliverable folders.

    ``subdir`` is a label under both ``CAD_STEP`` and ``CAD_STL``. For example,
    ``subdir='leg'`` writes ``CAD_STEP/leg/name.step`` and
    ``CAD_STL/leg/name.stl``. Empty/None uses the handoff root.

    This keeps STEP/STL deliverables separate from editable source code and makes
    generated output deterministic for humans and CI.
    """
    label = Path(subdir).as_posix().strip("/") if subdir else "handoff"
    # Historical callers sometimes passed CAD/Exports/... from the old layout.
    # Strip that prefix so the canonical output folders remain top-level.
    if label.startswith("CAD/Exports/"):
        label = label[len("CAD/Exports/"):]
    elif label == "CAD/Exports":
        label = "handoff"

    paths = {}
    if "step" in fmt:
        out = ROOT / "CAD_STEP" / label
        out.mkdir(parents=True, exist_ok=True)
        path = out / f"{name}.step"
        cq.exporters.export(part, str(path))
        paths["step"] = path
    if "stl" in fmt:
        out = ROOT / "CAD_STL" / label
        out.mkdir(parents=True, exist_ok=True)
        path = out / f"{name}.stl"
        cq.exporters.export(part, str(path))
        paths["stl"] = path
    return paths


def bounds(part):
    """Return ``(xmin, xmax, ymin, ymax, zmin, zmax)`` for a CAD part."""
    bb = part.val().BoundingBox() if hasattr(part, "val") else part.BoundingBox()
    return (bb.xmin, bb.xmax, bb.ymin, bb.ymax, bb.zmin, bb.zmax)


def size_of(part):
    """Return CAD bounding-box size as ``(X, Y, Z)``."""
    b = bounds(part)
    return (b[1] - b[0], b[3] - b[2], b[5] - b[4])


if __name__ == "__main__":
    print("build_common OK; ROOT =", ROOT)
