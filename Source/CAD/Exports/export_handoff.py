"""
Export the MECHANICAL HANDOFF deliverables (§13).

Generates STEP (CAD) + STL (print) for every printable part into a dedicated
  CAD/Exports/handoff/
directory, kept separate from any development/reference files. The untouched
baseline/reference files are NOT overwritten.

Run: python CAD/Exports/export_handoff.py
"""
import sys, os
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]  # export_handoff.py is in CAD/Exports/
for d in ("CAD/Master", "CAD/Legs", "CAD/Torso", "CAD/Head", "CAD/Arm", "CAD/Hardware/servos", "Parameters", "CAD/Assemblies"):
    sys.path.insert(0, str(ROOT / d))
import build_common as bc
import master_leg as ML, torso as TR, head as HD, arm as ARM
from Parameters.master_parameters import PARAMS as P

OUT = ROOT / "CAD" / "Exports" / "handoff"
OUT.mkdir(parents=True, exist_ok=True)

def exp(name, shape, step=True, stl=True):
    fmt = tuple([f for f, want in (("step", step), ("stl", stl)) if want]) or ("step",)
    bc.export(shape, name, subdir=str(OUT), fmt=fmt)

def main():
    # legs (upper/lower/foot) - one canonical leg
    exp("leg.upper_link", ML.make_upper_link(P))
    exp("leg.lower_link", ML.make_lower_link(P))
    ft, sw = ML.make_foot(P)
    exp("leg.foot", ft)
    # torso halves
    exp("torso.front_half", TR._half_shell(+1, P))
    exp("torso.rear_half", TR._half_shell(-1, P))
    # head parts
    h = HD.make_head(P)
    exp("head.dome", h["dome"])
    exp("head.neck", h["neck"])
    exp("head.pitch_yoke", h["pitch_yoke"])
    # arm sub-assemblies
    for nm, pt in ARM.make_arm(P)[0].items():
        exp(f"arm.{nm}", pt)
    # full robot (STEP reference only)
    import assembly as A
    exp("robot.full", A.make_robot(export_parts=False)["full"], stl=False)
    # metal shafts (STEP reference for machining)
    import shafts as SH
    for j in ("hip", "knee", "shoulder", "elbow", "wrist"):
        sh, br = SH.shaft_for_joint(j)
        exp(f"shaft.{j}", sh, stl=False)
    print(f"Exports written to: {OUT}")
    print(f"Files: {len(list(OUT.glob('*')))}")

if __name__ == "__main__":
    main()
