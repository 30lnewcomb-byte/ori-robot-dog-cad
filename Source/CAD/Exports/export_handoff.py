"""
Export the MECHANICAL HANDOFF deliverables (§13).

Generates STEP (CAD) + STL (print) for every printable part into a dedicated
  CAD/Exports/handoff/
directory, kept separate from any development/reference files. The untouched
baseline/reference files are NOT overwritten.

Run: python CAD/Exports/export_handoff.py
"""
import sys
from pathlib import Path

# export_handoff.py lives in Source/CAD/Exports/. Two parents up is Source/.
ROOT = Path(__file__).resolve().parents[2]
for d in (
    "CAD/Master",
    "CAD/Legs",
    "CAD/Torso",
    "CAD/Head",
    "CAD/Arm",
    "CAD/Hardware/servos",
    "Parameters",
    "CAD/Assemblies",
):
    sys.path.insert(0, str(ROOT / d))

import build_common as bc
import master_leg as ML
import torso as TR
import head as HD
import arm as ARM
from Parameters.master_parameters import PARAMS as P

OUT = ROOT / "CAD" / "Exports" / "handoff"
OUT.mkdir(parents=True, exist_ok=True)


def exp(name, shape, step=True, stl=True):
    fmt = tuple(
        f for f, want in (("step", step), ("stl", stl)) if want
    ) or ("step",)
    bc.export(shape, name, subdir=str(OUT), fmt=fmt)


def validate_existing_shaft_references() -> None:
    """Verify Hermes' tracked metal-shaft STEP deliverables are present.

    These are machining/reference deliverables, not regenerated printed geometry.
    Keeping them as existing assets avoids introducing a second, guessed shaft
    source that could drift from the established mechanical design.
    """
    repo_root = ROOT.parent
    shaft_files = [
        repo_root / "CAD_STEP" / "shaft.hip.step",
        repo_root / "CAD_STEP" / "shaft.knee.step",
        repo_root / "CAD_STEP" / "shaft.shoulder.step",
        repo_root / "CAD_STEP" / "shaft.elbow.step",
        repo_root / "CAD_STEP" / "shaft.wrist.step",
    ]
    missing = [str(path.relative_to(repo_root)) for path in shaft_files if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            "Required tracked metal-shaft STEP deliverables are missing: "
            + ", ".join(missing)
        )
    print("Metal shaft STEP references: PASS")
    for path in shaft_files:
        print(f"  {path.relative_to(repo_root)}")


def main():
    # legs (upper/lower/foot) - one canonical leg
    exp("leg.upper_link", ML.make_upper_link(P))
    exp("leg.lower_link", ML.make_lower_link(P))
    ft, sw = ML.make_foot(P)
    exp("leg.foot", ft)
    exp("leg.foot_switch", sw)

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

    # Metal shaft references are already tracked in CAD_STEP/ and must not be
    # silently regenerated from a guessed/duplicate source model.
    validate_existing_shaft_references()

    print(f"Exports written to: {OUT}")
    print(f"Files: {len(list(OUT.glob('*')))}")


if __name__ == "__main__":
    main()
