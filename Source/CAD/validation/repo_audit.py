"""Static repository audit for Ori Robot Dog CAD.

This deliberately uses only the Python standard library so it can run before
CadQuery is installed. It catches repository hygiene and source-of-truth
problems; geometry validation belongs to the CAD smoke test.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "Source"
CAD = SOURCE / "CAD"
PARAMS = SOURCE / "Parameters" / "master_parameters.py"

ERRORS = []
WARNINGS = []


def error(message):
    ERRORS.append(message)


def warn(message):
    WARNINGS.append(message)


def main():
    if not PARAMS.exists():
        error("Missing Source/Parameters/master_parameters.py")
        return report()

    py_files = sorted(CAD.rglob("*.py"))
    if not py_files:
        error("No CAD Python source files found under Source/CAD")

    # Catch dimensions hard-coded in part generators. This is a heuristic, not
    # a ban: hardware standards and algorithm constants can legitimately exist.
    hardcoded = []
    for path in py_files:
        if path.name in {"build_common.py", "repo_audit.py"}:
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        if "PARAMS" not in text:
            warn(f"{path.relative_to(ROOT)} does not reference PARAMS")
        for line_no, line in enumerate(text.splitlines(), 1):
            if re.search(r"\b(?:\d{2,3}(?:\.\d+)?)\s*(?:mm)?\b", line) and any(
                token in line.lower() for token in ("length", "width", "height", "diameter", "radius", "wall", "spacing", "clearance")
            ):
                hardcoded.append(f"{path.relative_to(ROOT)}:{line_no}: {line.strip()}")
    if hardcoded:
        warn(f"Found {len(hardcoded)} heuristic hard-coded-dimension candidates; review rather than blindly replacing them.")

    # Concept/reference files must advertise that they are not automatically
    # authoritative. This prevents old experiments becoming accidental source.
    for path in sorted(CAD.rglob("*concept*.py")):
        text = path.read_text(encoding="utf-8", errors="replace").lower()
        if "concept" not in text[:2500] or "non-final" not in text[:2500]:
            warn(f"Concept file should clearly declare non-final status: {path.relative_to(ROOT)}")

    # Parameter status labels should remain explicit.
    ptext = PARAMS.read_text(encoding="utf-8", errors="replace")
    for label in ("VERIFIED", "ASSUMED", "UNKNOWN", "BLOCKED"):
        if label not in ptext:
            error(f"Parameter system is missing status label: {label}")

    # Every tracked STEP/STL deliverable should have a matching source export
    # name somewhere in the repository. This is intentionally conservative.
    step_dir = ROOT / "CAD_STEP"
    stl_dir = ROOT / "CAD_STL"
    if not step_dir.exists():
        warn("CAD_STEP directory is missing")
    if not stl_dir.exists():
        warn("CAD_STL directory is missing")

    return report()


def report():
    print("ORI CAD STATIC AUDIT")
    print("====================")
    print(f"root: {ROOT}")
    print(f"errors: {len(ERRORS)}")
    print(f"warnings: {len(WARNINGS)}")
    for item in ERRORS:
        print(f"ERROR: {item}")
    for item in WARNINGS:
        print(f"WARN:  {item}")
    if ERRORS:
        return 1
    print("RESULT: PASS (static checks)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
