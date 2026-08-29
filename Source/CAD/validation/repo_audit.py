"""Static repository audit for Ori Robot Dog CAD.

This uses only the Python standard library so it can run before CadQuery is
installed. It checks repository structure, source-of-truth hygiene, current
implementation-state markers, and obvious stale-path hazards. Geometry checks
belong to the CAD/validation stage.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / "Source"
CAD = SOURCE / "CAD"
PARAMS = SOURCE / "Parameters" / "master_parameters.py"
JOINTMAP = ROOT / "Validation_and_Docs" / "Documentation" / "software_handoff_jointmap.json"
ENGINEERING_RULES = ROOT / "ENGINEERING_RULES.md"

ERRORS = []
WARNINGS = []


def error(message):
    ERRORS.append(message)


def warn(message):
    WARNINGS.append(message)


def main():
    required = [PARAMS, ENGINEERING_RULES, ROOT / "CAD_STEP", ROOT / "CAD_STL"]
    for path in required:
        if not path.exists():
            error(f"Missing required repository item: {path.relative_to(ROOT)}")

    py_files = sorted(CAD.rglob("*.py"))
    if not py_files:
        error("No CAD Python source files found under Source/CAD")

    # The shared helper must resolve the actual repository root from
    # Source/CAD/Master/build_common.py (parents[3]). A wrong root silently sends
    # exports into the source tree.
    build_common = CAD / "Master" / "build_common.py"
    if build_common.exists():
        text = build_common.read_text(encoding="utf-8", errors="replace")
        if "parents[3]" not in text.splitlines()[0:40].__str__():
            error("build_common.py must resolve the repository root with parents[3]")
        if "CAD_STEP" not in text or "CAD_STL" not in text:
            error("build_common.py must export to canonical CAD_STEP/CAD_STL folders")

    # Catch obvious stale path assumptions in source. Source files should not
    # assume the historical pre-Source layout when locating the repository root.
    for path in py_files:
        text = path.read_text(encoding="utf-8", errors="replace")
        rel = path.relative_to(ROOT)
        if "CAD/Master" in text and "parents[2]" in text and path.name != "repo_audit.py":
            warn(f"Historical path pattern found in {rel}; verify imports/root resolution")

    # Catch dimensions hard-coded in part generators. This is a heuristic, not
    # a ban: algorithm constants and hardware standards can legitimately exist.
    hardcoded = []
    for path in py_files:
        if path.name in {"build_common.py", "repo_audit.py"}:
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        if "PARAMS" not in text:
            warn(f"{path.relative_to(ROOT)} does not reference PARAMS")
        for line_no, line in enumerate(text.splitlines(), 1):
            if re.search(r"\b(?:\d{2,3}(?:\.\d+)?)\s*(?:mm)?\b", line) and any(
                token in line.lower() for token in ("length", "width", "height", "diameter", "radius", "spacing", "clearance")
            ):
                hardcoded.append(f"{path.relative_to(ROOT)}:{line_no}: {line.strip()}")
    if hardcoded:
        warn(f"Found {len(hardcoded)} heuristic hard-coded-dimension candidates; review rather than blindly replacing them.")

    # Concept/reference files must clearly declare non-final status.
    for path in sorted(CAD.rglob("*concept*.py")):
        text = path.read_text(encoding="utf-8", errors="replace").lower()
        if "concept" not in text[:2500] or "non-final" not in text[:2500]:
            warn(f"Concept file should clearly declare non-final status: {path.relative_to(ROOT)}")

    # Parameter status vocabulary is mandatory.
    if PARAMS.exists():
        ptext = PARAMS.read_text(encoding="utf-8", errors="replace")
        for label in ("VERIFIED", "ASSUMED", "UNKNOWN", "BLOCKED"):
            if label not in ptext:
                error(f"Parameter system is missing status label: {label}")

    # Current implementation state must be explicit in the software handoff.
    if JOINTMAP.exists():
        jtext = JOINTMAP.read_text(encoding="utf-8", errors="replace")
        for marker in ('"current_cad_leg_dof": 8', '"current_cad_actuated_dof": 16', '"target_actuated_dof": 20', '"implementation_status": "NOT_IN_CAD"'):
            if marker not in jtext:
                error(f"Joint map missing current/target implementation marker: {marker}")
        if '"head_pan",' in jtext and '"axis": "Z"' not in jtext:
            error("Joint map does not document head pan as Z axis")
        if '"head_pitch",' in jtext and '"axis": "Y"' not in jtext:
            error("Joint map does not document head pitch as Y axis")

    # Both root READMEs exist. They should not disagree about the core 16-vs-20
    # implementation state.
    readmes = [ROOT / "README.md", ROOT / "README.txt"]
    readme_texts = [p.read_text(encoding="utf-8", errors="replace") for p in readmes if p.exists()]
    if len(readme_texts) == 2:
        for text in readme_texts:
            if "16" not in text or "20" not in text or "hip-yaw" not in text.lower():
                warn("Root README pair should explicitly describe current 16-DOF state and 20-DOF target/hip-yaw distinction")

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
