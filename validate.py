"""One-command entry point for the Ori CAD validation suite.

Run from the repository root:
    python validate.py

The script deliberately executes the same checks used by CI so local and GitHub
results stay aligned. It never treats a skipped tool as a pass.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CHECKS = [
    ("static repository audit", [sys.executable, "Source/CAD/validation/repo_audit.py"]),
    ("leg geometry validation", [sys.executable, "Validation_and_Docs/Validation/validate_leg.py"]),
    ("robot geometry validation", [sys.executable, "Validation_and_Docs/Validation/validate_robot.py"]),
    ("wrist/gripper validation", [sys.executable, "Validation_and_Docs/Validation/validate_wrist.py"]),
    ("leg load analysis", [sys.executable, "Validation_and_Docs/Validation/analyze_loads.py"]),
    ("shaft sizing analysis", [sys.executable, "Validation_and_Docs/Validation/analyze_shafts.py"]),
    ("arm/wrist load analysis", [sys.executable, "Validation_and_Docs/Validation/analyze_arm_wrist.py"]),
]


def main() -> int:
    failures = []
    print("ORI ROBOT DOG — FULL CAD VALIDATION")
    print("=" * 42)
    for name, command in CHECKS:
        print(f"\n>>> {name}")
        result = subprocess.run(command, cwd=ROOT)
        if result.returncode != 0:
            failures.append((name, result.returncode))
            print(f"<<< FAIL ({result.returncode})")
        else:
            print("<<< PASS")

    print("\n" + "=" * 42)
    if failures:
        print(f"RESULT: FAIL — {len(failures)} check(s) failed")
        for name, code in failures:
            print(f"  - {name}: exit {code}")
        return 1
    print(f"RESULT: PASS — {len(CHECKS)}/{len(CHECKS)} checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
