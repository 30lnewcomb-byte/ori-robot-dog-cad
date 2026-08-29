# Ori Mechanical Handoff — Delta / Cleanup Record

**Date of this cleanup:** 2026-08-29

This record supersedes stale path/state claims in the older 2026-08-25 handoff delta. Changes below are repository-maintenance and correctness work; they do not imply physical validation.

## What was corrected

- Added `ENGINEERING_RULES.md` to make AI-generated CAD verification requirements explicit.
- Added a standard GitHub Actions CAD validation workflow.
- Added `validate.py` as the single local validation entry point.
- Fixed repository-root resolution in `Source/CAD/Master/build_common.py`.
- Canonicalized generated STEP/STL output to top-level `CAD_STEP/` and `CAD_STL/` deliverable folders.
- Fixed validation scripts that still assumed the old pre-`Source/` directory layout.
- Reworked the full-robot validator so it does not contain always-true checks and reports the actual current 16-DOF implementation state.
- Reworked wrist validation so it does not claim powered/unpowered behavior that has not been physically tested.
- Corrected the wrist grip servo/gear coordinate system: the grip servo shaft and spur gear are now both along +X, and the passive tool gear is aligned to the same wrist plane.
- Updated the software joint map to distinguish implemented joints from target-only hip-yaw joints and corrected head pan/pitch axes.
- Updated BOM and documentation to distinguish current hardware from target hardware.
- Corrected the arm analysis record's PETG density-unit error.
- Reworked FEA source paths so the optional structural analysis uses the current repository layout.

## Current engineering state

- Current generated leg architecture: 2 actuated joints per leg (hip pitch + knee) = 8 leg DOF.
- Current arm: 6 DOF.
- Current head: 2 DOF.
- Current generated actuated DOF: **16**.
- Target architecture: **20 DOF**, requiring four hip-yaw mechanisms that are not yet in the CAD.

## Validation status

GitHub Actions has successfully completed the CadQuery smoke build on the corrected validation pipeline. The latest pipeline is now configured to continue through the full validation suite after the smoke build.

The checks remain CAD/calculation checks only. They do not replace physical print, fit, strength, dynamic, or hardware tests.

## Remaining blockers

1. Decide whether to implement the four hip-yaw mechanisms or intentionally freeze the 8-DOF leg architecture.
2. Resolve Pi 3 vs Pi 4 for final electronics packaging.
3. Perform physical print and assembly validation.
4. Validate grip backlash/wear, insert pull-out, seam strength, dynamic loads, and FDM anisotropy.
5. Revisit the upper-link crash-case reinforcement only when the decision is supported by testing or better structural analysis.
