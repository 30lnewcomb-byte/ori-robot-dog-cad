# Ori Engineering Rules

## AI/CAD responsibility

AI-generated CAD is never considered correct merely because it looks good, imports successfully, or passes a superficial check. Before a change is called complete, verify geometry, interfaces, assembly, motion/clearance, manufacturability, and relevant engineering assumptions. If a property cannot be verified, mark it unverified.

The agent making a CAD change is responsible for checking the result. The user should not have to discover mistakes that the agent could reasonably have caught.

## Source of truth

- `Source/CAD/` is the authoritative parametric CAD source.
- `Source/Parameters/master_parameters.py` is the authoritative parameter source.
- `CAD_STEP/` and `CAD_STL/` are deliverables/exports, not editable source.
- `OpenSCAD/` is legacy/reference material unless explicitly promoted.
- `Validation_and_Docs/` records validation, analysis, BOM, and handoff information.

## Mechanical integrity

- Preserve working interfaces unless a deliberate design change requires otherwise.
- Hardware dimensions and axes must come from the authoritative hardware models/parameters.
- A component must be checked in context where its loads, mating parts, motion, and assembly matter.
- Do not claim a target architecture is implemented when the current CAD does not implement it.
- Do not weaken, remove, or rewrite a validator merely to obtain PASS.
- Prefer fixing the design or fixing an objectively incorrect test.

## Arm reinforcement requirement

The arm's metal shaft inserts are structural reinforcement members, not cosmetic or optional hardware. The arm CAD must positively locate and retain them, provide adequate surrounding material, and provide a credible load path through the reinforced joint/link. Any arm change affecting a shaft interface must re-check shaft dimensions, retention, bearing/servo clearance, assembly access, and load-transfer assumptions.

## Validation

A green test is evidence only for what the test actually checks. Keep implementation status, analytical validation, and physical verification distinct.

When changing geometry, run the narrowest relevant validation first, then the complete repository validation before declaring the change complete.
