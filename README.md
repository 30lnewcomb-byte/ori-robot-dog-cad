# Ori Robot Dog CAD

This repository is the mechanical/CAD workspace for the Ori Robot Dog.

## Source of truth

The authoritative design is the parametric Python/CadQuery source under `Source/CAD` plus `Source/Parameters/master_parameters.py`.

- `Source/Parameters/` — master dimensions and hardware parameters.
- `Source/CAD/` — parametric part and assembly generators.
- `Source/CAD/Master/` — shared CAD/build helpers and hardware primitives.
- `CAD_STEP/` — tracked STEP deliverables.
- `CAD_STL/` — tracked STL deliverables.
- `OpenSCAD/` — simplified/reference geometry; not the authoritative manufacturing model.
- `Source/CAD/Exports/` — generated exports from the parametric build.

## Verification

Run the lightweight repository audit:

```text
python Source/CAD/validation/repo_audit.py
```

The GitHub Actions workflow also installs CadQuery and performs a CAD smoke build of the leg, torso, head, arm, and full robot assembly.

A successful smoke build proves that the current source can construct valid CAD solids and report non-zero bounds. It does **not** by itself prove structural safety, real-world fit, servo performance, or print success.

## Engineering standard

Read `ENGINEERING_RULES.md` before changing mechanical source. In particular, generated CAD is not considered correct merely because it looks good or exports successfully; meaningful changes must be checked against interfaces, assembly, motion, manufacturing, and engineering assumptions.

## Design status

The repository contains both current design source and historical/concept/reference material. Concept and reference files must not silently override the current parametric design.

When a design decision changes, update the relevant source, validation, and documentation rather than leaving conflicting claims scattered through the repository.
