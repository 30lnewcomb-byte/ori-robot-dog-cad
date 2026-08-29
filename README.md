# Ori Robot Dog CAD

This repository is the **mechanical/CAD workspace** for Ori Robot Dog.

## Source of truth

The authoritative design is the parametric Python/CadQuery source plus the master parameter system:

- `Source/Parameters/master_parameters.py` — dimensional and hardware source of truth.
- `Source/CAD/` — parametric part and assembly generators.
- `CAD_STEP/` — tracked high-fidelity STEP deliverables.
- `CAD_STL/` — tracked print-ready STL deliverables.
- `OpenSCAD/` — simplified, text-readable preview/reference models only.
- `Validation_and_Docs/` — validators, analyses, BOM, handoff, and engineering records.
- `ENGINEERING_RULES.md` — quality rules for AI-assisted mechanical work.

Generated STEP/STL files are deliverables, not editable design authority. Regenerate them from source after changing the parametric CAD.

## Current implementation state

The repository currently generates:

- 4 mirrored instances of one master leg architecture.
- **2 actuated joints per leg**: hip pitch + knee.
- **8 implemented leg DOF** total.
- A **6-DOF arm** with the grip actuator kept in the wrist/forearm and a passive interchangeable gripper.
- A **2-DOF head**: pan about Z and pitch about Y.
- **16 implemented actuated DOF** total (8 legs + 6 arm + 2 head).

The project also carries a **20-DOF target architecture** (12 leg + 6 arm + 2 head). The additional four hip-yaw joints are specified in the software handoff but are **not implemented in the current leg CAD**. See `Validation_and_Docs/Documentation/KNOWN_BLOCKERS.md`.

## One-command validation

From the repository root:

```text
python validate.py
```

This runs the static repository audit and the geometry/engineering validation suite. GitHub Actions runs the same validation plus a separate CadQuery smoke build.

A successful validation run proves only what the automated checks actually test. It does **not** prove physical fit, print success, FDM layer strength, dynamic behavior, servo performance under load, or long-term gear wear.

## CAD quality standard

Read `ENGINEERING_RULES.md` before changing mechanical source.

The core rule is simple:

> **AI-generated CAD is not considered correct because it looks good. The person making the CAD owns the reasonable verification work.**

Meaningful changes should be checked against geometry, hardware interfaces, assembly, motion and clearance, printability, structural assumptions, mass effects, and downstream dependencies.

## Known engineering blockers

- Hip-yaw is a target feature but not yet implemented in the CAD.
- Raspberry Pi 3 vs Pi 4 remains unresolved for the final electronics packaging.
- The upper-link crash-case reinforcement decision remains intentionally deferred.
- No physical assembly/printing validation has been performed yet.
- Gear backlash/wear, heat-set insert pull-out, seam strength, dynamic loads, and FDM anisotropy remain physical-validation tasks.
