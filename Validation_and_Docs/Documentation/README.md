# Ori Robot Dog — CAD Documentation Index

This folder records the mechanical state of the Ori Robot Dog CAD repository.

## Current source layout

```text
Source/Parameters/master_parameters.py   master dimensional/hardware parameters
Source/CAD/                               parametric CadQuery generators
CAD_STEP/                                tracked STEP deliverables
CAD_STL/                                 tracked STL deliverables
Validation_and_Docs/Validation/           automated checks and engineering analyses
```

## Current implementation

The current CAD generates four mirrored instances of one master leg with **2 actuated joints per leg** (hip pitch + knee), plus a 6-DOF arm and 2-DOF head. That is **16 implemented actuated DOF**.

The project still contains a **20-DOF target architecture** with four hip-yaw joints. Those target hip-yaw joints are explicitly marked `NOT_IN_CAD` in `software_handoff_jointmap.json` and are tracked in `KNOWN_BLOCKERS.md`.

## Read these first

- `MECHANICAL_HANDOFF.md` — current mechanical handoff.
- `KNOWN_BLOCKERS.md` — unresolved engineering decisions and implementation gaps.
- `VERIFICATION_STATUS.md` — evidence/status matrix.
- `software_handoff_jointmap.json` — current software/mechanical interface map.
- `PRINTABLE_SPLITS.md` — current A1 Mini print strategy.

## Validation

From the repository root:

```text
python validate.py
```

GitHub Actions runs the same suite after a CadQuery smoke build.

Validation is intentionally evidence-based. Geometry construction is not the same thing as physical validation.

## Historical/reference material

Concept and baseline files are retained when they preserve useful engineering history, but they are not allowed to silently override current source. Older records should be labeled historical/reference when they describe superseded mechanisms or dimensions.
