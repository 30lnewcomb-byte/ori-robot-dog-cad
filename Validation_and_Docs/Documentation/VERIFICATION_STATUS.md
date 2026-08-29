# Ori Robot Dog — Verification Status

Legend: **VERIFIED** = supported by concrete source/data or an automated check. **ASSUMED** = engineering assumption. **UNKNOWN/BLOCKED** = unresolved.

## CAD infrastructure

- Parametric source: **VERIFIED** — `Source/CAD/` and `Source/Parameters/master_parameters.py`.
- Master-parameter architecture: **VERIFIED** — current generators consume `PARAMS`.
- Repository-root/export helper: **VERIFIED** — `Source/CAD/Master/build_common.py` resolves the repository root and targets `CAD_STEP/` + `CAD_STL/`.
- One-command validation: **VERIFIED** — `python validate.py` runs the core suite.
- GitHub Actions CAD smoke build: **VERIFIED** — a corrected pipeline completed successfully before the latest source refinements; latest runs continue to exercise the pipeline.

## Actuator / hardware dimensions

- HTD-45H body 51.1 × 20.14 × 40 mm: **VERIFIED in project record**.
- HTD-45H mass 64 g: **VERIFIED in project record**.
- HTD-45H nominal torque 4.41 N·m @ 11.1 V: **VERIFIED in project record**.
- Canonical servo model shaft axis = +Z: **VERIFIED from `htd45h.py`**.
- 626ZZ = 6 × 19 × 6 mm: **VERIFIED in project record**.
- 6 mm steel shaft interface to 626ZZ: **VERIFIED as the current design interface; physical fit untested**.
- M3 fastener / insert dimensions: **ASSUMED/typical where explicitly marked in parameters**.

## Current mechanical architecture

### Legs

- Four leg instances from one master architecture: **VERIFIED in `assembly.py`**.
- Two actuated joints per leg (hip pitch + knee): **VERIFIED in current source**.
- Current leg actuated DOF: **8**.
- Hip-yaw target joints: **NOT IN CAD**.
- Passive ankle/contact-foot concept: **DESIGNED; physical validation pending**.

### Arm

- Six arm joints: **VERIFIED in current `arm.py` architecture**.
- Grip actuator located in forearm/wrist rather than tool: **VERIFIED in current source**.
- Grip servo shaft and spur gear axis: **corrected to +X** in current source.
- Passive tool gear shares the wrist gear plane: **implemented in current source; physical mesh still untested**.
- Standard passive coupler: **DESIGNED** (Ø34 mm, 2× M3 + Ø4 dowel).
- 500 g payload: **engineering target only; not physically verified**.

### Head

- Two actuated joints: **VERIFIED in current source**.
- Pan axis: **Z**.
- Pitch axis: **Y**.
- HTD-45H installation transforms were corrected to match the canonical +Z servo model.

## Current DOF accounting

- Implemented leg DOF: **8**.
- Arm DOF: **6**.
- Head DOF: **2**.
- **Implemented total: 16 DOF.**
- Target architecture: **20 DOF**.
- Gap: **four hip-yaw mechanisms are not yet implemented**.

## Manufacturing

- Bambu Lab A1 Mini build volume 180³ mm: **VERIFIED project constraint**.
- Current part-envelope validation: **AUTOMATED** through `validate.py` / GitHub Actions.
- Torso split strategy: **DESIGNED**, because the full 300 mm torso exceeds the printer envelope.
- Physical print fit and dimensional accuracy: **UNKNOWN until printed**.

## Analysis

- Static torque/load calculations: **AUTOMATED calculation only**.
- Shaft sizing: **AUTOMATED calculation only**.
- PETG linear-static FEA: **ANALYSIS ONLY**; optional dependency set, no physical correlation.
- FDM layer anisotropy: **UNKNOWN**.
- Dynamic/fatigue behavior: **UNKNOWN**.
- Gear wear/backlash: **UNKNOWN**.
- Heat-set insert pull-out: **UNKNOWN**.
- Torso seam strength: **UNKNOWN**.

## Documentation integrity

- Software joint map explicitly separates `IN_CAD` from `NOT_IN_CAD`: **VERIFIED**.
- Historical worm/rack gripper discussion removed from the current engineering record: **VERIFIED**.
- Historical/reference files remain available where useful: **VERIFIED**.

## Open engineering decisions

1. Implement the four hip-yaw joints, or deliberately freeze the design at the current 8-DOF leg architecture.
2. Resolve Raspberry Pi 3 vs Pi 4 for final electronics packaging.
3. Perform physical print/assembly validation.
4. Validate the arm/gripper mechanism under real loads.
5. Revisit upper-link crash-case reinforcement based on evidence, not appearance.
