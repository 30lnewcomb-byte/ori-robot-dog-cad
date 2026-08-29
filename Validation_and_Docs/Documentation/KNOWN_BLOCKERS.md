# Ori Robot Dog CAD — Known Engineering Blockers

This file is intentionally blunt. These are issues that should be resolved before physical assembly or software is treated as mechanically authoritative.

## 1. Hip-yaw is specified but not implemented in the current CAD

The software handoff historically describes three actuated joints per leg (`hip_yaw`, `hip_pitch`, `knee`), but the current `Source/CAD/Legs/master_leg.py` generates only the hip/pitch servo and knee servo. There is no hip-yaw actuator, yaw-bearing stack, or yaw mechanism in the current leg generator.

### Consequence
- Current generated leg mechanics = **2 actuated joints per leg**.
- Current generated robot mechanics = **8 leg DOF + 6 arm DOF + 2 head DOF = 16 actuated DOF**.
- The previously documented 20-DOF / 12-leg-DOF figure is a **target architecture, not the implemented CAD state**.

### Required resolution
Either:
1. implement a real hip-yaw joint in the CAD (including actuator, bearing stack, shaft/load path, geometry, limits, assembly transforms, BOM, and validation), or
2. deliberately freeze the design as an 8-DOF leg system and propagate that decision into the software handoff and all documentation.

Do not silently choose one.

## 2. Head joint axes in the historical software handoff were reversed

The current CAD implementation has:
- `head_pan`: shaft axis **Z**
- `head_pitch`: shaft axis **Y**

The older joint map listed those axes in the opposite order. The joint map must match the generated CAD before firmware work relies on it.

## 3. Physical validation has not happened yet

All current CAD/FEA/torque claims are analysis or geometry checks. No printed assembly, bearing press-fit, insert pull-out, seam-strength, gear backlash/wear, servo loading, or hard-landing test has been completed.

## 4. Pi 3 vs Pi 4 remains unresolved

Mechanical documentation has both Pi 3 and Pi 4 references. Do not finalize the torso electronics layout until the controller board is selected.

## 5. Upper-link crash case remains documented, not solved

The recorded FEA crash case reaches a high local stress at the upper-link yoke root. The current decision is to document it and defer a gusset unless physical testing shows the need. This is not a claim that the crash case is safe.

## Rule
A documented blocker is better than a fictional completion state. The CAD repository should fail or loudly report when software/documentation starts treating an unimplemented mechanism as real.
