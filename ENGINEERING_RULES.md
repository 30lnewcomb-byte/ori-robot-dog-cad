# Ori Robot Dog CAD — Engineering Rules

These rules are the quality bar for every CAD change in this repository.

## 1. AI-generated CAD is never accepted on appearance alone
A model that looks plausible, exports successfully, or imports into a CAD viewer is not automatically correct.

## 2. The person making the CAD owns the verification work
When an AI or script creates or modifies geometry, it must perform the reasonable verification work itself. The user should not be expected to discover avoidable CAD mistakes.

## 3. Verify the whole interface chain
For every meaningful mechanical change, check as applicable:
- mating geometry and datums
- servo/bearing/fastener interfaces
- clearances and collision risks
- assembly order and tool access
- motion and joint limits
- printability and split strategy
- structural load paths and assumptions
- mass/center-of-mass implications
- downstream assembly and exports

## 4. One source of truth
Dimensions that drive geometry belong in `Source/Parameters/master_parameters.py` unless there is a documented reason otherwise. Do not silently duplicate authoritative dimensions inside individual part generators.

## 5. Do not silently revive old concepts
Files named `concept`, `reference`, `legacy`, or similar are not authoritative unless the current design explicitly promotes them. Historical analyses may be useful evidence but must not override current source code or parameters.

## 6. Changes must be traceable
A change should leave enough information in code, validation output, or documentation to answer:
- what changed?
- why did it change?
- what was checked?
- what remains unverified?

## 7. Validation claims must be earned
`VERIFIED` means there is a measurement, manufacturer source, reproducible calculation, or other concrete evidence. `ASSUMED`, `UNKNOWN`, and `BLOCKED` must remain explicit until resolved.

## 8. Generated deliverables must not become the design source
STEP/STL files are deliverables. The parametric source and master parameters remain the design authority. Regenerate deliverables from source after source changes.

## 9. Preserve working interfaces by default
An improvement that breaks an existing interface is not an improvement unless the interface change is deliberate, documented, and propagated through all affected parts.

## 10. Prefer automation over memory
If a check can be made repeatable, put it in validation/CI rather than relying on a human remembering to perform it.

## Definition of done
A CAD change is complete only when the relevant source builds, the relevant automated checks pass, the affected interfaces have been reviewed, and any remaining uncertainty is explicitly recorded.
