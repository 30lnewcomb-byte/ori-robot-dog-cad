# Ori Arm — Wrist / Gripper Engineering Record

**Status:** Current architecture record. CAD implementation is in `Source/CAD/Arm/arm.py`. No physical print or hardware test has been performed.

## Design goal

Keep the interchangeable tool light by keeping its grip actuator in the forearm/wrist. The current arm uses six HTD-45H actuators on the arm structure and a passive printed gripper.

## Current transmission

```text
HTD-45H grip servo in forearm
        ↓  (+X shaft)
printed spur gear
        ↓  18 mm center distance
coupler gear on tool
        ↓
eccentric cam representation
        ↓
two passive fingers
```

The grip servo and grip gear are now explicitly aligned to the same X shaft/gear axis in `arm.py`. Earlier concept code used a different arrangement and remains reference material only.

## Current arm geometry inputs

From `Source/Parameters/master_parameters.py`:

- Shoulder link: 70 mm
- Elbow link: 60 mm
- Wrist link: 28 mm
- Grip depth: 30 mm
- Grip target span: 44 mm baseline
- Coupler: Ø34 mm
- Gear module: 1.5
- Drive gear: 12 teeth
- Coupler gear: 12 teeth
- Derived gear center distance: 18 mm
- Grip-servo Y location: −18 mm relative to the wrist centerline

## What has been fixed in this record

The old analysis mixed several generations of the gripper architecture, including worm/rack text that no longer describes the current `arm.py`. That language has been removed from the current engineering record.

The current implementation is also careful not to call the servo's unpowered state "self-locking". Whether an unpowered actuator resists backdrive depends on the actual servo mechanics/electronics and must be verified on hardware.

## Payload calculation

The 500 g payload remains a **design target**. Static gravity torque is calculated from the current parameterized arm dimensions. This is useful for sizing but does not prove dynamic performance, joint stiffness, printed-layer strength, or reliable grasping.

## Mass calculation

The analysis must keep purchased servo mass separate from printed PETG mass. The current arm source embeds servo envelope geometry inside printed-part compounds for placement. Treating that entire compound as PETG would double-count or misclassify mass.

`analyze_arm_wrist.py` therefore reports the actuator count separately and should not be interpreted as a final weighed arm assembly.

## Physical validation still required

- Gear backlash and wear after printing.
- Actual finger travel and parallelism.
- Coupler fit and repeatability.
- Grip force with real pads.
- Servo behavior under load, including power-off/backdrive behavior.
- Wrist/arm deflection under the 500 g target.
- FDM layer-direction effects and fatigue.

**Engineering rule:** do not upgrade these items from analysis/assumption to VERIFIED until the relevant evidence exists.
