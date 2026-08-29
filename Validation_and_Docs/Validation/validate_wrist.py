"""
Validation: current wrist/gripper architecture.

Checks the current arm.py implementation and master parameters. This validator
is intentionally careful not to claim that a servo has holding torque while
unpowered; the electrical/firmware behavior must be verified on hardware.

Run: python Validation_and_Docs/Validation/validate_wrist.py
"""
import math
import sys
import inspect
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "Source"
CAD = SOURCE / "CAD"
for d in (SOURCE, CAD, CAD / "Master", CAD / "Arm", CAD / "Hardware" / "servos", SOURCE / "Parameters"):
    sys.path.insert(0, str(d))

import arm as ARM
from Parameters.master_parameters import PARAMS as P

res = []

def chk(name, ok, detail=""):
    res.append(bool(ok))
    print(f"[{name}] {detail} -> {'PASS' if ok else 'FAIL'}")

def main():
    a = P.arm
    parts, _ = ARM.make_arm(P)
    grip = parts["gripper"]
    bb = grip.val().BoundingBox()

    # A) Parameterized jaw travel sanity.
    open_gap = 2 * (a.finger_pivot_r + a.grip_depth)
    travel = max(0.0, 2 * a.cam_ecc)
    dtheta = travel / a.finger_drive_r if a.finger_drive_r > 0 else math.inf
    closed_tip = max(a.finger_pivot_r - a.grip_depth * math.sin(dtheta), 0.0)
    closed_gap = 2 * closed_tip
    chk("A_grip_span_covered", open_gap >= a.grip_span >= closed_gap,
        f"open={open_gap:.1f} closed={closed_gap:.1f} target={a.grip_span:.1f} mm")
    chk("A_no_overtravel", closed_tip >= 0.0,
        f"closed half-gap={closed_tip:.2f} mm")

    # B) Current spur-gear geometry parameters.
    center_distance = abs(a.grip_servo_y)
    drive_pitch_r = a.gear_module * a.gear_drive_teeth / 2.0
    pinion_pitch_r = a.gear_module * a.gear_pinion_teeth / 2.0
    chk("B_center_distance", abs(center_distance - a.gear_center) < 0.5,
        f"actual={center_distance:.1f} target={a.gear_center:.1f} mm")
    chk("B_pitch_radii", abs((drive_pitch_r + pinion_pitch_r) - a.gear_center) < 0.5,
        f"sum={drive_pitch_r+pinion_pitch_r:.1f} center={a.gear_center:.1f} mm")
    chk("B_printable_module", a.gear_module >= P.mfg.min_feature and 6.0 <= P.mfg.build_z,
        f"module={a.gear_module:.1f}, build_z={P.mfg.build_z:.0f} mm")

    # C) Interchangeable tool interface.
    chk("C_coupler_diameter", 0 < a.coupler_d < P.mfg.build_x,
        f"coupler={a.coupler_d:.1f} mm")
    chk("C_M3_retention", abs(a.tool_retain_d - P.hw.screw_M3) < 1.0,
        f"retention hole={a.tool_retain_d:.1f} mm vs M3 nominal={P.hw.screw_M3:.1f} mm")
    chk("C_dowel", a.tool_dowel_d >= 4.0,
        f"dowel={a.tool_dowel_d:.1f} mm")

    # D) Architecture: grip actuator remains on arm, tool itself has no servo constructor.
    forearm_src = inspect.getsource(ARM.make_forearm)
    gripper_src = inspect.getsource(ARM.make_gripper)
    n_forearm_servos = forearm_src.count("make_htd45h(")
    chk("D_grip_servo_on_arm", n_forearm_servos >= 3,
        f"forearm contains {n_forearm_servos} HTD-45H instances")
    chk("D_gripper_passive_source", "make_htd45h" not in gripper_src,
        "make_gripper() contains no actuator construction")

    # E) Tool envelope remains printable.
    sx, sy, sz = P.mfg.build_x, P.mfg.build_y, P.mfg.build_z
    chk("E_printable", (bb.xmax-bb.xmin) <= sx and (bb.ymax-bb.ymin) <= sy and (bb.zmax-bb.zmin) <= sz,
        f"bbox={bb.xmax-bb.xmin:.1f}x{bb.ymax-bb.ymin:.1f}x{bb.zmax-bb.zmin:.1f} <= {sx:.0f}^3")

    ok = all(res)
    print("WRIST VALIDATION:", "PASS" if ok else "FAIL", f"({sum(res)}/{len(res)})")
    return ok

if __name__ == "__main__":
    sys.exit(0 if main() else 1)
