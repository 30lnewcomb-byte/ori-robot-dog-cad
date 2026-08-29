"""
Ori Robot Dog - arm/wrist/gripper load analysis.

CAD-grounded calculation only. No physical testing is implied.
Run: python Validation_and_Docs/Validation/analyze_arm_wrist.py
"""
import sys
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "Source"
CAD = SOURCE / "CAD"
for d in (SOURCE, CAD, CAD / "Master", CAD / "Arm", CAD / "Hardware" / "servos", SOURCE / "Parameters"):
    sys.path.insert(0, str(d))

import cadquery as cq
import build_common as bc
import arm as ARM
from Parameters.master_parameters import PARAMS as P

RHO_PETG = 1.27e-3      # g/mm^3
G = 9.81
HTD45H_TORQUE_NM = P.servo.torque_nm
HTD45H_CONT_NM = HTD45H_TORQUE_NM * 0.55
HTD45H_MASS_G = P.servo.mass
MARGIN = 2.0

def part_mass_g(solid):
    return solid.val().Volume() * RHO_PETG

def main():
    a = P.arm
    parts, _ = ARM.make_arm(P)
    mass = {k: part_mass_g(v) for k, v in parts.items()}
    petg_total = sum(mass.values())
    actuator_total = a.dof * HTD45H_MASS_G
    all_up = petg_total + actuator_total

    reach_wrist = a.shoulder_len + a.elbow_len + a.wrist_len
    payload_torque = (a.payload_g / 1000.0) * G * (reach_wrist / 1000.0)
    wrist_torque = (a.payload_g / 1000.0) * G * (a.wrist_len / 1000.0)
    grip_torque = (a.payload_g / 1000.0) * G * (a.grip_depth / 1000.0) / 0.4

    rows = [
        ("wrist pose torque margin", HTD45H_CONT_NM / max(payload_torque, 1e-9), 1.0,
         f"continuous~{HTD45H_CONT_NM:.2f} / required~{payload_torque:.3f} N.m"),
        ("wrist joint torque margin", HTD45H_CONT_NM / max(wrist_torque, 1e-9), 1.0,
         f"continuous~{HTD45H_CONT_NM:.2f} / required~{wrist_torque:.3f} N.m"),
        ("grip actuator torque margin", HTD45H_CONT_NM / max(grip_torque, 1e-9), 1.0,
         f"continuous~{HTD45H_CONT_NM:.2f} / required~{grip_torque:.3f} N.m"),
    ]

    print("=" * 78)
    print("ORI ARM / WRIST LOAD ANALYSIS (CALCULATION ONLY)")
    print("=" * 78)
    for k, m in mass.items():
        print(f"{k:16s} {m:7.1f} g PETG")
    print(f"PETG total:       {petg_total:7.1f} g")
    print(f"6x HTD-45H:       {actuator_total:7.1f} g")
    print(f"Arm all-up:       {all_up:7.1f} g")
    print(f"Payload target:   {a.payload_g:.0f} g")
    print(f"Reach to wrist/tool chain: {reach_wrist:.1f} mm")
    print()
    ok = True
    for name, margin, minimum, detail in rows:
        passed = margin >= minimum
        ok = ok and passed
        print(f"{name:28s} {'PASS' if passed else 'FAIL'}  {margin:6.1f}x  {detail}")
    print(f"overall: {'PASS' if ok else 'FAIL'}")
    return ok

if __name__ == "__main__":
    sys.exit(0 if main() else 1)
