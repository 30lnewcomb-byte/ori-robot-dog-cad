"""Validation: full Ori robot (global self-check).

This validator checks the current parametric CAD rather than relying on stale
handoff numbers. It verifies assembly geometry, stance, ground contact, A1 Mini
part envelopes, current-vs-target DOF state, and key interfaces.

Run: python Validation_and_Docs/Validation/validate_robot.py
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "Source"
CAD = SOURCE / "CAD"
for d in (SOURCE, CAD, CAD / "Master", CAD / "Legs", CAD / "Torso", CAD / "Head", CAD / "Arm", SOURCE / "Parameters"):
    sys.path.insert(0, str(d))

import build_common as bc
from Parameters.master_parameters import PARAMS
import assembly as A
import torso as TR
import head as HD
import arm as ARM
import master_leg as ML

rows = []
def check(name, ok, detail=""):
    rows.append((name, "PASS" if ok else "FAIL", detail))

p = PARAMS
robot = A.make_robot(export_parts=False, merged=False)
full = robot["full"]
b = bc.bounds(full)
xs = b[1] - b[0]
ys = b[3] - b[2]
zs = b[5] - b[4]

check("assembly has positive dimensions", xs > 0 and ys > 0 and zs > 0, f"{xs:.1f}x{ys:.1f}x{zs:.1f} mm")
check("robot height exceeds old hobby target", zs > 340, f"Z={zs:.1f} mm")
check("robot width plausible", 150 < ys < 320, f"Y={ys:.1f} mm")
check("robot length plausible", 400 < xs < 750, f"X={xs:.1f} mm")

# --- ground / stance ---
foot_zs = []
foot_centers = {}
for nm in ("FL", "FR", "RL", "RR"):
    fb = bc.bounds(robot["legs"][nm]["foot"])
    foot_zs.append(fb[4])
    foot_centers[nm] = ((fb[0] + fb[1]) / 2, (fb[2] + fb[3]) / 2)
check("lowest point is at ground", b[4] >= -0.01 and b[4] < 2.0, f"Zmin={b[4]:.2f}")
check("all feet are on/near ground", max(foot_zs) - min(foot_zs) < 8.0, f"foot z={ [round(v,1) for v in foot_zs] }")
actual_stance_x = max(v[0] for v in foot_centers.values()) - min(v[0] for v in foot_centers.values())
actual_stance_y = max(v[1] for v in foot_centers.values()) - min(v[1] for v in foot_centers.values())
check("stance X tracks parameter", abs(actual_stance_x - p.scale.stance_length) < 70, f"actual={actual_stance_x:.1f} vs target={p.scale.stance_length:.1f}")
check("stance Y tracks parameter", abs(actual_stance_y - p.scale.stance_width) < 70, f"actual={actual_stance_y:.1f} vs target={p.scale.stance_width:.1f}")

# --- hip anchors ---
expected = {
    "FL": (+p.torso.hip_pitch_axis_x, +p.torso.hip_axis_y),
    "FR": (+p.torso.hip_pitch_axis_x, -p.torso.hip_axis_y),
    "RL": (-p.torso.hip_pitch_axis_x, +p.torso.hip_axis_y),
    "RR": (-p.torso.hip_pitch_axis_x, -p.torso.hip_axis_y),
}
for nm, (ex, ey) in expected.items():
    hb = bc.bounds(robot["legs"][nm]["hip_servo"])
    cx = (hb[0] + hb[1]) / 2
    cy = (hb[2] + hb[3]) / 2
    check(f"{nm} hip anchor", abs(cx - ex) < 35 and abs(cy - ey) < 35,
          f"actual=({cx:.1f},{cy:.1f}) expected=({ex:.1f},{ey:.1f})")

# --- A1 Mini printable envelopes ---
bx, by, bz = p.mfg.build_x, p.mfg.build_y, p.mfg.build_z
parts_to_check = [
    ("leg.upper", ML.make_upper_link(p)),
    ("leg.lower", ML.make_lower_link(p)),
]
foot, _switch = ML.make_foot(p)
parts_to_check.append(("leg.foot", foot))
parts_to_check.extend([
    ("torso.front", TR._half_shell(+1, p)),
    ("torso.rear", TR._half_shell(-1, p)),
    ("head.dome", HD.make_head(p, export_parts=False)["dome"]),
])
arm_parts, _arm_compound = ARM.make_arm(p)
parts_to_check.extend((f"arm.{nm}", pt) for nm, pt in arm_parts.items())
for nm, part in parts_to_check:
    sx, sy, sz = bc.size_of(part)
    check(f"A1 Mini fit {nm}", sx <= bx and sy <= by and sz <= bz,
          f"{sx:.1f}x{sy:.1f}x{sz:.1f} <= {bx:.0f}^3")

# --- current architecture versus target architecture ---
# The current master_leg implementation generates hip-pitch + knee only.
leg_parts = ML.make_leg_assembly(export_parts=False)
current_leg_servo_count = int("hip_servo" in leg_parts) + int("knee_servo" in leg_parts)
current_leg_dof = current_leg_servo_count * 4
current_total_dof = current_leg_dof + p.arm.dof + 2
target_leg_dof = 12
target_total_dof = target_leg_dof + p.arm.dof + 2
check("current leg CAD is 8-DOF", current_leg_dof == 8, f"{current_leg_servo_count} actuators/leg x 4 = {current_leg_dof}")
check("current CAD total DOF is 16", current_total_dof == 16, f"8 leg + {p.arm.dof} arm + 2 head")
check("target architecture documented", target_total_dof == 20, f"target={target_total_dof} DOF; hip-yaw remains unimplemented")

# --- arm interface ---
arm_mounted = ARM.make_arm_mounted(p)
ab = bc.bounds(arm_mounted)
check("arm extends forward of torso port", ab[1] > p.torso.arm_port_x, f"tip_x={ab[1]:.1f}, port_x={p.torso.arm_port_x:.1f}")
check("arm remains partially recessed", ab[0] < p.torso.arm_port_x + 10, f"arm_min_x={ab[0]:.1f}, port_x={p.torso.arm_port_x:.1f}")

print(f"{'CHECK':34s} {'RESULT':6s} DETAIL")
print("-" * 84)
for n, r, d in rows:
    print(f"{n:34s} {r:6s} {d}")
npass = sum(1 for _, r, _ in rows if r == "PASS")
print("-" * 84)
print(f"{npass}/{len(rows)} checks passed")
sys.exit(0 if npass == len(rows) else 1)
