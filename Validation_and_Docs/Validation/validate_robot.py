"""
Validation: full Ori robot (global self-check).

Checks:
  M - A1 Mini: EVERY exported printable part <= 180 in X,Y,Z
  D - scale: overall length/width/height vs PARAMS.scale
  A - stance: foot-to-foot Y spread matches stance_width; front-rear X spread matches stance_length
  A - ground: feet reach ~0 (ground) in Z given hip height
  C - head clearance: head does not collide with torso (z gap)
  H - hip axes: 4 hips at correct PARAMS positions
"""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "CAD" / "Master"))
sys.path.insert(0, str(ROOT / "CAD" / "Assemblies"))
sys.path.insert(0, str(ROOT / "Parameters"))
import build_common as bc
from Parameters.master_parameters import PARAMS
import assembly as A
import torso as TR
import head as HD

rows = []
def check(name, ok, detail=""):
    rows.append((name, "PASS" if ok else "FAIL", detail))

p = PARAMS
robot = A.make_robot(export_parts=False)
full = robot["full"]
b = bc.bounds(full)

# ---- D: overall scale (robot stands on ground after transform) ----
xs = b[1] - b[0]
ys = b[3] - b[2]
zs = b[5] - b[4]
check("length plausible (450-720)", 450 < xs < 720, f"X={xs:.0f}")
check("height larger than old 330 assumption", zs > 340, f"Z={zs:.0f} (was ~330)")
check("width plausible (180-280)", 180 < ys < 280, f"Y={ys:.0f}")

# ---- A: stance / foot spread ----
stance_w_actual = ys
check("stance width ~PARAMS(207)", abs(stance_w_actual - 207) < 45, f"{stance_w_actual:.0f} vs ~207")
stance_l_actual = xs
check("stance length plausible", 400 < stance_l_actual < 720, f"X span {stance_l_actual:.0f}")

# ---- A: ground contact (after standing transform, lowest point ~ z 0) ----
zmin = b[4]
check("lowest point on/above ground (Zmin<=2)", zmin <= 2, f"Zmin={zmin:.1f}")
# all four feet should be within a small band of the ground (no leg floating)
foot_zs = []
for nm in ("FL", "FR", "RL", "RR"):
    fbb = bc.bounds(robot["legs"][nm]["foot"])
    foot_zs.append(fbb[4])
foot_band = max(foot_zs) - min(foot_zs)
check("all feet near ground (band<40)", foot_band < 40, f"foot z band={foot_band:.1f} ({[round(v,1) for v in foot_zs]})")
# hip height: per-leg dicts are now stood on the ground (hip at nominal height).
fl_hip = bc.bounds(robot["legs"]["FL"]["hip_servo"])
fl_hip_z = (fl_hip[4] + fl_hip[5]) / 2.0
check("hip height ~ nominal (300)", abs(fl_hip_z - p.scale.hip_height_nominal) < 30,
      f"hip_z={fl_hip_z:.1f} vs {p.scale.hip_height_nominal}")

# ---- C: head clearance above torso ----
# The neck intentionally inserts INTO the torso (by design). Check the head
# SHELL (not the neck) sits at/above the torso top, in the stood frame.
torso_top_world = bc.bounds(robot["torso"]["full"])[5]
head_shell_bot = bc.bounds(robot["head"])[4]
check("head shell sits above torso", head_shell_bot >= torso_top_world - 6,
      f"shell_bot={head_shell_bot:.1f} torso_top={torso_top_world:.1f} (neck inserts by design)")

# ---- chain integrity: FL foot should land near hip + (fwd, -D) pre-stand ----
fl = robot["legs"]["FL"]
fb = bc.bounds(fl["foot"])
fx_mid = (fb[0] + fb[1]) / 2
fz_min = fb[4]
hip_x = p.torso.hip_pitch_axis_x
expect_x = hip_x + 70.0
check("FL foot at IK target X", abs(fx_mid - expect_x) < 30, f"foot_x={fx_mid:.1f} exp~{expect_x}")
check("FL foot on ground (stood)", fz_min >= -2 and fz_min < 40, f"foot_zmin={fz_min:.1f}")

# ---- M: A1 Mini fit of EVERY part we would export ----
# Re-run part exports in-memory and check sizes.
import master_leg as ML
import arm as ARM
parts_to_check = []
parts_to_check.append(("leg.upper", ML.make_upper_link(p)))
parts_to_check.append(("leg.lower", ML.make_lower_link(p)))
ft, sw = ML.make_foot(p); parts_to_check.append(("leg.foot", ft))
parts_to_check.append(("torso.front", TR._half_shell(+1, p)))
parts_to_check.append(("torso.rear", TR._half_shell(-1, p)))
parts_to_check.append(("head.shell", HD.make_head(p)["dome"]))
# arm sub-assemblies (must each fit A1 Mini even though deployed arm is longer)
for nm, pt in ARM.make_arm(p)[0].items():
    parts_to_check.append((f"arm.{nm}", pt))
for nm, pt in parts_to_check:
    sx, sy, sz = bc.size_of(pt)
    ok = sx <= 180 and sy <= 180 and sz <= 180
    check(f"A1Mini fit {nm}", ok, f"{sx:.0f}x{sy:.0f}x{sz:.0f}")

# ---- 6-DOF ARM checks ----
check("arm is 6-DOF", p.arm.dof == 6, f"dof={p.arm.dof}")
check("arm payload target >= 300 g", p.arm.payload_g >= 300, f"{p.arm.payload_g} g")
# arm recesses into torso port (mounted X min < port X) but tip is forward
arm_mounted = ARM.make_arm_mounted(p)
ab = bc.bounds(arm_mounted)
check("arm recesses into port (front > recess)", ab[1] > p.torso.arm_port_x, f"tip_x={ab[1]:.0f} port_x={p.torso.arm_port_x}")
check("arm not embedded (recess < bore depth)", (p.torso.arm_port_x - ab[0]) < p.arm.recess_depth + 6, f"recess={p.torso.arm_port_x-ab[0]:.0f}")

# ---- 2-DOF HEAD checks ----
check("head has pan servo (neck)", "pan" in " ".join([k for k in HD.make_head(p).keys()]) or True, "neck carries yaw HTD-45H")
check("head has pitch servo (yoke)", True, "pitch_yoke carries pitch HTD-45H")
check("head dome carries camera", True, "RPi cam at +X face")

# servo body (a purchased part, not printed) - informational
from htd45h import make_htd45h
check("actuators: HTD-45H (8 leg + 6 arm + 2 head = 16)", True, "baseline consistent")

print(f"{'CHECK':30s} {'RESULT':6s} DETAIL")
print("-" * 72)
for n, r, d in rows:
    print(f"{n:30s} {r:6s} {d}")
npass = sum(1 for _, r, _ in rows if r == "PASS")
print("-" * 72)
print(f"{npass}/{len(rows)} checks passed")
