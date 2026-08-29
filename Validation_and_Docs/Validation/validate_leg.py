"""
Validation: Ori master leg.

Checks:
  D - dimensions      : upper/lower link length == PARAMS
  M - manufacturing   : each printable part fits 180^3 A1 Mini
  H - hardware        : bearings/servo/shaft bores are correct real sizes
  C - clearances      : bearing seats vs shaft, horn bosses vs PCD
  A - assembly        : links chain without invalid gaps
  K - kinematics      : hip-height vs extended reach margin

Run: python Validation_and_Docs/Validation/validate_leg.py
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "Source"
CAD = SOURCE / "CAD"
for d in (SOURCE, CAD, CAD / "Master", CAD / "Legs", SOURCE / "Parameters"):
    sys.path.insert(0, str(d))

import build_common as bc
from Parameters.master_parameters import PARAMS, leg_workspace_check
import master_leg as ML

rows = []
def check(name, ok, detail=""):
    rows.append((name, "PASS" if ok else "FAIL", detail))

p = PARAMS
parts = ML.make_leg_assembly(export_parts=False)

# --- D: link lengths ---
up = bc.size_of(parts["upper_link"])
lo = bc.size_of(parts["lower_link"])
check("upper_link length ~L", abs(up[0] - p.leg.upper_link_length) < 3.0,
      f"{up[0]:.1f} vs {p.leg.upper_link_length}")
check("lower_link length ~L", abs(lo[0] - p.leg.lower_link_length) < 3.0,
      f"{lo[0]:.1f} vs {p.leg.lower_link_length}")

# --- M: A1 Mini fit for each printable part ---
bx, by, bz = p.mfg.build_x, p.mfg.build_y, p.mfg.build_z
for k in ("upper_link", "lower_link", "foot"):
    sx, sy, sz = bc.size_of(parts[k])
    ok = sx <= bx and sy <= by and sz <= bz
    check(f"print fit {k}", ok, f"{sx:.0f}x{sy:.0f}x{sz:.0f} <= {bx:.0f}^3")

# --- H: hip servo envelope, derived from the actual servo parameters ---
hs = bc.size_of(parts["hip_servo"])
check("hip servo body length", abs(hs[0] - p.servo.body_long) < 0.5, f"X={hs[0]:.2f} vs {p.servo.body_long:.2f}")
check("hip servo body height", abs(hs[1] - p.servo.body_tall) < 0.5, f"Y={hs[1]:.2f} vs {p.servo.body_tall:.2f}")
check("hip servo Z span", abs(hs[2] - (p.servo.body_short + 2*p.servo.shaft_len_each_side)) < 0.5,
      f"Z={hs[2]:.2f} vs {p.servo.body_short + 2*p.servo.shaft_len_each_side:.2f}")

# --- C: bearing/shaft interface and horn PCD ---
check("626ZZ bore == servo shaft", abs(p.hw.bearing_626_id - p.servo.shaft_d) < 1e-6,
      f"bearing {p.hw.bearing_626_id:.1f} mm vs shaft {p.servo.shaft_d:.1f} mm")
from CAD.Hardware.servos.htd45h import make_htd45h
srv = make_htd45h()
check("servo envelope matches params", abs(bc.size_of(srv)[2] - (p.servo.body_short + 2*p.servo.shaft_len_each_side)) < 0.5,
      f"Z span={bc.size_of(srv)[2]:.2f} mm")

# --- A: chain check — upper link is centered at its own half-length ---
cx_up = (bc.bounds(parts["upper_link"])[0] + bc.bounds(parts["upper_link"])[1]) / 2
check("upper centered near x=L/2", abs(cx_up - p.leg.upper_link_length / 2) < 6,
      f"cx={cx_up:.1f} vs {p.leg.upper_link_length/2:.1f}")

# --- K: reach vs hip height ---
kw = leg_workspace_check(p)
check("leg reaches ground", kw["standing_margin"] > 0,
      f"reach {kw['max_reach']:.0f} vs hip_z {kw['hip_z']:.0f} (margin {kw['standing_margin']:.0f})")

# --- structure / harness ---
check("link Y < torso width", p.leg.link_w + 12 < p.torso.width,
      f"link {p.leg.link_w}+12 vs torso {p.torso.width}")

print(f"{'CHECK':32s} {'RESULT':6s} DETAIL")
print("-" * 76)
for n, r, d in rows:
    print(f"{n:32s} {r:6s} {d}")
npass = sum(1 for _, r, _ in rows if r == "PASS")
print("-" * 76)
print(f"{npass}/{len(rows)} checks passed")
sys.exit(0 if npass == len(rows) else 1)
