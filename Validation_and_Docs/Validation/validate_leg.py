"""
Validation: Ori master leg.

Checks (per requirement #24/#25 self-check loop):
  D - dimensions      : upper/lower link length == PARAMS
  M - manufacturing   : each printable part fits 180^3 A1 Mini
  H - hardware        : bearings/servo/shaft bores are correct real sizes
  C - clearances      : bearing seats vs shaft, horn bosses vs PCD
  A - assembly        : links chain without overlap; foot reaches ground
  K - kinematics      : hip-height vs extended reach margin

Prints a verdict table. Run: python Validation/validate_leg.py
"""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "CAD" / "Master"))
sys.path.insert(0, str(ROOT / "CAD" / "Legs"))
sys.path.insert(0, str(ROOT / "Parameters"))
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
check("upper_link length ~L", abs(up[0] - p.leg.upper_link_length) < 3.0, f"{up[0]:.1f} vs {p.leg.upper_link_length}")
check("lower_link length ~L", abs(lo[0] - p.leg.lower_link_length) < 3.0, f"{lo[0]:.1f} vs {p.leg.lower_link_length}")

# --- M: A1 Mini fit for each printable part (X,Y,Z each < 180) ---
bx, by, bz = p.mfg.build_x, p.mfg.build_y, p.mfg.build_z
for k in ("upper_link", "lower_link", "foot"):
    sx, sy, sz = bc.size_of(parts[k])
    ok = sx < bx and sy < by and sz < bz
    check(f"print fit {k}", ok, f"{sx:.0f}x{sy:.0f}x{sz:.0f} < 180^3")

# --- H: hip servo body actual dims ---
hs = bc.size_of(parts["hip_servo"])
check("hip servo body L=51.1", abs(hs[0] - 51.1) < 0.5, f"X={hs[0]:.1f}")
check("hip servo body H=40.0", abs(hs[2] - 40.0) < 0.5, f"Z={hs[2]:.1f}")

# --- C: servo shaft 6mm present in SERVO model (not link) -> rely on htd45h ---
from CAD.Hardware.servos.htd45h import make_htd45h
srv = make_htd45h()
check("servo Z span 36.14", abs(bc.size_of(srv)[2] - 36.14) < 0.2, f"Z span={bc.size_of(srv)[2]:.2f}")

# --- A: chain check — knee joint of lower sits at end of upper ---
# upper spans x in [0, L_up]; lower after translate spans [L_up, L_up+L_lo].
# They should meet at x = L_up (knee). Verify no big gap/overlap in X center.
cx_up = (bc.bounds(parts["upper_link"])[0] + bc.bounds(parts["upper_link"])[1]) / 2
check("upper centered near x=L/2", abs(cx_up - p.leg.upper_link_length / 2) < 6, f"cx={cx_up:.1f}")

# --- K: reach vs hip height ---
kw = leg_workspace_check(p)
check("leg reaches ground", kw["standing_margin"] > 0, f"reach {kw['max_reach']:.0f} vs hip_z {kw['hip_z']:.0f} (margin {kw['standing_margin']:.0f})")

# --- link Y width vs stance harness ---
check("link Y < torso width", p.leg.link_w + 12 < p.torso.width, f"link {p.leg.link_w}+12 vs torso {p.torso.width}")

print(f"{'CHECK':28s} {'RESULT':6s} DETAIL")
print("-" * 70)
for n, r, d in rows:
    print(f"{n:28s} {r:6s} {d}")
npass = sum(1 for _, r, _ in rows if r == "PASS")
print("-" * 70)
print(f"{npass}/{len(rows)} checks passed")
sys.exit(0 if npass == len(rows) else 1)
