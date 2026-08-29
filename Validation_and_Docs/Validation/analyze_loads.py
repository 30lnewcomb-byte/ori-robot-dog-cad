"""
Ori Robot Dog - static load / torque margin analysis.

Calculation only; this does not replace physical testing or FEA.
Run: python Validation_and_Docs/Validation/analyze_loads.py
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "Source"
for d in (SOURCE, SOURCE / "CAD", SOURCE / "Parameters"):
    sys.path.insert(0, str(d))

from Parameters.master_parameters import PARAMS

p = PARAMS
g = 9.81
rows = []

def check(name, ok, detail=""):
    rows.append((name, "PASS" if ok else "FAIL", detail))

mass = p.scale.body_mass_budget / 1000.0
W = mass * g
per_leg = W / 4.0
tau = p.servo.torque_nm
horn_r = p.leg.servo_horn_r / 1000.0
F_out = tau / horn_r

margin_static = F_out / per_leg
check("hip torque margin (static, 1g)", margin_static > 2.0,
      f"output {F_out:.0f} N vs load {per_leg:.1f} N -> {margin_static:.1f}x")

dyn_factor = 3.0
per_leg_dyn = per_leg * dyn_factor
margin_dyn = F_out / per_leg_dyn
check("hip torque margin (dynamic 3g)", margin_dyn > 1.5,
      f"output {F_out:.0f} N vs dyn load {per_leg_dyn:.1f} N -> {margin_dyn:.1f}x")

L1 = p.leg.upper_link_length / 1000.0
M_hip = per_leg * L1
w = p.leg.link_w / 1000.0
t = p.leg.link_t / 1000.0
ww = p.leg.link_wall / 1000.0
Z = (w * t * t - (w - 2 * ww) * (t - 2 * ww) ** 2) / 6.0
sigma = M_hip / Z if Z > 0 else math.inf
petg = 35e6
check("upper-link bending stress < PETG", sigma < petg,
      f"sigma {sigma/1e6:.1f} MPa vs {petg/1e6:.0f} MPa (analysis assumption)")

known = {
    "12x HTD-45H": 12 * p.servo.mass,
    "battery": p.elec.batt_mass,
    "4x D2F": 4 * 0.7,
    "bearings 16x626ZZ": 16 * 8.0,
    "RPi4": 46.0,
    "3x Pico": 3 * 4.0,
    "prints (est)": 800.0,
}
total_known = sum(known.values())
check("known mass < budget", total_known < p.scale.body_mass_budget,
      f"known {total_known:.0f} g vs budget {p.scale.body_mass_budget:.0f} g")

print(f"{'CHECK':36s} {'RESULT':6s} DETAIL")
print("-" * 78)
for n, r, d in rows:
    print(f"{n:36s} {r:6s} {d}")
npass = sum(1 for _, r, _ in rows if r == "PASS")
print("-" * 78)
print(f"{npass}/{len(rows)} checks passed | W={W:.1f}N per_leg={per_leg:.1f}N output={F_out:.0f}N")
sys.exit(0 if npass == len(rows) else 1)
