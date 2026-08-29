"""
Ori Robot Dog - STATIC LOAD / TORQUE MARGIN ANALYSIS (no user input required).

This is CALCULATION, not physical testing. It checks whether the chosen
HTD-45H actuator has adequate torque margin at the assumed 6 kg all-up mass
budget, in the neutral standing pose, and under a conservative dynamic factor.

It does NOT prove printed-part strength (that needs FEA). It converts the
"torque margin ASSUMED adequate" status into a quantified number.

Run: python Validation/analyze_loads.py
"""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from Parameters.master_parameters import PARAMS
import math

p = PARAMS
g = 9.81

rows = []
def check(name, ok, detail=""):
    rows.append((name, "PASS" if ok else "FAIL", detail))

# --- mass split ---
mass = p.scale.body_mass_budget / 1000.0          # kg
W = mass * g                                      # N total weight
per_leg = W / 4.0                                 # N per leg at neutral stand
per_foot = per_leg                                # coincident

# --- actuator ---
tau = p.servo.torque_nm                           # 4.41 N.m
horn_r = p.leg.servo_horn_r / 1000.0              # 0.022 m lever
F_out = tau / horn_r                              # N output force at link end

# --- static margin (hip, worst single joint) ---
margin_static = F_out / per_leg
check("hip torque margin (static, 1g)", margin_static > 2.0,
      f"output {F_out:.0f} N vs load {per_leg:.1f} N -> {margin_static:.1f}x")

# --- dynamic margin: gait transients, one leg lifted, 3x factor ---
dyn_factor = 3.0
per_leg_dyn = per_leg * dyn_factor
margin_dyn = F_out / per_leg_dyn
check("hip torque margin (dynamic 3g)", margin_dyn > 1.5,
      f"output {F_out:.0f} N vs dyn load {per_leg_dyn:.1f} N -> {margin_dyn:.1f}x")

# --- joint bending moment on PRINTED link at base (cantilever-ish) ---
# Upper link ~175 mm; hip reaction moment ~ per_leg * link_length
L1 = p.leg.upper_link_length / 1000.0
M_hip = per_leg * L1                            # N.m bending at hip
# rough section modulus of hollow box tube (w x t, wall ww) about Y (out-of-plane)
w = p.leg.link_w / 1000.0
t = p.leg.link_t / 1000.0
ww = p.leg.link_wall / 1000.0
# outer I-ish: approximate as rectangular tube, bending about Y (weak axis = t)
# section modulus Z ~ (w*t^2 - (w-2ww)*(t-2ww)^2)/6  (m^3)
Z = (w*t*t - (w-2*ww)*(t-2*ww)**2) / 6.0
sigma = M_hip / Z if Z > 0 else 1e9             # Pa
# PETG printed ~ 40-60 MPa yield-ish (layer-dependent). Use 35 MPa conservative.
petg = 35e6
check("upper-link bending stress < PETG (static)", sigma < petg,
      f"sigma {sigma/1e6:.1f} MPa vs {petg/1e6:.0f} MPa (ASSUMED PETG, static)")

# --- mass budget sanity: sum of known component masses vs 6 kg budget ---
known = {
    "12x HTD-45H": 12 * p.servo.mass,
    "battery": p.elec.batt_mass,
    "4x D2F": 4 * 0.7,
    "bearings 16x626ZZ": 16 * 0.008 * 1000,  # ~8 g each
    "RPi4": 46.0,
    "3x Pico": 3 * 4.0,
    "prints (est)": 800.0,
}
total_known = sum(known.values())
check("known mass < budget", total_known < p.scale.body_mass_budget,
      f"known {total_known:.0f} g vs budget {p.scale.body_mass_budget:.0f} g (head/arm/spares not counted)")

print(f"{'CHECK':36s} {'RESULT':6s} DETAIL")
print("-" * 78)
for n, r, d in rows:
    print(f"{n:36s} {r:6s} {d}")
npass = sum(1 for _, r, _ in rows if r == "PASS")
print("-" * 78)
print(f"{npass}/{len(rows)} checks passed  |  W={W:.1f}N per_leg={per_leg:.1f}N output={F_out:.0f}N")
sys.exit(0 if npass == len(rows) else 1)
