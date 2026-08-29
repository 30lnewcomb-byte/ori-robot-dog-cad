"""
Validation: standardized 6 mm metal shaft sizing (§4).

Calculations only; bearing interface and loads are parameterized where practical.
Run: python Validation_and_Docs/Validation/analyze_shafts.py
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "Source"
for d in (SOURCE, SOURCE / "CAD", SOURCE / "Parameters"):
    sys.path.insert(0, str(d))
from Parameters.master_parameters import PARAMS as P

STEEL_RHO = 7.85e-3   # g/mm^3
ALLOW_BEND = 250.0    # MPa conservative allowable
E_STEEL = 200000.0    # N/mm^2

JOINTS = {
    "hip":      dict(span=34 + 18, load=58.9, torque=0.0, note="leg apex overload 58.9 N"),
    "knee":     dict(span=34 + 18, load=58.9, torque=0.0, note="leg apex overload 58.9 N"),
    "shoulder": dict(span=18 + 18, load=0.0, torque=1.06, note="arm horizontal extended 1.06 N.m"),
    "elbow":    dict(span=18 + 18, load=0.0, torque=1.06, note="arm horizontal extended 1.06 N.m"),
    "wrist":    dict(span=18 + 18, load=0.0, torque=0.137, note="wrist hold payload 0.137 N.m"),
}

def size(d_mm, j):
    d = float(d_mm)
    I = math.pi * d**4 / 64.0
    Z = math.pi * d**3 / 32.0
    L = j["span"]
    F = j["load"]
    M_bend = F * L / 4.0
    sig_b = M_bend / Z
    T = j["torque"] * 1000.0
    tau = T / (math.pi * d**3 / 16.0)
    vm = math.sqrt(sig_b**2 + 3 * tau**2)
    defl = (F * L**3) / (48.0 * E_STEEL * I) if F > 0 else 0.0
    mass = math.pi * (d / 2.0)**2 * (L + 8.0) * STEEL_RHO
    return dict(sig_b=sig_b, tau=tau, vm=vm, defl=defl, mass=mass, margin=ALLOW_BEND / max(vm, 1e-6))

def main():
    all_ok = True
    print(f"{'joint':10} {'d':>5} {'sig_b':>7} {'tau':>7} {'vM':>7} {'defl':>7} {'mass':>7} {'margin':>7}  result")
    for name, j in JOINTS.items():
        d = getattr(P.arm, f"shaft_d_{name}", P.servo.shaft_d)
        r = size(d, j)
        ok = r["vm"] < ALLOW_BEND
        all_ok = all_ok and ok
        print(f"{name:10} {d:5.1f} {r['sig_b']:7.1f} {r['tau']:7.2f} {r['vm']:7.1f} {r['defl']:7.3f} {r['mass']:7.1f} {r['margin']:7.1f}x  {'PASS' if ok else 'FAIL'}")
    shaft_d = P.servo.shaft_d
    br_ok = abs(P.hw.bearing_626_id - shaft_d) < 1e-6
    print(f"\nbearing interface: 626ZZ bore {P.hw.bearing_626_id:.1f} mm == servo shaft {shaft_d:.1f} mm -> {'PASS' if br_ok else 'FAIL'}")
    print(f"\nshaft sizing: {'STANDARD SHAFT ADEQUATE' if (all_ok and br_ok) else 'REVIEW REQUIRED'}")
    return all_ok and br_ok

if __name__ == "__main__":
    sys.exit(0 if main() else 1)
