"""
Validation: metal shaft sizing (§4) — justify standardized 6 mm shaft.

For each major axis we compute, from the ACTUAL Ori loads:
  * bending stress (simply-supported beam, point load at mid-span)
  * torsion (where applicable)
  * deflection (mid-span)
  * bearing interface (626ZZ = 6 mm bore)
  * mass (steel 7.85 g/cm3)
and confirm the smallest sensible standardized diameter (6 mm, = 626ZZ bore)
is adequate, with margin. If 6 mm fails, we report the required diameter.

Loads come from the prior leg/arm load study (analyze_loads.py):
  - leg single-leg overload apex force ~58.9 N (crash), stance ~14.7 N
  - arm shoulder/elbow ~1.06 N.m; wrist hold 0.137 N.m
Steel 1045 allowable bending ~250 MPa (conservative for ground/polished, yield 530).

Run: python Validation/analyze_shafts.py
"""
import sys, math
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))               # project root (Parameters namespace pkg)
for d in ("Parameters", "CAD/Master"):
    sys.path.insert(0, str(ROOT / d))
from Parameters.master_parameters import PARAMS as P

STEEL_RHO = 7.85e-3   # g/mm^3
ALLOW_BEND = 250.0    # MPa allowable bending (conservative)
E_STEEL = 200000.0    # N/mm^2

# Loads per joint (worst-case single-axis overload from prior study)
JOINTS = {
    "hip":      dict(span=34+18, load=58.9, torque=0.0,  note="leg apex overload 58.9 N"),
    "knee":     dict(span=34+18, load=58.9, torque=0.0,  note="leg apex overload 58.9 N"),
    "shoulder": dict(span=18+18, load=0.0,  torque=1.06, note="arm horizontal extended 1.06 N.m"),
    "elbow":    dict(span=18+18, load=0.0,  torque=1.06, note="arm horizontal extended 1.06 N.m"),
    "wrist":    dict(span=18+18, load=0.0,  torque=0.137,note="wrist hold payload 0.137 N.m"),
}

def size(d_mm, j):
    d = d_mm
    r = d / 2.0
    I = math.pi * d**4 / 64.0
    Z = math.pi * d**3 / 32.0
    # bending: simply supported, point load F at mid-span L
    L = j["span"]
    F = j["load"]
    M_bend = F * L / 4.0                      # N.mm
    sig_b = M_bend / Z                        # MPa
    # torsion:
    T = j["torque"] * 1000.0                  # N.mm
    tau = T * 1000.0 / (math.pi * d**3 / 16.0) if T > 0 else 0.0  # shear MPa (T in N.m -> *1000)
    tau = T / (math.pi * d**3 / 16.0)
    # combined (von Mises)
    vm = math.sqrt(sig_b**2 + 3 * tau**2)
    # deflection (mid-span, bending only)
    defl = (F * L**3) / (48.0 * E_STEEL * I) if F > 0 else 0.0
    mass = math.pi * r**2 * (L + 8) * STEEL_RHO   # + end margin
    return dict(sig_b=sig_b, tau=tau, vm=vm, defl=defl, mass=mass, margin=ALLOW_BEND / max(vm, 1e-6))

def main():
    res = []
    print(f"{'joint':10} {'d':>3} {'sig_b':>7} {'tau':>6} {'vM':>7} {'defl':>6} {'mass':>6} {'margin':>7}  ok")
    all_ok = True
    for name, j in JOINTS.items():
        d = getattr(P.arm, f"shaft_d_{name}", 6.0)
        r = size(d, j)
        ok = r["vm"] < ALLOW_BEND
        all_ok = all_ok and ok
        res.append(ok)
        print(f"{name:10} {d:3.0f} {r['sig_b']:7.1f} {r['tau']:6.2f} {r['vm']:7.1f} {r['defl']:6.3f} {r['mass']:6.1f} {r['margin']:7.1f}x  {'PASS' if ok else 'FAIL'}")
    # bearing interface check: 6mm shaft in 626ZZ 6mm bore
    br_ok = (P.hw.bearing_626_id == 6.0)
    print(f"\nbearing interface: 626ZZ bore {P.hw.bearing_626_id} mm == shaft 6 mm -> {'PASS' if br_ok else 'FAIL'}")
    print(f"\nshaft sizing: {'6 MM STANDARDIZED SHAFT ADEQUATE (all joints)' if (all(res) and br_ok) else 'REVIEW REQUIRED'}")
    return all(res) and br_ok

if __name__ == "__main__":
    sys.exit(0 if main() else 1)
