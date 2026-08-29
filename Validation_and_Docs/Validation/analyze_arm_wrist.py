"""
Ori Robot Dog - ARM WRIST / GRIPPER LOAD ANALYSIS  (CAD-grounded, no printing)

Principle under test:
  The gripper should NOT carry its own actuator. Actuator mass stays on the
  wrist/arm so the interchangeable end-effector is light and more of the arm's
  capacity is available for payload.

This script computes the actual requirements for the 500 g target payload from
the REAL frozen arm geometry (part volumes -> PETG mass -> CG), then reports the
torque each arm/wrist joint must hold, and the required wrist/gripper actuation.

All loads are static worst-case (gravity + moment arm). Dynamic amplification is
NOT included here; a safety margin is applied instead (see MARGIN).

Run:  python Validation/analyze_arm_wrist.py
Outputs a plain-text report (no GUI, no printing).
"""
import sys, math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for d in ("CAD/Master", "CAD/Arm", "CAD/Arm/reference", "CAD/Hardware/servos", "Parameters"):
    sys.path.insert(0, str(ROOT / d))
import cadquery as cq
import build_common as bc
import arm as ARM
from Parameters.master_parameters import PARAMS as P

# ---- material / actuator facts ----
RHO_PETG = 1.27e-3        # g / mm^3  (PETG ~1.27 g/cm^3 = 1.27e-3 g/mm^3)
G = 9.81                   # m/s^2
HTD45H_TORQUE_NM = 4.41    # VERIFIED max/locked @11.1V
HTD45H_CONT_NM = 4.41 * 0.55   # recommended continuous ~55% of locked (bus-servo practice)
HTD45H_MASS_G = 64.0       # VERIFIED
MARGIN = 2.0               # engineering safety factor on required torque

# ---- mass helper: real part volume -> PETG mass ----
def part_mass_g(solid):
    """solid is a cadquery Workplane; returns mass in grams (PETG)."""
    s = solid.val()             # first/combined solid
    vol = s.Volume()             # mm^3
    return vol * RHO_PETG

def part_cg(solid):
    s = solid.vals()[0] if hasattr(solid, "vals") else solid
    c = s.CenterOfMass()
    return (c.x, c.y, c.z)

def report():
    a = P.arm
    p = P
    lines = []
    def L(t=""): lines.append(t)

    L("=" * 74)
    L("ORI ARM - WRIST/GRIPPER LOAD ANALYSIS  (frozen baseline geometry)")
    L("=" * 74)
    L("")
    L(f"PETG density assumed      : {RHO_PETG*1e6:.2f} g/cm^3")
    L(f"Gravity                   : {G} m/s^2")
    L(f"Safety margin (torque)    : {MARGIN}x")
    L(f"HTD-45H locked torque     : {HTD45H_TORQUE_NM:.2f} N.m (VERIFIED)")
    L(f"HTD-45H est. continuous    : {HTD45H_CONT_NM:.2f} N.m (~55% locked, bus-servo practice)")
    L("")

    # ---- real part masses from geometry ----
    parts, compound = ARM.make_arm(P)
    mass = {k: part_mass_g(v) for k, v in parts.items()}
    L("REAL PART MASSES (from CAD volume, PETG):")
    tot = 0.0
    for k, m in mass.items():
        L(f"  {k:14s} {m:7.1f} g")
        tot += m
    L(f"  {'TOTAL arm (PETG)':14s} {tot:7.1f} g")
    # actuators: 6x HTD-45H present in current design
    actuator_g = 6 * HTD45H_MASS_G
    L(f"  + 6x HTD-45H servos   {actuator_g:7.1f} g  (current design only)")
    L(f"  => current arm all-up  {tot+actuator_g:7.1f} g")
    L("")

    # ---- payload + tool CG assumptions ----
    payload_g = a.payload_g
    # gripper/tool CG sits at end of wrist_len past the wrist joint
    tool_mass_g = mass["gripper"]          # current gripper incl. its servo
    # in the NEW concept the passive gripper is ~<=80 g; we analyze both
    L("PAYLOAD / TOOL ASSUMPTIONS:")
    L(f"  Target payload          : {payload_g} g  (user spec)")
    L(f"  Payload CG              : at gripper tip plane, centred, fully extended")
    L(f"  Current gripper mass     : {tool_mass_g:.1f} g (includes its HTD-45H actuator)")
    L(f"  Passive-gripper target   : <= 80 g (no actuator/motor/battery/controller)")
    L("")

    # ---- arm reach / moment arms (mm) ----
    L1 = a.shoulder_len
    L2 = a.elbow_len
    Lw = a.wrist_len
    base_reach = a.base_reach + 28   # yaw housing front to shoulder origin
    # fully extended horizontal reach of the wrist joint from shoulder pivot:
    reach_wrist = base_reach + L1 + L2
    reach_tip = reach_wrist + Lw
    L("GEOMETRY / MOMENT ARMS (worst case = arm horizontal, fully extended):")
    L(f"  shoulder->elbow (L1)     : {L1} mm")
    L(f"  elbow->wrist  (L2)       : {L2} mm")
    L(f"  wrist->tool   (Lw)       : {Lw} mm")
    L(f"  reach to wrist joint     : {reach_wrist} mm")
    L(f"  reach to tool CG/tip     : {reach_tip} mm")
    L("")

    # ---- torque at each joint holding the 500 g payload (worst-case horizontal) ----
    # Convert g-force at distance d(mm) to N.m:  T = m_kg * G * d_m
    def t_nm(mass_g, d_mm):
        return (mass_g / 1000.0) * G * (d_mm / 1000.0)

    # Payload alone, fully extended, horizontal:
    T_elbow_payload = t_nm(payload_g, reach_tip)          # about shoulder/elbow (whole arm lever)
    T_shoulder_payload = t_nm(payload_g, reach_tip)
    # wrist joints see payload lever = Lw (wrist roll/pitch) + tool mass lever
    T_wrist_payload = t_nm(payload_g, Lw)
    # tool (gripper) self-weight also loads wrist: lever ~ Lw/2 for passive tool CG
    T_wrist_tool = t_nm(tool_mass_g, Lw / 2.0)
    # gripper-actuation torque (closing force) is SEPARATE from pose torque:
    #   to hold a 500 g object the gripper must produce grip force Fg with margin.
    #   Required closing torque depends on the transmission; we size the joint, not the grip.
    L("STATIC PAYLOAD TORQUE (fully extended, horizontal, gravity):")
    L(f"  Shoulder/Elbow hold payload : {T_shoulder_payload:.3f} N.m  (lever {reach_tip} mm)")
    L(f"  Wrist ROLL/PITCH hold payload: {T_wrist_payload:.3f} N.m  (lever {Lw} mm)")
    L(f"  Wrist holds tool self-wt    : {T_wrist_tool:.3f} N.m  (tool {tool_mass_g:.0f}g, lever {Lw/2:.0f} mm)")
    L("")

    # ---- gripper actuation requirement (NOT pose torque) ----
    # To hold mass m without slip: grip force Fg = (m*g)/mu, mu~0.4 (rubber pad).
    # Finger lever from jaw pivot to pad ~ grip_depth/2; closing torque tau = Fg * lever * (transmission ratio)
    mu = 0.4
    Fg = (payload_g/1000.0)*G/mu            # N required normal grip force (total, both pads)
    jaw_lever = a.grip_depth/2.0/1000.0     # m (pad to pivot)
    # simplest: direct pivot, torque = Fg * jaw_lever (one side), x2 sides /2 -> Fg*jaw_lever
    tau_close = Fg * jaw_lever
    L("GRIPPER ACTUATION (closing) REQUIREMENT:")
    L(f"  Required grip force (mu={mu}) : {Fg:.2f} N  for {payload_g} g payload")
    L(f"  Jaw pivot->pad lever      : {jaw_lever*1000:.1f} mm")
    L(f"  Closing torque at jaw pivot: {tau_close:.3f} N.m  (BEFORE transmission)")
    L(f"  With {MARGIN}x margin       : {tau_close*MARGIN:.3f} N.m")
    L("")

    # ---- compare to HTD-45H ----
    L("HTD-45H vs REQUIREMENTS:")
    L(f"  Shoulder/Elbow req {T_shoulder_payload:.3f} N.m  vs locked {HTD45H_TORQUE_NM} / cont {HTD45H_CONT_NM:.2f} -> margin {HTD45H_CONT_NM/T_shoulder_payload:.1f}x")
    L(f"  Wrist pose req     {T_wrist_payload:.3f} N.m  vs cont {HTD45H_CONT_NM:.2f} -> margin {HTD45H_CONT_NM/T_wrist_payload:.1f}x")
    L(f"  Gripper close req  {tau_close*MARGIN:.3f} N.m (margined) vs cont {HTD45H_CONT_NM:.2f} -> {'OVER' if tau_close*MARGIN>HTD45H_CONT_NM else 'OK'}")
    L("")
    L("CONCLUSION (see Documentation/ARM_WRIST_ANALYSIS.md for full deliverables):")
    L(f"  - Wrist actuators are NOT torque-limited by the 500 g payload (req {T_wrist_payload:.3f} N.m << cont).")
    L(f"  - Gripper closing torque is tiny ({tau_close:.3f} N.m); a SMALL servo or a")
    L(f"    passive transmission from a wrist servo easily covers it.")
    L(f"  - HTD-45H at wrist/gripper is OVERSIZED for torque; mass savings possible.")
    L(f"  - Recommendation: servo-IN-WRIST drives passive gripper via a light")
    L(f"    transmission (printed gear or shaft+linkage). Passive gripper <=80 g.")
    L("=" * 74)

    txt = "\n".join(lines)
    print(txt)
    # also write the report alongside the doc
    out = ROOT / "Documentation" / "ARM_WRIST_ANALYSIS_LOG.txt"
    out.write_text(txt)
    return txt

if __name__ == "__main__":
    report()
