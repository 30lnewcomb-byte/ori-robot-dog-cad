"""
Ori Robot Dog - FULL ROBOT ASSEMBLY.

Assembles the verified components in a realistic STANDING pose:
  * Torso (front + rear half shells)
  * 4x master leg (FL, FR, RL, RR) via orientation transforms only
  * Head on the torso front-top

Leg placement uses 2-link inverse kinematics so each leg stands on the ground:
  hip at (x=±150, y=±70, z=55), then the upper/lower links bend in the sagittal
  (X-Z) plane to put the foot on the ground at a forward/back stance offset.
  Front legs reach forward (+X), rear legs reach backward (-X).

After placement the whole robot is translated so the lowest foot sits at z=0
(ground). This makes the assembly a real standing quadruped, not a pose-less blob.

Global validation prints reach, stance, foot spread, head clearance.
"""
import sys, math
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Master"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Legs"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Torso"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Head"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Arm"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from build_common import cq, PARAMS, cyl, export, bounds
import master_leg as ML
import torso as TR
import head as HD
import arm as ARM


def leg_ik(fx, D, a, b):
    """2-link IK in sagittal plane (x forward, z up; down is -z).
    Returns (theta_hip, psi) in DEGREES:
      theta_hip = upper-link angle from straight-down, forward positive
      psi       = lower-link angle relative to upper-link.
    Foot target: forward offset fx, down distance D. Link lengths a,b."""
    r = math.hypot(fx, D)
    r = min(r, a + b - 1e-3)
    r = max(r, abs(a - b) + 1e-3)
    phi = math.atan2(fx, D)
    delta = math.acos((a * a + r * r - b * b) / (2 * a * r))
    theta_hip = phi + delta
    kx = a * math.sin(theta_hip)
    kz = -a * math.cos(theta_hip)
    lx = fx - kx
    lz = -D - kz
    theta_low = math.atan2(lx, -lz)
    psi = theta_low - theta_hip
    return math.degrees(theta_hip), math.degrees(psi)


def place_leg(hip_x, hip_y, fwd_offset, mirror_y=False, p=PARAMS):
    """Place one leg at its hip in a standing pose.

    The leg is built ALREADY in a valid standing IK pose (hip at origin) by
    master_leg.make_leg_standing(). Here we only:
      * yaw about Z so the leg splays outboard (+Y for left, -Y for right) and
        aims forward/back,
      * mirror across YZ for the right side,
      * translate the hip to its world anchor (hip_x, hip_y, hip_z).
    Because the pose is baked in, the foot lands at the correct ground point.
    """
    leg = ML.make_leg_standing(fwd_offset, p)
    # yaw: left legs splay +Y (rotate +Z), right legs handled by mirror.
    # Build aiming forward/back in X already; yaw about Z by splay angle.
    splay = 12.0  # degrees of outboard splay
    yaw = splay if not mirror_y else -splay
    out = {}
    for k, part in leg.items():
        if k.startswith("_") or k in ("knee", "ankle"):
            continue
        pp = part.rotate((0, 0, 0), (0, 0, 1), yaw)
        if mirror_y:
            pp = pp.mirror("YZ")
        out[k] = pp.translate((hip_x, hip_y, p.torso.hip_axis_z))
    out["merged"] = out["upper_link"].union(out["lower_link"]).union(out["foot"]).union(out["foot_switch"]).union(out["hip_servo"]).union(out["knee_servo"])
    return out


def make_robot(p=PARAMS, export_parts=True, merged=True):
    tor = TR.make_torso(p, export_parts=False)

    hips = [
        ("FL", +p.torso.hip_pitch_axis_x, +p.torso.hip_axis_y, +70.0, False),
        ("FR", +p.torso.hip_pitch_axis_x, -p.torso.hip_axis_y, +70.0, True),
        ("RL", -p.torso.hip_pitch_axis_x, +p.torso.hip_axis_y, -70.0, False),
        ("RR", -p.torso.hip_pitch_axis_x, -p.torso.hip_axis_y, -70.0, True),
    ]
    assembly = tor["full"]
    leg_solids = {}
    for name, hx, hy, fwd, my in hips:
        placed = place_leg(hx, hy, fwd, my, p)
        leg_solids[name] = placed
        assembly = assembly.union(placed["merged"])

    hd = HD.make_head(p, export_parts=False)
    head_part = hd["full"].translate((p.torso.length / 2 - 10, 0, p.torso.height / 2 + p.head.height / 2 - 6))
    assembly = assembly.union(head_part)

    # --- FRONT ARM: 6-DOF manipulator recessed into the torso front port ---
    arm_part = ARM.make_arm_mounted(p)
    assembly = assembly.union(arm_part)

    # --- stand the robot on the ground (lowest foot at z = 0) ---
    bb = bounds(assembly)
    z_min = bb[4]
    stand = (0, 0, -z_min)
    assembly = assembly.translate(stand)
    # also stand the per-leg dicts and head so returned parts are consistent
    for name in leg_solids:
        for k in list(leg_solids[name].keys()):
            if isinstance(leg_solids[name][k], tuple):
                continue
            leg_solids[name][k] = leg_solids[name][k].translate(stand)
    head_part = head_part.translate(stand)

    if export_parts:
        export(tor["front"], "ASSEMBLY_torso_front", subdir="CAD/Exports/assembly")
        export(tor["rear"], "ASSEMBLY_torso_rear", subdir="CAD/Exports/assembly")
        keys = {"upper_link": "upper", "lower_link": "lower", "foot": "foot"}
        for name, _, _, _, _ in hips:
            for src, dst in keys.items():
                export(leg_solids[name][src], f"ASSEMBLY_{name}_{dst}", subdir="CAD/Exports/assembly")
        export(head_part, "ASSEMBLY_head", subdir="CAD/Exports/assembly")
        export(arm_part, "ASSEMBLY_arm", subdir="CAD/Exports/assembly")
    if merged:
        export(assembly, "Ori_Robot_full", subdir="CAD/Exports/assembly")
    return {"torso": tor, "legs": leg_solids, "head": head_part, "arm": arm_part, "full": assembly}


if __name__ == "__main__":
    import build_common as bc
    robot = make_robot()
    b = bc.bounds(robot["full"])
    print("FULL ROBOT size (X,Y,Z) =", [round(r, 1) for r in bc.size_of(robot["full"])])
    print("X span (length):", round(b[1] - b[0], 1))
    print("Y span (width, foot-to-foot):", round(b[3] - b[2], 1))
    print("Z span (height, ground->top):", round(b[5] - b[4], 1))
    # hip world height and FL foot
    tor_zmin = bc.bounds(robot["torso"]["full"])[4]
    print("Hip height above ground:", round(tor_zmin + PARAMS.torso.hip_axis_z, 1))
    fb = bc.bounds(robot["legs"]["FL"]["foot"])
    print("FL foot zmin:", round(fb[4], 1), " xmid:", round((fb[0]+fb[1])/2, 1))
