"""
Ori Robot Dog - TORSO CHASSIS.

Structural central body. Carries:
  * 4 hip bulkheads (FL/FR at +X end, RL/RR at -X end) at the four top corners.
  * Battery bay (3S 2200 mAh LiPo 106x34x24).
  * Electronics mounts: RPi 4, 3x Pico, servo controller, IMU.
  * Future arm interface port (front centre, recessed, blanked).
  * Internal ribs, cable channels, service deck (removable top panel).

Hip axes (torso-local, canonical): (x=±150, y=±70, z=+55). The hip servo
output axis is +Y; the servo body is recessed into the bulkhead, shaft pointing
outward (+/-Y) to the leg; the upper-link proximal hub bolts to the servo horn.

Manufacturing: full shell is 300 long (>180). It is SPLIT at x=0 into a front
half (x:0..150) and rear half (x:-150..0), joined by a flat seam with 2
alignment pins + 3 M3 screw bosses designed for plastic fusion.

All dims from Parameters/master_parameters.py (PARAMS.torso / .elec / .hw).
"""
import sys, math
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Master"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from build_common import cq, PARAMS, cyl, box_centered, export
import hardware_lib as hw


def _half_shell(sign: int, p=PARAMS):
    """One longitudinal half of the torso shell (sign=+1 front, -1 rear).
    x spans [0,150] for sign+1 (shifted to [-150,0] by caller), width 150, height 110."""
    L = p.torso.length / 2.0      # 150
    W = p.torso.width
    H = p.torso.height
    wall = p.torso.wall

    # outer shell
    outer = cq.Workplane("XY").box(L, W, H).edges("|Z").fillet(8.0)
    inner = cq.Workplane("XY").box(L - 2 * wall, W - 2 * wall, H - 2 * wall).translate((0, 0, 0))
    shell = outer.cut(inner)
    # shift so this half occupies x in [0, L]
    shell = shell.translate((sign * L / 2.0, 0, 0))

    # top deck flange (perimeter rim for service panel) - thin lip on top
    lip = cq.Workplane("XY").box(L, W, 6.0).translate((sign * L / 2.0, 0, H / 2 - 3.0))
    lip = lip.cut(cq.Workplane("XY").box(L - 2 * wall - 4, W - 2 * wall - 4, 8.0).translate((sign * L / 2.0, 0, H / 2 - 3.0)))
    shell = shell.union(lip)

    # internal ribs along X for stiffness (2 ribs per half)
    for rx in (sign * (L * 0.33), sign * (L * 0.66)):
        rib = cq.Workplane("XY").box(4.0, W - 2 * wall - 2, H - 2 * wall - 4).translate((rx, 0, 0))
        shell = shell.union(rib)

    # ---- hip bulkheads on THIS half (front has FL/FR, rear has RL/RR) ----
    for dy in (+1, -1):
        hx = sign * p.torso.hip_pitch_axis_x
        hy = dy * p.torso.hip_axis_y
        hz = p.torso.hip_axis_z
        shell = shell.union(_hip_bulkhead(hx, hy, hz, p))

    # ---- future ARM port (front half only) ----
    if sign == +1:
        shell = shell.union(_arm_port(p))

    # ---- battery bay (front half, within printable envelope) ----
    bay = cq.Workplane("XY").box(p.elec.batt_l + 6, p.elec.batt_w + 6, p.elec.batt_h + 6).translate((sign * 75, 0, p.torso.batt_bay_offset_z - 6))
    shell = shell.cut(bay)
    # battery retention ledges (one per half across the seam)
    for dy2 in (+1, -1):
        ledge = cq.Workplane("XY").box(p.elec.batt_l + 6, 8, 6).translate((sign * 75, dy2 * (p.elec.batt_w / 2 + 4), p.torso.batt_bay_offset_z - p.elec.batt_h / 2 - 3))
        shell = shell.union(ledge)

    # ---- electronics mounts on floor ----
    shell = shell.union(_elec_mounts(sign, p))

    # ---- seam pins + screw bosses at x=0 face ----
    shell = shell.union(_seam_features(sign, p))
    return shell


def _hip_bulkhead(hx, hy, hz, p=PARAMS):
    """Corner hip bulkhead: thick block with servo pocket + 626ZZ bearing seat
    facing outward (+/-Y) to the leg, plus gusset to the deck."""
    blk = cq.Workplane("XY").box(40.0, 30.0, 40.0).edges("|Z").fillet(3.0)
    blk = blk.translate((hx, hy * 0.7, hz - 10))   # sits at corner, slightly inward
    # servo pocket: 52 x 42 x 22 recess for HTD-45H body (axis Y)
    pocket = cq.Workplane("XY").box(54.0, 24.0, 42.0).translate((hx, hy * 0.5, hz - 10))
    blk = blk.cut(pocket)
    # servo shaft bore (6 mm) through the outer face toward the leg (+/-Y)
    shaft = cyl(p.servo.shaft_d / 2, 30.0, axis="Y").translate((hx, hy, hz - 10))
    blk = blk.cut(shaft)
    # 626ZZ bearing seat (counterbore) on outer face
    seat = cyl(p.hw.bearing_626_od / 2 + 1.5, 12.0, axis="Y").translate((hx, hy, hz - 10))
    blk = blk.union(seat)
    # gusset up to deck
    g = cq.Workplane("XY").box(20.0, 16.0, 30.0).translate((hx, hy * 0.7, hz + 8))
    blk = blk.union(g)
    return blk


def _arm_port(p=PARAMS):
    """Recessed 50 mm bore at front centre (x=+140, z=+10) for the future arm.
    Blanked with a service plate held by 4 M3 inserts. NOT destroyed to install."""
    x0 = p.torso.arm_port_x
    z0 = p.torso.arm_port_z
    # port tube through the front face (front face at x=+150)
    tube = cyl(p.torso.arm_port_d / 2, 16.0, axis="X").translate((x0, 0, z0))
    # flange ring
    flange = cyl(p.torso.arm_port_d / 2 + 8, 6.0, axis="X").translate((x0 + 4, 0, z0))
    # blanking plate (removable service cover)
    plate = cyl(p.torso.arm_port_d / 2 + 6, 4.0, axis="X").translate((x0 + 9, 0, z0))
    # 4 bolt holes for the blanking plate (M3) on a 60 PCD
    for i in range(4):
        a = math.radians(i * 90 + 45)
        bx, by = (p.torso.arm_port_d / 2 + 4) * math.cos(a), (p.torso.arm_port_d / 2 + 4) * math.sin(a)
        plate = plate.cut(cyl(3.0 / 2, 8.0, axis="X").translate((x0 + 9, bx, by + z0)))
    return tube.union(flange).union(plate)


def _elec_mounts(sign, p=PARAMS):
    """Floor-mounted trays for RPi (centre), 3x Pico (rear), servo ctrl (front),
    IMU (centre). Heat-set M3 inserts modelled as recessed bosses."""
    base_z = -p.torso.height / 2 + p.torso.wall + 2
    mounts = cq.Workplane("XY")
    # RPi tray (sits in the front half, within its own printable envelope)
    rpi = cq.Workplane("XY").box(p.elec.rpi_l + 4, p.elec.rpi_w + 4, 4).translate((sign * 75, 0, base_z))
    mounts = mounts.union(rpi)
    # 4 M3 insert bosses for RPi (58x49 PCD, inset)
    for dx in (+1, -1):
        for dy in (+1, -1):
            b = cyl(4.0 / 2, 6.0, axis="Z").translate((sign * 75 + dx * p.elec.rpi_hole_pcd_x / 2, dy * p.elec.rpi_hole_pcd_y / 2, base_z + 3))
            b = b.cut(cyl(3.0 / 2, 8.0, axis="Z"))
            mounts = mounts.union(b)
    # 3x Pico (rear half only) stacked along Y
    if sign == -1:
        for k in range(3):
            pc = cq.Workplane("XY").box(p.elec.pico_l + 4, p.elec.pico_w + 4, 4).translate((-40, (k - 1) * 26, base_z))
            mounts = mounts.union(pc)
    # servo controller (front half)
    if sign == +1:
        ssc = cq.Workplane("XY").box(p.elec.ssc_l + 4, p.elec.ssc_w + 4, 4).translate((40, -20, base_z))
        mounts = mounts.union(ssc)
    return mounts


def _seam_features(sign, p=PARAMS):
    """Flat seam face at x=0: 2 alignment pins (cylinders) + 3 M3 screw bosses
    for plastic-fusion joining of the two halves."""
    feats = cq.Workplane("XY")
    W = p.torso.width
    H = p.torso.height
    # seam plate (thin, closes the open cut face)
    plate = cq.Workplane("XY").box(6.0, W - 2 * p.torso.wall, H - 2 * p.torso.wall).translate((sign * 3.0, 0, 0))
    feats = feats.union(plate)
    # 2 alignment pins (male on +half, female counterbore handled by overlap fit)
    for dy in (+1, -1):
        pin = cyl(4.0 / 2, 14.0, axis="X").translate((sign * 6.0, dy * (W / 2 - 25), 0))
        feats = feats.union(pin)
    # 3 M3 screw bosses across the seam (z levels)
    for z in (-H / 4, 0, H / 4):
        boss = cyl(5.5 / 2 + 1.5, 10.0, axis="X").translate((sign * 6.0, 0, z))
        boss = boss.cut(cyl(3.0 / 2, 14.0, axis="X"))
        feats = feats.union(boss)
    return feats


def make_torso(p=PARAMS, export_parts=True):
    front = _half_shell(+1, p)
    rear = _half_shell(-1, p)
    torso = front.union(rear)
    if export_parts:
        export(front, "torso_front_half", subdir="CAD/Exports/torso")
        export(rear, "torso_rear_half", subdir="CAD/Exports/torso")
        export(torso, "torso_full", subdir="CAD/Exports/torso")
    return {"front": front, "rear": rear, "full": torso}


if __name__ == "__main__":
    import build_common as bc
    out = make_torso()
    for k, v in out.items():
        print(f"{k:6s} size(X,Y,Z)=", [round(r, 1) for r in bc.size_of(v)])
