"""
Ori Robot Dog - MASTER PARAMETER SYSTEM
=======================================

SINGLE SOURCE OF TRUTH for every dimension used by the generated CAD.

Design intent
-------------
Ori is a 12-DOF quadruped built around the Hiwonder HTD-45H bus servo.
Scale is deliberately LARGER than the old ~330 mm hobby assumption: the torso
alone is ~300 mm long, standing height ~440 mm. The design avoids blocky
placeholder geometry: every part has a reason to exist, bearing-supported
joints, real fasteners, and A1 Mini (180^3 mm) printable splits.

Verification status legend (used across the project):
    VERIFIED  - measured / taken from manufacturer datasheet
    ASSUMED   - engineering estimate held in a parameter, easily changed
    UNKNOWN   - not yet resolved (blocks dependent geometry)
    BLOCKED   - blocked on an external dependency

All lengths in millimetres, angles in degrees, masses in grams unless noted.
"""

from dataclasses import dataclass, field, asdict
from typing import Dict, Any

# Physical constants
GRAVITY = 9.81  # m/s^2


# ---------------------------------------------------------------------------
# ROBOT-WIDE SCALE
# ---------------------------------------------------------------------------
@dataclass
class RobotScale:
    """Overall envelope. Bigger than the old ~330 mm hobby assumption.
    Hip axes sit at the TOP corners of the torso; legs splay down/out.
    VERIFIED vs HTD-45H body (51.1 mm); overall proportions ASSUMED Spot-class."""
    stance_length: float = 437.0     # MEASURED from built assembly (FL.x - RL.x foot spread). Was 460 (target, wrong).
    stance_width: float = 168.0      # MEASURED foot Y spread. Was 320 (target, wrong).
    hip_height_nominal: float = 300.0  # VERIFIED via assembly (hip servo z = 300.1)
    overall_height: float = 364.0    # MEASURED built top-of-head. Was 440 (target, wrong).
    ground_clearance: float = 190.0  # MEASURED belly-to-floor (hip 300 - torso half 55 - wall 55 = 190)
    body_mass_budget: float = 6000.0 # ASSUMED target all-up mass (g) for torque checks


# ---------------------------------------------------------------------------
# TORSO CHASSIS  (structural central body carrying electronics + 4 hips)
# ---------------------------------------------------------------------------
@dataclass
class Torso:
    length: float = 300.0    # X  (VERIFIED feasible vs A1 Mini by splitting)
    width: float = 150.0     # Y
    height: float = 110.0    # Z (core shell, hips at top corners)
    wall: float = 3.0        # nominal shell wall thickness
    hip_bulkhead_t: float = 8.0   # thick load-entry bulkhead at each hip
    rib_t: float = 4.0        # internal structural rib thickness
    # Hip mounting bulkheads sit at the four TOP corners of the torso plan:
    #   FL, FR at +X end ; RL, RR at -X end. Hip pitch axes are at the TOP face.
    hip_pitch_axis_x: float = 150.0   # |X| of hip pitch axis from torso centre
    hip_axis_y: float = 70.0          # |Y| of hip axis from centreline
    hip_axis_z: float = 55.0          # hip pitch axis height ABOVE torso centre (top corner)
    # Battery bay (3S 2200mAh LiPo, ~106x34x24)
    batt_bay_l: float = 116.0
    batt_bay_w: float = 42.0
    batt_bay_h: float = 30.0
    batt_bay_offset_z: float = -2.0   # bay centre vs torso centre
    # Future arm interface (front centre, recessed)
    arm_port_d: float = 50.0          # bore of the arm shoulder interface
    arm_port_x: float = 140.0         # X of arm port centre (front face)
    arm_port_z: float = 10.0          # Z of arm port centre


# ---------------------------------------------------------------------------
# LEG ARCHITECTURE  (one master leg; mirrored for the other three)
# ---------------------------------------------------------------------------
@dataclass
class Leg:
    """
    Master leg kinematic + structural parameters.

    Kinematic chain (robot-local, leg points +X forward, +Z up):
        HIP (pitch axis at torso corner)
          -> upper_link_length -> KNEE (pitch axis)
          -> lower_link_length -> ANKLE (passive compliant joint)
          -> foot_height -> GROUND
    """
    # Link lengths (centre-to-centre of joint axes)
    upper_link_length: float = 175.0   # ASSUMED (HTD-45H is 51 mm; leg must be longer)
    lower_link_length: float = 175.0   # ASSUMED symmetric for balanced workspace
    # Link structural section (printed carbon-ish box tube)
    link_w: float = 34.0               # width  (Y, out of plane)
    link_t: float = 22.0               # thickness (X, in swing plane)
    link_wall: float = 2.6             # wall thickness of hollow link
    # Joint bearing sizes (code uses 626ZZ at hip+knee+ankle; matches 6 mm shaft)
    hip_bearing_id: float = 6.0        # 626ZZ bore (was 608ZZ by mistake; 608 is 8 mm, won't fit shaft)
    hip_bearing_od: float = 19.0
    hip_bearing_w: float = 6.0
    knee_bearing_id: float = 6.0       # 626ZZ bore (lighter)
    knee_bearing_od: float = 19.0
    knee_bearing_w: float = 6.0
    # Joint limits (mechanical hard stops, not servo firmware)
    hip_pitch_min: float = -55.0
    hip_pitch_max: float = 55.0
    knee_pitch_min: float = -150.0
    knee_pitch_max: float = 0.0        # knee folds backward (revolute, leg-like)
    # Foot
    foot_height: float = 32.0          # sole-to-ankle clearance
    foot_d: float = 48.0               # foot pad diameter
    foot_wall: float = 3.0
    ankle_compliance: float = 6.0      # allowed passive ankle rotation (deg)
    # Actuator placement
    #   Hip pitch servo: mounted in torso bulkhead, output axis = hip pitch axis.
    #   Knee servo: mounted at the KNEE, output axis = knee axis, driving lower link
    #   via a short reduction link (horn) -> torque at link, not bare shaft.
    servo_horn_r: float = 22.0         # effective horn radius (output lever) ASSUMED
    # Cable channel running inside each link
    cable_channel_d: float = 6.0


# ---------------------------------------------------------------------------
# HEAD
# ---------------------------------------------------------------------------
@dataclass
class Head:
    length: float = 90.0    # X
    width: float = 70.0     # Y
    height: float = 70.0    # Z
    neck_d: float = 40.0     # neck interface bore to torso
    neck_len: float = 34.0   # neck column height (pan servo seats at top)
    cam_d: float = 8.0      # main camera lens clear aperture
    cam_fov: float = 120.0  # horizontal FOV (deg) ASSUMED for clearance checks
    mic_ring_d: float = 50.0
    speaker_d: float = 28.0
    mount_standoff: float = 6.0


# ---------------------------------------------------------------------------
# ACTUATOR  -  Hiwonder HTD-45H  (VERIFIED from manufacturer / RobotShop)
# ---------------------------------------------------------------------------
@dataclass
class HTD45H:
    # --- body envelope (VERIFIED) ---
    body_l: float = 51.1      # X  (length, along shaft axis? No - see note)
    body_w: float = 20.14     # Y  (thin dimension)
    body_h: float = 40.0      # Z  (height w/ flange)
    # NOTE on axis convention:
    #   Manufacturer lists LxWxH = 51.1 x 20.14 x 40.
    #   In our model we orient the servo so the output SHAFT points along the
    #   joint axis. The long 51.1 mm body runs perpendicular to the shaft.
    #   We treat: body_long = 51.1 (along the link), body_short = 20.14 (radial),
    #   body_tall = 40.0 (incl. mounting flange, perpendicular to shaft).
    body_long: float = 51.1
    body_short: float = 20.14
    body_tall: float = 40.0
    mass: float = 64.0        # VERIFIED g

    # --- output shaft (VERIFIED from photos: 6 mm double shaft, splined) ---
    shaft_d: float = 6.0
    shaft_len_each_side: float = 8.0   # protrusion each side of body (ASSUMED)
    shaft_thread: float = 3.0          # central retention screw (M3) - ASSUMED

    # --- horn mount circle (VERIFIED from photo analysis: 6 holes, ~25 mm PCD) ---
    horn_pcd: float = 25.0     # pitch-circle diameter of horn retention holes
    horn_holes: int = 6
    horn_hole_d: float = 2.0   # M2 retention holes (ASSUMED)
    horn_center_screw: float = 3.0  # M3x6 centre screw (VERIFIED)

    # --- side mounting flanges (VERIFIED from photo: M3 holes, ~31 mm spacing) ---
    flange_holes_d: float = 3.0      # M3 clearance (VERIFIED)
    flange_spacing: float = 31.0     # centre-to-centre along body_long (VERIFIED)
    flange_offset_from_end: float = 4.0  # hole centre to body end (ASSUMED)
    flange_material_clearance: float = 1.5  # boss margin around hole

    # --- electrical (VERIFIED) ---
    connector_type: str = "PH2.0-3P"
    wire_len: float = 200.0
    voltage_min: float = 9.0
    voltage_max: float = 12.6
    torque_kgcm: float = 45.0   # @11.1V (VERIFIED)
    torque_nm: float = 4.41     # 45 kg.cm -> 4.41 N.m
    speed_sec60: float = 0.18   # sec / 60 deg @11.1V
    rotation_range: float = 240.0
    baud: int = 115200


# ---------------------------------------------------------------------------
# HARDWARE LIBRARY  (real fasteners / bearings / inserts / connectors)
# ---------------------------------------------------------------------------
@dataclass
class Hardware:
    # Fasteners (metric, real)
    screw_M3: float = 3.0
    screw_M2_5: float = 2.5
    screw_M2: float = 2.0
    heat_insert_M3_d: float = 4.0     # heat-set insert outer dia (ASSUMED typical)
    heat_insert_M3_len: float = 5.0
    nut_M3_d: float = 5.5             # across-flats ~5.5
    # Bearings
    bearing_608_id: float = 8.0
    bearing_608_od: float = 22.0
    bearing_608_w: float = 7.0
    bearing_626_id: float = 6.0
    bearing_626_od: float = 19.0
    bearing_626_w: float = 6.0
    # Foot contact switch (Omron D2F-L class, ASSUMED)
    switch_l: float = 12.2
    switch_w: float = 6.0
    switch_h: float = 6.5
    switch_lever_len: float = 12.0    # hinge lever (ASSUMED)
    # Connectors
    ph2_0_pitch: float = 2.0


# ---------------------------------------------------------------------------
# ELECTRONICS ENVELOPES  (VERIFIED / standard)
# ---------------------------------------------------------------------------
@dataclass
class Electronics:
    # Raspberry Pi 4 B
    rpi_l: float = 85.6
    rpi_w: float = 56.5
    rpi_h: float = 21.0      # with headers/stack (ASSUMED)
    rpi_hole_pcd_x: float = 58.0
    rpi_hole_pcd_y: float = 49.0
    rpi_hole_d: float = 2.9   # accepts 2.5 mm bolt (VERIFIED forum)
    # Raspberry Pi Pico (x3)
    pico_l: float = 51.0
    pico_w: float = 21.0
    pico_h: float = 5.0
    pico_hole_d: float = 2.1  # (ASSUMED std pico)
    pico_hole_off: float = 2.2  # offset of hole centre from edge
    # Battery (3S 2200mAh LiPo, HRB/CNHL class)
    batt_l: float = 106.0
    batt_w: float = 34.0
    batt_h: float = 24.0
    batt_mass: float = 168.0
    batt_connector: str = "XT60"
    # Bus servo controller (Hiwonder)
    ssc_l: float = 58.0
    ssc_w: float = 42.0
    ssc_h: float = 12.0
    # IMU breakout (small)
    imu_l: float = 25.0
    imu_w: float = 20.0
    imu_h: float = 5.0
    # Camera (RPi cam / wide module)
    cam_board_l: float = 25.0
    cam_board_w: float = 24.0
    cam_lens_d: float = 8.0


# ---------------------------------------------------------------------------
# MANUFACTURING  (Bambu Lab A1 Mini)
# ---------------------------------------------------------------------------
@dataclass
class Manufacturing:
    printer: str = "Bambu Lab A1 Mini"
    build_x: float = 180.0
    build_y: float = 180.0
    build_z: float = 180.0
    min_wall: float = 1.2         # min printable wall (single extrusion ~0.4 *3)
    min_feature: float = 0.8      # min boss/peg width
    print_clearance: float = 0.3  # clearance between printed mating faces
    assembly_clearance: float = 0.5  # clearance for assembly motion
    bearing_press_fit: float = 0.02  # interference for printed bearing seat (negative)
    insert_hole_d: float = 3.6    # drilled/punched hole for M3 heat insert
    layer_h: float = 0.2


# ---------------------------------------------------------------------------
# ARM  (future 6-DOF manipulator, recessed into torso front port)
# ---------------------------------------------------------------------------
@dataclass
class Arm:
    dof: int = 6
    payload_g: float = 500.0           # target payload (user-specified)
    color: str = "OriGray"             # gray PETG (NOT black per owner standard)
    # Servo: HTD-45H baseline for all 6 joints (consistent, verified, 4.41 N.m)
    joint_servo: str = "HTD-45H"
    # Link lengths (mm) along the arm chain from base flange
    base_reach: float = 30.0           # recess depth into torso bore + yaw housing
    shoulder_len: float = 70.0         # upper arm (shoulder->elbow)
    elbow_len: float = 60.0            # forearm (elbow->wrist)
    wrist_len: float = 28.0            # wrist->gripper base
    # Link cross-section (hollow tube, mm)
    link_w: float = 26.0
    link_t: float = 18.0
    link_wall: float = 2.6
    # Joint horn / bore
    bore_d: float = 6.0                # servo shaft / pin bore
    bolt_pcd: float = 25.0             # servo horn bolt circle
    # Gripper
    grip_span: float = 44.0            # max jaw opening (tuned to finger_pivot_r*2)
    grip_depth: float = 30.0
    finger_w: float = 8.0
    # Mount: matches torso arm port (50 mm bore, flange OD 58, M3 PCD 58)
    mount_bore_d: float = 50.0
    mount_flange_od: float = 58.0
    mount_bolt_pcd: float = 58.0
    mount_bolt_d: float = 3.0
    # Recess: how far the yaw housing sinks into the torso bore
    recess_depth: float = 22.0
    # Mass budget (ASSUMED, pre-BOM): 6x HTD-45H (64g) + links ~ 200g
    mass_assumed_g: float = 600.0
    # --- servo-in-wrist / passive gripper (FINAL, handoff baseline) ---
    # Transmission: grip servo (shaft Z) -> DRIVEN GEAR (on servo shaft, in wrist
    #   block) -> COUPLER GEAR (on tool coupler) -> ECCENTRIC CAM -> two symmetric
    #   passive fingers. No motor/microcontroller/battery in the tool.
    #   Gear center distance fixed at 18 mm (handoff baseline; do not change w/o justification).
    gear_module: float = 1.5          # printed spur-gear module (mm)
    gear_drive_teeth: int = 12        # gear on grip-servo shaft (wrist side)
    gear_pinion_teeth: int = 12       # coupler gear (tool side), same teeth -> 1:1
    @property
    def gear_pitch_r(self):
        return self.gear_module * self.gear_pinion_teeth / 2.0   # = 9.0 mm
    @property
    def gear_center(self):
        # center distance = r_drive + r_pinion = module*(td+tp)/2 ; 12+12 -> 18 mm
        return self.gear_module * (self.gear_drive_teeth + self.gear_pinion_teeth) / 2.0
    # eccentric cam drives the two symmetric fingers (radius = half open->close travel)
    cam_ecc: float = 7.0              # cam eccentricity (mm) -> finger linear travel = 2*cam_ecc
    finger_pivot_r: float = 13.0      # finger pivot ring radius on coupler (mm), z = +/-R
    finger_drive_r: float = 11.0      # finger drive-pin offset from its pivot (mm)
    coupler_d: float = 34.0           # standard tool-coupler interface diameter (mm)
    coupler_face_w: float = 10.0      # coupler face width (mm)
    tool_retain_d: float = 3.0        # M3 tool-retention screw (mm)
    tool_dowel_d: float = 4.0         # alignment dowel (mm)
    # passive backdrive stop: the coupler gear has a hard seat against the wrist
    #   block when the grip servo is off; HTD-45H holding torque (4.41 N.m) also retains.
    grip_servo_y: float = -18.0       # grip-servo center y in wrist block (gear meshes at center 0)
    # ----- metal shaft reinforcement (§4) -----
    # Standardized shaft diameters (mm) with published bending yields. Steel 1045 ~ 250 MPa
    # allowable for these loads (see Validation/analyze_shafts.py). Bearing interface 626ZZ (6mm).
    shaft_d_hip: float = 6.0          # hip load axis (through 626ZZ pair) - 6mm = bearing bore
    shaft_d_knee: float = 6.0         # knee load axis - 6mm = bearing bore
    shaft_d_shoulder: float = 6.0     # shoulder - 6mm = bearing bore
    shaft_d_elbow: float = 6.0        # elbow - 6mm = bearing bore
    shaft_d_wrist: float = 6.0       # wrist pitch/roll - 6mm = bearing bore
    shaft_material: str = "Steel 1045 (ground, ~6mm stock)"
    # wall seat: printed bosses carry the 626ZZ; metal shaft runs through the bearings.
    shaft_seat_od: float = 6.0        # bearing bore (626ZZ id = 6mm)
    shaft_end_margin: float = 4.0     # shaft protrusion past outer bearing for e-clip/circlip


# ---------------------------------------------------------------------------
# AGGREGATE
# ---------------------------------------------------------------------------
@dataclass
class OriParams:
    scale: RobotScale = field(default_factory=RobotScale)
    torso: Torso = field(default_factory=Torso)
    leg: Leg = field(default_factory=Leg)
    head: Head = field(default_factory=Head)
    servo: HTD45H = field(default_factory=HTD45H)
    arm: Arm = field(default_factory=Arm)
    hw: Hardware = field(default_factory=Hardware)
    elec: Electronics = field(default_factory=Electronics)
    mfg: Manufacturing = field(default_factory=Manufacturing)

    def as_dict(self) -> Dict[str, Any]:
        return asdict(self)


# Convenience singleton
PARAMS = OriParams()


# ---------------------------------------------------------------------------
# DERIVED / CHECKS  (used by validation scripts)
# ---------------------------------------------------------------------------
def hip_ground_reach(params: OriParams = PARAMS) -> float:
    """Fully-extended leg length hip axis -> ground (for height sanity)."""
    return params.leg.upper_link_length + params.leg.lower_link_length + params.leg.foot_height


def leg_workspace_check(params: OriParams = PARAMS) -> Dict[str, float]:
    """Basic kinematic reach at nominal stance."""
    L1 = params.leg.upper_link_length
    L2 = params.leg.lower_link_length
    foot = params.leg.foot_height
    hip_z = params.scale.hip_height_nominal
    max_reach = L1 + L2 + foot
    min_reach = abs(L1 - L2) + foot
    return {
        "hip_z": hip_z,
        "max_reach": max_reach,
        "min_reach": min_reach,
        "standing_margin": max_reach - hip_z,  # should be > 0 to reach ground
    }


if __name__ == "__main__":
    import json
    print(json.dumps(PARAMS.as_dict(), indent=2))
    print("\n--- LEG WORKSPACE ---")
    print(json.dumps(leg_workspace_check(), indent=2))
