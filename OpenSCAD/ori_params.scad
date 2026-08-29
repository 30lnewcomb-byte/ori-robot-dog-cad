// ori_params.scad — Ori Robot Dog master parameters (mirrors Parameters/master_parameters.py)
//
// NOTE: These OpenSCAD files are a parametric, human/AI-READABLE summary of Ori.
// The authoritative, higher-fidelity manufactured geometry is the STEP files in the
// CAD_STEP/ folder of this package (NOT here, but still present). Use those for printing.
//
// All dims in millimetres. These are representative values for a readable/viewable model;
// the Python/CadQuery source (Source/) and STEP exports are the source of truth.

// Robot scale
overall_height = 364;
hip_height     = 300;
ground_clear   = 190;

// Stance
stance_length  = 437;
stance_width   = 168;

// Legs (upper = lower = 175)
link_len = 175;
link_w   = 34;
link_t   = 24;
foot_h   = 40;

// Torso (split at x=0 into two halves)
torso_len = 320;
torso_w   = 150;
torso_h   = 110;

// Head
head_d   = 90;

// Arm (6-DOF), mounted front +X
arm_base_x = 140;
arm_yaw_h  = 30;
arm_upper  = 70;
arm_fore   = 60;
arm_wrist  = 28;
grip_r     = 17;   // coupler radius
finger_len = 30;

// Servo (Hiwonder HTD-45H) envelope
servo_l = 51.1;
servo_w = 20.14;
servo_h = 40;

// Bearing (626ZZ)
bearing_od = 19;
bearing_id = 6;
bearing_w  = 6;
