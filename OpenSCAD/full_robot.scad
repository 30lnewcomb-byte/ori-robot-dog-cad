// ori_full_robot.scad — TOP-LEVEL Ori Robot Dog assembly (viewable, AI-readable).
//
//  *** READ THIS FIRST ***
//  This OpenSCAD model is a PARAMETRIC, READABLE SUMMARY of Ori for viewing and
//  AI discussion. It is NOT the manufactured geometry.
//
//  The authoritative, higher-fidelity CAD is the STEP files in the CAD_STEP/ folder
//  of this package — they are NOT here, but they ARE still in the package (see README).
//  Use CAD_STEP/*.step for printing / machining. Use these .scad files to look at and
//  talk about the robot. ChatGPT and GitHub can read .scad (plain text); they cannot
//  see .step/.stl directly.
//
//  Mirrors CAD/Assemblies/assembly.py (4 legs via transform + arm + head).
include <ori_params.scad>
include <leg.scad>
include <torso.scad>
include <head.scad>
include <arm.scad>

module leg_at(x,y,yaw) {
  translate([x,y,0]) rotate([0,0,yaw]) leg_canonical();
}

// Torso
torso();

// 4 legs (FL,FR,RL,RR) — mirrored transforms
leg_at(+stance_length/4, +stance_width/2,   0);
leg_at(+stance_length/4, -stance_width/2,   0);
leg_at(-stance_length/4, +stance_width/2, 180);
leg_at(-stance_length/4, -stance_width/2, 180);

// Arm
arm();

// Head (on top, forward)
translate([0,0,torso_h/2+head_d/2+10]) head();
